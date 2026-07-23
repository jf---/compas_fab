"""Decision-independent arming, fire-once, and readback machinery.

Grasshopper re-runs a component's ``RunScript`` on every recompute. A controller
mutation that fired on recompute would re-command a physical robot. This module
is the pure machinery that guarantees a mutation fires exactly once per explicit
user action and is believed only after readback. It holds no network, no
Grasshopper, and no RWS client dependency; the controller layer arms against it.
"""

from __future__ import annotations

from hashlib import sha256
from typing import Optional
from typing import Set
from typing import TypeVar

from attrs import define
from attrs import field

from .errors import CommandAlreadyConsumedError
from .errors import CommandArmExpiredError
from .errors import CommandPreconditionError
from .errors import InvalidArmingInputError
from .errors import ReadbackMismatchError

# Width of the big-endian length prefix that separates concatenated fields so no
# reassignment of bytes across fields can collide two distinct intents.
_LENGTH_PREFIX_BYTES = 8
_SHA256_HEX_LENGTH = sha256().digest_size * 2

# Domain-separation tags: digests of one primitive never collide with another's.
_INTENT_DIGEST_SCHEMA = "compas_fab.abb.intent/v1"
_CONTROLLER_ID_SCHEMA = "compas_fab.abb.controller/v1"

# Factory-evidence sentinels: only the factories hold these, so a raw constructor
# call (whose token defaults to None) cannot forge a derived primitive.
_CONTROLLER_ID_FACTORY_TOKEN = object()
_ARMED_COMMAND_FACTORY_TOKEN = object()


def _length_prefixed(value: bytes) -> bytes:
    return len(value).to_bytes(_LENGTH_PREFIX_BYTES, "big") + value


def _is_hex_digest(value: object) -> bool:
    return isinstance(value, str) and len(value) == _SHA256_HEX_LENGTH and value == value.lower() and all(character in "0123456789abcdef" for character in value)


def _is_present_text(value: object) -> bool:
    return isinstance(value, str) and bool(value.strip())


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise InvalidArmingInputError(message)


def _empty_nonce_set() -> Set[str]:
    return set()


@define(frozen=True, slots=True)
class IntentDigest:
    """SHA-256 identity of a canonical controller-mutation intent."""

    value: str

    @classmethod
    def build(cls, kind: str, payload: bytes) -> IntentDigest:
        """Hash a mutation intent from its command kind and canonical payload.

        Args:
            kind: Non-empty command discriminator (e.g. ``"start_rapid"``).
            payload: Canonical parameter bytes for the command.

        Returns:
            The deterministic digest of the length-prefixed intent.

        Raises:
            InvalidArmingInputError: The kind is empty or the payload is not bytes.
        """
        if not _is_present_text(kind):
            raise InvalidArmingInputError("Intent kind must be non-empty text; got {!r}.".format(kind))
        if not isinstance(payload, bytes):
            raise InvalidArmingInputError("Intent payload must be raw bytes; got {}.".format(type(payload).__name__))
        canonical = _length_prefixed(_INTENT_DIGEST_SCHEMA.encode("ascii")) + _length_prefixed(kind.encode("utf-8")) + _length_prefixed(payload)
        return cls(sha256(canonical).hexdigest())

    def __attrs_post_init__(self) -> None:
        if not _is_hex_digest(self.value):
            raise InvalidArmingInputError("Intent digest must be a lowercase SHA-256 hexadecimal string.")


@define(frozen=True, slots=True)
class ObservationRevision:
    """Monotone marker of the controller state an arm was bound to."""

    value: int

    @classmethod
    def build(cls, value: int) -> ObservationRevision:
        """Wrap a non-negative controller-state revision.

        Args:
            value: The controller observation counter (>= 0).

        Returns:
            The validated revision.

        Raises:
            InvalidArmingInputError: The value is not a non-negative integer.
        """
        return cls(value)

    def __attrs_post_init__(self) -> None:
        # bool is an int subclass; reject it so a truthy flag cannot pose as a revision.
        if type(self.value) is not int or self.value < 0:
            raise InvalidArmingInputError("Observation revision must be a non-negative integer; got {!r}.".format(self.value))


@define(frozen=True, slots=True)
class InvocationNonce:
    """Edge-triggered token standing for exactly one explicit user action."""

    value: str

    @classmethod
    def build(cls, value: str) -> InvocationNonce:
        """Wrap a non-empty edge value carried from a Grasshopper button.

        The same value replayed by a recompute denotes the same action; the
        ledger, not this type, enforces that the replay does not re-fire.

        Args:
            value: The edge token carried on the wire.

        Returns:
            The validated nonce.

        Raises:
            InvalidArmingInputError: The value is empty or whitespace-only.
        """
        return cls(value)

    def __attrs_post_init__(self) -> None:
        if not _is_present_text(self.value):
            raise InvalidArmingInputError("Invocation nonce must be non-empty text; got {!r}.".format(self.value))


def _controller_digest(endpoint: str, rw_version: str, credential_handle: str) -> str:
    canonical = b"".join(
        (
            _length_prefixed(_CONTROLLER_ID_SCHEMA.encode("ascii")),
            _length_prefixed(endpoint.encode("utf-8")),
            _length_prefixed(rw_version.encode("utf-8")),
            _length_prefixed(credential_handle.encode("utf-8")),
        )
    )
    return sha256(canonical).hexdigest()


@define(frozen=True, slots=True)
class ControllerId:
    """Content identity of a controller endpoint, version, and credential handle."""

    endpoint: str
    rw_version: str
    credential_handle: str
    digest: str
    _factory_token: Optional[object] = field(default=None, eq=False, repr=False)

    @classmethod
    def build(cls, endpoint: str, rw_version: str, credential_handle: str) -> ControllerId:
        """Derive a controller identity from its addressing triple.

        Args:
            endpoint: Controller base URL.
            rw_version: RobotWare version string.
            credential_handle: Opaque credential handle (never a secret).

        Returns:
            The content-addressed controller identity.

        Raises:
            InvalidArmingInputError: Any field is empty or whitespace-only.
        """
        for name, value in (("endpoint", endpoint), ("rw_version", rw_version), ("credential_handle", credential_handle)):
            if not _is_present_text(value):
                raise InvalidArmingInputError("Controller {} must be non-empty text; got {!r}.".format(name, value))
        digest = _controller_digest(endpoint, rw_version, credential_handle)
        return cls(endpoint, rw_version, credential_handle, digest, _CONTROLLER_ID_FACTORY_TOKEN)

    def __attrs_post_init__(self) -> None:
        consistent = (
            _is_present_text(self.endpoint)
            and _is_present_text(self.rw_version)
            and _is_present_text(self.credential_handle)
            and self.digest == _controller_digest(self.endpoint, self.rw_version, self.credential_handle)
            and self._factory_token is _CONTROLLER_ID_FACTORY_TOKEN
        )
        if not consistent:
            raise InvalidArmingInputError("Controller identity must be produced by ControllerId.build.")


@define(frozen=True, slots=True)
class ArmedCommand:
    """Single-use binding of an intent to a controller, revision, and nonce."""

    intent: IntentDigest
    controller_id: ControllerId
    observed_revision: ObservationRevision
    nonce: InvocationNonce
    _factory_token: Optional[object] = field(default=None, eq=False, repr=False)

    @classmethod
    def arm(
        cls,
        intent: IntentDigest,
        controller_id: ControllerId,
        observed_revision: ObservationRevision,
        nonce: InvocationNonce,
    ) -> ArmedCommand:
        """Arm a mutation against the controller state it was decided on.

        Args:
            intent: Digest of the intended controller mutation.
            controller_id: The controller the mutation targets.
            observed_revision: Controller revision the arm is bound to.
            nonce: Edge token for the one user action arming this command.

        Returns:
            The armed, single-use command.

        Raises:
            InvalidArmingInputError: Any argument is not the exact primitive type.
        """
        _require(type(intent) is IntentDigest, "Armed command intent must be an IntentDigest.")
        _require(type(controller_id) is ControllerId, "Armed command controller must be a ControllerId.")
        _require(type(observed_revision) is ObservationRevision, "Armed command revision must be an ObservationRevision.")
        _require(type(nonce) is InvocationNonce, "Armed command nonce must be an InvocationNonce.")
        return cls(intent, controller_id, observed_revision, nonce, _ARMED_COMMAND_FACTORY_TOKEN)

    def __attrs_post_init__(self) -> None:
        consistent = (
            type(self.intent) is IntentDigest
            and type(self.controller_id) is ControllerId
            and type(self.observed_revision) is ObservationRevision
            and type(self.nonce) is InvocationNonce
            and self._factory_token is _ARMED_COMMAND_FACTORY_TOKEN
        )
        if not consistent:
            raise InvalidArmingInputError("Armed command must be produced by ArmedCommand.arm.")

    def require_current(self, current_revision: ObservationRevision) -> None:
        """Assert the controller has not moved since this command was armed.

        Args:
            current_revision: The controller's present observation revision.

        Raises:
            InvalidArmingInputError: The argument is not an ObservationRevision.
            CommandArmExpiredError: The controller revision moved since arming.
        """
        _require(type(current_revision) is ObservationRevision, "Current revision must be an ObservationRevision.")
        if current_revision != self.observed_revision:
            raise CommandArmExpiredError("Armed command bound to revision {} but controller is at revision {}.".format(self.observed_revision.value, current_revision.value))


@define(frozen=True, slots=True)
class FireReceipt:
    """Evidence that one intent fired exactly once against a controller."""

    intent_digest: IntentDigest
    nonce: InvocationNonce
    controller_id: ControllerId

    @classmethod
    def build(cls, intent_digest: IntentDigest, nonce: InvocationNonce, controller_id: ControllerId) -> FireReceipt:
        """Issue a receipt for a consumed command.

        Args:
            intent_digest: Digest of the intent that fired.
            nonce: The consumed edge token.
            controller_id: The controller the intent fired against.

        Returns:
            The fire receipt.

        Raises:
            InvalidArmingInputError: Any field is not the exact primitive type.
        """
        _require(type(intent_digest) is IntentDigest, "Fire receipt intent must be an IntentDigest.")
        _require(type(nonce) is InvocationNonce, "Fire receipt nonce must be an InvocationNonce.")
        _require(type(controller_id) is ControllerId, "Fire receipt controller must be a ControllerId.")
        return cls(intent_digest, nonce, controller_id)

    def __attrs_post_init__(self) -> None:
        consistent = type(self.intent_digest) is IntentDigest and type(self.nonce) is InvocationNonce and type(self.controller_id) is ControllerId
        if not consistent:
            raise InvalidArmingInputError("Fire receipt fields must be exact safety-core primitives.")


@define(slots=True, eq=False)
class FireOnceLedger:
    """In-memory register that fires each controller nonce at most once."""

    controller_id: ControllerId
    _consumed: Set[str] = field(factory=_empty_nonce_set, eq=False, repr=False)

    @classmethod
    def build(cls, controller_id: ControllerId) -> FireOnceLedger:
        """Open a fire-once ledger scoped to one controller.

        Args:
            controller_id: The controller whose mutations this ledger guards.

        Returns:
            An empty ledger.

        Raises:
            InvalidArmingInputError: The controller id is not a ControllerId.
        """
        _require(type(controller_id) is ControllerId, "Fire-once ledger must be scoped to a ControllerId.")
        return cls(controller_id)

    def consume(self, armed_command: ArmedCommand) -> FireReceipt:
        """Fire a command once; reject any replay of the same nonce.

        This is the linearization point of the safety core. A Grasshopper
        recompute replays the same nonce value; the second consume is a
        rejection, never a silent re-fire.

        Args:
            armed_command: The command to fire.

        Returns:
            A receipt proving the command fired.

        Raises:
            InvalidArmingInputError: The argument is not an ArmedCommand.
            CommandPreconditionError: The command targets a different controller.
            CommandAlreadyConsumedError: The nonce was already consumed.
        """
        _require(type(armed_command) is ArmedCommand, "Fire-once ledger can only consume an ArmedCommand.")
        if armed_command.controller_id != self.controller_id:
            raise CommandPreconditionError("Armed command targets controller {} but this ledger guards {}.".format(armed_command.controller_id.digest, self.controller_id.digest))
        nonce_value = armed_command.nonce.value
        if nonce_value in self._consumed:
            raise CommandAlreadyConsumedError("Invocation nonce {!r} already fired; a Grasshopper recompute must not re-fire.".format(nonce_value))
        self._consumed.add(nonce_value)
        return FireReceipt.build(armed_command.intent, armed_command.nonce, armed_command.controller_id)


T = TypeVar("T")


def verify_readback(expected: T, observed: T) -> None:
    """Confirm a controller readback matches what was commanded.

    Args:
        expected: The value that was commanded (e.g. an ``IntentDigest`` or an
            I/O value).
        observed: The value the controller reports after the mutation.

    Raises:
        ReadbackMismatchError: The observed value differs from the expected one.
    """
    if observed != expected:
        raise ReadbackMismatchError("Controller readback {!r} does not match commanded {!r}.".format(observed, expected))
