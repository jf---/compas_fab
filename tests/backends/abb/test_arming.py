import pytest

from compas_fab.backends.abb.arming import ArmedCommand
from compas_fab.backends.abb.arming import ControllerId
from compas_fab.backends.abb.arming import FireOnceLedger
from compas_fab.backends.abb.arming import FireReceipt
from compas_fab.backends.abb.arming import IntentDigest
from compas_fab.backends.abb.arming import InvocationNonce
from compas_fab.backends.abb.arming import ObservationRevision
from compas_fab.backends.abb.arming import verify_readback
from compas_fab.backends.abb.errors import CommandAlreadyConsumedError
from compas_fab.backends.abb.errors import CommandArmExpiredError
from compas_fab.backends.abb.errors import CommandPreconditionError
from compas_fab.backends.abb.errors import InvalidArmingInputError
from compas_fab.backends.abb.errors import ReadbackMismatchError


def _controller(endpoint: str = "http://192.168.125.1:80") -> ControllerId:
    return ControllerId.build(endpoint=endpoint, rw_version="7.9", credential_handle="operator-handle")


def _intent() -> IntentDigest:
    return IntentDigest.build("start_rapid", b"T_ROB1")


# ---------------------------------------------------------------- ControllerId


def test_controller_id_equal_by_content() -> None:
    assert _controller() == _controller()
    assert _controller().digest == _controller().digest


@pytest.mark.parametrize(
    ("endpoint", "rw_version", "credential_handle"),
    [
        ("http://other:80", "7.9", "operator-handle"),
        ("http://192.168.125.1:80", "6.15", "operator-handle"),
        ("http://192.168.125.1:80", "7.9", "other-handle"),
    ],
)
def test_controller_id_distinct_by_any_field(endpoint: str, rw_version: str, credential_handle: str) -> None:
    other = ControllerId.build(endpoint=endpoint, rw_version=rw_version, credential_handle=credential_handle)
    assert other != _controller()
    assert other.digest != _controller().digest


@pytest.mark.parametrize("field_name", ["endpoint", "rw_version", "credential_handle"])
def test_controller_id_rejects_empty_field(field_name: str) -> None:
    fields = {"endpoint": "http://x:80", "rw_version": "7.9", "credential_handle": "h"}
    fields[field_name] = "   "
    with pytest.raises(InvalidArmingInputError):
        ControllerId.build(**fields)


def test_controller_id_raw_construction_bypassing_factory_is_rejected() -> None:
    genuine = _controller()
    with pytest.raises(InvalidArmingInputError):
        ControllerId(genuine.endpoint, genuine.rw_version, genuine.credential_handle, genuine.digest)


# ----------------------------------------------------------------- IntentDigest


def test_intent_digest_is_deterministic() -> None:
    assert IntentDigest.build("start_rapid", b"T_ROB1") == IntentDigest.build("start_rapid", b"T_ROB1")


def test_intent_digest_distinct_by_kind_and_payload() -> None:
    assert IntentDigest.build("start_rapid", b"x") != IntentDigest.build("motors_on", b"x")
    assert IntentDigest.build("io", b"DO1=1") != IntentDigest.build("io", b"DO1=0")


def test_intent_digest_length_prefix_prevents_field_bleed() -> None:
    # Without length-prefixing, ("ab", b"c") and ("a", b"bc") would collide.
    assert IntentDigest.build("ab", b"c") != IntentDigest.build("a", b"bc")


@pytest.mark.parametrize(
    ("kind", "payload"),
    [
        ("", b"x"),
        ("start_rapid", "not-bytes"),
    ],
)
def test_intent_digest_rejects_bad_input(kind: object, payload: object) -> None:
    with pytest.raises(InvalidArmingInputError):
        IntentDigest.build(kind, payload)  # type: ignore[arg-type]


def test_intent_digest_raw_construction_rejects_non_hex() -> None:
    with pytest.raises(InvalidArmingInputError):
        IntentDigest("not-a-sha-256-hex-digest")


# ------------------------------------------------------------ ObservationRevision


def test_observation_revision_accepts_zero() -> None:
    assert ObservationRevision.build(0).value == 0


def test_observation_revision_rejects_negative() -> None:
    with pytest.raises(InvalidArmingInputError):
        ObservationRevision.build(-1)


# --------------------------------------------------------------- InvocationNonce


def test_invocation_nonce_rejects_empty() -> None:
    with pytest.raises(InvalidArmingInputError):
        InvocationNonce.build("   ")


def test_invocation_nonce_equal_by_value() -> None:
    assert InvocationNonce.build("edge-42") == InvocationNonce.build("edge-42")


# ------------------------------------------------------------------ ArmedCommand


def test_arm_binds_all_four_fields() -> None:
    intent = _intent()
    controller = _controller()
    revision = ObservationRevision.build(7)
    nonce = InvocationNonce.build("edge-1")

    command = ArmedCommand.arm(intent, controller, revision, nonce)

    assert command.intent == intent
    assert command.controller_id == controller
    assert command.observed_revision == revision
    assert command.nonce == nonce


def test_arm_raw_construction_bypassing_factory_is_rejected() -> None:
    intent = _intent()
    controller = _controller()
    revision = ObservationRevision.build(7)
    nonce = InvocationNonce.build("edge-1")
    with pytest.raises(InvalidArmingInputError):
        ArmedCommand(intent, controller, revision, nonce)


def test_require_current_passes_when_revision_unchanged() -> None:
    command = ArmedCommand.arm(_intent(), _controller(), ObservationRevision.build(4), InvocationNonce.build("edge-1"))
    command.require_current(ObservationRevision.build(4))


def test_require_current_raises_when_revision_moved() -> None:
    command = ArmedCommand.arm(_intent(), _controller(), ObservationRevision.build(4), InvocationNonce.build("edge-1"))
    with pytest.raises(CommandArmExpiredError):
        command.require_current(ObservationRevision.build(5))


# ---------------------------------------------------------------- FireOnceLedger


def test_consume_fires_once_and_returns_receipt() -> None:
    controller = _controller()
    ledger = FireOnceLedger.build(controller)
    intent = _intent()
    command = ArmedCommand.arm(intent, controller, ObservationRevision.build(3), InvocationNonce.build("edge-9"))

    receipt = ledger.consume(command)

    assert isinstance(receipt, FireReceipt)
    assert receipt.intent_digest == intent
    assert receipt.nonce == command.nonce
    assert receipt.controller_id == controller


def test_grasshopper_recompute_replay_same_nonce_is_rejected() -> None:
    # THE core safety property: a Grasshopper recompute re-runs RunScript with the
    # SAME carried nonce value; the ledger must reject the second consume, never re-fire.
    controller = _controller()
    ledger = FireOnceLedger.build(controller)
    intent = _intent()
    revision = ObservationRevision.build(3)

    first = ArmedCommand.arm(intent, controller, revision, InvocationNonce.build("button-edge-42"))
    ledger.consume(first)

    # A fresh nonce object with the identical value == the same user action replayed.
    replayed = ArmedCommand.arm(intent, controller, revision, InvocationNonce.build("button-edge-42"))
    with pytest.raises(CommandAlreadyConsumedError):
        ledger.consume(replayed)


def test_consuming_the_same_command_twice_is_rejected() -> None:
    controller = _controller()
    ledger = FireOnceLedger.build(controller)
    command = ArmedCommand.arm(_intent(), controller, ObservationRevision.build(3), InvocationNonce.build("edge-1"))

    ledger.consume(command)
    with pytest.raises(CommandAlreadyConsumedError):
        ledger.consume(command)


def test_distinct_nonces_each_fire_without_false_rejection() -> None:
    controller = _controller()
    ledger = FireOnceLedger.build(controller)
    intent = _intent()
    revision = ObservationRevision.build(3)

    ledger.consume(ArmedCommand.arm(intent, controller, revision, InvocationNonce.build("edge-1")))
    ledger.consume(ArmedCommand.arm(intent, controller, revision, InvocationNonce.build("edge-2")))


def test_consume_rejects_command_for_a_foreign_controller() -> None:
    ledger = FireOnceLedger.build(_controller("http://192.168.125.1:80"))
    foreign = ArmedCommand.arm(_intent(), _controller("http://10.0.0.9:80"), ObservationRevision.build(3), InvocationNonce.build("edge-1"))
    with pytest.raises(CommandPreconditionError):
        ledger.consume(foreign)


# ---------------------------------------------------------------- verify_readback


def test_verify_readback_passes_on_match() -> None:
    verify_readback(_intent(), _intent())


def test_verify_readback_raises_on_mismatch() -> None:
    with pytest.raises(ReadbackMismatchError):
        verify_readback(IntentDigest.build("io", b"DO1=1"), IntentDigest.build("io", b"DO1=0"))
