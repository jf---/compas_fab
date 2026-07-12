"""Immutable current-or-absent branch publication state."""

from __future__ import annotations

from enum import Enum
from typing import Dict
from typing import Generic
from typing import Optional
from typing import Tuple
from typing import TypeVar

from attrs import define
from attrs import field

from compas_fab.ghpython.branch_runtime_identity import BranchRuntimeIdentity
from compas_fab.ghpython.branch_runtime_identity import TreeRuntimeSnapshot
from compas_fab.ghpython.branch_runtime_identity import UnknownRuntimeBranchError
from compas_fab.ghpython.tree_coordinates import BranchCoordinate

T = TypeVar("T")


class BranchOutputContractError(ValueError):
    """Base failure for branch publication state."""


class InvalidBranchOutputStateError(BranchOutputContractError):
    """Raised when branch publication state is internally inconsistent."""


class InvalidBranchDecisionStateError(InvalidBranchOutputStateError):
    """Raised when decision entries or current-publication state disagree."""


class StaleBranchGenerationError(BranchOutputContractError):
    """Raised when a transition targets an obsolete exact branch identity."""


class BranchTerminalTransitionError(BranchOutputContractError):
    """Base failure for duplicate or conflicting terminal outcomes."""


class DuplicateBranchTerminalTransitionError(BranchTerminalTransitionError):
    """Raised when the same terminal outcome is reported twice."""


class ConflictingBranchTerminalTransitionError(BranchTerminalTransitionError):
    """Raised when one request receives incompatible terminal outcomes."""


class RetiredBranchRequestError(ConflictingBranchTerminalTransitionError):
    """Raised when success arrives after failure or cancellation retired a request."""


class BranchDecision(Enum):
    """Observable reconciliation decision for one current branch."""

    ABSENT = "absent"
    CURRENT = "current"
    CLEARED = "cleared"


class _BranchTerminal(Enum):
    PUBLISHED = "published"
    FAILED = "failed"
    CANCELLED = "cancelled"


_OUTPUT_FACTORY_TOKEN = object()


@define(frozen=True, slots=True)
class _PublishedBranch(Generic[T]):
    identity: BranchRuntimeIdentity
    value: T

    def __attrs_post_init__(self) -> None:
        if type(self.identity) is not BranchRuntimeIdentity:
            raise InvalidBranchOutputStateError("Published branch requires an exact runtime identity.")


@define(frozen=True, slots=True)
class _TerminalBranch:
    identity: BranchRuntimeIdentity
    terminal: _BranchTerminal

    def __attrs_post_init__(self) -> None:
        if type(self.identity) is not BranchRuntimeIdentity or type(self.terminal) is not _BranchTerminal:
            raise InvalidBranchOutputStateError("Terminal branch requires exact runtime identity and terminal outcome.")


@define(frozen=True, slots=True)
class BranchOutputState(Generic[T]):
    """Values published only for the exact runtime identity they computed."""

    expected: Tuple[BranchRuntimeIdentity, ...]
    _published: Tuple[_PublishedBranch[T], ...]
    _decisions: Tuple[Tuple[BranchCoordinate, BranchDecision], ...]
    _terminals: Tuple[_TerminalBranch, ...]
    _factory_token: Optional[object] = field(default=None, eq=False, repr=False)

    @classmethod
    def build(cls, snapshot: TreeRuntimeSnapshot) -> "BranchOutputState[T]":
        """Create an absent output slot for every current branch."""
        if type(snapshot) is not TreeRuntimeSnapshot:
            raise InvalidBranchOutputStateError("Branch output state requires an exact runtime snapshot.")
        decisions = tuple((branch.coordinate, BranchDecision.ABSENT) for branch in snapshot.branches)
        return cls(snapshot.branches, (), decisions, (), _OUTPUT_FACTORY_TOKEN)

    @classmethod
    def _from_parts(
        cls,
        expected: Tuple[BranchRuntimeIdentity, ...],
        published: Tuple[_PublishedBranch[T], ...],
        decisions: Tuple[Tuple[BranchCoordinate, BranchDecision], ...],
        terminals: Tuple[_TerminalBranch, ...],
    ) -> "BranchOutputState[T]":
        return cls(expected, published, decisions, terminals, _OUTPUT_FACTORY_TOKEN)

    def __attrs_post_init__(self) -> None:
        valid = (
            type(self.expected) is tuple
            and all(type(identity) is BranchRuntimeIdentity for identity in self.expected)
            and type(self._published) is tuple
            and all(type(entry) is _PublishedBranch for entry in self._published)
            and type(self._decisions) is tuple
            and type(self._terminals) is tuple
            and all(type(entry) is _TerminalBranch for entry in self._terminals)
            and self._factory_token is _OUTPUT_FACTORY_TOKEN
        )
        if not valid:
            raise InvalidBranchOutputStateError("Branch output state must exactly match unique expected identities and decisions.")
        if any(
            type(entry) is not tuple
            or len(entry) != 2
            or type(entry[0]) is not BranchCoordinate
            or type(entry[1]) is not BranchDecision
            for entry in self._decisions
        ):
            raise InvalidBranchDecisionStateError("Branch decisions require exact coordinate-decision pairs.")

        expected_coordinates = tuple(identity.coordinate for identity in self.expected)
        published_identities_in_order = tuple(entry.identity for entry in self._published)
        published_coordinates = tuple(identity.coordinate for identity in published_identities_in_order)
        terminal_identities = tuple(entry.identity for entry in self._terminals)
        decision_coordinates = tuple(entry[0] for entry in self._decisions)
        published_identity_set = set(published_identities_in_order)
        terminal_identity_set = set(terminal_identities)
        if (
            len(set(expected_coordinates)) != len(expected_coordinates)
            or len(set(published_coordinates)) != len(published_coordinates)
            or len(set(terminal_identities)) != len(terminal_identities)
            or any(entry.identity not in self.expected for entry in self._published)
            or any(entry.identity not in self.expected for entry in self._terminals)
            or published_identities_in_order
            != tuple(identity for identity in self.expected if identity in published_identity_set)
            or terminal_identities
            != tuple(identity for identity in self.expected if identity in terminal_identity_set)
            or decision_coordinates != expected_coordinates
        ):
            raise InvalidBranchOutputStateError("Branch output state identities must be unique, current, and canonically ordered.")

        published_identities = {entry.identity for entry in self._published}
        published_terminals = {
            entry.identity
            for entry in self._terminals
            if entry.terminal is _BranchTerminal.PUBLISHED
        }
        current_decisions = {
            coordinate
            for coordinate, decision in self._decisions
            if decision is BranchDecision.CURRENT
        }
        decision_by_coordinate = dict(self._decisions)
        retired_terminal_coordinates = {
            entry.identity.coordinate
            for entry in self._terminals
            if entry.terminal is not _BranchTerminal.PUBLISHED
        }
        if (
            published_identities != published_terminals
            or current_decisions
            != {identity.coordinate for identity in published_identities}
            or any(
                decision_by_coordinate[coordinate] is not BranchDecision.CLEARED
                for coordinate in retired_terminal_coordinates
            )
        ):
            raise InvalidBranchDecisionStateError("CURRENT decisions must correspond exactly to published terminal values.")

    @property
    def decisions(self) -> Dict[BranchCoordinate, BranchDecision]:
        """Return current decisions keyed by exact runtime branch coordinate."""
        return dict(self._decisions)

    def __getitem__(self, coordinate: BranchCoordinate) -> BranchDecision:
        if type(coordinate) is not BranchCoordinate:
            raise UnknownRuntimeBranchError("Branch decision lookup requires an exact BranchCoordinate.")
        for candidate, decision in self._decisions:
            if candidate == coordinate:
                return decision
        raise UnknownRuntimeBranchError("Branch output state has no decision for the requested exact coordinate.")

    def _expected_identity(self, identity: BranchRuntimeIdentity) -> BranchRuntimeIdentity:
        if type(identity) is not BranchRuntimeIdentity:
            raise StaleBranchGenerationError("Branch transition requires an exact runtime identity.")
        for expected in self.expected:
            if expected.coordinate == identity.coordinate:
                if expected != identity:
                    raise StaleBranchGenerationError("Branch transition targets a stale content or generation identity.")
                return expected
        raise StaleBranchGenerationError("Branch transition targets a stale or unknown coordinate.")

    def current(self, identity: BranchRuntimeIdentity) -> Optional[T]:
        """Return the exact current output, or absence for a changed generation."""
        if type(identity) is not BranchRuntimeIdentity:
            raise UnknownRuntimeBranchError("Current-output lookup requires an exact branch runtime identity.")
        matching_coordinate = any(expected.coordinate == identity.coordinate for expected in self.expected)
        if not matching_coordinate:
            raise UnknownRuntimeBranchError("Branch output state has no requested exact coordinate.")
        for entry in self._published:
            if entry.identity == identity:
                return entry.value
        return None

    def publish(self, identity: BranchRuntimeIdentity, value: T) -> "BranchOutputState[T]":
        """Publish only into the current exact content/generation identity."""
        expected = self._expected_identity(identity)
        self._require_open_terminal(expected, _BranchTerminal.PUBLISHED)
        published = self._published + (_PublishedBranch(expected, value),)
        published_by_coordinate = {entry.identity.coordinate: entry for entry in published}
        ordered = tuple(published_by_coordinate[item.coordinate] for item in self.expected if item.coordinate in published_by_coordinate)
        decisions = tuple(
            (item.coordinate, BranchDecision.CURRENT if item == expected else self[item.coordinate])
            for item in self.expected
        )
        terminals = self._ordered_terminals(self._terminals + (_TerminalBranch(expected, _BranchTerminal.PUBLISHED),))
        return self._from_parts(self.expected, ordered, decisions, terminals)

    def fail(self, identity: BranchRuntimeIdentity) -> "BranchOutputState[T]":
        """Clear one exact current branch after a failed computation."""
        return self._retire(identity, _BranchTerminal.FAILED)

    def cancel(self, identity: BranchRuntimeIdentity) -> "BranchOutputState[T]":
        """Clear one exact current branch after cancellation."""
        return self._retire(identity, _BranchTerminal.CANCELLED)

    def _require_open_terminal(
        self,
        identity: BranchRuntimeIdentity,
        requested: _BranchTerminal,
    ) -> None:
        for entry in self._terminals:
            if entry.identity != identity:
                continue
            if entry.terminal is requested:
                raise DuplicateBranchTerminalTransitionError("Branch request already received this terminal outcome.")
            if requested is _BranchTerminal.PUBLISHED:
                raise RetiredBranchRequestError("Failed or cancelled branch request cannot publish a late success.")
            raise ConflictingBranchTerminalTransitionError("Branch request already received a conflicting terminal outcome.")

    def _ordered_terminals(
        self,
        terminals: Tuple[_TerminalBranch, ...],
    ) -> Tuple[_TerminalBranch, ...]:
        by_identity = {entry.identity: entry for entry in terminals}
        return tuple(by_identity[identity] for identity in self.expected if identity in by_identity)

    def _retire(
        self,
        identity: BranchRuntimeIdentity,
        terminal: _BranchTerminal,
    ) -> "BranchOutputState[T]":
        expected = self._expected_identity(identity)
        self._require_open_terminal(expected, terminal)
        decisions = tuple(
            (item.coordinate, BranchDecision.CLEARED if item == expected else self[item.coordinate])
            for item in self.expected
        )
        terminals = self._ordered_terminals(self._terminals + (_TerminalBranch(expected, terminal),))
        return self._from_parts(self.expected, self._published, decisions, terminals)

    def reconcile(self, snapshot: TreeRuntimeSnapshot) -> "BranchOutputState[T]":
        """Retain exact siblings and clear the changed dependency closure."""
        if type(snapshot) is not TreeRuntimeSnapshot:
            raise InvalidBranchOutputStateError("Output reconciliation requires an exact runtime snapshot.")
        old_by_path = {identity.coordinate.path: identity for identity in self.expected}
        published_by_identity = {entry.identity: entry for entry in self._published}
        terminal_by_identity = {entry.identity: entry for entry in self._terminals}
        published = []
        terminals = []
        decisions = []
        for identity in snapshot.branches:
            previous = old_by_path.get(identity.coordinate.path)
            if previous == identity and identity in terminal_by_identity:
                terminal = terminal_by_identity[identity]
                terminals.append(terminal)
                if terminal.terminal is _BranchTerminal.PUBLISHED:
                    published.append(published_by_identity[identity])
                    decision = BranchDecision.CURRENT
                else:
                    decision = BranchDecision.CLEARED
            elif previous is not None and previous != identity:
                decision = BranchDecision.CLEARED
            else:
                decision = BranchDecision.ABSENT
            decisions.append((identity.coordinate, decision))
        return self._from_parts(
            snapshot.branches,
            tuple(published),
            tuple(decisions),
            tuple(terminals),
        )
