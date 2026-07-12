"""Immutable current-or-absent branch publication state."""

from __future__ import annotations

from enum import Enum
from typing import Dict
from typing import Generic
from typing import Optional
from typing import Tuple
from typing import TypeVar
from typing import Union

from attrs import define
from attrs import field

from compas_fab.ghpython.branch_runtime_identity import BranchRuntimeIdentity
from compas_fab.ghpython.branch_runtime_identity import TreeRuntimeSnapshot
from compas_fab.ghpython.branch_runtime_identity import UnknownRuntimeBranchError
from compas_fab.ghpython.tree_coordinates import BranchCoordinate
from compas_fab.ghpython.tree_coordinates import GhPath

T = TypeVar("T")


class BranchOutputContractError(ValueError):
    """Base failure for branch publication state."""


class InvalidBranchOutputStateError(BranchOutputContractError):
    """Raised when branch publication state is internally inconsistent."""


class StaleBranchGenerationError(BranchOutputContractError):
    """Raised when a transition targets an obsolete exact branch identity."""


class BranchDecision(Enum):
    """Observable reconciliation decision for one current branch."""

    ABSENT = "absent"
    CURRENT = "current"
    CLEARED = "cleared"


_OUTPUT_FACTORY_TOKEN = object()


@define(frozen=True, slots=True)
class _PublishedBranch(Generic[T]):
    identity: BranchRuntimeIdentity
    value: T

    def __attrs_post_init__(self) -> None:
        if type(self.identity) is not BranchRuntimeIdentity:
            raise InvalidBranchOutputStateError("Published branch requires an exact runtime identity.")


@define(frozen=True, slots=True)
class BranchOutputState(Generic[T]):
    """Values published only for the exact runtime identity they computed."""

    expected: Tuple[BranchRuntimeIdentity, ...]
    _published: Tuple[_PublishedBranch[T], ...]
    _decisions: Tuple[Tuple[BranchCoordinate, BranchDecision], ...]
    _factory_token: Optional[object] = field(default=None, eq=False, repr=False)

    @classmethod
    def build(cls, snapshot: TreeRuntimeSnapshot) -> "BranchOutputState[T]":
        """Create an absent output slot for every current branch."""
        if type(snapshot) is not TreeRuntimeSnapshot:
            raise InvalidBranchOutputStateError("Branch output state requires an exact runtime snapshot.")
        decisions = tuple((branch.coordinate, BranchDecision.ABSENT) for branch in snapshot.branches)
        return cls(snapshot.branches, (), decisions, _OUTPUT_FACTORY_TOKEN)

    @classmethod
    def _from_parts(
        cls,
        expected: Tuple[BranchRuntimeIdentity, ...],
        published: Tuple[_PublishedBranch[T], ...],
        decisions: Tuple[Tuple[BranchCoordinate, BranchDecision], ...],
    ) -> "BranchOutputState[T]":
        return cls(expected, published, decisions, _OUTPUT_FACTORY_TOKEN)

    def __attrs_post_init__(self) -> None:
        valid = (
            type(self.expected) is tuple
            and all(type(identity) is BranchRuntimeIdentity for identity in self.expected)
            and type(self._published) is tuple
            and all(type(entry) is _PublishedBranch for entry in self._published)
            and type(self._decisions) is tuple
            and self._factory_token is _OUTPUT_FACTORY_TOKEN
        )
        if valid:
            expected_coordinates = tuple(identity.coordinate for identity in self.expected)
            published_coordinates = tuple(entry.identity.coordinate for entry in self._published)
            decision_coordinates = tuple(entry[0] for entry in self._decisions)
            valid = (
                len(set(expected_coordinates)) == len(expected_coordinates)
                and len(set(published_coordinates)) == len(published_coordinates)
                and all(entry.identity in self.expected for entry in self._published)
                and all(
                    type(entry) is tuple
                    and len(entry) == 2
                    and type(entry[0]) is BranchCoordinate
                    and type(entry[1]) is BranchDecision
                    for entry in self._decisions
                )
                and decision_coordinates == tuple(identity.coordinate for identity in self.expected)
            )
        if not valid:
            raise InvalidBranchOutputStateError("Branch output state must exactly match unique expected identities and decisions.")

    @property
    def decisions(self) -> Dict[BranchCoordinate, BranchDecision]:
        """Return current decisions keyed by exact runtime branch coordinate."""
        return dict(self._decisions)

    def __getitem__(self, coordinate: Union[BranchCoordinate, GhPath]) -> BranchDecision:
        if type(coordinate) is not BranchCoordinate and type(coordinate) is not GhPath:
            raise UnknownRuntimeBranchError("Branch decision lookup requires an exact coordinate or GhPath.")
        for candidate, decision in self._decisions:
            if candidate == coordinate or candidate.path == coordinate:
                return decision
        raise UnknownRuntimeBranchError("Branch output state has no decision for the requested path.")

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
        retained = tuple(entry for entry in self._published if entry.identity.coordinate != expected.coordinate)
        published = retained + (_PublishedBranch(expected, value),)
        published_by_coordinate = {entry.identity.coordinate: entry for entry in published}
        ordered = tuple(published_by_coordinate[item.coordinate] for item in self.expected if item.coordinate in published_by_coordinate)
        decisions = tuple(
            (item.coordinate, BranchDecision.CURRENT if item == expected else self[item.coordinate])
            for item in self.expected
        )
        return self._from_parts(self.expected, ordered, decisions)

    def fail(self, identity: BranchRuntimeIdentity) -> "BranchOutputState[T]":
        """Clear one exact current branch after a failed computation."""
        return self._clear(identity)

    def cancel(self, identity: BranchRuntimeIdentity) -> "BranchOutputState[T]":
        """Clear one exact current branch after cancellation."""
        return self._clear(identity)

    def _clear(self, identity: BranchRuntimeIdentity) -> "BranchOutputState[T]":
        expected = self._expected_identity(identity)
        published = tuple(entry for entry in self._published if entry.identity.coordinate != expected.coordinate)
        decisions = tuple(
            (item.coordinate, BranchDecision.CLEARED if item == expected else self[item.coordinate])
            for item in self.expected
        )
        return self._from_parts(self.expected, published, decisions)

    def reconcile(self, snapshot: TreeRuntimeSnapshot) -> "BranchOutputState[T]":
        """Retain exact siblings and clear the changed dependency closure."""
        if type(snapshot) is not TreeRuntimeSnapshot:
            raise InvalidBranchOutputStateError("Output reconciliation requires an exact runtime snapshot.")
        old_by_path = {identity.coordinate.path: identity for identity in self.expected}
        published_by_identity = {entry.identity: entry for entry in self._published}
        published = []
        decisions = []
        for identity in snapshot.branches:
            previous = old_by_path.get(identity.coordinate.path)
            if previous == identity and identity in published_by_identity:
                published.append(published_by_identity[identity])
                decision = BranchDecision.CURRENT
            elif previous is not None and previous != identity:
                decision = BranchDecision.CLEARED
            else:
                decision = BranchDecision.ABSENT
            decisions.append((identity.coordinate, decision))
        return self._from_parts(snapshot.branches, tuple(published), tuple(decisions))
