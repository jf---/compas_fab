"""Runtime-only generations and branch dependency closure."""

from __future__ import annotations

from typing import Optional
from typing import Tuple
from typing import Union

from attrs import define
from attrs import field

from compas_fab.ghpython.component_identity import CanonicalField
from compas_fab.ghpython.tree_coordinates import BranchCoordinate
from compas_fab.ghpython.tree_coordinates import GhPath
from compas_fab.ghpython.tree_coordinates import TreeRootId
from compas_fab.ghpython.tree_identity import BranchContentDigest
from compas_fab.ghpython.tree_identity import SourceTreeIdentity
from compas_fab.ghpython.tree_identity import StageTreeIdentity
from compas_fab.ghpython.tree_identity import TreeIdentity


class BranchRuntimeContractError(ValueError):
    """Base failure for runtime-only branch identities."""


class InvalidSolveGenerationError(BranchRuntimeContractError):
    """Raised when a root/shared-input generation is invalid."""


class InvalidBranchRequestGenerationError(BranchRuntimeContractError):
    """Raised when a branch request generation is invalid."""


class InvalidBranchRuntimeIdentityError(BranchRuntimeContractError):
    """Raised when a branch runtime identity is malformed."""


class InvalidTreeRuntimeSnapshotError(BranchRuntimeContractError):
    """Raised when a runtime snapshot is inconsistent with its content."""


class InvalidSharedInputsChangedError(BranchRuntimeContractError):
    """Raised when shared-input change evidence is malformed."""


class InvalidSharedInputsUnchangedError(BranchRuntimeContractError):
    """Raised when unchanged shared-input evidence bypasses its factory."""


class InvalidSharedInputTransitionError(BranchRuntimeContractError):
    """Raised when runtime advancement receives an unknown shared-input state."""


class UnknownRuntimeBranchError(BranchRuntimeContractError):
    """Raised when a runtime snapshot has no requested exact branch."""


class StaleBranchRequestRetryError(BranchRuntimeContractError):
    """Raised when retry advancement targets an obsolete branch request."""


_RUNTIME_FACTORY_TOKEN = object()


@define(frozen=True, slots=True)
class SolveGeneration:
    """Monotonic root/shared-input epoch."""

    value: int

    @classmethod
    def build(cls, value: int) -> "SolveGeneration":
        return cls(value)

    def __attrs_post_init__(self) -> None:
        if type(self.value) is not int or self.value < 0:
            raise InvalidSolveGenerationError("Solve generation must be an exact non-negative integer.")

    def next(self) -> "SolveGeneration":
        """Return the next root/shared-input epoch."""
        return SolveGeneration.build(self.value + 1)


@define(frozen=True, slots=True)
class BranchRequestGeneration:
    """Monotonic request epoch scoped to one canonical branch path."""

    value: int

    @classmethod
    def build(cls, value: int) -> "BranchRequestGeneration":
        return cls(value)

    def __attrs_post_init__(self) -> None:
        if type(self.value) is not int or self.value < 0:
            raise InvalidBranchRequestGenerationError("Branch request generation must be an exact non-negative integer.")

    def next(self) -> "BranchRequestGeneration":
        """Return the next request epoch for this branch."""
        return BranchRequestGeneration.build(self.value + 1)


@define(frozen=True, slots=True)
class BranchRuntimeIdentity:
    """Exact coordinate, content, and generations required for publication."""

    coordinate: BranchCoordinate
    content_digest: BranchContentDigest
    solve_generation: SolveGeneration
    request_generation: BranchRequestGeneration

    @classmethod
    def build(
        cls,
        coordinate: BranchCoordinate,
        content_digest: BranchContentDigest,
        solve_generation: SolveGeneration,
        request_generation: BranchRequestGeneration,
    ) -> "BranchRuntimeIdentity":
        return cls(coordinate, content_digest, solve_generation, request_generation)

    def __attrs_post_init__(self) -> None:
        if (
            type(self.coordinate) is not BranchCoordinate
            or type(self.content_digest) is not BranchContentDigest
            or type(self.solve_generation) is not SolveGeneration
            or type(self.request_generation) is not BranchRequestGeneration
        ):
            raise InvalidBranchRuntimeIdentityError("Branch runtime identity requires exact coordinate, content, and generation values.")


@define(frozen=True, slots=True)
class SharedInputsChanged:
    """Exact non-empty evidence naming changed shared inputs."""

    fields: Tuple[CanonicalField, ...]
    _factory_token: Optional[object] = field(default=None, eq=False, repr=False)

    @classmethod
    def build(cls, fields: Tuple[CanonicalField, ...]) -> "SharedInputsChanged":
        return cls(fields, _RUNTIME_FACTORY_TOKEN)

    def __attrs_post_init__(self) -> None:
        valid = (
            type(self.fields) is tuple
            and bool(self.fields)
            and all(type(value) is CanonicalField for value in self.fields)
            and len({value.name for value in self.fields}) == len(self.fields)
            and self._factory_token is _RUNTIME_FACTORY_TOKEN
        )
        if not valid:
            raise InvalidSharedInputsChangedError("Shared-input changes require a factory-built non-empty tuple of unique exact fields.")


@define(frozen=True, slots=True)
class SharedInputsUnchanged:
    """Factory-built evidence that no shared input changed."""

    _factory_token: Optional[object] = field(default=None, eq=False, repr=False)

    @classmethod
    def build(cls) -> "SharedInputsUnchanged":
        return cls(_RUNTIME_FACTORY_TOKEN)

    def __attrs_post_init__(self) -> None:
        if self._factory_token is not _RUNTIME_FACTORY_TOKEN:
            raise InvalidSharedInputsUnchangedError("Unchanged shared-input evidence must be factory-built.")


SharedInputTransition = Union[SharedInputsChanged, SharedInputsUnchanged]


def _is_tree_identity(content: object) -> bool:
    return type(content) is SourceTreeIdentity or type(content) is StageTreeIdentity


@define(frozen=True, slots=True)
class TreeRuntimeSnapshot:
    """Runtime routing and generations paired with root-free content identity."""

    root_id: TreeRootId
    content: TreeIdentity
    solve_generation: SolveGeneration
    branches: Tuple[BranchRuntimeIdentity, ...]
    _factory_token: Optional[object] = field(default=None, eq=False, repr=False)

    @classmethod
    def initial(cls, content: TreeIdentity, root_id: TreeRootId) -> "TreeRuntimeSnapshot":
        """Create generation zero for every canonical content branch."""
        if not _is_tree_identity(content) or type(root_id) is not TreeRootId:
            raise InvalidTreeRuntimeSnapshotError("Initial runtime snapshot requires exact content and root routing identity.")
        solve = SolveGeneration.build(0)
        branches = tuple(
            BranchRuntimeIdentity.build(
                BranchCoordinate.build(root_id, path),
                digest,
                solve,
                BranchRequestGeneration.build(0),
            )
            for path, digest in zip(content.topology.paths, content.branch_digests)
        )
        return cls(root_id, content, solve, branches, _RUNTIME_FACTORY_TOKEN)

    @classmethod
    def build(
        cls,
        root_id: TreeRootId,
        content: TreeIdentity,
        solve_generation: SolveGeneration,
        branches: Tuple[BranchRuntimeIdentity, ...],
    ) -> "TreeRuntimeSnapshot":
        """Build a validated runtime snapshot from exact state."""
        return cls(root_id, content, solve_generation, branches, _RUNTIME_FACTORY_TOKEN)

    def __attrs_post_init__(self) -> None:
        valid = (
            type(self.root_id) is TreeRootId
            and _is_tree_identity(self.content)
            and type(self.solve_generation) is SolveGeneration
            and type(self.branches) is tuple
            and all(type(branch) is BranchRuntimeIdentity for branch in self.branches)
            and self._factory_token is _RUNTIME_FACTORY_TOKEN
        )
        if valid:
            expected_paths = self.content.topology.paths
            expected_digests = self.content.branch_digests
            actual_paths = tuple(branch.coordinate.path for branch in self.branches)
            actual_digests = tuple(branch.content_digest for branch in self.branches)
            valid = (
                actual_paths == expected_paths
                and actual_digests == expected_digests
                and all(branch.coordinate.root_id == self.root_id for branch in self.branches)
                and all(branch.solve_generation == self.solve_generation for branch in self.branches)
            )
        if not valid:
            raise InvalidTreeRuntimeSnapshotError("Runtime snapshot must exactly match content paths, digests, root, and solve generation.")

    def branch(self, path: GhPath) -> BranchRuntimeIdentity:
        """Return one branch runtime identity by exact canonical path."""
        if type(path) is not GhPath:
            raise UnknownRuntimeBranchError("Runtime branch lookup requires an exact GhPath.")
        for branch in self.branches:
            if branch.coordinate.path == path:
                return branch
        raise UnknownRuntimeBranchError("Runtime snapshot has no branch at the requested path.")


def _root_topology(content: TreeIdentity) -> Tuple[Tuple[GhPath, int], ...]:
    """Return branch-set and cardinality dependencies; null/order stay branch-local."""
    return tuple(zip(content.topology.paths, content.topology.item_counts))


def advance_branch_request(
    previous: TreeRuntimeSnapshot,
    identity: BranchRuntimeIdentity,
) -> TreeRuntimeSnapshot:
    """Advance only one exact branch request generation for retry."""
    if type(previous) is not TreeRuntimeSnapshot or type(identity) is not BranchRuntimeIdentity:
        raise StaleBranchRequestRetryError("Branch retry requires an exact current branch request identity.")
    branches = []
    found = False
    for branch in previous.branches:
        if branch.coordinate == identity.coordinate:
            if branch != identity:
                raise StaleBranchRequestRetryError("Branch retry targets obsolete content or generations.")
            found = True
            branches.append(
                BranchRuntimeIdentity.build(
                    branch.coordinate,
                    branch.content_digest,
                    branch.solve_generation,
                    branch.request_generation.next(),
                )
            )
        else:
            branches.append(branch)
    if not found:
        raise UnknownRuntimeBranchError("Runtime snapshot has no exact branch coordinate to retry.")
    return TreeRuntimeSnapshot.build(
        previous.root_id,
        previous.content,
        previous.solve_generation,
        tuple(branches),
    )


def advance_runtime(
    previous: TreeRuntimeSnapshot,
    content: TreeIdentity,
    shared_inputs: SharedInputTransition,
    *,
    root_id: Optional[TreeRootId] = None,
) -> TreeRuntimeSnapshot:
    """Advance only the exact generation dependency closure."""
    if type(previous) is not TreeRuntimeSnapshot or not _is_tree_identity(content):
        raise InvalidTreeRuntimeSnapshotError("Runtime advancement requires an exact prior snapshot and content identity.")
    if type(shared_inputs) is not SharedInputsChanged and type(shared_inputs) is not SharedInputsUnchanged:
        raise InvalidSharedInputTransitionError("Runtime advancement requires exact shared-input change evidence.")
    resolved_root = previous.root_id if root_id is None else root_id
    if type(resolved_root) is not TreeRootId:
        raise InvalidTreeRuntimeSnapshotError("Runtime advancement requires an exact root routing identity.")

    full_invalidation = (
        type(shared_inputs) is SharedInputsChanged
        or resolved_root != previous.root_id
        or _root_topology(content) != _root_topology(previous.content)
    )
    solve = previous.solve_generation.next() if full_invalidation else previous.solve_generation
    prior_by_path = {branch.coordinate.path: branch for branch in previous.branches}
    branches = []
    for path, digest in zip(content.topology.paths, content.branch_digests):
        prior = prior_by_path.get(path)
        if prior is None:
            request = BranchRequestGeneration.build(0)
        elif full_invalidation or prior.content_digest != digest:
            request = prior.request_generation.next()
        else:
            request = prior.request_generation
        branches.append(
            BranchRuntimeIdentity.build(
                BranchCoordinate.build(resolved_root, path),
                digest,
                solve,
                request,
            )
        )
    return TreeRuntimeSnapshot.build(resolved_root, content, solve, tuple(branches))
