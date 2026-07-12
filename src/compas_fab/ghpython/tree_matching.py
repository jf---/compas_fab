"""Exact branch-local matching without Grasshopper list heuristics."""

from __future__ import annotations

from enum import Enum
from typing import Optional
from typing import Tuple
from typing import TypeVar
from typing import cast

from attrs import define

from compas_fab.ghpython.item_values import Scalar
from compas_fab.ghpython.tree_coordinates import GhPath
from compas_fab.ghpython.tree_coordinates import TreeRootId
from compas_fab.ghpython.tree_values import Tree
from compas_fab.ghpython.tree_values import TreeBranch
from compas_fab.ghpython.tree_values import TreeItem

InputT = TypeVar("InputT")


class TreeMatchingError(ValueError):
    """Base failure for exact tree matching."""


class InvalidMatchInputError(TreeMatchingError):
    """Raised when a named match input violates its declared role."""


class InvalidMatchPolicyError(TreeMatchingError):
    """Raised when a match policy is structurally invalid."""


class MatchPolicyInputMismatchError(TreeMatchingError):
    """Raised when ordered policy declarations do not equal ordered inputs."""


class MissingExactTreeAnchorError(TreeMatchingError):
    """Raised when no exact tree can provide output topology."""


class BranchSetMismatchError(TreeMatchingError):
    """Raised when exact trees do not contain identical ordered paths."""


class BranchLengthMismatchError(TreeMatchingError):
    """Raised when corresponding exact branches have unequal lengths."""


class ImplicitSingletonBroadcastError(BranchLengthMismatchError):
    """Raised when a one-item tree would need implicit repetition."""


class InvalidMatchedTreeError(TreeMatchingError):
    """Raised when a matched output is structurally invalid."""


class MatchRole(Enum):
    EXACT_TREE = "exact_tree"
    BROADCASTABLE_SCALAR = "broadcastable_scalar"
    GLOBAL_ATOMIC = "global_atomic"


@define(frozen=True, slots=True)
class MatchInput:
    """One named payload with an explicit matching authority."""

    name: str
    role: MatchRole
    payload: object

    @classmethod
    def tree(cls, name: str, value: Tree[InputT]) -> "MatchInput":
        return cls(name, MatchRole.EXACT_TREE, value)

    @classmethod
    def scalar(cls, name: str, value: Scalar[InputT]) -> "MatchInput":
        return cls(name, MatchRole.BROADCASTABLE_SCALAR, value)

    @classmethod
    def global_atomic(cls, name: str, value: InputT) -> "MatchInput":
        return cls(name, MatchRole.GLOBAL_ATOMIC, value)

    def __attrs_post_init__(self) -> None:
        if type(self.name) is not str or not self.name or self.name != self.name.strip():
            raise InvalidMatchInputError("Match input name must be non-empty text without surrounding whitespace.")
        if type(self.role) is not MatchRole:
            raise InvalidMatchInputError("Match input role must be an exact MatchRole value.")
        if self.role is MatchRole.EXACT_TREE and type(self.payload) is not Tree:
            raise InvalidMatchInputError("EXACT_TREE input requires an exact Tree value.")
        if self.role is MatchRole.BROADCASTABLE_SCALAR and type(self.payload) is not Scalar:
            raise InvalidMatchInputError("BROADCASTABLE_SCALAR input requires an exact Scalar value.")

    def exact_tree(self) -> Tree[object]:
        if self.role is not MatchRole.EXACT_TREE or type(self.payload) is not Tree:
            raise InvalidMatchInputError("Only EXACT_TREE input contains a tree.")
        return cast(Tree[object], self.payload)

    def broadcast_scalar(self) -> Scalar[object]:
        if self.role is not MatchRole.BROADCASTABLE_SCALAR or type(self.payload) is not Scalar:
            raise InvalidMatchInputError("Only BROADCASTABLE_SCALAR input contains a scalar.")
        return cast(Scalar[object], self.payload)


@define(frozen=True, slots=True)
class MatchPolicy:
    """Exact ordered names and roles accepted by one consumer."""

    names: Tuple[str, ...]
    roles: Tuple[MatchRole, ...]

    @classmethod
    def build(cls, names: Tuple[str, ...], roles: Tuple[MatchRole, ...]) -> "MatchPolicy":
        return cls(names, roles)

    def __attrs_post_init__(self) -> None:
        if type(self.names) is not tuple or type(self.roles) is not tuple:
            raise InvalidMatchPolicyError("Match policy names and roles must be exact tuples.")
        if not self.names or len(self.names) != len(self.roles):
            raise InvalidMatchPolicyError("Match policy requires equal non-empty name and role tuples.")
        if any(type(name) is not str or not name or name != name.strip() for name in self.names):
            raise InvalidMatchPolicyError("Match policy names must be non-empty text without surrounding whitespace.")
        if len(set(self.names)) != len(self.names):
            raise InvalidMatchPolicyError("Match policy names must be unique.")
        if any(type(role) is not MatchRole for role in self.roles):
            raise InvalidMatchPolicyError("Match policy roles must be exact MatchRole values.")


@define(frozen=True, slots=True)
class MatchedGlobal:
    """One operation-scoped atomic value, retained exactly once."""

    name: str
    value: object

    @classmethod
    def build(cls, name: str, value: object) -> "MatchedGlobal":
        return cls(name, value)

    def __attrs_post_init__(self) -> None:
        if type(self.name) is not str or not self.name:
            raise InvalidMatchedTreeError("Matched global requires a non-empty exact name.")


@define(frozen=True, slots=True)
class MatchedRow:
    """One branch-local exact-tree row plus declared scalar values."""

    items: Tuple[TreeItem[object], ...]

    @classmethod
    def build(cls, items: Tuple[TreeItem[object], ...]) -> "MatchedRow":
        return cls(items)

    def __attrs_post_init__(self) -> None:
        if type(self.items) is not tuple or any(type(item) is not TreeItem for item in self.items):
            raise InvalidMatchedTreeError("Matched row requires an exact tuple of TreeItem values.")


@define(frozen=True, slots=True)
class MatchedBranch:
    """Rows belonging to one exact anchor path."""

    path: GhPath
    rows: Tuple[MatchedRow, ...]

    @classmethod
    def build(cls, path: GhPath, rows: Tuple[MatchedRow, ...]) -> "MatchedBranch":
        return cls(path, rows)

    def __attrs_post_init__(self) -> None:
        if type(self.path) is not GhPath or type(self.rows) is not tuple:
            raise InvalidMatchedTreeError("Matched branch requires an exact path and row tuple.")
        if any(type(row) is not MatchedRow for row in self.rows):
            raise InvalidMatchedTreeError("Matched branch rows must be exact MatchedRow values.")


@define(frozen=True, slots=True)
class MatchedTree:
    """Canonical matched rows and operation-scoped atomic inputs."""

    root_id: TreeRootId
    item_names: Tuple[str, ...]
    branches: Tuple[MatchedBranch, ...]
    global_items: Tuple[MatchedGlobal, ...]

    @classmethod
    def build(
        cls,
        root_id: TreeRootId,
        item_names: Tuple[str, ...],
        branches: Tuple[MatchedBranch, ...],
        global_items: Tuple[MatchedGlobal, ...],
    ) -> "MatchedTree":
        return cls(root_id, item_names, branches, global_items)

    def __attrs_post_init__(self) -> None:
        if type(self.root_id) is not TreeRootId or type(self.item_names) is not tuple:
            raise InvalidMatchedTreeError("Matched tree requires an exact root and item-name tuple.")
        if any(type(name) is not str or not name for name in self.item_names):
            raise InvalidMatchedTreeError("Matched item names must be exact non-empty text.")
        if type(self.branches) is not tuple or any(type(branch) is not MatchedBranch for branch in self.branches):
            raise InvalidMatchedTreeError("Matched tree branches must be an exact MatchedBranch tuple.")
        if type(self.global_items) is not tuple or any(type(item) is not MatchedGlobal for item in self.global_items):
            raise InvalidMatchedTreeError("Matched globals must be an exact MatchedGlobal tuple.")
        keys = tuple(branch.path.canonical_key() for branch in self.branches)
        if keys != tuple(sorted(keys)) or len(keys) != len(set(keys)):
            raise InvalidMatchedTreeError("Matched branches must retain unique canonical path order.")
        if any(len(row.items) != len(self.item_names) for branch in self.branches for row in branch.rows):
            raise InvalidMatchedTreeError("Every matched row must match the declared item names.")


def _paths(tree: Tree[object]) -> Tuple[Tuple[int, ...], ...]:
    return tuple(branch.path.canonical_key() for branch in tree.branches)


def _mismatch_error(anchor_count: int, candidate_count: int) -> TreeMatchingError:
    if anchor_count == 1 or candidate_count == 1:
        return ImplicitSingletonBroadcastError("A singleton Tree is an exact series and cannot broadcast.")
    return BranchLengthMismatchError("Exact trees require equal item counts in each corresponding branch.")


def _scalar_item(value: object) -> TreeItem[object]:
    if value is None:
        return TreeItem.null()
    return TreeItem.value(value)


def _rows_for_branch(
    branch_index: int,
    item_inputs: Tuple[MatchInput, ...],
    anchor_branch: TreeBranch[object],
) -> Tuple[MatchedRow, ...]:
    rows = []
    for item_index in range(len(anchor_branch.items)):
        row_items = []
        for match_input in item_inputs:
            if match_input.role is MatchRole.EXACT_TREE:
                row_items.append(match_input.exact_tree().branches[branch_index].items[item_index])
            else:
                row_items.append(_scalar_item(match_input.broadcast_scalar().value))
        rows.append(MatchedRow.build(tuple(row_items)))
    return tuple(rows)


def match_inputs(inputs: Tuple[MatchInput, ...], policy: MatchPolicy) -> MatchedTree:
    """Match exact trees branch-locally and only explicitly declared scalars."""
    if type(inputs) is not tuple or any(type(match_input) is not MatchInput for match_input in inputs):
        raise InvalidMatchInputError("Matching requires an exact tuple of MatchInput values.")
    if type(policy) is not MatchPolicy:
        raise InvalidMatchPolicyError("Matching requires an exact MatchPolicy value.")
    if tuple(match_input.name for match_input in inputs) != policy.names:
        raise MatchPolicyInputMismatchError("Ordered match input names must equal ordered policy names.")
    if tuple(match_input.role for match_input in inputs) != policy.roles:
        raise MatchPolicyInputMismatchError("Ordered match input roles must equal ordered policy roles.")

    anchor: Optional[Tree[object]] = None
    for match_input in inputs:
        if match_input.role is MatchRole.EXACT_TREE:
            anchor = match_input.exact_tree()
            break
    if anchor is None:
        raise MissingExactTreeAnchorError("Matching requires at least one EXACT_TREE topology anchor.")

    exact_inputs = tuple(match_input for match_input in inputs if match_input.role is MatchRole.EXACT_TREE)
    for match_input in exact_inputs[1:]:
        candidate = match_input.exact_tree()
        if _paths(candidate) != _paths(anchor):
            raise BranchSetMismatchError("Exact trees require identical canonical ordered branch sets.")
        for anchor_branch, candidate_branch in zip(anchor.branches, candidate.branches):
            if len(anchor_branch.items) != len(candidate_branch.items):
                raise _mismatch_error(len(anchor_branch.items), len(candidate_branch.items))

    item_inputs = tuple(match_input for match_input in inputs if match_input.role is not MatchRole.GLOBAL_ATOMIC)
    branches = tuple(
        MatchedBranch.build(
            anchor_branch.path,
            _rows_for_branch(branch_index, item_inputs, anchor_branch),
        )
        for branch_index, anchor_branch in enumerate(anchor.branches)
    )
    globals_ = tuple(MatchedGlobal.build(match_input.name, match_input.payload) for match_input in inputs if match_input.role is MatchRole.GLOBAL_ATOMIC)
    return MatchedTree.build(
        anchor.root_id,
        tuple(match_input.name for match_input in item_inputs),
        branches,
        globals_,
    )
