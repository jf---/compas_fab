"""Immutable null-aware ordered Grasshopper tree values."""

from __future__ import annotations

from typing import Generic
from typing import Optional
from typing import Tuple
from typing import TypeVar

from attrs import define

from compas_fab.ghpython.tree_coordinates import GhPath
from compas_fab.ghpython.tree_coordinates import TreeRootId
from compas_fab.ghpython.tree_errors import DuplicateTreePathError
from compas_fab.ghpython.tree_errors import InvalidTreeBranchError
from compas_fab.ghpython.tree_errors import InvalidTreeItemError
from compas_fab.ghpython.tree_errors import InvalidTreeRootIdError
from compas_fab.ghpython.tree_errors import NonCanonicalTreeOrderError

T = TypeVar("T")


@define(frozen=True, slots=True)
class TreeItem(Generic[T]):
    """One explicit value or null slot in a tree branch."""

    item: Optional[T]
    is_null: bool

    @classmethod
    def value(cls, item: T) -> "TreeItem[T]":
        return cls(item=item, is_null=False)

    @classmethod
    def null(cls) -> "TreeItem[T]":
        return cls(item=None, is_null=True)

    def __attrs_post_init__(self) -> None:
        if type(self.is_null) is not bool:
            raise InvalidTreeItemError("Tree item null state must be bool.")
        if self.is_null != (self.item is None):
            raise InvalidTreeItemError("Tree item must contain either one value or one explicit null slot.")


@define(frozen=True, slots=True)
class TreeBranch(Generic[T]):
    """One exact path and its ordered item slots, including emptiness."""

    path: GhPath
    items: Tuple[TreeItem[T], ...]

    @classmethod
    def build(cls, path: GhPath, items: Tuple[TreeItem[T], ...]) -> "TreeBranch[T]":
        return cls(path=path, items=items)

    def __attrs_post_init__(self) -> None:
        if type(self.path) is not GhPath or type(self.items) is not tuple:
            raise InvalidTreeBranchError("Tree branch requires an exact path and item tuple.")
        if any(type(item) is not TreeItem for item in self.items):
            raise InvalidTreeBranchError("Tree branch items must be exact TreeItem values.")


@define(frozen=True, slots=True)
class TreeTopology:
    """Ordered paths, branch counts, and null-slot bitmaps."""

    paths: Tuple[GhPath, ...]
    item_counts: Tuple[int, ...]
    null_bitmaps: Tuple[Tuple[bool, ...], ...]

    @classmethod
    def build(cls, branches: Tuple[TreeBranch[T], ...]) -> "TreeTopology":
        if type(branches) is not tuple or any(type(branch) is not TreeBranch for branch in branches):
            raise InvalidTreeBranchError("Tree topology requires an exact tuple of TreeBranch values.")
        return cls(
            paths=tuple(branch.path for branch in branches),
            item_counts=tuple(len(branch.items) for branch in branches),
            null_bitmaps=tuple(tuple(item.is_null for item in branch.items) for branch in branches),
        )

    def __attrs_post_init__(self) -> None:
        if type(self.paths) is not tuple or type(self.item_counts) is not tuple or type(self.null_bitmaps) is not tuple:
            raise InvalidTreeBranchError("Tree topology fields must be exact tuples.")
        if len(self.paths) != len(self.item_counts) or len(self.paths) != len(self.null_bitmaps):
            raise InvalidTreeBranchError("Tree topology fields must describe the same branches.")
        for path, count, bitmap in zip(self.paths, self.item_counts, self.null_bitmaps):
            if type(path) is not GhPath or type(count) is not int or count < 0 or type(bitmap) is not tuple:
                raise InvalidTreeBranchError("Tree topology contains invalid branch metadata.")
            if len(bitmap) != count or any(type(bit) is not bool for bit in bitmap):
                raise InvalidTreeBranchError("Tree topology null bitmap must match its item count.")
        keys = tuple(path.canonical_key() for path in self.paths)
        if len(set(keys)) != len(keys):
            raise DuplicateTreePathError("Tree topology paths must be unique.")
        if keys != tuple(sorted(keys)):
            raise NonCanonicalTreeOrderError("Tree topology paths must retain canonical host order.")


@define(frozen=True, slots=True)
class Tree(Generic[T]):
    """Canonical ordered tree retaining paths, empty branches, and nulls."""

    root_id: TreeRootId
    branches: Tuple[TreeBranch[T], ...]

    @classmethod
    def build(cls, root_id: TreeRootId, branches: Tuple[TreeBranch[T], ...]) -> "Tree[T]":
        return cls(root_id=root_id, branches=branches)

    def __attrs_post_init__(self) -> None:
        if type(self.root_id) is not TreeRootId:
            raise InvalidTreeRootIdError("Tree requires an exact TreeRootId routing value.")
        if type(self.branches) is not tuple or any(type(branch) is not TreeBranch for branch in self.branches):
            raise InvalidTreeBranchError("Tree branches must be an exact tuple of TreeBranch values.")
        keys = tuple(branch.path.canonical_key() for branch in self.branches)
        if len(set(keys)) != len(keys):
            raise DuplicateTreePathError("Tree paths must be unique.")
        if keys != tuple(sorted(keys)):
            raise NonCanonicalTreeOrderError("Tree branches must retain canonical host path order.")

    @property
    def topology(self) -> TreeTopology:
        """Return an immutable topology record without routing metadata."""
        branches = tuple(self.branches)
        return TreeTopology.build(branches)
