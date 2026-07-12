"""Immutable canonical paths and coordinates for Grasshopper trees."""

from __future__ import annotations

from typing import Tuple

from attrs import define

from compas_fab.ghpython.tree_errors import InvalidGhPathError
from compas_fab.ghpython.tree_errors import InvalidItemIndexError
from compas_fab.ghpython.tree_errors import InvalidTreeCoordinateError
from compas_fab.ghpython.tree_errors import InvalidTreeRootIdError


@define(frozen=True, slots=True)
class TreeRootId:
    """Runtime routing identity for one tree."""

    value: str

    @classmethod
    def build(cls, value: str) -> "TreeRootId":
        return cls(value)

    def __attrs_post_init__(self) -> None:
        if type(self.value) is not str or not self.value:
            raise InvalidTreeRootIdError("Tree root ID must be non-empty text.")


@define(frozen=True, slots=True)
class ItemIndex:
    """Exact zero-based item slot within a branch."""

    value: int

    @classmethod
    def build(cls, value: int) -> "ItemIndex":
        return cls(value)

    def __attrs_post_init__(self) -> None:
        if type(self.value) is not int or self.value < 0:
            raise InvalidItemIndexError("Item index must be an exact non-negative integer.")


@define(frozen=True, slots=True)
class GhPath:
    """Exact Grasshopper path with canonical lexicographic ordering."""

    indices: Tuple[int, ...]

    @classmethod
    def build(cls, *indices: int) -> "GhPath":
        return cls(indices)

    def __attrs_post_init__(self) -> None:
        if type(self.indices) is not tuple or not self.indices:
            raise InvalidGhPathError("Grasshopper path must contain at least one segment.")
        if any(type(index) is not int or index < 0 for index in self.indices):
            raise InvalidGhPathError("Grasshopper path segments must be exact non-negative integers.")

    def canonical_key(self) -> Tuple[int, ...]:
        """Return the exact host-order key, including prefix structure."""
        return self.indices


@define(frozen=True, slots=True)
class BranchCoordinate:
    """Routing identity and exact path of one branch."""

    root_id: TreeRootId
    path: GhPath

    @classmethod
    def build(cls, root_id: TreeRootId, path: GhPath) -> "BranchCoordinate":
        return cls(root_id, path)

    def __attrs_post_init__(self) -> None:
        if type(self.root_id) is not TreeRootId or type(self.path) is not GhPath:
            raise InvalidTreeCoordinateError("Branch coordinate requires exact tree-root and path values.")


@define(frozen=True, slots=True)
class TreeCoordinate:
    """Exact coordinate of an existing value or null item slot."""

    branch: BranchCoordinate
    item_index: ItemIndex

    @classmethod
    def build(cls, branch: BranchCoordinate, item_index: ItemIndex) -> "TreeCoordinate":
        return cls(branch, item_index)

    def __attrs_post_init__(self) -> None:
        if type(self.branch) is not BranchCoordinate or type(self.item_index) is not ItemIndex:
            raise InvalidTreeCoordinateError("Tree coordinate requires exact branch and item-index values.")
