"""Pure explicit and bounded Cartesian tree expansion."""

from __future__ import annotations

from typing import Generic
from typing import Tuple
from typing import TypeVar

from attrs import define

from compas_fab.ghpython.tree_coordinates import BranchCoordinate
from compas_fab.ghpython.tree_coordinates import GhPath
from compas_fab.ghpython.tree_coordinates import ItemIndex
from compas_fab.ghpython.tree_coordinates import TreeCoordinate
from compas_fab.ghpython.tree_coordinates import TreeRootId
from compas_fab.ghpython.tree_diagnostics import SourceCoordinateEntry
from compas_fab.ghpython.tree_diagnostics import SourceCoordinateMap
from compas_fab.ghpython.tree_expansion_codec import CrossProductCoordinate
from compas_fab.ghpython.tree_expansion_codec import ExpansionPathCodec
from compas_fab.ghpython.tree_values import Tree
from compas_fab.ghpython.tree_values import TreeBranch
from compas_fab.ghpython.tree_values import TreeItem

LeftT = TypeVar("LeftT")
RightT = TypeVar("RightT")
ItemT = TypeVar("ItemT")


class TreeExpansionError(ValueError):
    """Base failure for explicit tree expansion."""


class InvalidMaximumExpandedItemsError(TreeExpansionError):
    """Raised when an expansion bound is invalid."""


class InvalidCrossProductPolicyError(TreeExpansionError):
    """Raised when a cross-product policy is invalid."""


class InvalidCrossProductInputError(TreeExpansionError):
    """Raised when cross-product operands are not exact trees."""


class CrossProductLimitError(TreeExpansionError):
    """Raised before allocation when the derived product exceeds its bound."""


class InvalidCrossProductResultError(TreeExpansionError):
    """Raised when a cross-product output is structurally invalid."""


@define(frozen=True, slots=True)
class MaximumExpandedItems:
    """Maximum allowed pair count for one explicit product operation."""

    value: int

    @classmethod
    def build(cls, value: int) -> "MaximumExpandedItems":
        return cls(value)

    def __attrs_post_init__(self) -> None:
        if type(self.value) is not int or self.value <= 0:
            raise InvalidMaximumExpandedItemsError("Maximum expanded items must be an exact positive integer.")


@define(frozen=True, slots=True)
class CrossProductPolicy:
    """Explicit allocation bound for a pure Cartesian expansion."""

    maximum_expanded_items: MaximumExpandedItems

    @classmethod
    def build(cls, maximum_expanded_items: MaximumExpandedItems) -> "CrossProductPolicy":
        return cls(maximum_expanded_items)

    def __attrs_post_init__(self) -> None:
        if type(self.maximum_expanded_items) is not MaximumExpandedItems:
            raise InvalidCrossProductPolicyError("Cross-product policy requires an exact expansion bound.")


@define(frozen=True, slots=True)
class CrossProductPair(Generic[LeftT, RightT]):
    """One left-major pair retaining both values/nulls and exact sources."""

    left: TreeItem[LeftT]
    right: TreeItem[RightT]
    coordinate: CrossProductCoordinate

    @classmethod
    def build(
        cls,
        left: TreeItem[LeftT],
        right: TreeItem[RightT],
        coordinate: CrossProductCoordinate,
    ) -> "CrossProductPair[LeftT, RightT]":
        return cls(left, right, coordinate)

    def __attrs_post_init__(self) -> None:
        if type(self.left) is not TreeItem or type(self.right) is not TreeItem or type(self.coordinate) is not CrossProductCoordinate:
            raise InvalidCrossProductResultError("Cross-product pair requires exact left/right items and a source coordinate.")

    @property
    def path(self) -> GhPath:
        """Return the canonical reversible expanded path for this pair."""
        return ExpansionPathCodec.encode(self.coordinate)


@define(frozen=True, slots=True)
class CrossProductResult(Generic[LeftT, RightT]):
    """Deterministic left-major product pairs."""

    pairs: Tuple[CrossProductPair[LeftT, RightT], ...]
    source_coordinates: SourceCoordinateMap

    @classmethod
    def build(
        cls,
        pairs: Tuple[CrossProductPair[LeftT, RightT], ...],
        source_coordinates: SourceCoordinateMap,
    ) -> "CrossProductResult[LeftT, RightT]":
        return cls(pairs, source_coordinates)

    def __attrs_post_init__(self) -> None:
        if type(self.pairs) is not tuple or any(type(pair) is not CrossProductPair for pair in self.pairs):
            raise InvalidCrossProductResultError("Cross-product result requires an exact pair tuple.")
        if type(self.source_coordinates) is not SourceCoordinateMap:
            raise InvalidCrossProductResultError("Cross-product result requires an exact source-coordinate map.")
        coordinates = tuple(pair.coordinate for pair in self.pairs)
        if len(coordinates) != len(set(coordinates)):
            raise InvalidCrossProductResultError("Cross-product result coordinates must be unique.")
        expected_entries = _source_entries(self.pairs)
        if self.source_coordinates.entries != expected_entries:
            raise InvalidCrossProductResultError("Cross-product source map must exactly cover every product pair.")

    @property
    def paths(self) -> Tuple[GhPath, ...]:
        """Return all canonical expanded paths in left-major axis order."""
        return tuple(pair.path for pair in self.pairs)


def _coordinate(branch: TreeBranch[ItemT], root_id: TreeRootId, item_index: int) -> TreeCoordinate:
    if type(root_id) is not TreeRootId:
        raise InvalidCrossProductInputError("Cross-product operand requires an exact tree root.")
    return TreeCoordinate.build(
        BranchCoordinate.build(root_id, branch.path),
        ItemIndex.build(item_index),
    )


def _build_pair(
    left_item: TreeItem[LeftT],
    right_item: TreeItem[RightT],
    left_coordinate: TreeCoordinate,
    right_coordinate: TreeCoordinate,
) -> CrossProductPair[LeftT, RightT]:
    return CrossProductPair.build(
        left_item,
        right_item,
        CrossProductCoordinate.build(left_coordinate, right_coordinate),
    )


def _output_coordinate(pair: CrossProductPair[LeftT, RightT]) -> TreeCoordinate:
    return TreeCoordinate.build(
        BranchCoordinate.build(pair.coordinate.left.branch.root_id, pair.path),
        ItemIndex.build(0),
    )


def _source_entries(
    pairs: Tuple[CrossProductPair[LeftT, RightT], ...],
) -> Tuple[SourceCoordinateEntry, ...]:
    entries = tuple(
        SourceCoordinateEntry.build(
            _output_coordinate(pair),
            (pair.coordinate.left, pair.coordinate.right),
        )
        for pair in pairs
    )
    return tuple(
        sorted(
            entries,
            key=lambda entry: (
                entry.output.branch.path.canonical_key(),
                entry.output.item_index.value,
            ),
        )
    )


def cross_product(
    left: Tree[LeftT],
    right: Tree[RightT],
    policy: CrossProductPolicy,
) -> CrossProductResult[LeftT, RightT]:
    """Return a bounded left-major Cartesian product of all exact tree items."""
    if type(left) is not Tree or type(right) is not Tree:
        raise InvalidCrossProductInputError("Cross-product operands must be exact Tree values.")
    if type(policy) is not CrossProductPolicy:
        raise InvalidCrossProductPolicyError("Cross-product requires an exact policy.")

    left_count = sum(len(branch.items) for branch in left.branches)
    right_count = sum(len(branch.items) for branch in right.branches)
    expanded_count = left_count * right_count
    if expanded_count > policy.maximum_expanded_items.value:
        raise CrossProductLimitError(
            "Cross-product item count {0} exceeds declared maximum {1}.".format(
                expanded_count,
                policy.maximum_expanded_items.value,
            )
        )

    pairs = []
    for left_branch in left.branches:
        for left_index, left_item in enumerate(left_branch.items):
            left_coordinate = _coordinate(left_branch, left.root_id, left_index)
            for right_branch in right.branches:
                for right_index, right_item in enumerate(right_branch.items):
                    right_coordinate = _coordinate(right_branch, right.root_id, right_index)
                    pairs.append(_build_pair(left_item, right_item, left_coordinate, right_coordinate))
    result_pairs = tuple(pairs)
    return CrossProductResult.build(
        result_pairs,
        SourceCoordinateMap.build(_source_entries(result_pairs)),
    )
