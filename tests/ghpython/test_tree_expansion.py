from __future__ import annotations

from typing import Tuple

import pytest

from compas_fab.ghpython.tree_coordinates import BranchCoordinate
from compas_fab.ghpython.tree_coordinates import GhPath
from compas_fab.ghpython.tree_coordinates import ItemIndex
from compas_fab.ghpython.tree_coordinates import TreeCoordinate
from compas_fab.ghpython.tree_coordinates import TreeRootId
from compas_fab.ghpython.tree_expansion import CrossProductLimitError
from compas_fab.ghpython.tree_expansion import CrossProductPolicy
from compas_fab.ghpython.tree_expansion import InvalidCrossProductPolicyError
from compas_fab.ghpython.tree_expansion import InvalidMaximumExpandedItemsError
from compas_fab.ghpython.tree_expansion import MaximumExpandedItems
from compas_fab.ghpython.tree_expansion import cross_product
from compas_fab.ghpython.tree_expansion_codec import CrossProductCoordinate
from compas_fab.ghpython.tree_expansion_codec import ExpandedBranchCoordinate
from compas_fab.ghpython.tree_expansion_codec import ExpansionPathCodec
from compas_fab.ghpython.tree_expansion_codec import InvalidExpansionPathError
from compas_fab.ghpython.tree_expansion_codec import InvalidRequestOrdinalError
from compas_fab.ghpython.tree_expansion_codec import InvalidResultOrdinalError
from compas_fab.ghpython.tree_expansion_codec import RequestOrdinal
from compas_fab.ghpython.tree_expansion_codec import ResultOrdinal
from compas_fab.ghpython.tree_values import Tree
from compas_fab.ghpython.tree_values import TreeBranch
from compas_fab.ghpython.tree_values import TreeItem


def branch_coordinate(path: Tuple[int, ...], root: str = "source") -> BranchCoordinate:
    return BranchCoordinate.build(TreeRootId.build(root), GhPath.build(*path))


def tree_coordinate(path: Tuple[int, ...], index: int, root: str = "source") -> TreeCoordinate:
    return TreeCoordinate.build(branch_coordinate(path, root), ItemIndex.build(index))


def tree(root: str, branches: Tuple[Tuple[Tuple[int, ...], Tuple[object | None, ...]], ...]) -> Tree[object]:
    return Tree.build(
        TreeRootId.build(root),
        tuple(
            TreeBranch.build(
                GhPath.build(*path),
                tuple(TreeItem.null() if item is None else TreeItem.value(item) for item in items),
            )
            for path, items in branches
        ),
    )


def test_one_codec_round_trips_cross_product_and_reserved_result_coordinates() -> None:
    cross = CrossProductCoordinate.build(tree_coordinate((1,), 0), tree_coordinate((1, 2), 3))
    result = ExpandedBranchCoordinate.build(
        branch_coordinate((7, 4)),
        RequestOrdinal.build(2),
        ResultOrdinal.build(5),
    )

    assert ExpansionPathCodec.decode(ExpansionPathCodec.encode(cross)) == cross
    assert ExpansionPathCodec.decode(ExpansionPathCodec.encode(result)) == result
    assert ExpansionPathCodec.encode(cross) != ExpansionPathCodec.encode(result)


def test_codec_is_collision_free_for_prefix_paths_and_requires_exact_payload() -> None:
    prefix = CrossProductCoordinate.build(tree_coordinate((1,), 2), tree_coordinate((9,), 0))
    extension = CrossProductCoordinate.build(tree_coordinate((1, 2), 0), tree_coordinate((9,), 0))

    assert ExpansionPathCodec.encode(prefix) != ExpansionPathCodec.encode(extension)
    for malformed in (
        GhPath.build(99, 1, 0),
        GhPath.build(0, 2, 1),
        GhPath.build(0, 1, 4, 0, 1, 5),
        GhPath.build(1, 1, 7, 2),
    ):
        with pytest.raises(InvalidExpansionPathError):
            ExpansionPathCodec.decode(malformed)


def test_codec_round_trip_retains_distinct_unicode_runtime_roots() -> None:
    coordinate = CrossProductCoordinate.build(
        tree_coordinate((2,), 4, "læft"),
        tree_coordinate((3, 5), 6, "右"),
    )

    assert ExpansionPathCodec.decode(ExpansionPathCodec.encode(coordinate)) == coordinate


def test_cross_product_is_left_major_bounded_and_retains_null_operands() -> None:
    left = tree("left", (((0,), ("a", None)),))
    right = tree("right", (((1,), (10, 20)),))
    policy = CrossProductPolicy.build(MaximumExpandedItems.build(4))

    result = cross_product(left, right, policy)

    assert tuple((pair.left.item, pair.right.item) for pair in result.pairs) == (
        ("a", 10),
        ("a", 20),
        (None, 10),
        (None, 20),
    )
    assert result.pairs[2].left.is_null
    assert result.pairs[2].coordinate == CrossProductCoordinate.build(
        tree_coordinate((0,), 1, "left"),
        tree_coordinate((1,), 0, "right"),
    )


def test_cross_product_source_map_is_canonical_and_maps_both_exact_axes() -> None:
    left = tree("left", (((1, 9), ("first",)), ((2,), ("second",))))
    right = tree("right", (((4,), (10, 20)),))

    result = cross_product(
        left,
        right,
        CrossProductPolicy.build(MaximumExpandedItems.build(4)),
    )

    output_keys = tuple((entry.output.branch.path.indices, entry.output.item_index.value) for entry in result.source_coordinates.entries)
    assert output_keys == tuple(sorted(output_keys))
    for entry in result.source_coordinates.entries:
        decoded = ExpansionPathCodec.decode(entry.output.branch.path)
        assert isinstance(decoded, CrossProductCoordinate)
        assert entry.sources == (decoded.left, decoded.right)


def test_cross_product_checks_total_before_allocating(monkeypatch: pytest.MonkeyPatch) -> None:
    left = tree("left", (((0,), tuple(range(3))),))
    right = tree("right", (((1,), tuple(range(2))),))
    policy = CrossProductPolicy.build(MaximumExpandedItems.build(5))

    with pytest.raises(CrossProductLimitError):
        cross_product(left, right, policy)

    monkeypatch.setattr(
        "compas_fab.ghpython.tree_expansion._build_pair",
        lambda *_args: pytest.fail("allocation began before limit validation"),
    )
    with pytest.raises(CrossProductLimitError):
        cross_product(left, right, policy)


def test_expansion_factories_and_raw_constructors_enforce_exact_types() -> None:
    with pytest.raises(InvalidMaximumExpandedItemsError):
        MaximumExpandedItems(0)
    with pytest.raises(InvalidMaximumExpandedItemsError):
        MaximumExpandedItems(True)
    with pytest.raises(InvalidCrossProductPolicyError):
        CrossProductPolicy(4)  # type: ignore[arg-type]
    with pytest.raises(InvalidRequestOrdinalError):
        RequestOrdinal(-1)
    with pytest.raises(InvalidResultOrdinalError):
        ResultOrdinal(True)
