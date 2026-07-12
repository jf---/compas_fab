from __future__ import annotations

from typing import Tuple

import pytest
from hypothesis import given
from hypothesis import strategies as st

from compas_fab.ghpython.tree_coordinates import BranchCoordinate
from compas_fab.ghpython.tree_coordinates import GhPath
from compas_fab.ghpython.tree_coordinates import ItemIndex
from compas_fab.ghpython.tree_coordinates import TreeCoordinate
from compas_fab.ghpython.tree_coordinates import TreeRootId
from compas_fab.ghpython.tree_expansion import CrossProductLimitError
from compas_fab.ghpython.tree_expansion import CrossProductPair
from compas_fab.ghpython.tree_expansion import CrossProductPolicy
from compas_fab.ghpython.tree_expansion import CrossProductResult
from compas_fab.ghpython.tree_expansion import CrossProductSourceMapMismatchError
from compas_fab.ghpython.tree_expansion import IncompleteCrossProductGridError
from compas_fab.ghpython.tree_expansion import InvalidCrossProductResultError
from compas_fab.ghpython.tree_expansion import InvalidCrossProductPolicyError
from compas_fab.ghpython.tree_expansion import InvalidMaximumExpandedItemsError
from compas_fab.ghpython.tree_expansion import MaximumExpandedItems
from compas_fab.ghpython.tree_expansion import NonLeftMajorCrossProductOrderError
from compas_fab.ghpython.tree_expansion import cross_product
from compas_fab.ghpython.tree_expansion_codec import CrossProductCoordinate
from compas_fab.ghpython.tree_expansion_codec import ExpandedBranchCoordinate
from compas_fab.ghpython.tree_expansion_codec import ExpansionPathCodec
from compas_fab.ghpython.tree_expansion_codec import InvalidExpansionPathError
from compas_fab.ghpython.tree_expansion_codec import InvalidRequestOrdinalError
from compas_fab.ghpython.tree_expansion_codec import InvalidResultOrdinalError
from compas_fab.ghpython.tree_expansion_codec import RequestOrdinal
from compas_fab.ghpython.tree_expansion_codec import ResultOrdinal
from compas_fab.ghpython.tree_diagnostics import SourceCoordinateEntry
from compas_fab.ghpython.tree_diagnostics import SourceCoordinateMap
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


UNICODE_ROOTS = st.text(
    alphabet=st.characters(exclude_categories=("Cs",)),
    min_size=1,
    max_size=8,
)
PATHS = st.lists(st.integers(min_value=0, max_value=8), min_size=1, max_size=4).map(tuple)
ORDINALS = st.integers(min_value=0, max_value=32)


@st.composite
def cross_product_coordinates(draw: st.DrawFn) -> CrossProductCoordinate:
    return CrossProductCoordinate.build(
        tree_coordinate(draw(PATHS), draw(ORDINALS), draw(UNICODE_ROOTS)),
        tree_coordinate(draw(PATHS), draw(ORDINALS), draw(UNICODE_ROOTS)),
    )


@st.composite
def expanded_branch_coordinates(draw: st.DrawFn) -> ExpandedBranchCoordinate:
    return ExpandedBranchCoordinate.build(
        branch_coordinate(draw(PATHS), draw(UNICODE_ROOTS)),
        RequestOrdinal.build(draw(ORDINALS)),
        ResultOrdinal.build(draw(ORDINALS)),
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


@given(cross_product_coordinates(), cross_product_coordinates())
def test_cross_product_codec_is_round_trip_injective(
    first: CrossProductCoordinate,
    second: CrossProductCoordinate,
) -> None:
    first_path = ExpansionPathCodec.encode(first)
    second_path = ExpansionPathCodec.encode(second)

    assert ExpansionPathCodec.decode(first_path) == first
    assert ExpansionPathCodec.decode(second_path) == second
    assert (first_path == second_path) is (first == second)


@given(expanded_branch_coordinates(), expanded_branch_coordinates())
def test_expanded_branch_codec_is_round_trip_injective(
    first: ExpandedBranchCoordinate,
    second: ExpandedBranchCoordinate,
) -> None:
    first_path = ExpansionPathCodec.encode(first)
    second_path = ExpansionPathCodec.encode(second)

    assert ExpansionPathCodec.decode(first_path) == first
    assert ExpansionPathCodec.decode(second_path) == second
    assert (first_path == second_path) is (first == second)


@given(
    UNICODE_ROOTS,
    PATHS,
    st.lists(st.integers(min_value=0, max_value=8), min_size=1, max_size=3).map(tuple),
    ORDINALS,
    ORDINALS,
)
def test_codec_distinguishes_prefix_related_source_paths(
    root: str,
    prefix: Tuple[int, ...],
    suffix: Tuple[int, ...],
    request_ordinal: int,
    result_ordinal: int,
) -> None:
    prefix_coordinate = ExpandedBranchCoordinate.build(
        branch_coordinate(prefix, root),
        RequestOrdinal.build(request_ordinal),
        ResultOrdinal.build(result_ordinal),
    )
    extension_coordinate = ExpandedBranchCoordinate.build(
        branch_coordinate(prefix + suffix, root),
        RequestOrdinal.build(request_ordinal),
        ResultOrdinal.build(result_ordinal),
    )

    assert ExpansionPathCodec.encode(prefix_coordinate) != ExpansionPathCodec.encode(extension_coordinate)


def test_codec_rejects_invalid_utf8_tags_lengths_and_trailing_segments() -> None:
    valid = ExpansionPathCodec.encode(
        ExpandedBranchCoordinate.build(
            branch_coordinate((2,), "valid"),
            RequestOrdinal.build(3),
            ResultOrdinal.build(5),
        )
    )
    malformed_paths = (
        GhPath.build(0, 1, 128),
        GhPath.build(77),
        GhPath.build(1, 0),
        GhPath.build(1, 4, 65),
        GhPath.build(*valid.indices, 0),
    )

    for malformed in malformed_paths:
        with pytest.raises(InvalidExpansionPathError):
            ExpansionPathCodec.decode(malformed)


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


def test_cross_product_count_guard_handles_zero_exact_bound_and_overbound_before_multiplication(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    empty = tree("empty", (((0,), ()),))
    populated = tree("populated", (((1,), (1, 2, 3)),))
    exact_left = tree("left", (((0,), (1, 2)),))
    exact_right = tree("right", (((1,), (3, 4, 5)),))

    exact_result = cross_product(
        exact_left,
        exact_right,
        CrossProductPolicy.build(MaximumExpandedItems.build(6)),
    )
    assert len(exact_result.pairs) == 6

    monkeypatch.setattr(
        "compas_fab.ghpython.tree_expansion._multiply_item_counts",
        lambda *_args: pytest.fail("multiplication reached before zero/limit guard"),
    )
    empty_policy = CrossProductPolicy.build(MaximumExpandedItems.build(1))
    assert cross_product(empty, populated, empty_policy).pairs == ()
    assert cross_product(populated, empty, empty_policy).pairs == ()
    with pytest.raises(CrossProductLimitError):
        cross_product(
            exact_left,
            exact_right,
            CrossProductPolicy.build(MaximumExpandedItems.build(5)),
        )


def test_cross_product_result_rejects_non_left_major_permutations() -> None:
    result = cross_product(
        tree("left", (((0,), (1, 2)),)),
        tree("right", (((1,), (3, 4)),)),
        CrossProductPolicy.build(MaximumExpandedItems.build(4)),
    )

    with pytest.raises(NonLeftMajorCrossProductOrderError):
        CrossProductResult.build(tuple(reversed(result.pairs)), result.source_coordinates)


def test_cross_product_result_rejects_incomplete_coordinate_grid() -> None:
    result = cross_product(
        tree("left", (((0,), (1, 2)),)),
        tree("right", (((1,), (3, 4)),)),
        CrossProductPolicy.build(MaximumExpandedItems.build(4)),
    )
    retained_pairs = result.pairs[:-1]
    retained_paths = {pair.path for pair in retained_pairs}
    retained_map = SourceCoordinateMap.build(tuple(entry for entry in result.source_coordinates.entries if entry.output.branch.path in retained_paths))

    with pytest.raises(IncompleteCrossProductGridError):
        CrossProductResult(retained_pairs, retained_map)


def test_cross_product_result_rejects_diagonal_grid_before_expected_pair_generation(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    axis_size = 128
    valid_item = TreeItem.value(1)
    diagonal_pairs = tuple(
        CrossProductPair.build(
            valid_item,
            valid_item,
            CrossProductCoordinate.build(
                tree_coordinate((0,), index, "left"),
                tree_coordinate((1,), index, "right"),
            ),
        )
        for index in range(axis_size)
    )
    monkeypatch.setattr(
        CrossProductCoordinate,
        "build",
        lambda *_args: pytest.fail("expected pair generation began before cardinality rejection"),
    )

    with pytest.raises(IncompleteCrossProductGridError):
        CrossProductResult.build(diagonal_pairs, SourceCoordinateMap.build(()))


def test_cross_product_pair_revalidates_nested_tree_items() -> None:
    valid_item = TreeItem.value(1)
    invalid_null = TreeItem.null()
    object.__setattr__(invalid_null, "is_null", False)
    coordinate = CrossProductCoordinate.build(
        tree_coordinate((0,), 0, "left"),
        tree_coordinate((1,), 0, "right"),
    )

    for left, right in ((invalid_null, valid_item), (valid_item, invalid_null)):
        with pytest.raises(InvalidCrossProductResultError):
            CrossProductPair.build(left, right, coordinate)


def test_cross_product_result_rejects_pair_path_source_map_mismatch() -> None:
    result = cross_product(
        tree("left", (((0,), (1,)),)),
        tree("right", (((1,), (2,)),)),
        CrossProductPolicy.build(MaximumExpandedItems.build(1)),
    )
    entry = result.source_coordinates.entries[0]
    wrong_sources = (entry.sources[1], entry.sources[0])
    wrong_map = SourceCoordinateMap.build((SourceCoordinateEntry.build(entry.output, wrong_sources),))

    with pytest.raises(CrossProductSourceMapMismatchError):
        CrossProductResult.build(result.pairs, wrong_map)


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
