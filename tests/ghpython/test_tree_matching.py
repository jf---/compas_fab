from __future__ import annotations

from typing import Tuple

import pytest
from hypothesis import given
from hypothesis import strategies as st

from compas_fab.ghpython.item_values import Scalar
from compas_fab.ghpython.tree_coordinates import GhPath
from compas_fab.ghpython.tree_coordinates import TreeRootId
from compas_fab.ghpython.tree_matching import BranchLengthMismatchError
from compas_fab.ghpython.tree_matching import BranchSetMismatchError
from compas_fab.ghpython.tree_matching import ImplicitSingletonBroadcastError
from compas_fab.ghpython.tree_matching import InvalidMatchPolicyError
from compas_fab.ghpython.tree_matching import InvalidMatchInputError
from compas_fab.ghpython.tree_matching import InvalidMatchedBranchOrderError
from compas_fab.ghpython.tree_matching import InvalidMatchedGlobalNamesError
from compas_fab.ghpython.tree_matching import InvalidMatchedItemNamesError
from compas_fab.ghpython.tree_matching import InvalidMatchedRowsError
from compas_fab.ghpython.tree_matching import MatchPolicyInputMismatchError
from compas_fab.ghpython.tree_matching import MatchInput
from compas_fab.ghpython.tree_matching import MatchPolicy
from compas_fab.ghpython.tree_matching import MatchRole
from compas_fab.ghpython.tree_matching import MatchedBranch
from compas_fab.ghpython.tree_matching import MatchedGlobal
from compas_fab.ghpython.tree_matching import MatchedRow
from compas_fab.ghpython.tree_matching import MatchedTree
from compas_fab.ghpython.tree_matching import MatchedTreeNameOverlapError
from compas_fab.ghpython.tree_matching import MissingExactTreeAnchorError
from compas_fab.ghpython.tree_matching import match_inputs
from compas_fab.ghpython.tree_values import Tree
from compas_fab.ghpython.tree_values import TreeBranch
from compas_fab.ghpython.tree_values import TreeItem


EXACT_PAIR_POLICY = MatchPolicy.build(
    ("left", "right"),
    (MatchRole.EXACT_TREE, MatchRole.EXACT_TREE),
)
BROADCAST_POLICY = MatchPolicy.build(
    ("pose", "profile"),
    (MatchRole.EXACT_TREE, MatchRole.BROADCASTABLE_SCALAR),
)


def tree(
    root: str,
    branches: Tuple[Tuple[Tuple[int, ...], Tuple[object | None, ...]], ...],
) -> Tree[object]:
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


def one_branch(items: list[int]) -> Tree[object]:
    return tree("one", (((0,), tuple(items)),))


@st.composite
def ragged_nullable_branches(draw: st.DrawFn) -> Tuple[Tuple[object | None, ...], ...]:
    counts = draw(st.lists(st.integers(min_value=0, max_value=4), min_size=2, max_size=5).filter(lambda values: len(set(values)) > 1))
    nullable_items = st.one_of(st.none(), st.integers())
    return tuple(tuple(draw(st.lists(nullable_items, min_size=count, max_size=count))) for count in counts)


@given(st.lists(st.integers(), min_size=0), st.lists(st.integers(), min_size=0))
def test_equal_branch_zip_never_repeats_or_truncates(left: list[int], right: list[int]) -> None:
    trees = (MatchInput.tree("left", one_branch(left)), MatchInput.tree("right", one_branch(right)))
    if len(left) != len(right):
        expected_error = ImplicitSingletonBroadcastError if 1 in (len(left), len(right)) else BranchLengthMismatchError
        with pytest.raises(expected_error):
            match_inputs(trees, EXACT_PAIR_POLICY)
    else:
        rows = match_inputs(trees, EXACT_PAIR_POLICY).branches[0].rows
        assert [tuple(item.item for item in row.items) for row in rows] == list(zip(left, right))


def test_only_declared_scalar_broadcasts() -> None:
    poses = tree("pose", (((0,), ("a", "b")),))
    scalar = MatchInput.scalar("profile", Scalar.build("DEFAULT"))

    assert len(match_inputs((MatchInput.tree("pose", poses), scalar), BROADCAST_POLICY).branches[0].rows) == 2
    with pytest.raises(ImplicitSingletonBroadcastError):
        match_inputs(
            (
                MatchInput.tree("left", poses),
                MatchInput.tree("right", tree("profile", (((0,), ("DEFAULT",)),))),
            ),
            EXACT_PAIR_POLICY,
        )


def test_matching_retains_ragged_empty_null_branches_and_canonical_order() -> None:
    left = tree("left", (((0,), (1, None)), ((1, 2), ()), ((3,), (4,))))
    right = tree("right", (((0,), (10, 20)), ((1, 2), ()), ((3,), (40,))))

    matched = match_inputs((MatchInput.tree("left", left), MatchInput.tree("right", right)), EXACT_PAIR_POLICY)

    assert tuple(branch.path.indices for branch in matched.branches) == ((0,), (1, 2), (3,))
    assert tuple(len(branch.rows) for branch in matched.branches) == (2, 0, 1)
    assert matched.branches[0].rows[1].items[0].is_null
    assert matched.branches[0].rows[1].items[1].item == 20


@given(ragged_nullable_branches(), st.one_of(st.none(), st.integers()))
def test_matching_property_preserves_ragged_zip_null_scalar_and_canonical_branches(
    left_values: Tuple[Tuple[object | None, ...], ...],
    scalar_value: object | None,
) -> None:
    right_values = tuple(tuple(None if value is None else -value for value in branch) for branch in left_values)
    left = tree(
        "left",
        tuple(((branch_index,), values) for branch_index, values in enumerate(left_values)),
    )
    right = tree(
        "right",
        tuple(((branch_index,), values) for branch_index, values in enumerate(right_values)),
    )
    policy = MatchPolicy.build(
        ("left", "right", "scalar"),
        (
            MatchRole.EXACT_TREE,
            MatchRole.EXACT_TREE,
            MatchRole.BROADCASTABLE_SCALAR,
        ),
    )

    matched = match_inputs(
        (
            MatchInput.tree("left", left),
            MatchInput.tree("right", right),
            MatchInput.scalar("scalar", Scalar.build(scalar_value)),
        ),
        policy,
    )

    assert tuple(branch.path.indices for branch in matched.branches) == tuple((index,) for index in range(len(left_values)))
    for branch, expected_left, expected_right in zip(
        matched.branches,
        left_values,
        right_values,
    ):
        assert tuple(tuple(item.item for item in row.items) for row in branch.rows) == tuple(
            (left_item, right_item, scalar_value) for left_item, right_item in zip(expected_left, expected_right)
        )
        assert tuple(tuple(item.is_null for item in row.items) for row in branch.rows) == tuple(
            (left_item is None, right_item is None, scalar_value is None) for left_item, right_item in zip(expected_left, expected_right)
        )


def test_matching_rejects_branch_set_mismatch_and_requires_exact_anchor() -> None:
    with pytest.raises(BranchSetMismatchError):
        match_inputs(
            (
                MatchInput.tree("left", tree("left", (((0,), (1,)),))),
                MatchInput.tree("right", tree("right", (((1,), (1,)),))),
            ),
            EXACT_PAIR_POLICY,
        )
    with pytest.raises(MissingExactTreeAnchorError):
        match_inputs(
            (MatchInput.scalar("profile", Scalar.build("DEFAULT")),),
            MatchPolicy.build(("profile",), (MatchRole.BROADCASTABLE_SCALAR,)),
        )


def test_global_atomic_is_retained_once_not_repeated_into_rows() -> None:
    native = object()
    policy = MatchPolicy.build(
        ("poses", "native"),
        (MatchRole.EXACT_TREE, MatchRole.GLOBAL_ATOMIC),
    )

    matched = match_inputs(
        (MatchInput.tree("poses", tree("poses", (((4,), (1, 2)),))), MatchInput.global_atomic("native", native)),
        policy,
    )

    assert matched.item_names == ("poses",)
    assert tuple(len(row.items) for row in matched.branches[0].rows) == (1, 1)
    assert matched.global_items[0].name == "native"
    assert matched.global_items[0].value is native


def test_raw_policy_constructor_cannot_bypass_exact_contract() -> None:
    with pytest.raises(InvalidMatchPolicyError):
        MatchPolicy(("left",), (MatchRole.EXACT_TREE, MatchRole.EXACT_TREE))
    with pytest.raises(InvalidMatchPolicyError):
        MatchPolicy(["left"], (MatchRole.EXACT_TREE,))  # type: ignore[arg-type]


def test_match_input_and_policy_must_agree_exactly_in_name_role_and_order() -> None:
    source = tree("source", (((0,), (1,)),))

    with pytest.raises(InvalidMatchInputError):
        MatchInput("source", MatchRole.EXACT_TREE, Scalar.build(1))
    with pytest.raises(MatchPolicyInputMismatchError):
        match_inputs(
            (MatchInput.tree("source", source),),
            MatchPolicy.build(("other",), (MatchRole.EXACT_TREE,)),
        )
    with pytest.raises(MatchPolicyInputMismatchError):
        match_inputs(
            (MatchInput.tree("source", source),),
            MatchPolicy.build(("source",), (MatchRole.GLOBAL_ATOMIC,)),
        )


def test_matched_tree_raw_constructor_rejects_invalid_item_and_global_names() -> None:
    matched = match_inputs(
        (MatchInput.tree("source", tree("source", (((0,), (1,)),))),),
        MatchPolicy.build(("source",), (MatchRole.EXACT_TREE,)),
    )

    for names in ((" source",), ("source", "source"), ("",)):
        with pytest.raises(InvalidMatchedItemNamesError):
            MatchedTree(matched.root_id, names, matched.branches, ())
    with pytest.raises(InvalidMatchedGlobalNamesError):
        MatchedGlobal(" global", object())
    with pytest.raises(InvalidMatchedGlobalNamesError):
        MatchedTree(
            matched.root_id,
            matched.item_names,
            matched.branches,
            (MatchedGlobal.build("first", object()), MatchedGlobal.build("first", object())),
        )


def test_matched_tree_raw_constructor_rejects_name_overlap_and_branch_permutation() -> None:
    matched = match_inputs(
        (MatchInput.tree("source", tree("source", (((0,), (1,)), ((1,), (2,))))),),
        MatchPolicy.build(("source",), (MatchRole.EXACT_TREE,)),
    )

    with pytest.raises(MatchedTreeNameOverlapError):
        MatchedTree(
            matched.root_id,
            matched.item_names,
            matched.branches,
            (MatchedGlobal.build("source", object()),),
        )
    with pytest.raises(InvalidMatchedBranchOrderError):
        MatchedTree(
            matched.root_id,
            matched.item_names,
            tuple(reversed(matched.branches)),
            (),
        )


def test_matched_rows_reject_name_count_and_nested_null_bypass() -> None:
    matched = match_inputs(
        (MatchInput.tree("source", tree("source", (((0,), (1,)),))),),
        MatchPolicy.build(("source",), (MatchRole.EXACT_TREE,)),
    )

    with pytest.raises(InvalidMatchedRowsError):
        MatchedTree(
            matched.root_id,
            ("source", "missing"),
            matched.branches,
            (),
        )

    invalid_null = TreeItem.null()
    object.__setattr__(invalid_null, "is_null", False)
    with pytest.raises(InvalidMatchedRowsError):
        MatchedRow.build((invalid_null,))

    mutated_row = MatchedRow.build((TreeItem.value(1),))
    object.__setattr__(mutated_row, "items", (invalid_null,))
    with pytest.raises(InvalidMatchedRowsError):
        MatchedBranch.build(GhPath.build(0), (mutated_row,))


def test_matched_tree_revalidates_nested_branch_rows() -> None:
    matched = match_inputs(
        (MatchInput.tree("source", tree("source", (((0,), (1,)),))),),
        MatchPolicy.build(("source",), (MatchRole.EXACT_TREE,)),
    )
    list_mutated_branch = matched.branches[0]
    object.__setattr__(list_mutated_branch, "rows", list(list_mutated_branch.rows))

    with pytest.raises(InvalidMatchedRowsError):
        MatchedTree.build(
            matched.root_id,
            matched.item_names,
            (list_mutated_branch,),
            matched.global_items,
        )

    invalid_null = TreeItem.null()
    object.__setattr__(invalid_null, "is_null", False)
    invalid_row = MatchedRow.build((TreeItem.value(1),))
    object.__setattr__(invalid_row, "items", (invalid_null,))
    nested_mutated_branch = MatchedBranch.build(GhPath.build(0), (MatchedRow.build((TreeItem.value(1),)),))
    object.__setattr__(nested_mutated_branch, "rows", (invalid_row,))

    with pytest.raises(InvalidMatchedRowsError):
        MatchedTree.build(
            matched.root_id,
            matched.item_names,
            (nested_mutated_branch,),
            matched.global_items,
        )


def test_matched_tree_rejects_non_row_nested_value_with_named_error() -> None:
    matched = match_inputs(
        (MatchInput.tree("source", tree("source", (((0,), (1,)),))),),
        MatchPolicy.build(("source",), (MatchRole.EXACT_TREE,)),
    )
    mutated_branch = matched.branches[0]
    object.__setattr__(mutated_branch, "rows", (object(),))

    with pytest.raises(InvalidMatchedRowsError):
        MatchedTree.build(
            matched.root_id,
            matched.item_names,
            (mutated_branch,),
            matched.global_items,
        )
