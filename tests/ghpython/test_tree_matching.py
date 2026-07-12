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
from compas_fab.ghpython.tree_matching import MatchPolicyInputMismatchError
from compas_fab.ghpython.tree_matching import MatchInput
from compas_fab.ghpython.tree_matching import MatchPolicy
from compas_fab.ghpython.tree_matching import MatchRole
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
