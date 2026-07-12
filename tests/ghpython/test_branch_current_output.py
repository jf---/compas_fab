from __future__ import annotations

from typing import Optional
from typing import Tuple
from typing import cast

import pytest

from compas_fab.ghpython.branch_current_output import BranchDecision
from compas_fab.ghpython.branch_current_output import BranchOutputState
from compas_fab.ghpython.branch_current_output import InvalidBranchOutputStateError
from compas_fab.ghpython.branch_current_output import StaleBranchGenerationError
from compas_fab.ghpython.branch_runtime_identity import BranchRequestGeneration
from compas_fab.ghpython.branch_runtime_identity import BranchRuntimeIdentity
from compas_fab.ghpython.branch_runtime_identity import InvalidBranchRequestGenerationError
from compas_fab.ghpython.branch_runtime_identity import InvalidBranchRuntimeIdentityError
from compas_fab.ghpython.branch_runtime_identity import InvalidSharedInputsChangedError
from compas_fab.ghpython.branch_runtime_identity import InvalidSharedInputsUnchangedError
from compas_fab.ghpython.branch_runtime_identity import InvalidSolveGenerationError
from compas_fab.ghpython.branch_runtime_identity import InvalidTreeRuntimeSnapshotError
from compas_fab.ghpython.branch_runtime_identity import SharedInputsChanged
from compas_fab.ghpython.branch_runtime_identity import SharedInputsUnchanged
from compas_fab.ghpython.branch_runtime_identity import SolveGeneration
from compas_fab.ghpython.branch_runtime_identity import TreeRuntimeSnapshot
from compas_fab.ghpython.branch_runtime_identity import UnknownRuntimeBranchError
from compas_fab.ghpython.branch_runtime_identity import advance_runtime
from compas_fab.ghpython.component_identity import CanonicalField
from compas_fab.ghpython.item_values import ItemShape
from compas_fab.ghpython.port_semantics import BranchSemantics
from compas_fab.ghpython.port_semantics import PortSemantics
from compas_fab.ghpython.port_semantics import TopologyRole
from compas_fab.ghpython.tree_coordinates import BranchCoordinate
from compas_fab.ghpython.tree_coordinates import GhPath
from compas_fab.ghpython.tree_coordinates import TreeRootId
from compas_fab.ghpython.tree_identity import BranchContentDigest
from compas_fab.ghpython.tree_identity import SourceTreeIdentity
from compas_fab.ghpython.tree_identity import TEXT_CODEC
from compas_fab.ghpython.tree_values import Tree
from compas_fab.ghpython.tree_values import TreeBranch
from compas_fab.ghpython.tree_values import TreeItem


TEXT_TREE_SEMANTICS = PortSemantics.build(
    TopologyRole.TREE,
    BranchSemantics.ORDERED_SEQUENCE,
    ItemShape.scalar(),
)


def content_tree(
    *branches: Tuple[Tuple[int, ...], Tuple[Optional[str], ...]],
    root: str = "content-source",
) -> SourceTreeIdentity:
    tree = Tree.build(
        TreeRootId.build(root),
        tuple(
            TreeBranch.build(
                GhPath.build(*path),
                tuple(TreeItem.null() if item is None else TreeItem.value(item) for item in items),
            )
            for path, items in branches
        ),
    )
    return SourceTreeIdentity.build(tree, TEXT_CODEC, TEXT_TREE_SEMANTICS)


def initial(
    content: SourceTreeIdentity,
    root: str = "runtime",
) -> TreeRuntimeSnapshot:
    return TreeRuntimeSnapshot.initial(content, TreeRootId.build(root))


def path(index: int) -> GhPath:
    return GhPath.build(index)


def test_branch_edit_preserves_unchanged_sibling_but_shared_edit_clears_all() -> None:
    first = initial(content_tree(((0,), ("a",)), ((1,), ("b",))))
    state = BranchOutputState[str].build(first)
    state = state.publish(first.branch(path(0)), "left")
    state = state.publish(first.branch(path(1)), "right")

    local = advance_runtime(
        first,
        content_tree(((0,), ("changed",)), ((1,), ("b",))),
        SharedInputsUnchanged.build(),
    )
    reconciled = state.reconcile(local)

    assert local.solve_generation == first.solve_generation
    assert local.branch(path(0)).request_generation == BranchRequestGeneration.build(1)
    assert local.branch(path(1)) == first.branch(path(1))
    assert reconciled[path(0)] is BranchDecision.CLEARED
    assert reconciled[path(1)] is BranchDecision.CURRENT
    assert tuple(reconciled.decisions) == tuple(branch.coordinate for branch in local.branches)
    assert reconciled.current(local.branch(path(0))) is None
    assert reconciled.current(local.branch(path(1))) == "right"

    shared = advance_runtime(
        local,
        local.content,
        SharedInputsChanged.build((CanonicalField.text("scene", "next"),)),
    )
    cleared = reconciled.reconcile(shared)

    assert shared.solve_generation == SolveGeneration.build(1)
    assert all(
        branch.request_generation.value
        == local.branch(branch.coordinate.path).request_generation.value + 1
        for branch in shared.branches
    )
    assert set(cleared.decisions.values()) == {BranchDecision.CLEARED}


def test_null_and_order_edits_invalidate_only_the_reduced_branch() -> None:
    first = initial(
        content_tree(
            ((0,), ("a", None, "b")),
            ((1,), ("sibling",)),
        )
    )
    null_changed = advance_runtime(
        first,
        content_tree(
            ((0,), ("a", "now-present", "b")),
            ((1,), ("sibling",)),
        ),
        SharedInputsUnchanged.build(),
    )
    reordered = advance_runtime(
        null_changed,
        content_tree(
            ((0,), ("b", "now-present", "a")),
            ((1,), ("sibling",)),
        ),
        SharedInputsUnchanged.build(),
    )

    assert reordered.solve_generation == SolveGeneration.build(0)
    assert null_changed.branch(path(0)).request_generation == BranchRequestGeneration.build(1)
    assert reordered.branch(path(0)).request_generation == BranchRequestGeneration.build(2)
    assert reordered.branch(path(1)) == first.branch(path(1))


def test_topology_change_invalidates_survivors_and_initializes_new_branch() -> None:
    first = initial(content_tree(((0,), ("a",)), ((2,), ())))
    changed = advance_runtime(
        first,
        content_tree(((0,), ("a",)), ((1,), ("new",))),
        SharedInputsUnchanged.build(),
    )

    assert changed.solve_generation == SolveGeneration.build(1)
    assert changed.branch(path(0)).request_generation == BranchRequestGeneration.build(1)
    assert changed.branch(path(1)).request_generation == BranchRequestGeneration.build(0)
    with pytest.raises(UnknownRuntimeBranchError):
        changed.branch(path(2))


def test_unchanged_advance_is_idempotent() -> None:
    first = initial(content_tree(((0,), ("a",)), ((2,), ())))

    same = advance_runtime(first, first.content, SharedInputsUnchanged.build())

    assert same == first


def test_runtime_root_change_reroutes_without_changing_content_digest() -> None:
    content = content_tree(((0,), ("a",)))
    first = initial(content, root="root-a")

    rerouted = advance_runtime(
        first,
        content,
        SharedInputsUnchanged.build(),
        root_id=TreeRootId.build("root-b"),
    )

    assert rerouted.content.digest == first.content.digest
    assert rerouted.branch(path(0)).content_digest == first.branch(path(0)).content_digest
    assert rerouted.branch(path(0)).coordinate.root_id == TreeRootId.build("root-b")
    assert rerouted.solve_generation == SolveGeneration.build(1)
    assert rerouted.branch(path(0)).request_generation == BranchRequestGeneration.build(1)


def test_shared_change_requires_exact_nonempty_unique_validated_fields() -> None:
    field = CanonicalField.text("scene", "next")
    assert SharedInputsChanged.build((field,)).fields == (field,)

    with pytest.raises(InvalidSharedInputsChangedError):
        SharedInputsChanged.build(cast(Tuple[CanonicalField, ...], [field]))
    with pytest.raises(InvalidSharedInputsChangedError):
        SharedInputsChanged.build(())
    with pytest.raises(InvalidSharedInputsChangedError):
        SharedInputsChanged.build((field, CanonicalField.text("scene", "other")))

    class DerivedCanonicalField(CanonicalField):
        pass

    with pytest.raises(InvalidSharedInputsChangedError):
        SharedInputsChanged.build((DerivedCanonicalField("scene", b"next"),))

    with pytest.raises(InvalidSharedInputsChangedError):
        SharedInputsChanged((field,))
    with pytest.raises(InvalidSharedInputsUnchangedError):
        SharedInputsUnchanged()


def test_stale_publish_fail_and_cancel_are_rejected_after_reconcile() -> None:
    first = initial(content_tree(((0,), ("a",))))
    state = BranchOutputState[str].build(first).publish(first.branch(path(0)), "current")
    next_snapshot = advance_runtime(
        first,
        content_tree(((0,), ("changed",))),
        SharedInputsUnchanged.build(),
    )
    current = state.reconcile(next_snapshot)

    assert current.current(next_snapshot.branch(path(0))) is None
    for transition in (
        lambda: current.publish(first.branch(path(0)), "late"),
        lambda: current.fail(first.branch(path(0))),
        lambda: current.cancel(first.branch(path(0))),
    ):
        with pytest.raises(StaleBranchGenerationError):
            transition()

    published = current.publish(next_snapshot.branch(path(0)), "next")
    assert published.current(next_snapshot.branch(path(0))) == "next"


def test_output_state_uses_exact_coordinates_and_rejects_unknown_paths() -> None:
    snapshot = initial(content_tree(((0,), ("a",))))
    state = BranchOutputState[str].build(snapshot)
    wrong_root = BranchRuntimeIdentity.build(
        BranchCoordinate.build(TreeRootId.build("other"), path(0)),
        snapshot.branch(path(0)).content_digest,
        snapshot.solve_generation,
        snapshot.branch(path(0)).request_generation,
    )

    with pytest.raises(StaleBranchGenerationError):
        state.publish(wrong_root, "wrong")
    with pytest.raises(UnknownRuntimeBranchError):
        snapshot.branch(path(4))
    with pytest.raises(UnknownRuntimeBranchError):
        state.current(
            BranchRuntimeIdentity.build(
                BranchCoordinate.build(snapshot.root_id, path(4)),
                snapshot.branch(path(0)).content_digest,
                snapshot.solve_generation,
                BranchRequestGeneration.build(0),
            )
        )


def test_generation_and_runtime_raw_constructors_fail_loudly() -> None:
    with pytest.raises(InvalidSolveGenerationError):
        SolveGeneration(-1)
    with pytest.raises(InvalidSolveGenerationError):
        SolveGeneration(True)
    with pytest.raises(InvalidBranchRequestGenerationError):
        BranchRequestGeneration(-1)

    snapshot = initial(content_tree(((0,), ("a",))))
    branch = snapshot.branch(path(0))
    with pytest.raises(InvalidBranchRuntimeIdentityError):
        BranchRuntimeIdentity(
            branch.coordinate,
            cast(BranchContentDigest, object()),
            branch.solve_generation,
            branch.request_generation,
        )
    with pytest.raises(InvalidTreeRuntimeSnapshotError):
        TreeRuntimeSnapshot(
            snapshot.root_id,
            snapshot.content,
            snapshot.solve_generation,
            snapshot.branches,
        )
    with pytest.raises(InvalidBranchOutputStateError):
        BranchOutputState(snapshot.branches, (), ())
