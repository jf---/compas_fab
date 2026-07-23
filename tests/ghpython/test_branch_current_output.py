from __future__ import annotations

from typing import Optional
from typing import Tuple
from typing import cast

import pytest

import compas_fab.ghpython.branch_current_output as branch_current_output
from compas_fab.ghpython.branch_current_output import BranchDecision
from compas_fab.ghpython.branch_current_output import BranchOutputState
from compas_fab.ghpython.branch_current_output import ConflictingBranchTerminalTransitionError
from compas_fab.ghpython.branch_current_output import DuplicateBranchTerminalTransitionError
from compas_fab.ghpython.branch_current_output import InvalidBranchReconciliationTransitionError
from compas_fab.ghpython.branch_current_output import InvalidBranchDecisionStateError
from compas_fab.ghpython.branch_current_output import InvalidBranchOutputStateError
from compas_fab.ghpython.branch_current_output import RetiredBranchRequestError
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
from compas_fab.ghpython.branch_runtime_identity import StaleBranchRequestRetryError
from compas_fab.ghpython.branch_runtime_identity import TreeRuntimeSnapshot
from compas_fab.ghpython.branch_runtime_identity import UnknownRuntimeBranchError
from compas_fab.ghpython.branch_runtime_identity import advance_branch_request
from compas_fab.ghpython.branch_runtime_identity import advance_runtime
from compas_fab.ghpython.component_identity import CanonicalField
from compas_fab.ghpython.item_values import ItemShape
from compas_fab.ghpython.port_semantics import BranchSemantics
from compas_fab.ghpython.port_semantics import PortSemantics
from compas_fab.ghpython.port_semantics import TopologyRole
from compas_fab.ghpython.tree_coordinates import BranchCoordinate
from compas_fab.ghpython.tree_coordinates import GhPath
from compas_fab.ghpython.tree_coordinates import ItemIndex
from compas_fab.ghpython.tree_coordinates import TreeCoordinate
from compas_fab.ghpython.tree_coordinates import TreeRootId
from compas_fab.ghpython.tree_diagnostics import SourceCoordinateEntry
from compas_fab.ghpython.tree_diagnostics import SourceCoordinateMap
from compas_fab.ghpython.tree_identity import BranchContentDigest
from compas_fab.ghpython.tree_identity import IdentityVerification
from compas_fab.ghpython.tree_identity import SourceTreeIdentity
from compas_fab.ghpython.tree_identity import StageTreeIdentity
from compas_fab.ghpython.tree_identity import TEXT_CODEC
from compas_fab.ghpython.tree_values import Tree
from compas_fab.ghpython.tree_values import TreeBranch
from compas_fab.ghpython.tree_values import TreeItem
from compas_fab.ghpython.tree_values import TreeTopology


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


def forged_snapshot(
    previous: TreeRuntimeSnapshot,
    content: SourceTreeIdentity,
    *,
    solve: int,
    requests: Tuple[int, ...],
    root: Optional[str] = None,
) -> TreeRuntimeSnapshot:
    root_id = previous.root_id if root is None else TreeRootId.build(root)
    solve_generation = SolveGeneration.build(solve)
    branches = tuple(
        BranchRuntimeIdentity.build(
            BranchCoordinate.build(root_id, branch_path),
            digest,
            solve_generation,
            BranchRequestGeneration.build(request),
        )
        for branch_path, digest, request in zip(
            content.topology.paths,
            content.branch_digests,
            requests,
        )
    )
    return TreeRuntimeSnapshot.build(root_id, content, solve_generation, branches)


def reduction_identity(source: SourceTreeIdentity) -> StageTreeIdentity:
    output_root = TreeRootId.build("aggregate-output")
    source_root = TreeRootId.build("aggregate-source")
    output_topology = TreeTopology.build(tuple(TreeBranch.build(source_path, (TreeItem.value("aggregate"),)) for source_path in source.topology.paths))
    entries = []
    for source_path, item_count in zip(source.topology.paths, source.topology.item_counts):
        output = TreeCoordinate.build(
            BranchCoordinate.build(output_root, source_path),
            ItemIndex.build(0),
        )
        sources = tuple(
            TreeCoordinate.build(
                BranchCoordinate.build(source_root, source_path),
                ItemIndex.build(item_index),
            )
            for item_index in range(item_count)
        )
        entries.append(SourceCoordinateEntry.build(output, sources))
    return StageTreeIdentity.build(
        source,
        "tests.ghpython.sequence_reduction/v1",
        (),
        output_topology,
        SourceCoordinateMap.build(tuple(entries)),
        IdentityVerification.UNVERIFIABLE,
    )


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
    assert reconciled[local.branch(path(0)).coordinate] is BranchDecision.CLEARED
    assert reconciled[local.branch(path(1)).coordinate] is BranchDecision.CURRENT
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
    assert all(branch.request_generation.value == local.branch(branch.coordinate.path).request_generation.value + 1 for branch in shared.branches)
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


def test_empty_branch_add_and_remove_each_close_the_root_dependency() -> None:
    first = initial(content_tree(((0,), ("a",))))
    added = advance_runtime(
        first,
        content_tree(((0,), ("a",)), ((2,), ())),
        SharedInputsUnchanged.build(),
    )
    removed = advance_runtime(
        added,
        content_tree(((0,), ("a",))),
        SharedInputsUnchanged.build(),
    )

    assert added.solve_generation == SolveGeneration.build(1)
    assert added.branch(path(0)).request_generation == BranchRequestGeneration.build(1)
    assert added.branch(path(2)).request_generation == BranchRequestGeneration.build(0)
    assert removed.solve_generation == SolveGeneration.build(2)
    assert removed.branch(path(0)).request_generation == BranchRequestGeneration.build(2)


def test_branch_cardinality_change_closes_the_root_dependency() -> None:
    first = initial(content_tree(((0,), ("a",)), ((1,), ("sibling",))))
    changed = advance_runtime(
        first,
        content_tree(((0,), ("a", "b")), ((1,), ("sibling",))),
        SharedInputsUnchanged.build(),
    )

    assert changed.solve_generation == SolveGeneration.build(1)
    assert changed.branch(path(0)).request_generation == BranchRequestGeneration.build(1)
    assert changed.branch(path(1)).request_generation == BranchRequestGeneration.build(1)


def test_sequence_reduction_provenance_keeps_element_edit_branch_local() -> None:
    first_content = reduction_identity(content_tree(((0,), ("a", "b")), ((1,), ("sibling",))))
    changed_content = reduction_identity(content_tree(((0,), ("changed", "b")), ((1,), ("sibling",))))
    first = TreeRuntimeSnapshot.initial(first_content, TreeRootId.build("runtime"))

    changed = advance_runtime(first, changed_content, SharedInputsUnchanged.build())

    assert changed.solve_generation == first.solve_generation
    assert changed.branch(path(0)).request_generation == BranchRequestGeneration.build(1)
    assert changed.branch(path(1)) == first.branch(path(1))


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


@pytest.mark.parametrize("terminal", ["fail", "cancel"])
def test_failed_or_cancelled_request_is_retired_until_branch_generation_advances(
    terminal: str,
) -> None:
    snapshot = initial(content_tree(((0,), ("a",)), ((1,), ("sibling",))))
    identity = snapshot.branch(path(0))
    state = BranchOutputState[str].build(snapshot)
    retired = state.fail(identity) if terminal == "fail" else state.cancel(identity)

    with pytest.raises(RetiredBranchRequestError):
        retired.publish(identity, "late")
    with pytest.raises(DuplicateBranchTerminalTransitionError):
        retired.fail(identity) if terminal == "fail" else retired.cancel(identity)
    with pytest.raises(ConflictingBranchTerminalTransitionError):
        retired.cancel(identity) if terminal == "fail" else retired.fail(identity)

    retry = advance_branch_request(snapshot, identity)
    assert retry.solve_generation == snapshot.solve_generation
    assert retry.branch(path(0)).content_digest == identity.content_digest
    assert retry.branch(path(0)).request_generation == BranchRequestGeneration.build(1)
    assert retry.branch(path(1)) == snapshot.branch(path(1))
    with pytest.raises(StaleBranchRequestRetryError):
        advance_branch_request(retry, identity)
    retried = retired.reconcile(retry).publish(retry.branch(path(0)), "retry")
    assert retried.current(retry.branch(path(0))) == "retry"


def test_published_success_is_terminal_and_cannot_be_erased() -> None:
    snapshot = initial(content_tree(((0,), ("a",))))
    identity = snapshot.branch(path(0))
    published = BranchOutputState[str].build(snapshot).publish(identity, "success")

    with pytest.raises(DuplicateBranchTerminalTransitionError):
        published.publish(identity, "duplicate")
    with pytest.raises(ConflictingBranchTerminalTransitionError):
        published.fail(identity)
    with pytest.raises(ConflictingBranchTerminalTransitionError):
        published.cancel(identity)
    assert published.current(identity) == "success"


def test_reconcile_rejects_branch_request_and_solve_rollbacks_without_mutation() -> None:
    first = initial(content_tree(((0,), ("a",))))
    retry = advance_branch_request(first, first.branch(path(0)))
    state = BranchOutputState[str].build(retry).publish(retry.branch(path(0)), "success")
    before_bytes = repr(state).encode("utf-8")

    with pytest.raises(InvalidBranchReconciliationTransitionError):
        state.reconcile(first)
    assert state == BranchOutputState[str].build(retry).publish(
        retry.branch(path(0)),
        "success",
    )
    assert repr(state).encode("utf-8") == before_bytes
    assert state.current(retry.branch(path(0))) == "success"

    shared = advance_runtime(
        retry,
        retry.content,
        SharedInputsChanged.build((CanonicalField.text("scene", "next"),)),
    )
    advanced = state.reconcile(shared)
    with pytest.raises(InvalidBranchReconciliationTransitionError):
        advanced.reconcile(retry)


def test_reconcile_rejects_same_solve_digest_root_and_topology_forgery() -> None:
    first = initial(content_tree(((0,), ("a",))))
    retired = BranchOutputState[str].build(first).cancel(first.branch(path(0)))
    before_bytes = repr(retired).encode("utf-8")
    forged = (
        forged_snapshot(
            first,
            content_tree(((0,), ("changed",))),
            solve=0,
            requests=(0,),
        ),
        forged_snapshot(
            first,
            first.content,
            solve=0,
            requests=(0,),
            root="other-root",
        ),
        forged_snapshot(
            first,
            content_tree(((0,), ("a",)), ((1,), ())),
            solve=0,
            requests=(0, 0),
        ),
        forged_snapshot(
            first,
            content_tree(((0,), ("a", "extra"))),
            solve=0,
            requests=(0,),
        ),
        forged_snapshot(
            first,
            content_tree(),
            solve=0,
            requests=(),
        ),
    )

    for target in forged:
        with pytest.raises(InvalidBranchReconciliationTransitionError):
            retired.reconcile(target)
        assert repr(retired).encode("utf-8") == before_bytes
        with pytest.raises(RetiredBranchRequestError):
            retired.publish(first.branch(path(0)), "late")


def test_reconcile_accepts_skipped_forward_solve_and_remove_readd_generation() -> None:
    first = initial(content_tree(((0,), ("a",)), ((1,), ("b",))))
    state = BranchOutputState[str].build(first).publish(first.branch(path(0)), "old")
    skipped = forged_snapshot(
        first,
        content_tree(((0,), ("a",)), ((1,), ("b",))),
        solve=5,
        requests=(7, 9),
        root="rerouted",
    )

    forward = state.reconcile(skipped)

    assert forward.snapshot == skipped
    assert set(forward.decisions.values()) == {BranchDecision.CLEARED}

    removed = forged_snapshot(
        skipped,
        content_tree(((1,), ("b",))),
        solve=6,
        requests=(0,),
        root="rerouted",
    )
    without_zero = forward.reconcile(removed)
    assert without_zero.snapshot == removed

    readded = forged_snapshot(
        removed,
        content_tree(((0,), ("readded",)), ((1,), ("b",))),
        solve=7,
        requests=(0, 1),
        root="rerouted",
    )
    accepted = without_zero.reconcile(readded)
    assert accepted.snapshot == readded
    assert accepted.current(readded.branch(path(0))) is None


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
        state[wrong_root.coordinate]
    with pytest.raises(UnknownRuntimeBranchError):
        state[BranchCoordinate.build(snapshot.root_id, path(4))]
    with pytest.raises(UnknownRuntimeBranchError):
        state[cast(BranchCoordinate, path(0))]
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
        BranchOutputState(snapshot, (), (), ())


def test_output_state_factory_rejects_malformed_decision_entries_before_indexing() -> None:
    snapshot = initial(content_tree(((0,), ("a",))))
    state = BranchOutputState[str].build(snapshot)

    with pytest.raises(InvalidBranchDecisionStateError):
        state._from_parts(
            state.snapshot,
            (),
            cast(Tuple[Tuple[BranchCoordinate, BranchDecision], ...], ((),)),
            (),
        )
    with pytest.raises(InvalidBranchDecisionStateError):
        state._from_parts(
            state.snapshot,
            (),
            ((state.expected[0].coordinate, BranchDecision.CURRENT),),
            (),
        )

    published = branch_current_output._PublishedBranch(state.expected[0], "value")
    with pytest.raises(InvalidBranchDecisionStateError):
        state._from_parts(
            state.snapshot,
            (published,),
            ((state.expected[0].coordinate, BranchDecision.ABSENT),),
            (),
        )
    with pytest.raises(InvalidBranchOutputStateError):
        state._from_parts(
            state.snapshot,
            (),
            ((state.expected[0].coordinate, BranchDecision.ABSENT),),
            cast(Tuple[branch_current_output._TerminalBranch, ...], ((),)),
        )

    failed = branch_current_output._TerminalBranch(
        state.expected[0],
        branch_current_output._BranchTerminal.FAILED,
    )
    with pytest.raises(InvalidBranchDecisionStateError):
        state._from_parts(
            state.snapshot,
            (),
            ((state.expected[0].coordinate, BranchDecision.ABSENT),),
            (failed,),
        )
