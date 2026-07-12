from attrs import evolve
import pytest
from typing import Any
from typing import cast

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
from compas_fab.ghpython.tree_identity import DuplicateAuditedStageSchemaError
from compas_fab.ghpython.tree_identity import InvalidStageBranchEvidenceError
from compas_fab.ghpython.tree_identity import InvalidAuditedStageSchemaError
from compas_fab.ghpython.tree_identity import InvalidStagePriorBindingError
from compas_fab.ghpython.tree_identity import IdentityVerification
from compas_fab.ghpython.tree_identity import SourceCoordinateRequirement
from compas_fab.ghpython.tree_identity import SourceTreeIdentity
from compas_fab.ghpython.tree_identity import StageBranchEvidence
from compas_fab.ghpython.tree_identity import StagePriorBinding
from compas_fab.ghpython.tree_identity import StageTreeIdentity
from compas_fab.ghpython.tree_identity import TEXT_CODEC
from compas_fab.ghpython.tree_identity import InvalidStageBuilderAuthorityError
from compas_fab.ghpython.tree_identity import _register_audited_stage_schema
from compas_fab.ghpython.tree_values import Tree
from compas_fab.ghpython.tree_values import TreeBranch
from compas_fab.ghpython.tree_values import TreeItem

SEMANTICS = PortSemantics.build(TopologyRole.TREE, BranchSemantics.ORDERED_SEQUENCE, ItemShape.scalar())
SCHEMA = "tests.ghpython.multi_prior/v1"
OTHER_SCHEMA = "tests.ghpython.other_authority/v1"


def _builder() -> None:
    pass


def _other_builder() -> None:
    pass


_AUTHORITY = _register_audited_stage_schema(SCHEMA, _builder, SourceCoordinateRequirement.COMPLETE_OUTPUTS)
_OTHER_AUTHORITY = _register_audited_stage_schema(OTHER_SCHEMA, _other_builder, SourceCoordinateRequirement.COMPLETE_OUTPUTS)


def _tree(root: str, first: str, second: str) -> Tree[str]:
    return Tree.build(
        TreeRootId.build(root),
        (
            TreeBranch.build(GhPath.build(0), (TreeItem.value(first),)),
            TreeBranch.build(GhPath.build(4, 1), (TreeItem.value(second),)),
        ),
    )


def _coordinate(root: str, path: GhPath) -> TreeCoordinate:
    return TreeCoordinate.build(BranchCoordinate.build(TreeRootId.build(root), path), ItemIndex.build(0))


def _mapping() -> SourceCoordinateMap:
    return SourceCoordinateMap.build(
        tuple(
            SourceCoordinateEntry.build(
                _coordinate("output", path),
                (_coordinate("pose", path), _coordinate("move", path)),
            )
            for path in (GhPath.build(0), GhPath.build(4, 1))
        )
    )


def _identity(move_second: str, *, evidence_second: bytes = b"program-b") -> StageTreeIdentity:
    pose_tree = _tree("pose", "pose-a", "pose-b")
    move_tree = _tree("move", "move-a", move_second)
    pose = SourceTreeIdentity.build(pose_tree, TEXT_CODEC, SEMANTICS)
    move = SourceTreeIdentity.build(move_tree, TEXT_CODEC, SEMANTICS)
    return StageTreeIdentity.build(
        pose,
        SCHEMA,
        (),
        pose_tree.topology,
        _mapping(),
        authority=_AUTHORITY,
        prior_binding=StagePriorBinding.build("pose", pose_tree.root_id, pose),
        additional_priors=(StagePriorBinding.build("move", move_tree.root_id, move),),
        branch_evidence=(
            StageBranchEvidence.build(GhPath.build(0), (CanonicalField.bytes("native_program", b"program-a"),)),
            StageBranchEvidence.build(GhPath.build(4, 1), (CanonicalField.bytes("native_program", evidence_second),)),
        ),
    )


def test_secondary_branch_edit_changes_only_corresponding_output_branch() -> None:
    baseline = _identity("move-b")
    changed = _identity("changed")

    assert baseline.digest != changed.digest
    assert baseline.branch(GhPath.build(0)) == changed.branch(GhPath.build(0))
    assert baseline.branch(GhPath.build(4, 1)) != changed.branch(GhPath.build(4, 1))


def test_branch_evidence_changes_only_its_output_branch() -> None:
    baseline = _identity("move-b")
    changed = _identity("move-b", evidence_second=b"changed")

    assert baseline.branch(GhPath.build(0)) == changed.branch(GhPath.build(0))
    assert baseline.branch(GhPath.build(4, 1)) != changed.branch(GhPath.build(4, 1))


def test_any_unverifiable_bound_prior_downgrades_complete_stage() -> None:
    pose_tree = _tree("pose", "pose-a", "pose-b")
    move_tree = _tree("move", "move-a", "move-b")
    pose = SourceTreeIdentity.build(pose_tree, TEXT_CODEC, SEMANTICS)
    move_source = SourceTreeIdentity.build(move_tree, TEXT_CODEC, SEMANTICS)
    move = StageTreeIdentity.build(
        move_source,
        "external.native.move_series/v1",
        (),
        move_tree.topology,
        verification=IdentityVerification.UNVERIFIABLE,
    )
    identity = StageTreeIdentity.build(
        pose,
        SCHEMA,
        (),
        pose_tree.topology,
        _mapping(),
        prior_binding=StagePriorBinding.build("pose", pose_tree.root_id, pose),
        additional_priors=(StagePriorBinding.build("move", move_tree.root_id, move),),
    )

    assert identity.verification is IdentityVerification.UNVERIFIABLE
    assert identity.requires_fresh_compute_token
    assert not identity.reusable_from_content_cache


def test_registered_schema_requires_exact_implementation() -> None:
    with pytest.raises(InvalidStageBuilderAuthorityError):
        _identity_with_authority(_OTHER_AUTHORITY)

    with pytest.raises(InvalidStageBuilderAuthorityError):
        _identity_with_authority(None)


def _identity_with_authority(authority: object) -> StageTreeIdentity:
    tree = _tree("pose", "a", "b")
    identity = SourceTreeIdentity.build(tree, TEXT_CODEC, SEMANTICS)
    mapping = SourceCoordinateMap.build(
        tuple(SourceCoordinateEntry.build(_coordinate("output", path), (_coordinate("pose", path),)) for path in tree.topology.paths)
    )
    return StageTreeIdentity.build(
        identity,
        SCHEMA,
        (),
        tree.topology,
        mapping,
        authority=cast(Any, authority),
        prior_binding=StagePriorBinding.build("pose", tree.root_id, identity),
    )


def test_duplicate_registration_is_rejected() -> None:
    with pytest.raises(DuplicateAuditedStageSchemaError):
        _register_audited_stage_schema(SCHEMA, _builder, SourceCoordinateRequirement.COMPLETE_OUTPUTS)


def test_registration_rejects_unhashable_schema_with_named_error() -> None:
    with pytest.raises(InvalidAuditedStageSchemaError):
        _register_audited_stage_schema(cast(Any, []), _builder, SourceCoordinateRequirement.COMPLETE_OUTPUTS)


def test_raw_prior_and_evidence_bypass_is_rejected() -> None:
    identity = _identity("move-b")
    with pytest.raises(InvalidStageBuilderAuthorityError):
        evolve(identity, builder_authority=None)
    with pytest.raises(InvalidStageBuilderAuthorityError):
        evolve(identity, builder_authority=_OTHER_AUTHORITY)
    with pytest.raises(InvalidStagePriorBindingError):
        evolve(identity, prior_bindings=())
    with pytest.raises(InvalidStageBranchEvidenceError):
        evolve(identity, branch_evidence=tuple(reversed(identity.branch_evidence)))


@pytest.mark.parametrize("invalid", [None, []])
def test_additional_prior_container_fails_with_named_error(invalid: object) -> None:
    tree = _tree("pose", "a", "b")
    identity = SourceTreeIdentity.build(tree, TEXT_CODEC, SEMANTICS)
    mapping = SourceCoordinateMap.build(
        tuple(SourceCoordinateEntry.build(_coordinate("output", path), (_coordinate("pose", path),)) for path in tree.topology.paths)
    )
    with pytest.raises(InvalidStagePriorBindingError):
        StageTreeIdentity.build(
            identity,
            SCHEMA,
            (),
            tree.topology,
            mapping,
            authority=_AUTHORITY,
            prior_binding=StagePriorBinding.build("pose", tree.root_id, identity),
            additional_priors=cast(Any, invalid),
        )


def test_duplicate_prior_roles_and_roots_are_rejected() -> None:
    tree = _tree("pose", "a", "b")
    identity = SourceTreeIdentity.build(tree, TEXT_CODEC, SEMANTICS)
    primary = StagePriorBinding.build("pose", tree.root_id, identity)
    with pytest.raises(InvalidStagePriorBindingError):
        StageTreeIdentity.build(
            identity,
            SCHEMA,
            (),
            tree.topology,
            authority=_AUTHORITY,
            prior_binding=primary,
            additional_priors=(StagePriorBinding.build("pose", TreeRootId.build("other"), identity),),
        )
    with pytest.raises(InvalidStagePriorBindingError):
        StageTreeIdentity.build(
            identity,
            SCHEMA,
            (),
            tree.topology,
            authority=_AUTHORITY,
            prior_binding=primary,
            additional_priors=(StagePriorBinding.build("other", tree.root_id, identity),),
        )


def test_raw_legacy_identity_cannot_gain_unhashed_branch_evidence() -> None:
    tree = _tree("source", "a", "b")
    source = SourceTreeIdentity.build(tree, TEXT_CODEC, SEMANTICS)
    legacy = StageTreeIdentity.build(
        source,
        "external.legacy/v1",
        (),
        tree.topology,
        verification=IdentityVerification.UNVERIFIABLE,
    )
    evidence = tuple(
        StageBranchEvidence.build(path, (CanonicalField.bytes("external", b"value"),))
        for path in tree.topology.paths
    )

    with pytest.raises(InvalidStageBranchEvidenceError):
        evolve(legacy, branch_evidence=evidence)


def test_legacy_stage_v1_canonical_vector_remains_byte_stable() -> None:
    tree = Tree.build(
        TreeRootId.build("runtime-root"),
        (TreeBranch.build(GhPath.build(0), (TreeItem.value("a"),)),),
    )
    source = SourceTreeIdentity.build(tree, TEXT_CODEC, SEMANTICS)
    legacy = StageTreeIdentity.build(
        source,
        "external.legacy/v1",
        (),
        tree.topology,
        verification=IdentityVerification.UNVERIFIABLE,
    )

    assert legacy.digest.value == "0dc81b7810c3c17c0416aaf84a7f3076486ec87c1b4cbec8982a21fb7b2a5a88"
    assert legacy.branch_digests[0].value == "c19f7c436eb7285d2ead6d4ac90e763bfaf72a29b6b322b6dd19bba7f33a863a"


def test_consistent_runtime_root_reroute_preserves_bound_identity() -> None:
    def build_with_roots(pose_root: str, move_root: str, output_root: str) -> StageTreeIdentity:
        pose_tree = _tree(pose_root, "a", "b")
        move_tree = _tree(move_root, "c", "d")
        pose = SourceTreeIdentity.build(pose_tree, TEXT_CODEC, SEMANTICS)
        move = SourceTreeIdentity.build(move_tree, TEXT_CODEC, SEMANTICS)
        mapping = SourceCoordinateMap.build(
            tuple(
                SourceCoordinateEntry.build(
                    _coordinate(output_root, path),
                    (_coordinate(pose_root, path), _coordinate(move_root, path)),
                )
                for path in pose_tree.topology.paths
            )
        )
        return StageTreeIdentity.build(
            pose,
            SCHEMA,
            (),
            pose_tree.topology,
            mapping,
            authority=_AUTHORITY,
            prior_binding=StagePriorBinding.build("pose", pose_tree.root_id, pose),
            additional_priors=(StagePriorBinding.build("move", move_tree.root_id, move),),
        )

    assert build_with_roots("pose-a", "move-a", "out-a") == build_with_roots("pose-b", "move-b", "out-b")


def test_equal_content_priors_are_separated_by_semantic_role() -> None:
    left_tree = _tree("left", "same-a", "same-b")
    right_tree = _tree("right", "same-a", "same-b")
    left = SourceTreeIdentity.build(left_tree, TEXT_CODEC, SEMANTICS)
    right = SourceTreeIdentity.build(right_tree, TEXT_CODEC, SEMANTICS)

    def build_for_source(source_root: str) -> StageTreeIdentity:
        mapping = SourceCoordinateMap.build(
            tuple(
                SourceCoordinateEntry.build(
                    _coordinate("output", path),
                    (_coordinate(source_root, path),),
                )
                for path in left_tree.topology.paths
            )
        )
        return StageTreeIdentity.build(
            left,
            SCHEMA,
            (),
            left_tree.topology,
            mapping,
            authority=_AUTHORITY,
            prior_binding=StagePriorBinding.build("left_role", left_tree.root_id, left),
            additional_priors=(StagePriorBinding.build("right_role", right_tree.root_id, right),),
        )

    left_sourced = build_for_source("left")
    right_sourced = build_for_source("right")
    assert left_sourced.digest != right_sourced.digest
    assert left_sourced.branch(GhPath.build(0)) != right_sourced.branch(GhPath.build(0))
