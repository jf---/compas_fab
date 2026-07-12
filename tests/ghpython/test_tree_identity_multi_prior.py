from attrs import evolve
import pytest

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
from compas_fab.ghpython.tree_identity import InvalidStagePriorBindingError
from compas_fab.ghpython.tree_identity import SourceCoordinateRequirement
from compas_fab.ghpython.tree_identity import SourceTreeIdentity
from compas_fab.ghpython.tree_identity import StageBranchEvidence
from compas_fab.ghpython.tree_identity import StagePriorBinding
from compas_fab.ghpython.tree_identity import StageTreeIdentity
from compas_fab.ghpython.tree_identity import TEXT_CODEC
from compas_fab.ghpython.tree_identity import UnregisteredStageImplementationError
from compas_fab.ghpython.tree_identity import _register_audited_stage_schema
from compas_fab.ghpython.tree_values import Tree
from compas_fab.ghpython.tree_values import TreeBranch
from compas_fab.ghpython.tree_values import TreeItem

SEMANTICS = PortSemantics.build(TopologyRole.TREE, BranchSemantics.ORDERED_SEQUENCE, ItemShape.scalar())
SCHEMA = "tests.ghpython.multi_prior/v1"


def _builder() -> None:
    pass


def _other_builder() -> None:
    pass


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
        implementation=_builder,
        prior_binding=StagePriorBinding.build("pose", pose_tree.root_id, pose),
        additional_priors=(StagePriorBinding.build("move", move_tree.root_id, move),),
        branch_evidence=(
            StageBranchEvidence.build(GhPath.build(0), (CanonicalField.bytes("native_program", b"program-a"),)),
            StageBranchEvidence.build(GhPath.build(4, 1), (CanonicalField.bytes("native_program", evidence_second),)),
        ),
    )


@pytest.fixture(scope="module", autouse=True)
def _registered_schema() -> None:
    _register_audited_stage_schema(SCHEMA, _builder, SourceCoordinateRequirement.COMPLETE_OUTPUTS)


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


def test_registered_schema_requires_exact_implementation() -> None:
    with pytest.raises(UnregisteredStageImplementationError):
        _identity_with_implementation(_other_builder)


def _identity_with_implementation(implementation: object) -> StageTreeIdentity:
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
        implementation=implementation,  # type: ignore[arg-type]
        prior_binding=StagePriorBinding.build("pose", tree.root_id, identity),
    )


def test_duplicate_registration_is_rejected() -> None:
    with pytest.raises(DuplicateAuditedStageSchemaError):
        _register_audited_stage_schema(SCHEMA, _builder, SourceCoordinateRequirement.COMPLETE_OUTPUTS)


def test_raw_prior_and_evidence_bypass_is_rejected() -> None:
    identity = _identity("move-b")
    with pytest.raises(InvalidStagePriorBindingError):
        evolve(identity, prior_bindings=())
    with pytest.raises(InvalidStageBranchEvidenceError):
        evolve(identity, branch_evidence=tuple(reversed(identity.branch_evidence)))
