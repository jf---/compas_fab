import pytest
from attrs import evolve
from compas.geometry import Frame
from tesseract_robotics.planning import MotionProgram
from tesseract_robotics.planning import MoveType

from compas_fab.ghpython.group_shape import GroupShapeFactory
from compas_fab.ghpython.group_shape import PlanningGroupId
from compas_fab.ghpython.item_values import Scalar
from compas_fab.ghpython.optional_tree_input import OptionalTreeInput
from compas_fab.ghpython.group_shape import GroupFixedVector
from compas_fab.ghpython.tesseract_cartesian_target_series import CartesianTargetSeriesParameters
from compas_fab.ghpython.tesseract_cartesian_target_series import build_cartesian_target_series
from compas_fab.ghpython.tesseract_joint_target_series import build_joint_target_series
from compas_fab.ghpython.tesseract_pose_series import PoseSeriesParameters
from compas_fab.ghpython.tesseract_pose_series import build_pose_series
from compas_fab.ghpython.tesseract_state_target_series import build_state_target_series
from compas_fab.ghpython.tesseract_program_series import InvalidProgramSeriesBuildError
from compas_fab.ghpython.tesseract_program_series import ProgramSeriesParameters
from compas_fab.ghpython.tesseract_program_series import build_motion_program_series
from compas_fab.ghpython.tesseract_vector_target_common import VectorTargetSeriesParameters
from compas_fab.ghpython.tree_coordinates import GhPath
from compas_fab.ghpython.tree_coordinates import TreeRootId
from compas_fab.ghpython.tree_diagnostics import ReducedTopologyOutput
from compas_fab.ghpython.tree_identity import IdentityVerification
from compas_fab.ghpython.tree_values import Tree
from compas_fab.ghpython.tree_values import TreeBranch
from compas_fab.ghpython.tree_values import TreeItem


def _frame_source(root: str = "frames") -> Tree[Frame]:
    return Tree.build(
        TreeRootId.build(root),
        (
            TreeBranch.build(GhPath.build(0), (TreeItem.value(Frame.worldXY()),)),
            TreeBranch.build(GhPath.build(4, 1), (TreeItem.value(Frame([1, 0, 0], [1, 0, 0], [0, 1, 0])),)),
        ),
    )


def _pose_parameters() -> PoseSeriesParameters:
    return PoseSeriesParameters.build(1.0, "base")


def _target_parameters() -> CartesianTargetSeriesParameters:
    return CartesianTargetSeriesParameters.build(Scalar.build(MoveType.LINEAR), Scalar.build("DEFAULT"))


def _program_parameters() -> ProgramSeriesParameters:
    return ProgramSeriesParameters.build("manipulator", None, "base", "DEFAULT")


def _cartesian_targets(root: str = "frames"):
    poses = build_pose_series(_frame_source(root), _pose_parameters())
    return poses, build_cartesian_target_series(poses, _target_parameters())


def _pipeline(native_robot, source_root: TreeRootId):
    _poses, targets = _cartesian_targets(source_root.value)
    return build_motion_program_series(native_robot, targets, _program_parameters())


def test_exact_series_builds_one_program_per_branch_with_provenance(native_robot) -> None:
    poses = build_pose_series(_frame_source(), _pose_parameters())
    targets = build_cartesian_target_series(poses, _target_parameters())
    programs = build_motion_program_series(native_robot, targets, _program_parameters())

    assert [branch.path for branch in programs.output.values.branches] == [GhPath.build(0), GhPath.build(4, 1)]
    assert all(len(branch.items) == 1 for branch in programs.output.values.branches)
    first_target = targets.output.values.branches[0].items[0].item
    assert first_target.pose is poses.output.values.branches[0].items[0].item
    assert programs.output.values.branches[0].items[0].item.motion_program.targets[0] is first_target
    assert isinstance(programs.output.values.branches[0].items[0].item.motion_program, MotionProgram)
    assert programs.identity.verification is IdentityVerification.VERIFIED


def test_output_is_reduced_topology_with_complete_source_map(native_robot) -> None:
    programs = _pipeline(native_robot, TreeRootId.build("frames"))

    assert isinstance(programs.output, ReducedTopologyOutput)
    assert len(programs.output.source_coordinates.entries) == 2
    assert programs.output.branch_diagnostics.entries == ()


def test_full_native_authoring_identity_ignores_routing_root(native_robot) -> None:
    left = _pipeline(native_robot, TreeRootId.build("document-a"))
    right = _pipeline(native_robot, TreeRootId.build("document-b"))

    assert left.identity == right.identity
    assert left.output.source_coordinates.root_free_identity_bytes() == right.output.source_coordinates.root_free_identity_bytes()


def test_null_target_invalidates_only_its_program_branch(native_robot) -> None:
    frames = Tree.build(
        TreeRootId.build("frames"),
        (
            TreeBranch.build(GhPath.build(0), (TreeItem.value(Frame.worldXY()), TreeItem.null())),
            TreeBranch.build(GhPath.build(4, 1), (TreeItem.value(Frame.worldXY()),)),
        ),
    )
    poses = build_pose_series(frames, _pose_parameters())
    targets = build_cartesian_target_series(poses, _target_parameters())
    programs = build_motion_program_series(native_robot, targets, _program_parameters())

    assert programs.output.values.branches[0].items[0].is_null
    assert programs.output.values.branches[1].items[0].item.motion_program.targets[0] is targets.output.values.branches[1].items[0].item
    assert programs.identity.verification is IdentityVerification.VERIFIED
    branch_paths = tuple(coordinate.path for coordinate, _diagnostic in programs.output.branch_diagnostics.entries)
    assert branch_paths == (GhPath.build(0),)


def test_empty_ordered_branch_yields_branch_diagnostic(native_robot) -> None:
    frames = Tree.build(
        TreeRootId.build("frames"),
        (
            TreeBranch.build(GhPath.build(0), (TreeItem.value(Frame.worldXY()),)),
            TreeBranch.build(GhPath.build(1), ()),
        ),
    )
    poses = build_pose_series(frames, _pose_parameters())
    targets = build_cartesian_target_series(poses, _target_parameters())
    programs = build_motion_program_series(native_robot, targets, _program_parameters())

    assert programs.output.values.branches[1].items[0].is_null
    empty_entry = programs.output.source_coordinates.entries[1]
    assert empty_entry.sources == ()
    assert empty_entry.empty_source_branch is not None
    branch_paths = tuple(coordinate.path for coordinate, _diagnostic in programs.output.branch_diagnostics.entries)
    assert branch_paths == (GhPath.build(1),)


def test_joint_targets_feed_program_series(native_robot) -> None:
    shape = GroupShapeFactory.from_native_robot(native_robot, PlanningGroupId.build("manipulator"))
    positions = Tree.build(
        TreeRootId.build("positions"),
        (TreeBranch.build(GhPath.build(0), (TreeItem.value(GroupFixedVector.positions(shape, (0.1,))),)),),
    )
    parameters = VectorTargetSeriesParameters.build(Scalar.build(MoveType.LINEAR), Scalar.build("DEFAULT"))
    targets = build_joint_target_series(shape, positions, OptionalTreeInput.absent(), parameters)
    programs = build_motion_program_series(native_robot, targets, _program_parameters())

    assert programs.output.values.branches[0].items[0].item.motion_program.targets[0] is targets.output.values.branches[0].items[0].item
    assert programs.identity.verification is IdentityVerification.VERIFIED


def test_state_targets_feed_program_series(native_robot) -> None:
    shape = GroupShapeFactory.from_native_robot(native_robot, PlanningGroupId.build("manipulator"))
    positions = Tree.build(
        TreeRootId.build("positions"),
        (TreeBranch.build(GhPath.build(0), (TreeItem.value(GroupFixedVector.positions(shape, (0.1,))),)),),
    )
    parameters = VectorTargetSeriesParameters.build(Scalar.build(MoveType.LINEAR), Scalar.build("DEFAULT"))
    targets = build_state_target_series(
        shape,
        positions,
        OptionalTreeInput.absent(),
        OptionalTreeInput.absent(),
        OptionalTreeInput.absent(),
        OptionalTreeInput.absent(),
        parameters,
    )
    programs = build_motion_program_series(native_robot, targets, _program_parameters())

    assert programs.output.values.branches[0].items[0].item.motion_program.targets[0] is targets.output.values.branches[0].items[0].item
    assert programs.identity.verification is IdentityVerification.VERIFIED


def test_raw_constructor_cannot_swap_parameters(native_robot) -> None:
    programs = _pipeline(native_robot, TreeRootId.build("frames"))
    with pytest.raises(InvalidProgramSeriesBuildError):
        evolve(programs, parameters=ProgramSeriesParameters.build("manipulator", None, "base", "OTHER"))


def test_provenance_native_program_digest_mismatch_fails(native_robot) -> None:
    poses = build_pose_series(_frame_source(), _pose_parameters())
    targets = build_cartesian_target_series(poses, _target_parameters())
    first = build_motion_program_series(native_robot, targets, _program_parameters())
    other = build_motion_program_series(native_robot, targets, ProgramSeriesParameters.build("manipulator", None, "base", "OTHER"))

    with pytest.raises(InvalidProgramSeriesBuildError):
        evolve(first, output=other.output)
