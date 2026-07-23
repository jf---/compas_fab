import pytest
from tesseract_robotics.planning import MoveType

from compas_fab.ghpython.group_shape import GroupFixedVector
from compas_fab.ghpython.group_shape import GroupShape
from compas_fab.ghpython.group_shape import JointId
from compas_fab.ghpython.group_shape import PlanningGroupId
from compas_fab.ghpython.item_values import Scalar
from compas_fab.ghpython.optional_tree_input import OptionalTreeInput
from compas_fab.ghpython.tesseract_state_target_series import StateTargetSeriesBuild
from compas_fab.ghpython.tesseract_state_target_series import StateTargetSeriesValueChangedError
from compas_fab.ghpython.tesseract_state_target_series import build_state_target_series
from compas_fab.ghpython.tesseract_vector_target_common import IncompatibleGroupVectorTreeError
from compas_fab.ghpython.tesseract_vector_target_common import VectorTargetSeriesParameters
from compas_fab.ghpython.tree_coordinates import GhPath
from compas_fab.ghpython.tree_coordinates import TreeRootId
from compas_fab.ghpython.tree_values import Tree
from compas_fab.ghpython.tree_values import TreeBranch
from compas_fab.ghpython.tree_values import TreeItem


def _shape() -> GroupShape:
    return GroupShape.build(PlanningGroupId.build("manipulator"), (JointId.build("joint_1"), JointId.build("joint_2")))


def _positions(shape: GroupShape) -> Tree[GroupFixedVector]:
    return Tree.build(
        TreeRootId.build("positions"),
        (TreeBranch.build(GhPath.build(2), (TreeItem.value(GroupFixedVector.positions(shape, (0.0, 1.0))),)),),
    )


def _parameters() -> VectorTargetSeriesParameters:
    return VectorTargetSeriesParameters.build(Scalar.build(MoveType.FREESPACE), Scalar.build("DEFAULT"))


def test_absent_state_optionals_remain_exact_native_absence() -> None:
    shape = _shape()
    built = build_state_target_series(
        shape,
        _positions(shape),
        OptionalTreeInput.absent(),
        OptionalTreeInput.absent(),
        OptionalTreeInput.absent(),
        OptionalTreeInput.absent(),
        _parameters(),
    )
    target = built.output.values.branches[0].items[0].item

    assert target.names is None
    assert target.velocities is None
    assert target.accelerations is None
    assert target.time is None


def test_present_null_time_is_topology_bearing_invalid_input() -> None:
    shape = _shape()
    times = Tree.build(
        TreeRootId.build("times"),
        (TreeBranch.build(GhPath.build(2), (TreeItem.null(),)),),
    )
    built = build_state_target_series(
        shape,
        _positions(shape),
        OptionalTreeInput.absent(),
        OptionalTreeInput.absent(),
        OptionalTreeInput.absent(),
        OptionalTreeInput.present(times),
        _parameters(),
    )

    assert built.output.values.branches[0].items[0].is_null
    assert built.time_input.is_present


def _optional_branch(root: str, vector: GroupFixedVector) -> Tree[GroupFixedVector]:
    return Tree.build(TreeRootId.build(root), (TreeBranch.build(GhPath.build(2), (TreeItem.value(vector),)),))


def _absent_build(shape: GroupShape) -> StateTargetSeriesBuild:
    return build_state_target_series(
        shape,
        _positions(shape),
        OptionalTreeInput.absent(),
        OptionalTreeInput.absent(),
        OptionalTreeInput.absent(),
        OptionalTreeInput.absent(),
        _parameters(),
    )


def test_present_state_dynamics_land_as_exact_native_values() -> None:
    shape = _shape()
    times = Tree.build(TreeRootId.build("times"), (TreeBranch.build(GhPath.build(2), (TreeItem.value(1.5),)),))
    built = build_state_target_series(
        shape,
        _positions(shape),
        OptionalTreeInput.present(_optional_branch("names", GroupFixedVector.names(shape, ("joint_1", "joint_2")))),
        OptionalTreeInput.present(_optional_branch("velocities", GroupFixedVector.velocities(shape, (0.1, 0.2)))),
        OptionalTreeInput.present(_optional_branch("accelerations", GroupFixedVector.accelerations(shape, (0.3, 0.4)))),
        OptionalTreeInput.present(times),
        _parameters(),
    )
    target = built.output.values.branches[0].items[0].item

    assert target.names == ["joint_1", "joint_2"]
    assert list(target.velocities) == [0.1, 0.2]
    assert list(target.accelerations) == [0.3, 0.4]
    assert target.time == 1.5
    assert built.identity != _absent_build(shape).identity


def test_state_target_series_rejects_wrong_group_before_native_work() -> None:
    shape = _shape()
    other = GroupShape.build(PlanningGroupId.build("other"), (JointId.build("joint_1"), JointId.build("joint_2")))
    with pytest.raises(IncompatibleGroupVectorTreeError):
        build_state_target_series(
            shape,
            _positions(other),
            OptionalTreeInput.absent(),
            OptionalTreeInput.absent(),
            OptionalTreeInput.absent(),
            OptionalTreeInput.absent(),
            _parameters(),
        )


def test_state_require_current_catches_mutated_native_target() -> None:
    shape = _shape()
    built = build_state_target_series(
        shape,
        _positions(shape),
        OptionalTreeInput.absent(),
        OptionalTreeInput.absent(),
        OptionalTreeInput.absent(),
        OptionalTreeInput.absent(),
        _parameters(),
    )
    built.require_current()
    built.output.values.branches[0].items[0].item.profile = "MUTATED"
    with pytest.raises(StateTargetSeriesValueChangedError):
        built.require_current()
