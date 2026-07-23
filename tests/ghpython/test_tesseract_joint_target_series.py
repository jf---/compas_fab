import pytest
from attrs import evolve
from tesseract_robotics.planning import MoveType

from compas_fab.ghpython.group_shape import GroupFixedVector
from compas_fab.ghpython.group_shape import GroupShape
from compas_fab.ghpython.group_shape import JointId
from compas_fab.ghpython.group_shape import PlanningGroupId
from compas_fab.ghpython.item_values import Scalar
from compas_fab.ghpython.optional_tree_input import OptionalTreeInput
from compas_fab.ghpython.tesseract_joint_target_series import InvalidJointTargetSeriesBuildError
from compas_fab.ghpython.tesseract_joint_target_series import JointTargetSeriesValueChangedError
from compas_fab.ghpython.tesseract_joint_target_series import build_joint_target_series
from compas_fab.ghpython.tesseract_vector_target_common import IncompatibleGroupVectorTreeError
from compas_fab.ghpython.tesseract_vector_target_common import VectorTargetSeriesParameters
from compas_fab.ghpython.tree_coordinates import GhPath
from compas_fab.ghpython.tree_coordinates import TreeRootId
from compas_fab.ghpython.tree_values import Tree
from compas_fab.ghpython.tree_values import TreeBranch
from compas_fab.ghpython.tree_values import TreeItem


def _shape() -> GroupShape:
    return GroupShape.build(
        PlanningGroupId.build("manipulator"),
        (JointId.build("joint_1"), JointId.build("joint_2")),
    )


def _positions(shape: GroupShape) -> Tree[GroupFixedVector]:
    return Tree.build(
        TreeRootId.build("positions"),
        (
            TreeBranch.build(
                GhPath.build(0),
                (
                    TreeItem.value(GroupFixedVector.positions(shape, (0.0, 1.0))),
                    TreeItem.value(GroupFixedVector.positions(shape, (2.0, 3.0))),
                ),
            ),
        ),
    )


def _parameters() -> VectorTargetSeriesParameters:
    return VectorTargetSeriesParameters.build(Scalar.build(MoveType.FREESPACE), Scalar.build("DEFAULT"))


def test_joint_target_absent_names_remain_exact_native_none() -> None:
    shape = _shape()
    built = build_joint_target_series(shape, _positions(shape), OptionalTreeInput.absent(), _parameters())

    assert all(item.item.names is None for item in built.output.values.branches[0].items)
    assert built.output.values.branches[0].items[0].item.positions.tolist() == [0.0, 1.0]
    assert len(built.output.values.branches[0].items) == 2


def test_present_null_name_slot_invalidates_only_exact_target_slot() -> None:
    shape = _shape()
    names = Tree.build(
        TreeRootId.build("names"),
        (
            TreeBranch.build(
                GhPath.build(0),
                (TreeItem.value(GroupFixedVector.names(shape, ("joint_1", "joint_2"))), TreeItem.null()),
            ),
        ),
    )
    built = build_joint_target_series(shape, _positions(shape), OptionalTreeInput.present(names), _parameters())

    assert built.output.values.branches[0].items[0].item.names == ["joint_1", "joint_2"]
    assert built.output.values.branches[0].items[1].is_null


def test_joint_target_series_rejects_wrong_group_before_native_work() -> None:
    shape = _shape()
    other = GroupShape.build(PlanningGroupId.build("other"), (JointId.build("joint_1"), JointId.build("joint_2")))
    with pytest.raises(IncompatibleGroupVectorTreeError):
        build_joint_target_series(shape, _positions(other), OptionalTreeInput.absent(), _parameters())


def test_joint_target_series_raw_constructor_cannot_swap_group() -> None:
    shape = _shape()
    built = build_joint_target_series(shape, _positions(shape), OptionalTreeInput.absent(), _parameters())
    other = GroupShape.build(PlanningGroupId.build("other"), shape.ordered_joint_ids)
    with pytest.raises(InvalidJointTargetSeriesBuildError):
        evolve(built, group_shape=other)


def test_joint_require_current_catches_mutated_native_target() -> None:
    shape = _shape()
    built = build_joint_target_series(shape, _positions(shape), OptionalTreeInput.absent(), _parameters())
    built.require_current()
    built.output.values.branches[0].items[0].item.profile = "MUTATED"
    with pytest.raises(JointTargetSeriesValueChangedError):
        built.require_current()
