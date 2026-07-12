from attrs import evolve
import pytest

from compas_fab.ghpython.group_shape import DegreesOfFreedom
from compas_fab.ghpython.group_shape import DuplicateJointIdError
from compas_fab.ghpython.group_shape import EmptyGroupShapeError
from compas_fab.ghpython.group_shape import GroupShape
from compas_fab.ghpython.group_shape import GroupShapeFactory
from compas_fab.ghpython.group_shape import InvalidGroupShapeError
from compas_fab.ghpython.group_shape import JointId
from compas_fab.ghpython.group_shape import PlanningGroupId


class NativeRobot:
    def get_joint_names(self, group: str) -> list[str]:
        assert group == "manipulator"
        return ["joint_3", "joint_1", "joint_2"]


def test_group_shape_retains_native_order_and_derived_dof(monkeypatch) -> None:
    monkeypatch.setattr("compas_fab.ghpython.group_shape.Robot", NativeRobot)
    group = PlanningGroupId.build("manipulator")
    shape = GroupShapeFactory.from_native_robot(NativeRobot(), group)

    assert shape.group_id == group
    assert shape.ordered_joint_ids == tuple(JointId.build(name) for name in ("joint_3", "joint_1", "joint_2"))
    assert shape.dof == DegreesOfFreedom.build(3)


def test_group_shape_rejects_empty_and_duplicate_joint_ids() -> None:
    group = PlanningGroupId.build("manipulator")
    with pytest.raises(EmptyGroupShapeError):
        GroupShape.build(group, ())
    with pytest.raises(DuplicateJointIdError):
        GroupShape.build(group, (JointId.build("joint_1"), JointId.build("joint_1")))


def test_raw_constructor_cannot_forge_derived_dof() -> None:
    shape = GroupShape.build(PlanningGroupId.build("manipulator"), (JointId.build("joint_1"),))
    with pytest.raises(InvalidGroupShapeError):
        evolve(shape, dof=DegreesOfFreedom.build(2))
