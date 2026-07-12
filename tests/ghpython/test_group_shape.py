from attrs import evolve
import pytest

from compas_fab.ghpython.group_shape import DegreesOfFreedom
from compas_fab.ghpython.group_shape import DuplicateJointIdError
from compas_fab.ghpython.group_shape import EmptyGroupShapeError
from compas_fab.ghpython.group_shape import GroupShape
from compas_fab.ghpython.group_shape import GroupShapeFactory
from compas_fab.ghpython.group_shape import GroupFixedVector
from compas_fab.ghpython.group_shape import GroupVectorQuantity
from compas_fab.ghpython.group_shape import InvalidGroupFixedVectorError
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


def test_group_fixed_vector_is_atomic_and_quantity_specific() -> None:
    shape = GroupShape.build(
        PlanningGroupId.build("manipulator"),
        (JointId.build("joint_1"), JointId.build("joint_2")),
    )
    positions = GroupFixedVector.positions(shape, (1.0, 2.0))
    velocities = GroupFixedVector.velocities(shape, (1.0, 2.0))

    assert positions.group_shape is shape
    assert positions.quantity is GroupVectorQuantity.POSITION
    assert positions.vector.values == (1.0, 2.0)
    assert positions.vector.shape != velocities.vector.shape


def test_group_fixed_vector_rejects_wrong_length_and_name_order() -> None:
    shape = GroupShape.build(
        PlanningGroupId.build("manipulator"),
        (JointId.build("joint_1"), JointId.build("joint_2")),
    )
    with pytest.raises(InvalidGroupFixedVectorError):
        GroupFixedVector.positions(shape, (1.0,))
    with pytest.raises(InvalidGroupFixedVectorError):
        GroupFixedVector.names(shape, ("joint_2", "joint_1"))
