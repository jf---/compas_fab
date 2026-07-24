"""Contract tests for arm-vs-external joint classification (W4 E1-T2).

The load-bearing contract: on the shipped `abb_irb2400_on_positioner` cell, the
coupled `full_manipulator` group splits into the six arm joints (in order) plus one
external TRACK axis -- the prismatic positioner, emitted in millimetres. A synthetic
rotary-table cell proves the revolute -> POSITIONER/degree branch that no shipped
cell exercises, and an exotic-axis cell proves an unclassifiable external joint is
rejected rather than silently mis-typed.
"""

from pathlib import Path

import pytest
import tesseract_robotics
from compas_robots import RobotModel

from compas_fab.backends.tesseract.errors import UnknownKinematicTopologyError
from compas_fab.backends.tesseract.external_axes import CoupledGroupLayout
from compas_fab.backends.tesseract.external_axes import ExternalAxis
from compas_fab.backends.tesseract.external_axes import ExternalAxisRole
from compas_fab.backends.tesseract.external_axes import ExternalAxisUnit
from compas_fab.robots import RobotCell
from compas_fab.robots import RobotSemantics

_SUPPORT_URDF = Path(tesseract_robotics.__file__).parent / "data" / "tesseract" / "support" / "urdf"

_ARM_JOINTS = ("joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6")

# A minimal synthetic cell: a revolute rotary table (`table_axis`) carrying a 2R arm.
# No shipped reference cell has a revolute external axis, so this is how the
# POSITIONER/degree classification branch gets exercised end-to-end.
_ROTARY_URDF = """<?xml version="1.0"?>
<robot name="table_arm">
  <link name="base"/><link name="table"/><link name="link_a"/><link name="link_b"/>
  <joint name="table_axis" type="revolute">
    <parent link="base"/><child link="table"/><axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/></joint>
  <joint name="arm_1" type="revolute">
    <parent link="table"/><child link="link_a"/><axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/></joint>
  <joint name="arm_2" type="revolute">
    <parent link="link_a"/><child link="link_b"/><axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/></joint>
</robot>
"""

_ROTARY_SRDF = """<?xml version="1.0"?>
<robot name="table_arm">
  <group name="arm"><chain base_link="table" tip_link="link_b"/></group>
  <group name="coupled"><chain base_link="base" tip_link="link_b"/></group>
</robot>
"""

# Same arm, but the manipulator group lists its joints out of chain order. The
# coupled chain is [table_axis, arm_1, arm_2]; the arm group is [arm_2, arm_1].
_SCRAMBLED_SRDF = """<?xml version="1.0"?>
<robot name="table_arm">
  <group name="arm"><joint name="arm_2"/><joint name="arm_1"/></group>
  <group name="coupled"><chain base_link="base" tip_link="link_b"/></group>
</robot>
"""

# A coupled group whose external axis is a floating joint -- a configurable type
# that is neither a track nor a positioner, so it must be rejected.
_EXOTIC_URDF = """<?xml version="1.0"?>
<robot name="exotic">
  <link name="base"/><link name="table"/><link name="link_a"/>
  <joint name="exotic_axis" type="floating">
    <parent link="base"/><child link="table"/></joint>
  <joint name="arm_1" type="revolute">
    <parent link="table"/><child link="link_a"/><axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/></joint>
</robot>
"""

_EXOTIC_SRDF = """<?xml version="1.0"?>
<robot name="exotic">
  <group name="arm"><chain base_link="table" tip_link="link_a"/></group>
  <group name="coupled"><chain base_link="base" tip_link="link_a"/></group>
</robot>
"""


def _cell(urdf: str, srdf: str) -> RobotCell:
    model = RobotModel.from_urdf_string(urdf)
    semantics = RobotSemantics.from_srdf_string(srdf, model)
    return RobotCell(model, semantics)


@pytest.fixture
def positioner_cell() -> RobotCell:
    """The shipped IRB2400-on-a-prismatic-track ROP reference cell."""
    urdf = (_SUPPORT_URDF / "abb_irb2400_on_positioner.urdf").read_text()
    srdf = (_SUPPORT_URDF / "abb_irb2400_on_positioner.srdf").read_text()
    return _cell(urdf, srdf)


@pytest.fixture
def rotary_cell() -> RobotCell:
    return _cell(_ROTARY_URDF, _ROTARY_SRDF)


# --------------------------------------------------------------------------
# Classification contract
# --------------------------------------------------------------------------


def test_track_layout_splits_arm_from_prismatic_external(positioner_cell: RobotCell) -> None:
    layout = CoupledGroupLayout.build(positioner_cell, "full_manipulator", "manipulator")

    assert layout.arm_joint_names == _ARM_JOINTS
    assert layout.external_axes == (ExternalAxis("positioner_joint_1", ExternalAxisRole.TRACK, ExternalAxisUnit.MILLIMETRE, 0),)


def test_positioner_layout_classifies_revolute_external(rotary_cell: RobotCell) -> None:
    layout = CoupledGroupLayout.build(rotary_cell, "coupled", "arm")

    assert layout.arm_joint_names == ("arm_1", "arm_2")
    assert layout.external_axes == (ExternalAxis("table_axis", ExternalAxisRole.POSITIONER, ExternalAxisUnit.DEGREE, 0),)


def test_layout_has_no_external_axes_when_group_is_the_manipulator(positioner_cell: RobotCell) -> None:
    layout = CoupledGroupLayout.build(positioner_cell, "manipulator", "manipulator")

    assert layout.arm_joint_names == _ARM_JOINTS
    assert layout.external_axes == ()


# --------------------------------------------------------------------------
# Topology rejections
# --------------------------------------------------------------------------


def test_missing_manipulator_joints_rejected(positioner_cell: RobotCell) -> None:
    # The `positioner` group holds only the track joint, not the six arm joints.
    with pytest.raises(UnknownKinematicTopologyError):
        CoupledGroupLayout.build(positioner_cell, "positioner", "manipulator")


def test_scrambled_manipulator_order_rejected() -> None:
    cell = _cell(_ROTARY_URDF, _SCRAMBLED_SRDF)
    with pytest.raises(UnknownKinematicTopologyError):
        CoupledGroupLayout.build(cell, "coupled", "arm")


def test_unclassifiable_external_joint_rejected() -> None:
    cell = _cell(_EXOTIC_URDF, _EXOTIC_SRDF)
    with pytest.raises(UnknownKinematicTopologyError):
        CoupledGroupLayout.build(cell, "coupled", "arm")


# --------------------------------------------------------------------------
# Factory invariants
# --------------------------------------------------------------------------


def test_external_axis_build_derives_unit_from_role() -> None:
    assert ExternalAxis.build("rail", ExternalAxisRole.TRACK, 0) == ExternalAxis("rail", ExternalAxisRole.TRACK, ExternalAxisUnit.MILLIMETRE, 0)
    assert ExternalAxis.build("table", ExternalAxisRole.POSITIONER, 1) == ExternalAxis("table", ExternalAxisRole.POSITIONER, ExternalAxisUnit.DEGREE, 1)


def test_external_axis_build_rejects_empty_name() -> None:
    with pytest.raises(UnknownKinematicTopologyError):
        ExternalAxis.build("", ExternalAxisRole.TRACK, 0)


def test_external_axis_build_rejects_negative_index() -> None:
    with pytest.raises(UnknownKinematicTopologyError):
        ExternalAxis.build("track", ExternalAxisRole.TRACK, -1)
