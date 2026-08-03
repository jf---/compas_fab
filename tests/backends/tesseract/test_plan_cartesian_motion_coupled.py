"""Coordinated Cartesian planning over the reference ROP cell.

The load-bearing proof: planning a `FrameWaypoints` path whose two poses are 1.8 m
apart in Y — beyond the arm's reach envelope from any single positioner position
(the arm spans ~0.8 m in Y at this pose) — must succeed by *coordinating* the
prismatic positioner with the six arm joints. A decoupled solve (frozen
positioner) cannot reach both poses, so a plan that both succeeds and follows the
tool path is only possible if the positioner column actually travels. Anything
less would leave the external axis as a fabricated constant, which is exactly what
world-class coordinated support must not do.
"""

import gc
from pathlib import Path

import numpy as np
import pytest
import tesseract_robotics
from compas.geometry import Frame
from compas_robots import Configuration
from compas_robots import RobotModel
from compas_robots.model import Joint

from compas_fab.backends.interfaces.planner_operation import PlannerOperation
from compas_fab.backends.tesseract.artifact import CollisionMeshPolicy
from compas_fab.backends.tesseract.artifact import CoupledKinematics
from compas_fab.backends.tesseract.artifact import CoupledTopology
from compas_fab.backends.tesseract.artifact import OpwParameters
from compas_fab.backends.tesseract.artifact import RobotArtifact
from compas_fab.backends.tesseract.artifact_loader import ResourceRoot
from compas_fab.backends.tesseract.artifact_loader import RobotArtifactLoader
from compas_fab.backends.tesseract.client import TesseractClient
from compas_fab.backends.tesseract.errors import UnsupportedTesseractTargetError
from compas_fab.backends.tesseract.errors import UnsupportedTesseractToleranceError
from compas_fab.backends.tesseract.planner import TesseractPlanner
from compas_fab.robots import FrameWaypoints
from compas_fab.robots import RobotCell
from compas_fab.robots import RobotCellState
from compas_fab.robots import RobotSemantics
from compas_fab.robots import TargetMode

# The shipped reference cell: an IRB2400 riding a prismatic positioner (a track
# carrying the robot -> ROP). Its full_manipulator group spans positioner + arm.
_DATA = Path(tesseract_robotics.__file__).parent / "data" / "tesseract"
_URDF = _DATA / "support" / "urdf" / "abb_irb2400_on_positioner.urdf"
_SRDF = _DATA / "support" / "urdf" / "abb_irb2400_on_positioner.srdf"
_RESOURCE_ROOT = _DATA.parent  # the referenced 'tesseract' package is its child
_FULL_GROUP = "full_manipulator"
_EXPECTED_JOINTS = ["positioner_joint_1", "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
_JOINT_TYPES = [Joint.PRISMATIC, Joint.REVOLUTE, Joint.REVOLUTE, Joint.REVOLUTE, Joint.REVOLUTE, Joint.REVOLUTE, Joint.REVOLUTE]

# IRB2400 OPW constants, from tesseract's online_planning_example_plugins.yaml.
_IRB2400_OPW = dict(
    a1=0.100,
    a2=-0.135,
    b=0.00,
    c1=0.615,
    c2=0.705,
    c3=0.755,
    c4=0.086,
    offsets=[0.0, 0.0, -1.57079632679, 0.0, 0.0, 0.0],
    sign_corrections=[1, 1, 1, 1, 1, 1],
)

# Two tool-down poses 1.8 m apart in Y (the positioner's prismatic axis, limits
# +/-1.0 m). Beyond the arm's Y reach from a fixed base -> forces coordination.
_WAYPOINT_Y = 0.9
_WAYPOINT_X = 0.9
_WAYPOINT_Z = 1.0
# The positioner must cover the span the arm cannot; empirically it sweeps ~1.0 m.
# Assert well above the 0.1 m positioner sample step so the motion is unambiguous.
_MIN_POSITIONER_TRAVEL_M = 0.5
# tool0 position match at each waypoint, bounded by the 0.1 m positioner sample
# step and the Descartes ladder-graph discretisation (observed error ~1 mm).
_CARTESIAN_FOLLOW_TOL_M = 5e-3


def _reference_rop_config() -> CoupledKinematics:
    return CoupledKinematics.build(
        group=_FULL_GROUP,
        topology=CoupledTopology.ROBOT_ON_POSITIONER,
        positioner_base_link="positioner_base_link",
        positioner_tip_link="base_link",
        manipulator_base_link="base_link",
        manipulator_tip_link="tool0",
        manipulator=OpwParameters.build(**_IRB2400_OPW),
        manipulator_reach=2.55,
        positioner_sample_resolution=[("positioner_joint_1", 0.1)],
    )


def _reference_artifact() -> RobotArtifact:
    """The full reference cell (meshes resolved) plus the coordinated ROP solver."""
    loader = RobotArtifactLoader.build(_URDF, _SRDF, [ResourceRoot.build(_RESOURCE_ROOT)])
    return loader.load(CollisionMeshPolicy.CONVEX_HULL).with_coupled_kinematics(_reference_rop_config())


def _coupled_group_cell() -> RobotCell:
    """A COMPAS cell exposing only the coordinated full_manipulator group.

    The reference SRDF also declares arm/positioner sub-groups whose SRDF-chain
    base link is not the native kinematic root (the arm base rides the positioner
    through a fixed joint), which structural validation rejects. The coordinated
    planner needs only full_manipulator, whose base link IS the root.
    """
    import xml.etree.ElementTree as ElementTree

    srdf_root = ElementTree.fromstring(_SRDF.read_text())
    for group_element in list(srdf_root.findall("group")):
        if group_element.get("name") != _FULL_GROUP:
            srdf_root.remove(group_element)
    srdf_full_only = ElementTree.tostring(srdf_root, encoding="unicode")

    model = RobotModel.from_urdf_string(_URDF.read_text())
    semantics = RobotSemantics.from_srdf_string(srdf_full_only, model)
    return RobotCell(model, semantics)


def _tool_down(y: float) -> Frame:
    # zaxis = xaxis x yaxis = [1,0,0] x [0,-1,0] = [0,0,-1] -> tool points down.
    return Frame([_WAYPOINT_X, y, _WAYPOINT_Z], [1.0, 0.0, 0.0], [0.0, -1.0, 0.0])


def _start_state(robot_cell: RobotCell) -> RobotCellState:
    state = RobotCellState.from_robot_cell(robot_cell)
    state.robot_configuration = Configuration([0.0] * 7, _JOINT_TYPES, _EXPECTED_JOINTS)
    return state


def test_plan_cartesian_motion_capability_is_exposed():
    assert PlannerOperation.PLAN_CARTESIAN_MOTION in TesseractPlanner.capabilities.operations


def test_coordinated_cartesian_plan_moves_the_positioner_column(tmp_path):
    robot_cell = _coupled_group_cell()
    start_state = _start_state(robot_cell)
    waypoints = FrameWaypoints([_tool_down(-_WAYPOINT_Y), _tool_down(_WAYPOINT_Y)], TargetMode.ROBOT)

    with TesseractClient(_reference_artifact(), cache_root=tmp_path) as client:
        planner = TesseractPlanner(client)
        planner.set_robot_cell(robot_cell, start_state)

        trajectory = planner.plan_cartesian_motion(waypoints, start_state, group=_FULL_GROUP)

        # tool0 pose at each waypoint endpoint, measured on the native robot.
        robot = client.environment.clone_robot()
        kinematic_group = robot.env.getKinematicGroup(_FULL_GROUP, "")
        endpoint_tool0 = [
            kinematic_group.calcFwdKin(np.asarray(point.joint_values, dtype=float))["tool0"].matrix[:3, 3] for point in (trajectory.points[0], trajectory.points[-1])
        ]
        del kinematic_group, robot
        gc.collect()

    # A coordinated >6-DOF trajectory spanning the positioner joint + six arm joints.
    assert trajectory.joint_names == _EXPECTED_JOINTS
    assert trajectory.attributes["tesseract_pipeline"] == "DescartesFPipeline"

    positioner_column = [point.joint_values[0] for point in trajectory.points]
    positioner_travel = max(positioner_column) - min(positioner_column)
    assert positioner_travel > _MIN_POSITIONER_TRAVEL_M, "positioner did not coordinate: travel {:.4f} m".format(positioner_travel)

    for tool0_position, waypoint in zip(endpoint_tool0, (_tool_down(-_WAYPOINT_Y), _tool_down(_WAYPOINT_Y))):
        want = np.asarray(waypoint.point, dtype=float)
        error = float(np.linalg.norm(tool0_position - want))
        assert error < _CARTESIAN_FOLLOW_TOL_M, "tool0 left the Cartesian path by {:.4f} m".format(error)


def test_plan_cartesian_motion_rejects_non_frame_waypoints(tmp_path):
    robot_cell = _coupled_group_cell()
    start_state = _start_state(robot_cell)

    with TesseractClient(_reference_artifact(), cache_root=tmp_path) as client:
        planner = TesseractPlanner(client)
        planner.set_robot_cell(robot_cell, start_state)

        with pytest.raises(UnsupportedTesseractTargetError):
            planner.plan_cartesian_motion("not-waypoints", start_state, group=_FULL_GROUP)


def test_plan_cartesian_motion_rejects_waypoint_tolerances(tmp_path):
    robot_cell = _coupled_group_cell()
    start_state = _start_state(robot_cell)
    waypoints = FrameWaypoints([_tool_down(0.0)], TargetMode.ROBOT, tolerance_position=0.001)

    with TesseractClient(_reference_artifact(), cache_root=tmp_path) as client:
        planner = TesseractPlanner(client)
        planner.set_robot_cell(robot_cell, start_state)

        with pytest.raises(UnsupportedTesseractToleranceError):
            planner.plan_cartesian_motion(waypoints, start_state, group=_FULL_GROUP)
