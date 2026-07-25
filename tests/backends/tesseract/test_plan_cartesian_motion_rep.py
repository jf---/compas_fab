"""Coordinated REP (robot-with-external-positioner) Cartesian proof.

The load-bearing proof: a FrameWaypoints path authored in the workpiece frame
(positioner_tool0), stepping ~1.2 m in the workpiece X, is followed by coordinating
the prismatic XY positioner with the six arm joints -- the positioner moves the
workpiece under the tool so the TCP tracks the workpiece-relative path. The robot
base is fixed at the world origin, so world-fixed reach is not the mechanism: only a
genuine coordinated solve tracks the moving workpiece target. Verified live: the
positioner sweeps ~1.0 m on the correct (X) axis and the TCP tracks to ~1 mm.
"""

import gc
import xml.etree.ElementTree as ElementTree
from pathlib import Path

import numpy as np
import tesseract_robotics
from compas.geometry import Frame
from compas.geometry import Transformation
from compas_robots import Configuration
from compas_robots import RobotModel
from compas_robots.model import Joint

from compas_fab.backends.tesseract.artifact import CollisionMeshPolicy
from compas_fab.backends.tesseract.artifact import CoupledKinematics
from compas_fab.backends.tesseract.artifact import CoupledTopology
from compas_fab.backends.tesseract.artifact import OpwParameters
from compas_fab.backends.tesseract.artifact_loader import ResourceRoot
from compas_fab.backends.tesseract.artifact_loader import RobotArtifactLoader
from compas_fab.backends.tesseract.client import TesseractClient
from compas_fab.backends.tesseract.planner import TesseractPlanner
from compas_fab.robots import FrameWaypoints
from compas_fab.robots import RobotCell
from compas_fab.robots import RobotCellState
from compas_fab.robots import RobotSemantics
from compas_fab.robots import TargetMode

_DATA = Path(tesseract_robotics.__file__).parent / "data" / "tesseract"
_URDF = _DATA / "support" / "urdf" / "abb_irb2400_external_positioner.urdf"
_SRDF = _DATA / "support" / "urdf" / "abb_irb2400_external_positioner.srdf"
_RESOURCE_ROOT = _DATA.parent
_FULL_GROUP = "full_manipulator"
_COUPLED_JOINTS = ["positioner_joint_1", "positioner_joint_2", "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
_COUPLED_TYPES = [Joint.PRISMATIC, Joint.PRISMATIC, Joint.REVOLUTE, Joint.REVOLUTE, Joint.REVOLUTE, Joint.REVOLUTE, Joint.REVOLUTE, Joint.REVOLUTE]
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
# The positioner (X axis, positioner_joint_2) must sweep to keep the TCP on the
# stepping workpiece path; observed ~1.0 m. Assert well above the 0.1 m sample step.
_MIN_POSITIONER_TRAVEL_M = 0.5
# tool0 tracks the workpiece-relative target; observed ~1 mm, bounded by the 0.1 m
# positioner sample step + Descartes discretisation (raw numpy L2, matching the ROP proof).
_CARTESIAN_FOLLOW_TOL_M = 5e-3


def _joint_list_srdf() -> str:
    root = ElementTree.fromstring(_SRDF.read_text())
    for group in root.findall("group"):
        if group.get("name") == _FULL_GROUP:
            for chain in list(group.findall("chain")):
                group.remove(chain)
            for joint_name in _COUPLED_JOINTS:
                ElementTree.SubElement(group, "joint", {"name": joint_name})
    return ElementTree.tostring(root, encoding="unicode")


def _rep_config() -> CoupledKinematics:
    return CoupledKinematics.build(
        group=_FULL_GROUP,
        topology=CoupledTopology.ROBOT_WITH_EXTERNAL_POSITIONER,
        positioner_base_link="positioner_base_link",
        positioner_tip_link="positioner_tool0",
        manipulator_base_link="base_link",
        manipulator_tip_link="tool0",
        manipulator=OpwParameters.build(**_IRB2400_OPW),
        manipulator_reach=2.55,
        positioner_sample_resolution=[("positioner_joint_1", 0.1), ("positioner_joint_2", 0.1)],
    )


def _rep_artifact(tmp_path):
    srdf_path = tmp_path / "rep_jointgroup.srdf"
    srdf_path.write_text(_joint_list_srdf())
    loader = RobotArtifactLoader.build(_URDF, srdf_path, [ResourceRoot.build(_RESOURCE_ROOT)])
    return loader.load(CollisionMeshPolicy.CONVEX_HULL).with_coupled_kinematics(_rep_config())


def _rep_cell() -> RobotCell:
    model = RobotModel.from_urdf_string(_URDF.read_text())
    semantics = RobotSemantics.from_srdf_string(_joint_list_srdf(), model)
    return RobotCell(model, semantics)


def _tool_down_in_workpiece(dx: float) -> Frame:
    # A tool-down pose at workpiece-X = dx, authored relative to positioner_tool0.
    return Frame([dx, 0.0, -0.1], [1.0, 0.0, 0.0], [0.0, -1.0, 0.0])


def _start_state(cell: RobotCell) -> RobotCellState:
    state = RobotCellState.from_robot_cell(cell)
    state.robot_configuration = Configuration([0.0] * 8, _COUPLED_TYPES, _COUPLED_JOINTS)
    return state


def test_coordinated_rep_plan_tracks_the_moving_workpiece(tmp_path):
    cell = _rep_cell()
    start_state = _start_state(cell)
    frames = [_tool_down_in_workpiece(-0.6), _tool_down_in_workpiece(0.6)]
    waypoints = FrameWaypoints(frames, TargetMode.ROBOT)

    with TesseractClient(_rep_artifact(tmp_path), cache_root=tmp_path) as client:
        planner = TesseractPlanner(client)
        planner.set_robot_cell(cell, start_state)
        trajectory = planner.plan_cartesian_motion(waypoints, start_state, group=_FULL_GROUP)

        # tool0 and the (moving) workpiece frame at each endpoint, measured natively.
        robot = client.environment.clone_robot()
        kinematic_group = robot.env.getKinematicGroup(_FULL_GROUP, "")
        ixx = list(kinematic_group.getJointNames()).index("positioner_joint_2")
        endpoints = []
        for point, frame in ((trajectory.points[0], frames[0]), (trajectory.points[-1], frames[-1])):
            fk = kinematic_group.calcFwdKin(np.asarray(point.joint_values, dtype=float))
            tool0_world = np.asarray(fk["tool0"].matrix, dtype=float)
            workpiece_world = np.asarray(fk["positioner_tool0"].matrix, dtype=float)
            want_world = workpiece_world @ np.asarray(Transformation.from_frame(frame).matrix, dtype=float)
            endpoints.append(float(np.linalg.norm(tool0_world[:3, 3] - want_world[:3, 3])))
        positioner_x = [point.joint_values[ixx] for point in trajectory.points]
        del kinematic_group, robot
        gc.collect()

    # A coordinated 8-DOF trajectory in positioner-forward order.
    assert trajectory.joint_names == _COUPLED_JOINTS
    assert trajectory.attributes["tesseract_pipeline"] == "DescartesFPipeline"

    # The X positioner coordinated (swept to keep the TCP on the workpiece path).
    positioner_travel = max(positioner_x) - min(positioner_x)
    assert positioner_travel > _MIN_POSITIONER_TRAVEL_M, "positioner did not coordinate: X travel {:.4f} m".format(positioner_travel)

    # The TCP tracked the moving workpiece-relative target at each endpoint.
    for error in endpoints:
        assert error < _CARTESIAN_FOLLOW_TOL_M, "tool0 left the workpiece path by {:.4f} m".format(error)
