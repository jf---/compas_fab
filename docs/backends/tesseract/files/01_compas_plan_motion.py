from pathlib import Path

import compas_fab
from compas_fab.backends.tesseract.artifact import ContinuousContactManager
from compas_fab.backends.tesseract.artifact import DiscreteContactManager
from compas_fab.backends.tesseract.artifact import KdlInverseKinematics
from compas_fab.backends.tesseract.artifact_loader import ResourceRoot
from compas_fab.backends.tesseract.artifact_loader import RobotArtifactLoader
from compas_fab.backends.tesseract.client import TesseractClient
from compas_fab.backends.tesseract.compas_artifact import CompasRobotArtifactCompiler
from compas_fab.backends.tesseract.planner import TesseractPlanner
from compas_fab.robots import ConfigurationTarget
from compas_fab.robots.robot_library import RobotCellLibrary

robot_cell, start_state = RobotCellLibrary.ur5(load_geometry=False)
start_state.robot_configuration = robot_cell.get_configuration_from_group_state("manipulator", "up")
resource_root = Path(compas_fab.get("robot_library/ur5_robot"))
loader = RobotArtifactLoader.build(
    resource_root / "urdf" / "robot_description.urdf",
    resource_root / "robot_description_semantic.srdf",
    [ResourceRoot.build(resource_root)],
)
compiler = CompasRobotArtifactCompiler.build(
    loader=loader,
    groups=["manipulator"],
    inverse_kinematics=KdlInverseKinematics.LMA,
    discrete_contact_manager=DiscreteContactManager.BULLET_BVH,
    continuous_contact_manager=ContinuousContactManager.BULLET_CAST_BVH,
)
artifact = compiler.compile(robot_cell)

goal_configuration = start_state.robot_configuration.copy()
goal_configuration["shoulder_pan_joint"] = 0.1
target = ConfigurationTarget(goal_configuration)

with TesseractClient(artifact) as client:
    planner = TesseractPlanner(client)
    planner.set_robot_cell(robot_cell, start_state)
    trajectory = planner.plan_motion(target, start_state, group="manipulator")

print(len(trajectory.points), trajectory.joint_names)
