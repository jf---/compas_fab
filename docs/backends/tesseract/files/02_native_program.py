from math import radians
from pathlib import Path

import numpy as np
from tesseract_robotics.planning import CartesianTarget
from tesseract_robotics.planning import MotionProgram
from tesseract_robotics.planning.profiles import create_descartes_pipeline_profiles

import compas_fab
from compas_fab.backends.tesseract.artifact import ContinuousContactManager
from compas_fab.backends.tesseract.artifact import DiscreteContactManager
from compas_fab.backends.tesseract.artifact import KdlInverseKinematics
from compas_fab.backends.tesseract.artifact_loader import ResourceRoot
from compas_fab.backends.tesseract.artifact_loader import RobotArtifactLoader
from compas_fab.backends.tesseract.client import TesseractClient
from compas_fab.backends.tesseract.compas_artifact import CompasRobotArtifactCompiler
from compas_fab.backends.tesseract.native import TesseractPlanningRequest
from compas_fab.backends.tesseract.planner import TesseractPlanner
from compas_fab.robots.robot_library import RobotCellLibrary

TOOL_Z_AXIS = (0.0, 0.0, 1.0)
TOOL_AXIS_SAMPLE_STEP = radians(30.0)

robot_cell, start_state = RobotCellLibrary.ur5(load_geometry=False)
start_state.robot_configuration = robot_cell.get_configuration_from_group_state("manipulator", "up")
resource_root = Path(compas_fab.get("robot_library/ur5_robot"))
loader = RobotArtifactLoader.build(
    resource_root / "urdf" / "robot_description.urdf",
    resource_root / "robot_description_semantic.srdf",
    [ResourceRoot.build(resource_root)],
)
artifact = CompasRobotArtifactCompiler.build(
    loader=loader,
    groups=["manipulator"],
    inverse_kinematics=KdlInverseKinematics.LMA,
    discrete_contact_manager=DiscreteContactManager.BULLET_BVH,
    continuous_contact_manager=ContinuousContactManager.BULLET_CAST_BVH,
).compile(robot_cell)

joint_names = robot_cell.get_configurable_joint_names("manipulator")
start_configuration = start_state.robot_configuration
start = np.asarray(start_configuration.joint_values, dtype=np.float64)
goal = start.copy()
goal[0] = 0.1

with TesseractClient(artifact) as client:
    planner = TesseractPlanner(client)
    planner.set_robot_cell(robot_cell, start_state)
    native_robot = client.clone_robot()
    start_pose = native_robot.fk("manipulator", start, tip_link="tool0")
    goal_pose = native_robot.fk("manipulator", goal, tip_link="tool0")
    program = (
        MotionProgram(
            "manipulator",
            tcp_frame="tool0",
            working_frame="base_link",
            profile="DEFAULT",
        )
        .set_joint_names(joint_names)
        .move_to(CartesianTarget(start_pose, profile="DEFAULT"))
        .move_to(CartesianTarget(goal_pose, profile="DEFAULT"))
        .to_composite_instruction(joint_names, "tool0")
    )
    profiles = create_descartes_pipeline_profiles(
        sample_axis=TOOL_Z_AXIS,
        sample_resolution=TOOL_AXIS_SAMPLE_STEP,
        use_redundant_joint_solutions=True,
    )
    request = TesseractPlanningRequest.build(
        program=program,
        pipeline="DescartesFPipeline",
        profiles=profiles,
        auto_seed=True,
    )
    result = planner.plan_native(request)

assert result.request is request
assert result.native_result.successful
print(len(result.raw_program), result.native_result.message)
