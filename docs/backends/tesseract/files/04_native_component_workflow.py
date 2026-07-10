from math import pi
from math import radians
from typing import NewType

import numpy as np
from tesseract_robotics.planning import MoveType
from tesseract_robotics.planning import Robot
from tesseract_robotics.planning import TaskComposer

from compas_fab.backends.tesseract.descartes_profiles import build_descartes_profiles
from compas_fab.backends.tesseract.frames import robot_frame_from_isometry
from compas_fab.backends.tesseract.native import TesseractPlanningRequest
from compas_fab.backends.tesseract.native_pose import WorkingFrameUserUnits
from compas_fab.backends.tesseract.native_pose import pose_from_working_frame
from compas_fab.backends.tesseract.native_program_builder import build_motion_program
from compas_fab.backends.tesseract.native_result_view import TesseractNativeResultView
from compas_fab.backends.tesseract.native_targets import cartesian_target_from_native
from compas_fab.backends.tesseract.runtime import TesseractRuntime

Radians = NewType("Radians", float)

MANIPULATOR = "manipulator"
TCP_FRAME = "tool0"
WORKING_FRAME = "base_link"
PROFILE_NAME = "DEFAULT"
TOOL_Z_AXIS = (0.0, 0.0, 1.0)
TOOL_AXIS_SAMPLE_STEP = Radians(radians(1.0))
TOOL_AXIS_SAMPLE_MIN = Radians(-pi)
TOOL_AXIS_SAMPLE_MAX = Radians(pi)

robot = Robot.from_tesseract_support("abb_irb2400")
joint_names = robot.get_joint_names(MANIPULATOR)
start_joints = np.zeros(len(joint_names), dtype=np.float64)
goal_joints = start_joints.copy()
goal_joints[0] = 0.1
robot.set_joints(start_joints, joint_names=joint_names)

# FK guarantees both Cartesian goals are reachable. Pose is an Isometry3d, so
# the exact native result can cross the same explicit COMPAS/unit boundary used
# by Tesseract Pose without approximating its transform.
start_frame = robot_frame_from_isometry(robot.fk(MANIPULATOR, start_joints, tip_link=TCP_FRAME)).value
goal_frame = robot_frame_from_isometry(robot.fk(MANIPULATOR, goal_joints, tip_link=TCP_FRAME)).value
start_pose = pose_from_working_frame(WorkingFrameUserUnits.build(start_frame, 1.0))
goal_pose = pose_from_working_frame(WorkingFrameUserUnits.build(goal_frame, 1.0))

targets = [
    cartesian_target_from_native(
        start_pose,
        MoveType.LINEAR,
        PROFILE_NAME,
    ),
    cartesian_target_from_native(
        goal_pose,
        MoveType.LINEAR,
        PROFILE_NAME,
    ),
]
built = build_motion_program(
    robot,
    targets,
    MANIPULATOR,
    TCP_FRAME,
    WORKING_FRAME,
    PROFILE_NAME,
)
profiles = build_descartes_profiles(
    profile_names=[PROFILE_NAME],
    enable_collision=True,
    enable_edge_collision=False,
    num_threads=None,
    sample_axis=TOOL_Z_AXIS,
    sample_resolution=TOOL_AXIS_SAMPLE_STEP,
    sample_min=TOOL_AXIS_SAMPLE_MIN,
    sample_max=TOOL_AXIS_SAMPLE_MAX,
    ik_solver=None,
    use_redundant_joint_solutions=True,
    move_profile=None,
)
request = TesseractPlanningRequest.build(
    program=built.composite_instruction,
    pipeline="DescartesFPipeline",
    profiles=profiles,
    auto_seed=True,
)
result = TesseractRuntime.build(TaskComposer.from_config()).execute(
    robot,
    request,
)
view = TesseractNativeResultView.build(result)

assert view.trajectory_points
print(view.message)
print(len(view.trajectory_points))
print(view.joint_names)
print(view.velocities is not None, view.accelerations is not None, view.times is not None)
