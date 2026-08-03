from math import pi
from math import radians
from typing import NewType

import numpy as np
from tesseract_robotics.emitters.rapid import RapidProfile
from tesseract_robotics.planning import CartesianTarget
from tesseract_robotics.planning import MotionProgram
from tesseract_robotics.planning import Pose
from tesseract_robotics.planning import Robot
from tesseract_robotics.planning import TaskComposer
from tesseract_robotics.planning import create_descartes_pipeline_profiles

from compas_fab.backends.tesseract.rapid_emitter import TesseractRapidEmitter

Metres = NewType("Metres", float)
Radians = NewType("Radians", float)

MANIPULATOR = "manipulator"
TCP_FRAME = "tool0"
PROFILE_NAME = "DEFAULT"
TOOL_Z_AXIS = (0.0, 0.0, 1.0)
TOOL_AXIS_SAMPLE_STEP = Radians(radians(1.0))
TOOL_AXIS_SAMPLE_MIN = Radians(-pi)
TOOL_AXIS_SAMPLE_MAX = Radians(pi)
TOOL_DOWN_QUAT_XYZW = (0.0, -1.0, 0.0, 0.0)
START_POSITION_M = (Metres(0.8), Metres(-0.2), Metres(0.8))
GOAL_POSITION_M = (Metres(0.8), Metres(0.2), Metres(0.8))

robot = Robot.from_tesseract_support("abb_irb2400")
joint_names = robot.get_joint_names(MANIPULATOR)
robot.set_joints(np.zeros(len(joint_names)), joint_names=joint_names)

program = (
    MotionProgram(
        MANIPULATOR,
        tcp_frame=TCP_FRAME,
        profile=PROFILE_NAME,
    )
    .set_joint_names(joint_names)
    .linear_to(
        CartesianTarget(
            Pose.from_xyz_quat(START_POSITION_M, TOOL_DOWN_QUAT_XYZW),
            profile=PROFILE_NAME,
        )
    )
    .linear_to(
        CartesianTarget(
            Pose.from_xyz_quat(GOAL_POSITION_M, TOOL_DOWN_QUAT_XYZW),
            profile=PROFILE_NAME,
        )
    )
)

# Planning proves that Tesseract can exploit the process symmetry. Emission is
# intentionally independent: RAPID consumes the authored Cartesian program,
# not the dense StateWaypoint program produced for offline result inspection.
planning_profiles = create_descartes_pipeline_profiles(
    sample_axis=TOOL_Z_AXIS,
    sample_resolution=TOOL_AXIS_SAMPLE_STEP,
    sample_min=TOOL_AXIS_SAMPLE_MIN,
    sample_max=TOOL_AXIS_SAMPLE_MAX,
    use_redundant_joint_solutions=True,
)
result = TaskComposer.from_config().plan(
    robot,
    program,
    pipeline="DescartesFPipeline",
    profiles=planning_profiles,
)
assert result.successful, result.message
assert len(result) > 0

authored_program = program.to_composite_instruction(joint_names, TCP_FRAME)
rapid_program = TesseractRapidEmitter.emit(
    authored_program,
    {PROFILE_NAME: RapidProfile()},
    module_name="AxisSymmetricPath",
    procedure_name="main",
)

print(rapid_program.identity.digest)
print(rapid_program.source)
