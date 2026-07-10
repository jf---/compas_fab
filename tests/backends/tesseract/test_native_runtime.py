import numpy as np
from tesseract_robotics.planning import MotionProgram
from tesseract_robotics.planning import Robot
from tesseract_robotics.planning import StateTarget
from tesseract_robotics.planning import TaskComposer
from tesseract_robotics.planning.profiles import create_freespace_pipeline_profiles
from tesseract_robotics.tesseract_serialization import composite_instruction_to_binary

from compas_fab.backends.tesseract.native import TesseractPlanningRequest
from compas_fab.backends.tesseract.runtime import TesseractRuntime


def test_runtime_executes_real_pipeline_without_mutating_native_program():
    robot = Robot.from_tesseract_support("lbr_iiwa_14_r820")
    joint_names = robot.get_joint_names("manipulator")
    start = np.array([-0.4, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0])
    goal = np.array([-0.2, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0])
    robot.set_joints(start, joint_names=joint_names)
    program = (
        MotionProgram("manipulator", tcp_frame="tool0", profile="FREESPACE")
        .set_joint_names(joint_names)
        .move_to(StateTarget(start, names=joint_names, profile="FREESPACE"))
        .move_to(StateTarget(goal, names=joint_names, profile="FREESPACE"))
        .to_composite_instruction(joint_names, "tool0")
    )
    program_before = composite_instruction_to_binary(program)
    request = TesseractPlanningRequest.build(
        program=program,
        pipeline="FreespacePipeline",
        profiles=create_freespace_pipeline_profiles(),
        auto_seed=True,
    )
    runtime = TesseractRuntime.build(TaskComposer.from_config())

    result = runtime.execute(robot, request)

    assert result.request is request
    assert result.native_result.successful
    assert result.native_result.raw_results is result.raw_program
    assert result.raw_program is not program
    assert len(result.raw_program) > 0
    assert composite_instruction_to_binary(program) == program_before
