import numpy as np
import pytest
from tesseract_robotics.planning import CartesianTarget
from tesseract_robotics.planning import JointTarget
from tesseract_robotics.planning import MotionProgram
from tesseract_robotics.planning import MoveType
from tesseract_robotics.planning import Pose
from tesseract_robotics.planning import StateTarget
from tesseract_robotics.tesseract_command_language import CompositeInstruction

from compas_fab.backends.tesseract.errors import InvalidTesseractMotionProgramError
from compas_fab.backends.tesseract.native_program_builder import NativeProgramBuild
from compas_fab.backends.tesseract.native_program_builder import build_motion_program


def test_builder_resolves_group_joint_order_and_tcp(tesseract_robot):
    target = JointTarget(
        [0.1],
        move_type=MoveType.LINEAR,
        profile="CUSTOM",
    )

    built = build_motion_program(
        tesseract_robot,
        [target],
        "manipulator",
        None,
        "base",
        "PROGRAM",
    )

    assert isinstance(built.motion_program, MotionProgram)
    assert isinstance(built.composite_instruction, CompositeInstruction)
    assert built.motion_program.targets[0] is target
    assert target.move_type is MoveType.LINEAR
    assert built.joint_names == ("joint1",)
    assert built.tcp_frame == "tip"
    assert built.composite_instruction.getProfile() == "PROGRAM"


def test_builder_does_not_mutate_robot_or_targets(tesseract_robot):
    target = JointTarget([0.1], names=None, profile="DEFAULT")
    before = tesseract_robot.get_state(["joint1"]).joint_positions.copy()

    build_motion_program(
        tesseract_robot,
        [target],
        "manipulator",
        None,
        "base",
        "DEFAULT",
    )

    assert target.names is None
    np.testing.assert_array_equal(
        tesseract_robot.get_state(["joint1"]).joint_positions,
        before,
    )


def test_builder_preserves_order_and_all_native_target_types(
    tesseract_robot,
):
    first = CartesianTarget(
        Pose.from_xyz(0.0, 0.0, 1.0),
        move_type=MoveType.FREESPACE,
    )
    second = JointTarget([0.0], move_type=MoveType.LINEAR)
    third = StateTarget([0.1], move_type=MoveType.CIRCULAR)

    built = build_motion_program(
        tesseract_robot,
        [first, second, third],
        "manipulator",
        "tip",
        "base",
        "DEFAULT",
    )

    assert built.motion_program.targets == [first, second, third]
    assert len(built.composite_instruction) == 3
    assert first.move_type is MoveType.FREESPACE
    assert second.move_type is MoveType.LINEAR
    assert third.move_type is MoveType.CIRCULAR


@pytest.mark.parametrize(
    ("targets", "group", "tcp", "working", "profile"),
    [
        ([], "manipulator", None, "base", "DEFAULT"),
        ([object()], "manipulator", None, "base", "DEFAULT"),
        ([JointTarget([0.0])], "", None, "base", "DEFAULT"),
        ([JointTarget([0.0])], "manipulator", "missing", "base", "DEFAULT"),
        ([JointTarget([0.0])], "manipulator", None, "missing", "DEFAULT"),
        ([JointTarget([0.0])], "manipulator", None, "base", ""),
    ],
)
def test_builder_rejects_invalid_program_inputs(
    tesseract_robot,
    targets,
    group,
    tcp,
    working,
    profile,
):
    with pytest.raises(InvalidTesseractMotionProgramError):
        build_motion_program(
            tesseract_robot,
            targets,
            group,
            tcp,
            working,
            profile,
        )


def test_builder_rejects_explicit_names_outside_selected_group(
    tesseract_robot,
):
    target = JointTarget([0.0], names=["other"])

    with pytest.raises(
        InvalidTesseractMotionProgramError,
        match="joint order",
    ):
        build_motion_program(
            tesseract_robot,
            [target],
            "manipulator",
            None,
            "base",
            "DEFAULT",
        )


def test_builder_rejects_unknown_group(tesseract_robot):
    with pytest.raises(
        InvalidTesseractMotionProgramError,
        match="unknown",
    ):
        build_motion_program(
            tesseract_robot,
            [JointTarget([0.0])],
            "unknown",
            None,
            "base",
            "DEFAULT",
        )


def test_builder_rejects_non_native_robot():
    with pytest.raises(InvalidTesseractMotionProgramError):
        build_motion_program(
            object(),
            [JointTarget([0.0])],
            "manipulator",
            None,
            "base",
            "DEFAULT",
        )


def test_raw_build_result_cannot_bypass_invariants():
    with pytest.raises(InvalidTesseractMotionProgramError):
        NativeProgramBuild(
            object(),
            object(),
            (),
            "",
        )
