"""Validated native MotionProgram and CompositeInstruction construction."""

from __future__ import annotations

from collections.abc import Iterable
from typing import Union

from attrs import define
from tesseract_robotics.planning import CartesianTarget
from tesseract_robotics.planning import JointTarget
from tesseract_robotics.planning import MotionProgram
from tesseract_robotics.planning import Robot
from tesseract_robotics.planning import StateTarget
from tesseract_robotics.tesseract_command_language import CompositeInstruction

from .errors import InvalidTesseractMotionProgramError
from .native_program_consistency import native_program_semantic_identity
from .native_targets import WorkingFrameCartesianTarget

NativeTarget = Union[CartesianTarget, JointTarget, StateTarget]
_NATIVE_TARGET_TYPES = (CartesianTarget, JointTarget, StateTarget)


@define(frozen=True, slots=True)
class NativeProgramBuild:
    """Exact native program representations and resolved group authority."""

    motion_program: MotionProgram
    composite_instruction: CompositeInstruction
    joint_names: tuple[str, ...]
    tcp_frame: str

    def __attrs_post_init__(self) -> None:
        _validate_built_program(
            self.motion_program,
            self.composite_instruction,
            self.joint_names,
            self.tcp_frame,
        )


def build_motion_program(
    robot: object,
    targets: object,
    group_name: object,
    tcp_frame: object,
    working_frame: object,
    profile: object,
) -> NativeProgramBuild:
    """Build exact native program forms without mutating robot or targets."""
    native_robot = _robot(robot)
    group = _name(group_name, "group")
    working = _name(working_frame, "working frame")
    program_profile = _name(profile, "program profile")
    requested_tcp = None if tcp_frame is None else _name(tcp_frame, "TCP frame")

    try:
        joint_names = tuple(native_robot.get_joint_names(group))
        manipulator = native_robot.get_manipulator_info(
            group,
            requested_tcp,
            working,
        )
    except (AttributeError, RuntimeError, ValueError) as group_error:
        raise InvalidTesseractMotionProgramError("Cannot resolve native group {!r}: {}.".format(group, group_error)) from group_error
    _validate_joint_names(joint_names, "native group {!r}".format(group))

    resolved_tcp = _name(manipulator.tcp_frame, "resolved TCP frame")
    link_names = set(native_robot.get_link_names())
    if working not in link_names:
        raise InvalidTesseractMotionProgramError("Native working frame {!r} is not a robot link.".format(working))
    if resolved_tcp not in link_names:
        raise InvalidTesseractMotionProgramError("Native TCP frame {!r} is not a robot link.".format(resolved_tcp))

    native_targets = _targets(targets, joint_names, working)
    motion_program = MotionProgram(
        group,
        tcp_frame=resolved_tcp,
        working_frame=working,
        profile=program_profile,
    ).set_joint_names(list(joint_names))
    for target in native_targets:
        motion_program.add_target(target)
    try:
        composite = motion_program.to_composite_instruction(
            list(joint_names),
            resolved_tcp,
        )
    except (RuntimeError, TypeError, ValueError) as program_error:
        raise InvalidTesseractMotionProgramError("Native MotionProgram construction failed: {}.".format(program_error)) from program_error
    return NativeProgramBuild(
        motion_program,
        composite,
        joint_names,
        resolved_tcp,
    )


def _robot(value: object) -> Robot:
    if not isinstance(value, Robot):
        raise InvalidTesseractMotionProgramError("Motion program requires exact native Robot, got {}.".format(type(value).__name__))
    return value


def _name(value: object, kind: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise InvalidTesseractMotionProgramError("Motion program {} must be an exact non-empty string.".format(kind))
    return value


def _targets(
    value: object,
    joint_names: tuple[str, ...],
    working_frame: str,
) -> tuple[NativeTarget, ...]:
    if isinstance(value, (str, bytes)) or not isinstance(value, Iterable):
        raise InvalidTesseractMotionProgramError("Motion program targets must be an ordered native sequence.")
    targets: list[NativeTarget] = []
    for index, target in enumerate(value):
        if not isinstance(target, _NATIVE_TARGET_TYPES):
            raise InvalidTesseractMotionProgramError("Motion program target {} must be CartesianTarget, JointTarget, or StateTarget; got {}.".format(index, type(target).__name__))
        if isinstance(target, (JointTarget, StateTarget)) and target.names is not None:
            if tuple(target.names) != joint_names:
                raise InvalidTesseractMotionProgramError(
                    "Motion program target {} joint order {} differs from native group order {}.".format(index, tuple(target.names), joint_names)
                )
        _validate_target_working_frame(target, index, working_frame)
        targets.append(target)
    if not targets:
        raise InvalidTesseractMotionProgramError("Motion program requires at least one native target.")
    return tuple(targets)


def _validate_joint_names(names: tuple[str, ...], context: str) -> None:
    if not names or any(not name for name in names) or len(names) != len(set(names)):
        raise InvalidTesseractMotionProgramError("{} must expose non-empty unique joint names.".format(context))


def _validate_built_program(
    motion_program: object,
    composite_instruction: object,
    joint_names: object,
    tcp_frame: object,
) -> None:
    if not isinstance(motion_program, MotionProgram):
        raise InvalidTesseractMotionProgramError("NativeProgramBuild.motion_program must be MotionProgram.")
    if not isinstance(composite_instruction, CompositeInstruction):
        raise InvalidTesseractMotionProgramError("NativeProgramBuild.composite_instruction must be CompositeInstruction.")
    if not isinstance(joint_names, tuple) or any(not isinstance(name, str) for name in joint_names):
        raise InvalidTesseractMotionProgramError("NativeProgramBuild.joint_names must be a tuple of strings.")
    _validate_joint_names(joint_names, "NativeProgramBuild")
    resolved_tcp = _name(tcp_frame, "resolved TCP frame")
    if not motion_program.targets or len(motion_program) != len(composite_instruction):
        raise InvalidTesseractMotionProgramError("NativeProgramBuild program representations disagree in length.")
    if motion_program.tcp_frame != resolved_tcp:
        raise InvalidTesseractMotionProgramError("NativeProgramBuild TCP does not match its MotionProgram.")
    for index, target in enumerate(motion_program.targets):
        _validate_target_working_frame(
            target,
            index,
            motion_program.working_frame,
        )
    if composite_instruction.getManipulatorInfo().tcp_frame != resolved_tcp:
        raise InvalidTesseractMotionProgramError("NativeProgramBuild TCP does not match its CompositeInstruction.")
    if tuple(motion_program._joint_names or ()) != joint_names:
        raise InvalidTesseractMotionProgramError("NativeProgramBuild joint names do not match its MotionProgram.")
    try:
        regenerated = motion_program.to_composite_instruction(
            list(joint_names),
            resolved_tcp,
        )
    except (RuntimeError, TypeError, ValueError) as program_error:
        raise InvalidTesseractMotionProgramError("NativeProgramBuild cannot regenerate its MotionProgram: {}.".format(program_error)) from program_error
    if native_program_semantic_identity(regenerated) != native_program_semantic_identity(composite_instruction):
        raise InvalidTesseractMotionProgramError("NativeProgramBuild program representations disagree in content.")


def _validate_target_working_frame(
    target: NativeTarget,
    index: int,
    working_frame: str,
) -> None:
    if isinstance(target, WorkingFrameCartesianTarget) and target.working_frame != working_frame:
        raise InvalidTesseractMotionProgramError(
            "Motion program target {} working frame {!r} differs from program working frame {!r}.".format(
                index,
                target.working_frame,
                working_frame,
            )
        )
