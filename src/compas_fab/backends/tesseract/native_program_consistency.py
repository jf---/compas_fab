"""Semantic identity for native programs whose generated UUIDs differ."""

from __future__ import annotations

from collections.abc import Iterable
from typing import Union
from typing import cast

from tesseract_robotics.tesseract_command_language import CompositeInstruction
from tesseract_robotics.tesseract_command_language import InstructionPoly
from tesseract_robotics.tesseract_command_language import WaypointPoly
from tesseract_robotics.tesseract_command_language import WaypointPoly_as_CartesianWaypointPoly
from tesseract_robotics.tesseract_command_language import WaypointPoly_as_JointWaypointPoly
from tesseract_robotics.tesseract_command_language import WaypointPoly_as_StateWaypointPoly
from tesseract_robotics.tesseract_common import Isometry3d
from tesseract_robotics.tesseract_common import ManipulatorInfo

SemanticAtom = Union[str, int, float, bool, None]
SemanticValue = Union[SemanticAtom, tuple["SemanticValue", ...]]


def native_program_semantic_identity(
    program: CompositeInstruction,
) -> tuple[SemanticValue, ...]:
    """Return every planning-relevant value except generated instruction UUIDs."""
    return (
        "composite",
        program.getProfile(),
        program.getOrder().value,
        program.getDescription(),
        _manipulator_identity(program.getManipulatorInfo()),
        tuple(_instruction_identity(instruction) for instruction in program),
    )


def _instruction_identity(
    instruction: InstructionPoly,
) -> tuple[SemanticValue, ...]:
    if not instruction.isMoveInstruction():
        return (
            "unsupported",
            instruction.getDescription(),
        )
    move = instruction.asMoveInstruction()
    waypoint = move.getWaypoint()
    return (
        "move",
        move.getMoveType().value,
        move.getProfile(),
        move.getPathProfile(),
        move.getDescription(),
        _manipulator_identity(move.getManipulatorInfo()),
        _waypoint_identity(waypoint),
    )


def _waypoint_identity(waypoint: WaypointPoly) -> tuple[SemanticValue, ...]:
    if waypoint.isCartesianWaypoint():
        cartesian = WaypointPoly_as_CartesianWaypointPoly(waypoint)
        seed: SemanticValue = None
        if cartesian.hasSeed():
            native_seed = cartesian.getSeed()
            seed = (
                tuple(native_seed.joint_names),
                _vector(native_seed.position),
                _vector(native_seed.velocity),
                _vector(native_seed.acceleration),
                _vector(native_seed.effort),
                float(native_seed.time),
            )
        return (
            "cartesian",
            cartesian.getName(),
            _matrix(cartesian.getTransform().matrix),
            seed,
        )
    if waypoint.isJointWaypoint():
        joint = WaypointPoly_as_JointWaypointPoly(waypoint)
        return (
            "joint",
            joint.getName(),
            tuple(joint.getNames()),
            _vector(joint.getPosition()),
            joint.isConstrained(),
        )
    if waypoint.isStateWaypoint():
        state = WaypointPoly_as_StateWaypointPoly(waypoint)
        return (
            "state",
            state.getName(),
            tuple(state.getNames()),
            _vector(state.getPosition()),
            _vector(state.getVelocity()),
            _vector(state.getAcceleration()),
            float(state.getTime()),
        )
    return ("null",)


def _manipulator_identity(
    info: ManipulatorInfo,
) -> tuple[SemanticValue, ...]:
    return (
        info.manipulator,
        info.manipulator_ik_solver,
        info.working_frame,
        info.tcp_frame,
        _matrix(cast(Isometry3d, info.tcp_offset).matrix),
    )


def _vector(values: Iterable[float]) -> tuple[SemanticValue, ...]:
    return tuple(float(value) for value in values)


def _matrix(values: Iterable[Iterable[float]]) -> tuple[SemanticValue, ...]:
    return tuple(float(value) for row in values for value in row)
