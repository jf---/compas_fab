"""Validated construction of exact native Tesseract motion targets."""

from __future__ import annotations

import math
from collections.abc import Iterable
from typing import NewType
from typing import Optional

import numpy as np
from numpy.typing import NDArray
from tesseract_robotics.planning import CartesianTarget
from tesseract_robotics.planning import JointTarget
from tesseract_robotics.planning import MoveType
from tesseract_robotics.planning import Pose
from tesseract_robotics.planning import StateTarget

from .errors import InvalidTesseractTargetError

NativeJointPosition = NewType("NativeJointPosition", float)
NativeJointVelocity = NewType("NativeJointVelocity", float)
NativeJointAcceleration = NewType("NativeJointAcceleration", float)
NativeTimeSeconds = NewType("NativeTimeSeconds", float)


def move_type_from_name(value: object) -> MoveType:
    """Resolve one UI name to the exact native move enum."""
    if not isinstance(value, str) or not value.strip():
        raise InvalidTesseractTargetError("Native move type name must be a non-empty string.")
    try:
        return MoveType[value.strip().upper()]
    except KeyError as move_type_error:
        raise InvalidTesseractTargetError("Unknown native move type {!r}.".format(value)) from move_type_error


def build_cartesian_target(
    pose: object,
    move_type: object,
    profile: object,
) -> CartesianTarget:
    """Build a Cartesian target from one exact native pose."""
    if not isinstance(pose, Pose):
        raise InvalidTesseractTargetError("Cartesian target requires exact native Pose, got {}.".format(type(pose).__name__))
    return CartesianTarget(
        pose=pose,
        move_type=_move_type(move_type),
        profile=_profile(profile),
    )


def build_joint_target(
    positions: object,
    names: object,
    move_type: object,
    profile: object,
) -> JointTarget:
    """Build a joint target in native joint units and supplied name order."""
    position_values = _finite_vector(positions, "positions")
    joint_names = _joint_names(names, position_values.size)
    return JointTarget(
        position_values,
        names=joint_names,
        move_type=_move_type(move_type),
        profile=_profile(profile),
    )


def build_state_target(
    positions: object,
    names: object,
    velocities: object,
    accelerations: object,
    time: object,
    move_type: object,
    profile: object,
) -> StateTarget:
    """Build a native state target without manufacturing absent dynamics."""
    position_values = _finite_vector(positions, "positions")
    joint_names = _joint_names(names, position_values.size)
    velocity_values = _optional_vector(
        velocities,
        "velocities",
        position_values.size,
    )
    acceleration_values = _optional_vector(
        accelerations,
        "accelerations",
        position_values.size,
    )
    seconds = _optional_non_negative_time(time)
    return StateTarget(
        position_values,
        names=joint_names,
        velocities=velocity_values,
        accelerations=acceleration_values,
        time=seconds,
        move_type=_move_type(move_type),
        profile=_profile(profile),
    )


def _move_type(value: object) -> MoveType:
    if not isinstance(value, MoveType):
        raise InvalidTesseractTargetError("Target move_type must be native MoveType, got {}.".format(type(value).__name__))
    return value


def _profile(value: object) -> str:
    if not isinstance(value, str) or not value.strip():
        raise InvalidTesseractTargetError("Target profile must be an exact non-empty string.")
    return value


def _finite_vector(value: object, name: str) -> NDArray[np.float64]:
    if isinstance(value, (str, bytes)):
        raise InvalidTesseractTargetError("Target {} must be a finite one-dimensional numeric sequence.".format(name))
    try:
        raw = np.asarray(value, dtype=object)
        values = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError) as vector_error:
        raise InvalidTesseractTargetError("Target {} must be a finite one-dimensional numeric sequence.".format(name)) from vector_error
    if raw.ndim != 1 or any(isinstance(item, (bool, np.bool_)) for item in raw):
        raise InvalidTesseractTargetError("Target {} must be a finite one-dimensional numeric sequence.".format(name))
    if values.ndim != 1 or values.size == 0 or not np.isfinite(values).all():
        raise InvalidTesseractTargetError("Target {} must be a non-empty finite one-dimensional vector.".format(name))
    return values


def _optional_vector(
    value: object,
    name: str,
    expected_size: int,
) -> Optional[NDArray[np.float64]]:
    if value is None:
        return None
    values = _finite_vector(value, name)
    if values.size != expected_size:
        raise InvalidTesseractTargetError(
            "Target {} length {} does not match {} positions.".format(
                name,
                values.size,
                expected_size,
            )
        )
    return values


def _joint_names(value: object, expected_size: int) -> Optional[list[str]]:
    if value is None:
        return None
    if isinstance(value, (str, bytes)):
        raise InvalidTesseractTargetError("Target joint names must be a sequence, not one string.")
    if not isinstance(value, Iterable):
        raise InvalidTesseractTargetError("Target joint names must be a sequence of exact names.")
    names: list[str] = []
    for name in value:
        if not isinstance(name, str) or not name.strip():
            raise InvalidTesseractTargetError("Target joint names must be non-empty strings.")
        names.append(name)
    if len(names) != expected_size:
        raise InvalidTesseractTargetError(
            "Target has {} joint names for {} positions.".format(
                len(names),
                expected_size,
            )
        )
    if len(names) != len(set(names)):
        raise InvalidTesseractTargetError("Target joint names must be unique.")
    return names


def _optional_non_negative_time(value: object) -> Optional[float]:
    if value is None:
        return None
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0.0:
        raise InvalidTesseractTargetError("Target time must be finite non-negative seconds or None, got {!r}.".format(value))
    return float(NativeTimeSeconds(float(value)))
