"""Typed frame and unit transformations at the Tesseract boundary."""

from __future__ import annotations

import math
from typing import Generic
from typing import NewType
from typing import Optional
from typing import TypeVar

import numpy as np
from attrs import define
from compas.geometry import Frame  # type: ignore[import-untyped]
from compas.geometry import Transformation
from tesseract_robotics.planning import Pose
from tesseract_robotics.tesseract_common import Isometry3d

from .errors import InvalidTesseractFrameError
from .errors import InvalidTesseractNativeScaleError

MetersPerUserUnit = NewType("MetersPerUserUnit", float)


class RobotCoordinates:
    """Type tag for coordinates relative to the robot base."""


class WorldCoordinates:
    """Type tag for coordinates relative to the COMPAS world frame."""


class Meters:
    """Type tag for native Tesseract metre coordinates."""


FrameTag = TypeVar("FrameTag")
UnitTag = TypeVar("UnitTag")


@define(frozen=True, slots=True)
class TaggedFrame(Generic[FrameTag, UnitTag]):
    """A COMPAS frame carrying coordinate-frame and unit tags."""

    value: Frame

    def __attrs_post_init__(self) -> None:
        if not isinstance(self.value, Frame):
            raise InvalidTesseractFrameError("Tagged Tesseract frame must contain compas.geometry.Frame, got {}.".format(type(self.value).__name__))


RobotFrameMeters = TaggedFrame[RobotCoordinates, Meters]
WorldFrameMeters = TaggedFrame[WorldCoordinates, Meters]


def meters_per_user_unit(value: Optional[float]) -> MetersPerUserUnit:
    """Validate the factor mapping one user coordinate unit to native metres."""
    scale: object = 1.0 if value is None else value
    if isinstance(scale, bool) or not isinstance(scale, (int, float)) or not math.isfinite(scale) or scale <= 0.0:
        raise InvalidTesseractNativeScaleError("native_scale must be a finite positive metres-per-user-unit value, got {!r}.".format(scale))
    return MetersPerUserUnit(float(scale))


def robot_frame_from_isometry(pose: Isometry3d) -> RobotFrameMeters:
    """Project an exact native robot-relative metre pose to a COMPAS frame."""
    transformation = Transformation(pose.matrix.tolist())
    return TaggedFrame(Frame.from_transformation(transformation))


def isometry_from_robot_frame(frame: RobotFrameMeters) -> Pose:
    """Convert a robot-relative metre frame to a native Tesseract pose."""
    matrix = np.asarray(Transformation.from_frame(frame.value).matrix, dtype=np.float64)
    return Pose.from_matrix_position(matrix[:3, :3], matrix[:3, 3])


def world_frame_from_robot(
    frame: RobotFrameMeters,
    robot_base_frame: WorldFrameMeters,
) -> WorldFrameMeters:
    """Transform a robot-relative metre frame into the COMPAS world frame."""
    transformation = Transformation.from_frame(robot_base_frame.value) * Transformation.from_frame(frame.value)
    return TaggedFrame(Frame.from_transformation(transformation))


def world_meters_frame(frame: Frame) -> WorldFrameMeters:
    """Tag a validated COMPAS world frame whose coordinates are metres."""
    return TaggedFrame(frame)


def robot_frame_from_world(
    frame: WorldFrameMeters,
    robot_base_frame: WorldFrameMeters,
) -> RobotFrameMeters:
    """Transform a COMPAS world metre frame into the robot base frame."""
    transformation = Transformation.from_frame(robot_base_frame.value).inverse() * Transformation.from_frame(frame.value)
    return TaggedFrame(Frame.from_transformation(transformation))


def frame_in_user_units(
    frame: WorldFrameMeters,
    scale: MetersPerUserUnit,
) -> Frame:
    """Return a copy whose coordinates use the requested user length unit."""
    projected = frame.value.copy()
    projected.scale(1.0 / scale)
    return projected
