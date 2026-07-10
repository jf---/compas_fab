"""Explicit COMPAS user-unit frames converted to native metre poses."""

from __future__ import annotations

import math

import numpy as np
from attrs import define
from compas.geometry import Frame  # type: ignore[import-untyped]
from compas.geometry import Transformation
from tesseract_robotics.planning import Pose

from .errors import InvalidTesseractPoseError
from .frames import MetersPerUserUnit


@define(frozen=True, slots=True)
class WorkingFrameUserUnits:
    """A COMPAS frame relative to the program working frame plus unit scale."""

    frame: Frame
    metres_per_user_unit: MetersPerUserUnit

    def __attrs_post_init__(self) -> None:
        _frame_matrix(self.frame)
        if type(self.metres_per_user_unit) is not float:
            raise InvalidTesseractPoseError("WorkingFrameUserUnits scale must be MetersPerUserUnit.")
        _explicit_scale(self.metres_per_user_unit)

    @classmethod
    def build(
        cls,
        frame: object,
        metres_per_user_unit: object,
    ) -> WorkingFrameUserUnits:
        if not isinstance(frame, Frame):
            raise InvalidTesseractPoseError("Tesseract pose requires compas.geometry.Frame, got {}.".format(type(frame).__name__))
        return cls(frame.copy(), _explicit_scale(metres_per_user_unit))


def pose_from_user_frame(
    frame: Frame,
    metres_per_user_unit: object,
) -> Pose:
    """Convert one explicit user-unit frame to a native metre pose.

    Args:
        frame: COMPAS frame relative to the consuming program's working frame.
        metres_per_user_unit: Finite positive scale from one user unit to metres.

    Returns:
        Exact native pose whose translation is expressed in metres.

    Raises:
        InvalidTesseractPoseError: The frame or explicit scale is invalid.
    """
    return pose_from_working_frame(WorkingFrameUserUnits.build(frame, metres_per_user_unit))


def pose_from_working_frame(value: WorkingFrameUserUnits) -> Pose:
    """Convert one typed working-frame/user-unit boundary to native metres."""
    if not isinstance(value, WorkingFrameUserUnits):
        raise InvalidTesseractPoseError("Typed Tesseract pose requires WorkingFrameUserUnits, got {}.".format(type(value).__name__))
    matrix = _frame_matrix(value.frame)
    return Pose.from_matrix_position(
        matrix[:3, :3],
        matrix[:3, 3] * value.metres_per_user_unit,
    )


def _explicit_scale(value: object) -> MetersPerUserUnit:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0.0:
        raise InvalidTesseractPoseError("metres_per_user_unit must be an explicit finite positive value, got {!r}.".format(value))
    return MetersPerUserUnit(float(value))


def _frame_matrix(frame: object) -> np.ndarray:
    if not isinstance(frame, Frame):
        raise InvalidTesseractPoseError("Tesseract pose requires compas.geometry.Frame, got {}.".format(type(frame).__name__))
    matrix = np.asarray(
        Transformation.from_frame(frame).matrix,
        dtype=np.float64,
    )
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise InvalidTesseractPoseError("Tesseract pose frame must contain finite coordinates and axes.")
    return matrix
