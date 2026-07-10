"""Explicit COMPAS user-unit frames converted to native metre poses."""

from __future__ import annotations

import math

import numpy as np
from compas.geometry import Frame  # type: ignore[import-untyped]
from compas.geometry import Transformation
from tesseract_robotics.planning import Pose

from .errors import InvalidTesseractPoseError
from .frames import MetersPerUserUnit


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
    if not isinstance(frame, Frame):
        raise InvalidTesseractPoseError("Tesseract pose requires compas.geometry.Frame, got {}.".format(type(frame).__name__))
    scale = _explicit_scale(metres_per_user_unit)
    matrix = np.asarray(
        Transformation.from_frame(frame).matrix,
        dtype=np.float64,
    )
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise InvalidTesseractPoseError("Tesseract pose frame must contain finite coordinates and axes.")
    return Pose.from_matrix_position(
        matrix[:3, :3],
        matrix[:3, 3] * scale,
    )


def _explicit_scale(value: object) -> MetersPerUserUnit:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0.0:
        raise InvalidTesseractPoseError("metres_per_user_unit must be an explicit finite positive value, got {!r}.".format(value))
    return MetersPerUserUnit(float(value))
