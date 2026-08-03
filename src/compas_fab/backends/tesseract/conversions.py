"""Loss-checked projections between native Tesseract and COMPAS values."""

from __future__ import annotations

import math
from typing import Mapping
from typing import Optional

from compas_fab.robots import Duration
from compas_fab.robots import JointTrajectory
from compas_fab.robots import JointTrajectoryPoint

from .errors import EmptyTesseractTrajectoryError
from .errors import InconsistentTesseractJointOrderError
from .errors import MalformedTesseractTrajectoryError
from .errors import MissingTesseractJointTypeError
from .errors import MissingTesseractTrajectoryFieldError
from .errors import MissingTesseractTrajectoryNativeResultError
from .errors import NonMonotonicTesseractTrajectoryError
from .native import TesseractPlanningResult
from .native_trajectory_vector import finite_native_vector

NANOSECONDS_PER_SECOND = 1_000_000_000
_NATIVE_RESULT_ATTRIBUTE = "_tesseract_native_result"


def joint_trajectory_from_result(
    result: TesseractPlanningResult,
    joint_types: Mapping[str, int],
) -> JointTrajectory:
    """Project a complete native trajectory without hiding missing fields.

    Args:
        result: Successful native planning result.
        joint_types: COMPAS joint type keyed by every native joint name.

    Returns:
        Exact point-for-point COMPAS projection.

    Raises:
        EmptyTesseractTrajectoryError: Native planning returned no points.
        InconsistentTesseractJointOrderError: Point name order changes.
        MissingTesseractJointTypeError: A native joint has no COMPAS type.
        MissingTesseractTrajectoryFieldError: Dynamics are unavailable.
        MalformedTesseractTrajectoryError: Native field shapes are inconsistent.
        NonMonotonicTesseractTrajectoryError: Time is negative or decreases.
    """
    native_points = result.native_result.trajectory
    if not native_points:
        raise EmptyTesseractTrajectoryError("Tesseract pipeline {!r} returned no trajectory points.".format(result.request.pipeline))

    names = list(native_points[0].joint_names)
    if not names or len(names) != len(set(names)):
        raise MalformedTesseractTrajectoryError("Native trajectory joint names must be non-empty and unique.")
    missing_types = [name for name in names if name not in joint_types]
    if missing_types:
        raise MissingTesseractJointTypeError("Missing COMPAS joint types for: {}.".format(", ".join(missing_types)))
    ordered_joint_types = [joint_types[name] for name in names]

    points = []
    native_times = []
    previous_time: Optional[float] = None
    for point_index, native_point in enumerate(native_points):
        point_names = list(native_point.joint_names)
        if point_names != names:
            raise InconsistentTesseractJointOrderError("Native point {} joint order {} differs from {}.".format(point_index, point_names, names))

        velocities = _required_field(native_point.velocities, "velocities", point_index)
        accelerations = _required_field(native_point.accelerations, "accelerations", point_index)
        time = native_point.time
        if time is None:
            raise MissingTesseractTrajectoryFieldError("Native trajectory field 'time' is missing at point {}.".format(point_index))
        if not math.isfinite(time):
            raise MalformedTesseractTrajectoryError("Native trajectory time at point {} is not finite.".format(point_index))
        if time < 0.0 or (previous_time is not None and time < previous_time):
            raise NonMonotonicTesseractTrajectoryError("Native trajectory time decreases from {} to {} at point {}.".format(previous_time, time, point_index))

        positions = finite_native_vector(
            native_point.positions,
            "positions",
            point_index,
            len(names),
            MalformedTesseractTrajectoryError,
        )
        velocity_values = finite_native_vector(
            velocities,
            "velocities",
            point_index,
            len(names),
            MalformedTesseractTrajectoryError,
        )
        acceleration_values = finite_native_vector(
            accelerations,
            "accelerations",
            point_index,
            len(names),
            MalformedTesseractTrajectoryError,
        )

        points.append(
            JointTrajectoryPoint(
                joint_values=list(positions),
                joint_types=ordered_joint_types,
                velocities=list(velocity_values),
                accelerations=list(acceleration_values),
                effort=[0.0] * len(names),
                time_from_start=_duration_from_seconds(time),
                joint_names=names,
            )
        )
        native_times.append(time)
        previous_time = time

    trajectory = JointTrajectory(
        trajectory_points=points,
        joint_names=names,
        fraction=1.0,
        attributes={
            "tesseract_pipeline": result.request.pipeline,
            "tesseract_time_seconds": native_times,
            "tesseract_native_effort_available": False,
        },
    )
    # The exact native object is intentionally attached outside COMPAS data so
    # conventional serialization remains portable while the producing in-memory
    # trajectory owns a concurrency-safe reference to its exact result.
    setattr(trajectory, _NATIVE_RESULT_ATTRIBUTE, result)
    return trajectory


def native_result_from_trajectory(trajectory: JointTrajectory) -> TesseractPlanningResult:
    """Return the exact native result that produced a conventional trajectory."""
    result = getattr(trajectory, _NATIVE_RESULT_ATTRIBUTE, None)
    if not isinstance(result, TesseractPlanningResult):
        raise MissingTesseractTrajectoryNativeResultError("JointTrajectory does not retain a Tesseract native planning result.")
    return result


def _required_field(
    value: object,
    name: str,
    point_index: int,
) -> object:
    if value is None:
        raise MissingTesseractTrajectoryFieldError("Native trajectory field {!r} is missing at point {}.".format(name, point_index))
    return value


def _duration_from_seconds(seconds: float) -> Duration:
    whole_seconds = math.floor(seconds)
    nanoseconds = round((seconds - whole_seconds) * NANOSECONDS_PER_SECOND)
    if nanoseconds == NANOSECONDS_PER_SECOND:
        whole_seconds += 1
        nanoseconds = 0
    return Duration(whole_seconds, nanoseconds)
