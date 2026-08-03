"""Lossless inspection views over exact native Tesseract planning results."""

from __future__ import annotations

import math
from collections.abc import Iterable
from typing import Optional
from typing import TypeVar

from attrs import define
from tesseract_robotics.planning import TrajectoryPoint
from tesseract_robotics.planning.composer import PlanningResult
from tesseract_robotics.tesseract_command_language import CompositeInstruction

from .errors import MalformedTesseractNativeResultError
from .native import TesseractPlanningRequest
from .native import TesseractPlanningResult
from .native_quantities import NativeJointAcceleration
from .native_quantities import NativeJointPosition
from .native_quantities import NativeJointVelocity
from .native_quantities import NativeTimeSeconds
from .native_trajectory_vector import finite_native_vector

OptionalRowValue = TypeVar("OptionalRowValue")


@define(frozen=True, slots=True)
class _ValidatedTrajectory:
    points: tuple[TrajectoryPoint, ...]
    joint_names: tuple[str, ...]
    positions: tuple[tuple[NativeJointPosition, ...], ...]
    velocities: Optional[tuple[tuple[NativeJointVelocity, ...], ...]]
    accelerations: Optional[tuple[tuple[NativeJointAcceleration, ...], ...]]
    times: Optional[tuple[NativeTimeSeconds, ...]]


@define(frozen=True, slots=True)
class TesseractNativeResultView:
    """Exact native planning objects plus validated immutable array views."""

    result: TesseractPlanningResult
    request: TesseractPlanningRequest
    native_result: PlanningResult
    raw_program: CompositeInstruction
    message: str
    trajectory_points: tuple[TrajectoryPoint, ...]
    joint_names: tuple[str, ...]
    positions: tuple[tuple[NativeJointPosition, ...], ...]
    velocities: Optional[tuple[tuple[NativeJointVelocity, ...], ...]]
    accelerations: Optional[tuple[tuple[NativeJointAcceleration, ...], ...]]
    times: Optional[tuple[NativeTimeSeconds, ...]]

    def __attrs_post_init__(self) -> None:
        exact = _exact_result(self.result)
        validated = _validated_trajectory(exact)
        if self.request is not exact.request:
            raise MalformedTesseractNativeResultError("Native result view request is not the exact producing request.")
        if self.native_result is not exact.native_result:
            raise MalformedTesseractNativeResultError("Native result view does not retain the exact PlanningResult.")
        if self.raw_program is not exact.raw_program:
            raise MalformedTesseractNativeResultError("Native result view does not retain the exact raw program.")
        if self.message != exact.native_result.message:
            raise MalformedTesseractNativeResultError("Native result view message differs from PlanningResult.")
        if len(self.trajectory_points) != len(validated.points) or any(
            actual is not expected
            for actual, expected in zip(
                self.trajectory_points,
                validated.points,
            )
        ):
            raise MalformedTesseractNativeResultError("Native result view trajectory points are not exact references.")
        if (
            self.joint_names != validated.joint_names
            or self.positions != validated.positions
            or self.velocities != validated.velocities
            or self.accelerations != validated.accelerations
            or self.times != validated.times
        ):
            raise MalformedTesseractNativeResultError("Native result view fields differ from its PlanningResult.")

    @classmethod
    def build(cls, result: object) -> TesseractNativeResultView:
        """Validate one exact result without projecting it to COMPAS."""
        exact = _exact_result(result)
        validated = _validated_trajectory(exact)
        return cls(
            exact,
            exact.request,
            exact.native_result,
            exact.raw_program,
            exact.native_result.message,
            validated.points,
            validated.joint_names,
            validated.positions,
            validated.velocities,
            validated.accelerations,
            validated.times,
        )


def _exact_result(value: object) -> TesseractPlanningResult:
    if not isinstance(value, TesseractPlanningResult):
        raise MalformedTesseractNativeResultError("Native result view requires TesseractPlanningResult, got {}.".format(type(value).__name__))
    if not isinstance(value.request, TesseractPlanningRequest):
        raise MalformedTesseractNativeResultError("Native result view request must be TesseractPlanningRequest.")
    if not isinstance(value.native_result, PlanningResult):
        raise MalformedTesseractNativeResultError("Native result view must retain exact PlanningResult.")
    if not isinstance(value.raw_program, CompositeInstruction):
        raise MalformedTesseractNativeResultError("Native result view raw program must be CompositeInstruction.")
    if value.native_result.raw_results is not value.raw_program:
        raise MalformedTesseractNativeResultError("PlanningResult raw output differs from retained raw program.")
    if not isinstance(value.native_result.message, str):
        raise MalformedTesseractNativeResultError("PlanningResult message must be text.")
    return value


def _validated_trajectory(
    result: TesseractPlanningResult,
) -> _ValidatedTrajectory:
    native_points = result.native_result.trajectory
    if not isinstance(native_points, list):
        raise MalformedTesseractNativeResultError("PlanningResult trajectory must be a native point list.")
    points: list[TrajectoryPoint] = []
    for index, point in enumerate(native_points):
        if not isinstance(point, TrajectoryPoint):
            raise MalformedTesseractNativeResultError(
                "Native trajectory point {} must be TrajectoryPoint, got {}.".format(
                    index,
                    type(point).__name__,
                )
            )
        points.append(point)
    if not points:
        return _ValidatedTrajectory((), (), (), None, None, None)

    joint_names = _joint_names(points[0].joint_names)
    position_rows: list[tuple[NativeJointPosition, ...]] = []
    velocity_rows: list[Optional[tuple[NativeJointVelocity, ...]]] = []
    acceleration_rows: list[Optional[tuple[NativeJointAcceleration, ...]]] = []
    time_values: list[Optional[NativeTimeSeconds]] = []
    previous_time: Optional[float] = None
    for index, point in enumerate(points):
        point_names = _joint_names(point.joint_names)
        if point_names != joint_names:
            raise MalformedTesseractNativeResultError(
                "Native trajectory point {} joint order {} differs from {}.".format(
                    index,
                    point_names,
                    joint_names,
                )
            )
        position_rows.append(
            tuple(
                NativeJointPosition(value)
                for value in finite_native_vector(
                    point.positions,
                    "positions",
                    index,
                    len(joint_names),
                    MalformedTesseractNativeResultError,
                )
            )
        )
        velocity_rows.append(_optional_velocity_row(point.velocities, index, len(joint_names)))
        acceleration_rows.append(
            _optional_acceleration_row(
                point.accelerations,
                index,
                len(joint_names),
            )
        )
        current_time = _optional_time(point.time, index)
        if current_time is not None:
            if current_time < 0.0 or (previous_time is not None and current_time < previous_time):
                raise MalformedTesseractNativeResultError("Native trajectory time is negative or decreases at point {}.".format(index))
            previous_time = current_time
        time_values.append(current_time)

    return _ValidatedTrajectory(
        tuple(points),
        joint_names,
        tuple(position_rows),
        _complete_optional_rows(velocity_rows, "velocities"),
        _complete_optional_rows(acceleration_rows, "accelerations"),
        _complete_optional_scalars(time_values, "time"),
    )


def _joint_names(value: object) -> tuple[str, ...]:
    if isinstance(value, (str, bytes)) or not isinstance(value, Iterable):
        raise MalformedTesseractNativeResultError("Native trajectory joint names must be a sequence.")
    names: list[str] = []
    for name in value:
        if not isinstance(name, str) or not name:
            raise MalformedTesseractNativeResultError("Native trajectory joint names must be non-empty strings.")
        names.append(name)
    if not names or len(names) != len(set(names)):
        raise MalformedTesseractNativeResultError("Native trajectory joint names must be non-empty and unique.")
    return tuple(names)


def _optional_velocity_row(
    value: object,
    point_index: int,
    expected_size: int,
) -> Optional[tuple[NativeJointVelocity, ...]]:
    if value is None:
        return None
    return tuple(
        NativeJointVelocity(item)
        for item in finite_native_vector(
            value,
            "velocities",
            point_index,
            expected_size,
            MalformedTesseractNativeResultError,
        )
    )


def _optional_acceleration_row(
    value: object,
    point_index: int,
    expected_size: int,
) -> Optional[tuple[NativeJointAcceleration, ...]]:
    if value is None:
        return None
    return tuple(
        NativeJointAcceleration(item)
        for item in finite_native_vector(
            value,
            "accelerations",
            point_index,
            expected_size,
            MalformedTesseractNativeResultError,
        )
    )


def _optional_time(
    value: object,
    point_index: int,
) -> Optional[NativeTimeSeconds]:
    if value is None:
        return None
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise MalformedTesseractNativeResultError("Native trajectory time at point {} must be finite or absent.".format(point_index))
    return NativeTimeSeconds(float(value))


def _complete_optional_rows(
    rows: list[Optional[tuple[OptionalRowValue, ...]]],
    name: str,
) -> Optional[tuple[tuple[OptionalRowValue, ...], ...]]:
    if all(row is None for row in rows):
        return None
    if any(row is None for row in rows):
        raise MalformedTesseractNativeResultError("Native trajectory {} must be present at every point or absent.".format(name))
    return tuple(row for row in rows if row is not None)


def _complete_optional_scalars(
    values: list[Optional[NativeTimeSeconds]],
    name: str,
) -> Optional[tuple[NativeTimeSeconds, ...]]:
    if all(value is None for value in values):
        return None
    if any(value is None for value in values):
        raise MalformedTesseractNativeResultError("Native trajectory {} must be present at every point or absent.".format(name))
    return tuple(value for value in values if value is not None)
