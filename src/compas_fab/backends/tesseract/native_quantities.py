"""Validated native joint quantities with non-interchangeable static types."""

from __future__ import annotations

import math
from collections.abc import Callable
from collections.abc import Iterable
from typing import NewType
from typing import TypeVar

import numpy as np
from attrs import define

from .errors import InvalidTesseractTargetError

NativeJointPosition = NewType("NativeJointPosition", float)
NativeJointVelocity = NewType("NativeJointVelocity", float)
NativeJointAcceleration = NewType("NativeJointAcceleration", float)
NativeTimeSeconds = NewType("NativeTimeSeconds", float)

NativeQuantity = TypeVar("NativeQuantity")


@define(frozen=True, slots=True)
class NativeJointPositions:
    """Finite non-empty native joint positions in radians or metres."""

    values: tuple[NativeJointPosition, ...]

    def __attrs_post_init__(self) -> None:
        _validate_stored_vector(self.values, "positions")

    @classmethod
    def build(cls, value: object) -> NativeJointPositions:
        return cls(_finite_vector(value, "positions", NativeJointPosition))


@define(frozen=True, slots=True)
class NativeJointVelocities:
    """Finite non-empty native joint velocities in radians/s or metres/s."""

    values: tuple[NativeJointVelocity, ...]

    def __attrs_post_init__(self) -> None:
        _validate_stored_vector(self.values, "velocities")

    @classmethod
    def build(cls, value: object) -> NativeJointVelocities:
        return cls(_finite_vector(value, "velocities", NativeJointVelocity))


@define(frozen=True, slots=True)
class NativeJointAccelerations:
    """Finite non-empty native accelerations in radians/s² or metres/s²."""

    values: tuple[NativeJointAcceleration, ...]

    def __attrs_post_init__(self) -> None:
        _validate_stored_vector(self.values, "accelerations")

    @classmethod
    def build(cls, value: object) -> NativeJointAccelerations:
        return cls(_finite_vector(value, "accelerations", NativeJointAcceleration))


@define(frozen=True, slots=True)
class NativeJointNames:
    """Immutable unique joint names in exact native order."""

    values: tuple[str, ...]

    def __attrs_post_init__(self) -> None:
        if (
            not isinstance(self.values, tuple)
            or not self.values
            or any(not isinstance(name, str) or not name.strip() for name in self.values)
            or len(self.values) != len(set(self.values))
        ):
            raise InvalidTesseractTargetError("Target joint names must be non-empty unique strings.")

    @classmethod
    def build(cls, value: object, expected_size: int) -> NativeJointNames:
        names = _names(value)
        if len(names) != expected_size:
            raise InvalidTesseractTargetError(
                "Target has {} joint names for {} positions.".format(
                    len(names),
                    expected_size,
                )
            )
        return cls(names)


@define(frozen=True, slots=True)
class NativeTime:
    """Finite non-negative native trajectory time in seconds."""

    value: NativeTimeSeconds

    def __attrs_post_init__(self) -> None:
        if type(self.value) is not float or not math.isfinite(self.value) or self.value < 0.0:
            raise InvalidTesseractTargetError("Target time must be finite non-negative seconds, got {!r}.".format(self.value))

    @classmethod
    def build(cls, value: object) -> NativeTime:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise InvalidTesseractTargetError("Target time must be finite non-negative seconds, got {!r}.".format(value))
        return cls(NativeTimeSeconds(float(value)))


def _finite_vector(
    value: object,
    name: str,
    quantity: Callable[[float], NativeQuantity],
) -> tuple[NativeQuantity, ...]:
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
    return tuple(quantity(float(value)) for value in values)


def _validate_stored_vector(values: object, name: str) -> None:
    if not isinstance(values, tuple) or not values or any(type(value) is not float or not math.isfinite(value) for value in values):
        raise InvalidTesseractTargetError("Target {} must be a non-empty finite native quantity tuple.".format(name))


def _names(value: object) -> tuple[str, ...]:
    if isinstance(value, (str, bytes)):
        raise InvalidTesseractTargetError("Target joint names must be a sequence, not one string.")
    if not isinstance(value, Iterable):
        raise InvalidTesseractTargetError("Target joint names must be a sequence of exact names.")
    names: list[str] = []
    for name in value:
        if not isinstance(name, str) or not name.strip():
            raise InvalidTesseractTargetError("Target joint names must be non-empty strings.")
        names.append(name)
    if len(names) != len(set(names)):
        raise InvalidTesseractTargetError("Target joint names must be unique.")
    return tuple(names)
