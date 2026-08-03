"""Shared validation and forward-kwarg types for the pipeline profile factories.

Every thin profile-factory wrapper (Cartesian, Freespace, OMPL, TrajOpt, ...)
validates the same small set of optional native arguments the same way the
Descartes wrapper does, then forwards only the arguments the caller supplied so
that omitted controls fall through to the native factory default. These helpers
carry that logic once, parameterized by the caller's named exception, so each
factory module stays a thin marshal without re-declaring identical validators.
"""

from __future__ import annotations

import math
from collections.abc import Iterable
from typing import Optional
from typing import TypedDict

from .errors import TesseractBackendError


def optional_profile_names(value: object, error: type[TesseractBackendError]) -> Optional[list[str]]:
    """Validate an optional sequence of unique, non-empty profile names."""
    if value is None:
        return None
    if isinstance(value, (str, bytes)) or not isinstance(value, Iterable):
        raise error("profile_names must be a sequence of exact names or None.")
    names: list[str] = []
    for name in value:
        if not isinstance(name, str) or not name.strip():
            raise error("Profile names must be non-empty strings.")
        names.append(name)
    if not names:
        raise error("profile_names cannot be an empty connected sequence.")
    if len(names) != len(set(names)):
        raise error("Profile names must be unique.")
    return names


def optional_bool(value: object, name: str, error: type[TesseractBackendError]) -> Optional[bool]:
    """Validate an optional strict boolean control."""
    if value is None:
        return None
    if not isinstance(value, bool):
        raise error("{} must be bool or None, got {}.".format(name, type(value).__name__))
    return value


def optional_positive_int(value: object, name: str, error: type[TesseractBackendError]) -> Optional[int]:
    """Validate an optional strictly positive integer control."""
    if value is None:
        return None
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise error("{} must be a positive integer or None, got {!r}.".format(name, value))
    return value


def optional_positive_float(value: object, name: str, error: type[TesseractBackendError]) -> Optional[float]:
    """Validate an optional strictly positive finite float control."""
    result = _optional_finite_float(value, name, error)
    if result is not None and result <= 0.0:
        raise error("{} must be positive when connected.".format(name))
    return result


def optional_non_negative_float(value: object, name: str, error: type[TesseractBackendError]) -> Optional[float]:
    """Validate an optional non-negative finite float control."""
    result = _optional_finite_float(value, name, error)
    if result is not None and result < 0.0:
        raise error("{} must be non-negative when connected.".format(name))
    return result


def _optional_finite_float(value: object, name: str, error: type[TesseractBackendError]) -> Optional[float]:
    if value is None:
        return None
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise error("{} must be finite or None, got {!r}.".format(name, value))
    return float(value)


class PipelineForwardKwargs(TypedDict, total=False):
    """Typed native arguments forwarded to the OMPL-plus-optimizer pipelines.

    Every key is optional: an argument absent from the mapping falls through to
    the native factory default, which is how omitted (``None``) controls are
    preserved without this layer duplicating the native default values.
    """

    profile_names: list[str]
    num_planners: int
    planning_time: float
    optimize: bool
    max_solutions: int
    planner_range: float


class OmplForwardKwargs(PipelineForwardKwargs, total=False):
    """Typed native arguments forwarded to the standalone OMPL profile factory."""

    simplify: bool
