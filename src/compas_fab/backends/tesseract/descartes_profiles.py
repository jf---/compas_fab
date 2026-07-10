"""Validated forwarding to Tesseract's complete Descartes profile factory."""

from __future__ import annotations

import math
from collections.abc import Iterable
from typing import Optional

import numpy as np
from numpy.typing import NDArray
from tesseract_robotics.planning import create_descartes_pipeline_profiles
from tesseract_robotics.tesseract_command_language import ProfileDictionary
from tesseract_robotics.tesseract_common import EIGEN_DEFAULT_PREC
from tesseract_robotics.tesseract_motion_planners_descartes import DescartesMoveProfileD

from .errors import InvalidTesseractDescartesProfileError


def build_descartes_profiles(
    profile_names: object,
    enable_collision: object,
    enable_edge_collision: object,
    num_threads: object,
    sample_axis: object,
    sample_resolution: object,
    sample_min: object,
    sample_max: object,
    ik_solver: object,
    use_redundant_joint_solutions: object,
    move_profile: object,
) -> ProfileDictionary:
    """Validate and forward every native Descartes profile option."""
    names = _optional_profile_names(profile_names)
    collision = _optional_bool(enable_collision, "enable_collision")
    edge_collision = _optional_bool(
        enable_edge_collision,
        "enable_edge_collision",
    )
    threads = _optional_positive_int(num_threads, "num_threads")
    axis = _optional_axis(sample_axis)
    resolution = _optional_positive_float(
        sample_resolution,
        "sample_resolution",
    )
    lower = _optional_finite_float(sample_min, "sample_min")
    upper = _optional_finite_float(sample_max, "sample_max")
    if lower is not None and upper is not None and lower > upper:
        raise InvalidTesseractDescartesProfileError("sample_min must be less than or equal to sample_max.")
    solver = _optional_name(ik_solver, "ik_solver")
    redundant = _optional_bool(
        use_redundant_joint_solutions,
        "use_redundant_joint_solutions",
    )
    custom = _optional_move_profile(move_profile)
    _reject_custom_conflicts(
        custom,
        collision,
        edge_collision,
        axis,
        resolution,
        lower,
        upper,
        solver,
        redundant,
    )
    return create_descartes_pipeline_profiles(
        profile_names=names,
        enable_collision=collision,
        enable_edge_collision=edge_collision,
        num_threads=threads,
        sample_axis=axis,
        sample_resolution=resolution,
        sample_min=lower,
        sample_max=upper,
        ik_solver=solver,
        use_redundant_joint_solutions=redundant,
        move_profile=custom,
    )


def _optional_profile_names(value: object) -> Optional[list[str]]:
    if value is None:
        return None
    if isinstance(value, (str, bytes)) or not isinstance(value, Iterable):
        raise InvalidTesseractDescartesProfileError("profile_names must be a sequence of exact names or None.")
    names: list[str] = []
    for name in value:
        if not isinstance(name, str) or not name.strip():
            raise InvalidTesseractDescartesProfileError("Descartes profile names must be non-empty strings.")
        names.append(name)
    if not names:
        raise InvalidTesseractDescartesProfileError("profile_names cannot be an empty connected sequence.")
    if len(names) != len(set(names)):
        raise InvalidTesseractDescartesProfileError("Descartes profile names must be unique.")
    return names


def _optional_bool(value: object, name: str) -> Optional[bool]:
    if value is None:
        return None
    if not isinstance(value, bool):
        raise InvalidTesseractDescartesProfileError(
            "{} must be bool or None, got {}.".format(
                name,
                type(value).__name__,
            )
        )
    return value


def _optional_positive_int(value: object, name: str) -> Optional[int]:
    if value is None:
        return None
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise InvalidTesseractDescartesProfileError(
            "{} must be a positive integer or None, got {!r}.".format(
                name,
                value,
            )
        )
    return value


def _optional_axis(value: object) -> Optional[NDArray[np.float64]]:
    if value is None:
        return None
    if isinstance(value, (str, bytes)):
        raise InvalidTesseractDescartesProfileError("sample_axis must be a finite non-degenerate three-vector.")
    try:
        axis = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError) as axis_error:
        raise InvalidTesseractDescartesProfileError("sample_axis must be a finite non-degenerate three-vector.") from axis_error
    if axis.shape != (3,) or not np.isfinite(axis).all() or np.linalg.norm(axis) <= EIGEN_DEFAULT_PREC:
        raise InvalidTesseractDescartesProfileError("sample_axis must be a finite non-degenerate three-vector.")
    return axis


def _optional_positive_float(value: object, name: str) -> Optional[float]:
    result = _optional_finite_float(value, name)
    if result is not None and result <= 0.0:
        raise InvalidTesseractDescartesProfileError("{} must be positive when connected.".format(name))
    return result


def _optional_finite_float(value: object, name: str) -> Optional[float]:
    if value is None:
        return None
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise InvalidTesseractDescartesProfileError("{} must be finite or None, got {!r}.".format(name, value))
    return float(value)


def _optional_name(value: object, name: str) -> Optional[str]:
    if value is None:
        return None
    if not isinstance(value, str) or not value.strip():
        raise InvalidTesseractDescartesProfileError("{} must be an exact non-empty string or None.".format(name))
    return value


def _optional_move_profile(value: object) -> Optional[DescartesMoveProfileD]:
    if value is None:
        return None
    if not isinstance(value, DescartesMoveProfileD):
        raise InvalidTesseractDescartesProfileError("move_profile must be exact native DescartesMoveProfileD or None.")
    return value


def _reject_custom_conflicts(
    custom: Optional[DescartesMoveProfileD],
    collision: Optional[bool],
    edge_collision: Optional[bool],
    axis: Optional[NDArray[np.float64]],
    resolution: Optional[float],
    lower: Optional[float],
    upper: Optional[float],
    solver: Optional[str],
    redundant: Optional[bool],
) -> None:
    if custom is None:
        return
    values = {
        "enable_collision": collision,
        "enable_edge_collision": edge_collision,
        "sample_axis": axis,
        "sample_resolution": resolution,
        "sample_min": lower,
        "sample_max": upper,
        "ik_solver": solver,
        "use_redundant_joint_solutions": redundant,
    }
    conflicts = [name for name, value in values.items() if value is not None]
    if conflicts:
        raise InvalidTesseractDescartesProfileError("move_profile is a full override; conflicting options: {}.".format(", ".join(conflicts)))
