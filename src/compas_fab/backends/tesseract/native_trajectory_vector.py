"""Shared finite-vector validation for native trajectory boundaries."""

from __future__ import annotations

import numpy as np

from .errors import TesseractBackendError


def finite_native_vector(
    value: object,
    name: str,
    point_index: int,
    expected_size: int,
    error_type: type[TesseractBackendError],
) -> tuple[float, ...]:
    """Return one finite, non-Boolean native trajectory vector."""
    try:
        raw = np.asarray(value, dtype=object)
        array = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError) as vector_error:
        raise error_type(
            "Native point {} {} is not a numeric vector.".format(
                point_index,
                name,
            )
        ) from vector_error
    if raw.ndim != 1 or any(isinstance(item, (bool, np.bool_)) for item in raw) or array.ndim != 1 or array.size != expected_size or not np.isfinite(array).all():
        raise error_type(
            "Native point {} {} must contain {} finite values.".format(
                point_index,
                name,
                expected_size,
            )
        )
    return tuple(float(item) for item in array)
