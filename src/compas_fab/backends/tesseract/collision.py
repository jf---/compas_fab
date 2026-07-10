"""Lossless native contact-query result value."""

from __future__ import annotations

from attrs import define
from tesseract_robotics.tesseract_collision import ContactRequest
from tesseract_robotics.tesseract_collision import ContactResultMap
from tesseract_robotics.tesseract_collision import ContactResultVector

CONTACT_BOUNDARY_METRES = 0.0  # Tesseract signed distance: contact is <= zero.


@define(frozen=True, slots=True)
class TesseractCollisionResult:
    """Exact native request/map plus a non-destructive flattened view."""

    request: ContactRequest
    native_map: ContactResultMap
    native_results: ContactResultVector


def is_collision_distance(distance_metres: float) -> bool:
    """Use Tesseract's exact signed-distance contact boundary."""
    return distance_metres <= CONTACT_BOUNDARY_METRES
