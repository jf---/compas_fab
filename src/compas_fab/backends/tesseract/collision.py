"""Lossless native contact-query result value."""

from __future__ import annotations

from typing import Optional

from attrs import define
from tesseract_robotics.tesseract_collision import ContactRequest
from tesseract_robotics.tesseract_collision import ContactResultMap
from tesseract_robotics.tesseract_collision import ContactResultVector
from tesseract_robotics.tesseract_collision import ContactTestType_ALL
from tesseract_robotics.tesseract_collision import ContactTestType_CLOSEST
from tesseract_robotics.tesseract_collision import ContactTestType_FIRST
from tesseract_robotics.tesseract_collision import ContactTestType_LIMITED

from .errors import TesseractContactQueryError

CONTACT_BOUNDARY_METRES = 0.0  # Tesseract signed distance: contact is <= zero.


@define(frozen=True, slots=True)
class TesseractCollisionResult:
    """Exact native request/map plus a non-destructive flattened view."""

    request: ContactRequest
    native_map: ContactResultMap
    native_results: ContactResultVector


def build_contact_request(
    test_type: Optional[str],
    calculate_distance: Optional[bool],
    calculate_penetration: Optional[bool],
    contact_limit: Optional[int],
) -> ContactRequest:
    """Build an exact native contact request without replacing native defaults."""
    test_types = {
        "FIRST": ContactTestType_FIRST,
        "CLOSEST": ContactTestType_CLOSEST,
        "ALL": ContactTestType_ALL,
        "LIMITED": ContactTestType_LIMITED,
    }
    if test_type is not None and (not isinstance(test_type, str) or test_type not in test_types):
        raise TesseractContactQueryError("test_type must be FIRST, CLOSEST, ALL, LIMITED, or None.")
    if calculate_distance is not None and type(calculate_distance) is not bool:
        raise TesseractContactQueryError("calculate_distance must be an exact bool or None.")
    if calculate_penetration is not None and type(calculate_penetration) is not bool:
        raise TesseractContactQueryError("calculate_penetration must be an exact bool or None.")
    if contact_limit is not None and (type(contact_limit) is not int or contact_limit < 0):
        raise TesseractContactQueryError("contact_limit must be a non-negative exact integer or None.")

    request = ContactRequest() if test_type is None else ContactRequest(test_types[test_type])
    if calculate_distance is not None:
        request.calculate_distance = calculate_distance
    if calculate_penetration is not None:
        request.calculate_penetration = calculate_penetration
    if contact_limit is not None:
        request.contact_limit = contact_limit
    return request


def is_collision_distance(distance_metres: float) -> bool:
    """Use Tesseract's exact signed-distance contact boundary."""
    return distance_metres <= CONTACT_BOUNDARY_METRES
