"""Validation and lossless merging of native RAPID profile maps."""

from collections.abc import Iterable
from collections.abc import Mapping

from tesseract_robotics.emitters.rapid import RapidProfile

from .errors import DuplicateRapidProfileError
from .errors import InvalidRapidProfileMapError


def normalize_rapid_profiles(profiles: Mapping[str, RapidProfile]) -> dict[str, RapidProfile]:
    """Validate a native RAPID profile map without replacing its values.

    Args:
        profiles: Exact Tesseract profile names mapped to native profiles.

    Returns:
        A concrete dictionary retaining every supplied profile object.

    Raises:
        InvalidRapidProfileMapError: The mapping boundary is malformed.
    """
    if not isinstance(profiles, Mapping):
        raise InvalidRapidProfileMapError("RAPID profiles must be a mapping.")

    normalized: dict[str, RapidProfile] = {}
    for name, profile in profiles.items():
        if not isinstance(name, str) or not name.strip():
            raise InvalidRapidProfileMapError("RAPID profile names must be non-empty strings.")
        if not isinstance(profile, RapidProfile):
            raise InvalidRapidProfileMapError(
                "RAPID profile {!r} must be RapidProfile, got {}.".format(
                    name,
                    type(profile).__name__,
                )
            )
        normalized[name] = profile
    return normalized


def merge_rapid_profile_maps(
    profile_maps: Iterable[Mapping[str, RapidProfile]],
) -> dict[str, RapidProfile]:
    """Merge native profile maps while rejecting ambiguous names.

    Args:
        profile_maps: Profile maps in no precedence order.

    Returns:
        One validated native profile dictionary.

    Raises:
        DuplicateRapidProfileError: More than one map defines the same name.
        InvalidRapidProfileMapError: A supplied map is malformed.
    """
    merged: dict[str, RapidProfile] = {}
    for profiles in profile_maps:
        for name, profile in normalize_rapid_profiles(profiles).items():
            if name in merged:
                raise DuplicateRapidProfileError("Duplicate Tesseract RAPID profile {!r}.".format(name))
            merged[name] = profile
    return merged
