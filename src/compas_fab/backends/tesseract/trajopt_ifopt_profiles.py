"""Validated forwarding to Tesseract's TrajOptIfopt default profile factory."""

from __future__ import annotations

from tesseract_robotics.planning import create_trajopt_ifopt_default_profiles
from tesseract_robotics.tesseract_command_language import ProfileDictionary

from .errors import InvalidTesseractTrajOptIfoptProfileError
from .profile_arguments import optional_profile_names


def build_trajopt_ifopt_profiles(
    profile_names: object,
) -> ProfileDictionary:
    """Validate and forward every native TrajOptIfopt default profile option.

    Args:
        profile_names: Exact native profile names, or None for the library default.

    Returns:
        The native ProfileDictionary produced by the TrajOptIfopt default profile factory.

    Raises:
        InvalidTesseractTrajOptIfoptProfileError: An argument violates its native contract.
    """
    names = optional_profile_names(profile_names, InvalidTesseractTrajOptIfoptProfileError)
    return create_trajopt_ifopt_default_profiles(profile_names=names)
