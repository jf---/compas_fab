"""Validated forwarding to Tesseract's TrajOpt default profile factory."""

from __future__ import annotations

from tesseract_robotics.planning import create_trajopt_default_profiles
from tesseract_robotics.tesseract_command_language import ProfileDictionary

from .errors import InvalidTesseractTrajOptProfileError
from .profile_arguments import optional_profile_names


def build_trajopt_profiles(
    profile_names: object,
) -> ProfileDictionary:
    """Validate and forward every native TrajOpt default profile option.

    Args:
        profile_names: Exact native profile names, or None for the library default.

    Returns:
        The native ProfileDictionary produced by the TrajOpt default profile factory.

    Raises:
        InvalidTesseractTrajOptProfileError: An argument violates its native contract.
    """
    names = optional_profile_names(profile_names, InvalidTesseractTrajOptProfileError)
    return create_trajopt_default_profiles(profile_names=names)
