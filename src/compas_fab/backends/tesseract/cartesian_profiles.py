"""Validated forwarding to Tesseract's Cartesian pipeline profile factory."""

from __future__ import annotations

from tesseract_robotics.planning import create_cartesian_pipeline_profiles
from tesseract_robotics.tesseract_command_language import ProfileDictionary

from .errors import InvalidTesseractCartesianProfileError
from .profile_arguments import optional_positive_int
from .profile_arguments import optional_profile_names


def build_cartesian_profiles(
    profile_names: object,
    num_threads: object,
) -> ProfileDictionary:
    """Validate and forward every native Cartesian pipeline profile option.

    Args:
        profile_names: Exact native profile names, or None for the library default.
        num_threads: Descartes solver thread count, or None for the library default.

    Returns:
        The native ProfileDictionary produced by the Cartesian pipeline factory.

    Raises:
        InvalidTesseractCartesianProfileError: An argument violates its native contract.
    """
    names = optional_profile_names(profile_names, InvalidTesseractCartesianProfileError)
    threads = optional_positive_int(num_threads, "num_threads", InvalidTesseractCartesianProfileError)
    return create_cartesian_pipeline_profiles(
        profile_names=names,
        num_threads=threads,
    )
