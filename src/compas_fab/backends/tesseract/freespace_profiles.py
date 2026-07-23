"""Validated forwarding to Tesseract's Freespace pipeline profile factory."""

from __future__ import annotations

from tesseract_robotics.planning import create_freespace_pipeline_profiles
from tesseract_robotics.tesseract_command_language import ProfileDictionary

from .errors import InvalidTesseractFreespaceProfileError
from .profile_arguments import PipelineForwardKwargs
from .profile_arguments import optional_bool
from .profile_arguments import optional_non_negative_float
from .profile_arguments import optional_positive_float
from .profile_arguments import optional_positive_int
from .profile_arguments import optional_profile_names


def build_freespace_profiles(
    profile_names: object,
    num_planners: object,
    planning_time: object,
    optimize: object,
    max_solutions: object,
    planner_range: object,
) -> ProfileDictionary:
    """Validate and forward every native Freespace pipeline profile option.

    Only the arguments the caller supplies are forwarded; an omitted (None)
    argument is left off the native call so the native factory default applies.
    The native factory rejects None for its scalar controls, so preserving the
    native default means omitting the argument rather than forwarding None.

    Args:
        profile_names: Exact native profile names, or None for the library default.
        num_planners: Parallel OMPL planner count, or None for the library default.
        planning_time: OMPL planning time in seconds, or None for the library default.
        optimize: Whether to keep optimizing until timeout, or None for the library default.
        max_solutions: Early-exit solution count, or None for the library default.
        planner_range: OMPL planner extension range, or None for the library default.

    Returns:
        The native ProfileDictionary produced by the Freespace pipeline factory.

    Raises:
        InvalidTesseractFreespaceProfileError: An argument violates its native contract.
    """
    error = InvalidTesseractFreespaceProfileError
    kwargs: PipelineForwardKwargs = {}
    names = optional_profile_names(profile_names, error)
    if names is not None:
        kwargs["profile_names"] = names
    planners = optional_positive_int(num_planners, "num_planners", error)
    if planners is not None:
        kwargs["num_planners"] = planners
    planning = optional_positive_float(planning_time, "planning_time", error)
    if planning is not None:
        kwargs["planning_time"] = planning
    keep_optimizing = optional_bool(optimize, "optimize", error)
    if keep_optimizing is not None:
        kwargs["optimize"] = keep_optimizing
    solutions = optional_positive_int(max_solutions, "max_solutions", error)
    if solutions is not None:
        kwargs["max_solutions"] = solutions
    extension = optional_non_negative_float(planner_range, "planner_range", error)
    if extension is not None:
        kwargs["planner_range"] = extension
    return create_freespace_pipeline_profiles(**kwargs)
