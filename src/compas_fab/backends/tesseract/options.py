"""Typed translation of existing COMPAS planning options."""

from __future__ import annotations

from typing import Mapping
from typing import Optional

from attrs import define
from tesseract_robotics.planning.profiles import create_freespace_pipeline_profiles
from tesseract_robotics.tesseract_command_language import ProfileDictionary

from compas_fab.backends.interfaces.planner_errors import ConflictingPlannerOptionsError
from compas_fab.backends.interfaces.planner_options import PlanMotionLegacyOptions
from compas_fab.backends.interfaces.planner_options import ResolvedPlannerOptions

from .errors import InvalidTesseractPipelineError
from .errors import InvalidTesseractProfilesError
from .errors import MissingTesseractProfilesError
from .errors import UnknownTesseractOptionError

DEFAULT_PIPELINE = "FreespacePipeline"
KNOWN_OPTION_NAMES = frozenset(("pipeline", "profiles", "auto_seed"))


@define(frozen=True, slots=True)
class TesseractPlanOptions:
    """Validated native controls translated from an existing COMPAS call."""

    pipeline: str
    profiles: ProfileDictionary
    auto_seed: bool

    @classmethod
    def resolve(
        cls,
        legacy: PlanMotionLegacyOptions,
        native: Optional[Mapping[str, object]],
    ) -> ResolvedPlannerOptions:
        if legacy.connected:
            raise ConflictingPlannerOptionsError("Tesseract has no legacy Plan Motion option ports.")
        options = cls.build(native)
        return ResolvedPlannerOptions.unverifiable(
            {
                "pipeline": options.pipeline,
                "profiles": options.profiles,
                "auto_seed": options.auto_seed,
            }
        )

    @classmethod
    def build(cls, options: Optional[Mapping[str, object]]) -> TesseractPlanOptions:
        """Validate the narrow dictionary boundary used by existing interfaces.

        Args:
            options: Existing COMPAS backend option dictionary.

        Returns:
            Typed native planning controls.

        Raises:
            UnknownTesseractOptionError: An option name is unknown.
            InvalidTesseractPipelineError: Pipeline is not a non-empty string.
            MissingTesseractProfilesError: A custom pipeline lacks profiles.
            InvalidTesseractProfilesError: Profiles have the wrong native type.
        """
        values = dict(options or {})
        unknown = sorted(set(values) - KNOWN_OPTION_NAMES)
        if unknown:
            raise UnknownTesseractOptionError("Unknown Tesseract planning options: {}.".format(", ".join(unknown)))

        pipeline_value = values.get("pipeline", DEFAULT_PIPELINE)
        if not isinstance(pipeline_value, str) or not pipeline_value.strip():
            raise InvalidTesseractPipelineError("Tesseract pipeline must be a non-empty string.")

        profiles_value = values.get("profiles")
        if profiles_value is None:
            if pipeline_value != DEFAULT_PIPELINE:
                raise MissingTesseractProfilesError("Pipeline {!r} requires an explicit ProfileDictionary.".format(pipeline_value))
            profiles = create_freespace_pipeline_profiles()
        else:
            if not isinstance(profiles_value, ProfileDictionary):
                raise InvalidTesseractProfilesError("profiles must be ProfileDictionary, got {}.".format(type(profiles_value).__name__))
            profiles = profiles_value

        auto_seed_value = values.get("auto_seed", False)
        if not isinstance(auto_seed_value, bool):
            raise UnknownTesseractOptionError("auto_seed must be bool.")
        return cls(pipeline_value, profiles, auto_seed_value)
