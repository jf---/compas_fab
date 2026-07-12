"""Exact native planning calls and observable cache signatures."""

from __future__ import annotations

from attrs import define
from tesseract_robotics.tesseract_command_language import CompositeInstruction
from tesseract_robotics.tesseract_command_language import ProfileDictionary

from .errors import InvalidTesseractNativePlanError
from .errors import NativePlanInputsChangedBeforeExecutionError
from .errors import NativePlanInputsChangedDuringExecutionError
from .native import TesseractPlanningRequest
from .native import TesseractPlanningResult
from .native_program_identity import NativeProgramIdentity
from .native_program_identity import native_program_digest
from .planner import TesseractPlanner


@define(frozen=True, slots=True)
class NativePlanSignature:
    """Every observable input controlling one native planning result."""

    planner_identity: int
    scene_revision: int
    program_identity: NativeProgramIdentity
    pipeline: str
    profile_identity: int
    auto_seed: bool

    def __attrs_post_init__(self) -> None:
        if not isinstance(self.planner_identity, int) or self.planner_identity <= 0:
            raise InvalidTesseractNativePlanError("Native plan signature requires planner object identity.")
        if not isinstance(self.scene_revision, int) or self.scene_revision < 0:
            raise InvalidTesseractNativePlanError("Native plan signature requires non-negative scene revision.")
        if not isinstance(self.program_identity, NativeProgramIdentity):
            raise InvalidTesseractNativePlanError("Native plan signature requires program content identity.")
        if not isinstance(self.pipeline, str) or not self.pipeline.strip():
            raise InvalidTesseractNativePlanError("Native plan signature requires an exact pipeline name.")
        if not isinstance(self.profile_identity, int) or self.profile_identity <= 0:
            raise InvalidTesseractNativePlanError("Native plan signature requires profile object identity.")
        required_native_plan_bool(self.auto_seed, "auto_seed")


@define(frozen=True, slots=True)
class NativePlanCall:
    """Validated exact native planning request plus cache signature."""

    planner: TesseractPlanner
    request: TesseractPlanningRequest
    signature: NativePlanSignature

    def __attrs_post_init__(self) -> None:
        if not isinstance(self.planner, TesseractPlanner):
            raise InvalidTesseractNativePlanError("Native plan call requires exact TesseractPlanner.")
        if not isinstance(self.request, TesseractPlanningRequest):
            raise InvalidTesseractNativePlanError("Native plan call requires TesseractPlanningRequest.")
        expected = _signature(self.planner, self.request)
        if self.signature != expected:
            raise InvalidTesseractNativePlanError("Native plan signature does not cover its exact request.")

    @classmethod
    def build(
        cls,
        planner: object,
        program: object,
        pipeline: object,
        profiles: object,
        auto_seed: object,
    ) -> NativePlanCall:
        """Build one exact call without inferring profiles or pipeline."""
        if not isinstance(planner, TesseractPlanner):
            raise InvalidTesseractNativePlanError("Native plan requires exact TesseractPlanner, got {}.".format(type(planner).__name__))
        if not isinstance(program, CompositeInstruction):
            raise InvalidTesseractNativePlanError("Native plan requires exact CompositeInstruction, got {}.".format(type(program).__name__))
        if not isinstance(pipeline, str) or not pipeline.strip():
            raise InvalidTesseractNativePlanError("Native plan pipeline must be an exact non-empty string.")
        if not isinstance(profiles, ProfileDictionary):
            raise InvalidTesseractNativePlanError("Native plan requires exact ProfileDictionary, got {}.".format(type(profiles).__name__))
        seed = required_native_plan_bool(auto_seed, "auto_seed")
        request = TesseractPlanningRequest.build(
            program,
            pipeline,
            profiles,
            seed,
        )
        return cls(planner, request, _signature(planner, request))

    def execute(self) -> TesseractPlanningResult:
        """Execute once and discard a result if observable inputs drift."""
        self.validate_inputs_before_execution()
        result = self.planner.plan_native(self.request)
        if _signature(self.planner, self.request) != self.signature:
            raise NativePlanInputsChangedDuringExecutionError("Native planning inputs changed while the planner call was active; result discarded.")
        return result

    def validate_inputs_before_execution(self) -> None:
        """Require every observable queued input to match its sealed signature."""
        if _signature(self.planner, self.request) != self.signature:
            raise NativePlanInputsChangedBeforeExecutionError("Native planning inputs changed before planner execution.")


def required_native_plan_bool(value: object, name: str) -> bool:
    """Require a real bool for a native planning control."""
    if not isinstance(value, bool):
        raise InvalidTesseractNativePlanError("{} must be bool, got {}.".format(name, type(value).__name__))
    return value


def _signature(
    planner: TesseractPlanner,
    request: TesseractPlanningRequest,
) -> NativePlanSignature:
    return NativePlanSignature(
        id(planner),
        planner.native_scene_revision,
        native_program_digest(request.program),
        request.pipeline,
        id(request.profiles),
        request.auto_seed,
    )
