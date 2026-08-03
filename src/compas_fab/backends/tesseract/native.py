"""Lossless native request and result values for Tesseract planning."""

from __future__ import annotations

from attrs import define
from tesseract_robotics.planning.composer import PlanningResult
from tesseract_robotics.tesseract_command_language import CompositeInstruction
from tesseract_robotics.tesseract_command_language import ProfileDictionary

from .errors import EmptyTesseractProgramError
from .errors import InvalidTesseractPipelineError
from .errors import InvalidTesseractProfilesError
from .errors import InvalidTesseractProgramError
from .errors import MalformedTesseractNativeResultError
from .errors import MissingTesseractOutputError
from .errors import TesseractPlanningFailedError
from .errors import UnknownTesseractOptionError


@define(frozen=True, slots=True)
class TesseractPlanningRequest:
    """Exact native inputs for one Task Composer execution."""

    program: CompositeInstruction
    pipeline: str
    profiles: ProfileDictionary
    auto_seed: bool

    def __attrs_post_init__(self) -> None:
        _validate_request(
            self.program,
            self.pipeline,
            self.profiles,
            self.auto_seed,
        )

    @classmethod
    def build(
        cls,
        program: CompositeInstruction,
        pipeline: str,
        profiles: ProfileDictionary,
        auto_seed: bool,
    ) -> TesseractPlanningRequest:
        """Validate and retain an exact native planning request.

        Args:
            program: Native program to execute. Caller ownership is retained.
            pipeline: Exact Task Composer pipeline name.
            profiles: Exact native profile dictionary.
            auto_seed: Whether Tesseract should assign its current-state seed.

        Returns:
            Validated request retaining the supplied native objects.

        Raises:
            EmptyTesseractProgramError: The program has no instructions.
            InvalidTesseractPipelineError: The pipeline name is empty.
        """
        return cls(program, pipeline, profiles, auto_seed)


@define(frozen=True, slots=True)
class TesseractPlanningResult:
    """Complete native result paired with the exact request that produced it."""

    request: TesseractPlanningRequest
    native_result: PlanningResult
    raw_program: CompositeInstruction

    def __attrs_post_init__(self) -> None:
        _validate_result(self.request, self.native_result, self.raw_program)

    @classmethod
    def build(
        cls,
        request: TesseractPlanningRequest,
        native_result: PlanningResult,
    ) -> TesseractPlanningResult:
        """Validate a native result without projecting or discarding it.

        Args:
            request: Exact request that was executed.
            native_result: Complete result returned by nanobind planning.

        Returns:
            A result retaining the request, native result, and raw program.

        Raises:
            TesseractPlanningFailedError: Native planning reported failure.
            MissingTesseractOutputError: Success lacked a raw output program.
        """
        if not isinstance(request, TesseractPlanningRequest):
            raise MalformedTesseractNativeResultError("Native result request must be TesseractPlanningRequest.")
        if not isinstance(native_result, PlanningResult):
            raise MalformedTesseractNativeResultError("Native result must retain exact PlanningResult.")
        raw_program = _validate_result(
            request,
            native_result,
            native_result.raw_results,
        )
        return cls(request, native_result, raw_program)


def _validate_request(
    program: CompositeInstruction,
    pipeline: str,
    profiles: ProfileDictionary,
    auto_seed: bool,
) -> None:
    if not isinstance(program, CompositeInstruction):
        raise InvalidTesseractProgramError("Native program must be CompositeInstruction, got {}.".format(type(program).__name__))
    if not isinstance(pipeline, str) or not pipeline.strip():
        raise InvalidTesseractPipelineError("Task Composer pipeline must be a non-empty string.")
    if not isinstance(profiles, ProfileDictionary):
        raise InvalidTesseractProfilesError("Native profiles must be ProfileDictionary, got {}.".format(type(profiles).__name__))
    if not isinstance(auto_seed, bool):
        raise UnknownTesseractOptionError("auto_seed must be bool, got {}.".format(type(auto_seed).__name__))
    if program.empty():
        raise EmptyTesseractProgramError("Native Tesseract program contains no instructions.")


def _validate_result(
    request: object,
    native_result: object,
    raw_program: object,
) -> CompositeInstruction:
    if not isinstance(request, TesseractPlanningRequest):
        raise MalformedTesseractNativeResultError("Native result request must be TesseractPlanningRequest.")
    if not isinstance(native_result, PlanningResult):
        raise MalformedTesseractNativeResultError("Native result must retain exact PlanningResult.")
    if not native_result.successful:
        raise TesseractPlanningFailedError(request.pipeline, native_result.message)
    if native_result.raw_results is None:
        raise MissingTesseractOutputError("Tesseract pipeline {!r} reported success without a raw program.".format(request.pipeline))
    if not isinstance(raw_program, CompositeInstruction):
        raise MalformedTesseractNativeResultError("Native result raw output must be CompositeInstruction.")
    if raw_program is not native_result.raw_results:
        raise MalformedTesseractNativeResultError("Retained raw output differs from PlanningResult raw output.")
    return raw_program
