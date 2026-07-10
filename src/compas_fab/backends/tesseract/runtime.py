"""Task Composer execution with caller-owned native inputs preserved."""

from __future__ import annotations

from attrs import define
from tesseract_robotics.planning import Robot
from tesseract_robotics.planning import TaskComposer
from tesseract_robotics.tesseract_command_language import CompositeInstruction
from tesseract_robotics.tesseract_command_language import InstructionPoly

from .errors import TesseractPlanningFailedError
from .errors import TesseractProgramCopyError
from .native import TesseractPlanningRequest
from .native import TesseractPlanningResult


def _copy_program(program: CompositeInstruction) -> CompositeInstruction:
    """Copy through Tesseract's native polymorphic instruction container."""
    return InstructionPoly(program).asCompositeInstruction()


@define(slots=True)
class TesseractRuntime:
    """Own Task Composer and execute validated native requests."""

    composer: TaskComposer

    @classmethod
    def build(cls, composer: TaskComposer) -> TesseractRuntime:
        """Build a runtime around an initialized Task Composer.

        Args:
            composer: Initialized native Task Composer.

        Returns:
            Runtime retaining the composer for repeated executions.
        """
        return cls(composer)

    def execute(
        self,
        robot: Robot,
        request: TesseractPlanningRequest,
    ) -> TesseractPlanningResult:
        """Execute exactly one native request without mutating caller input.

        Args:
            robot: Native robot environment to plan against.
            request: Validated native request.

        Returns:
            Complete validated native result.
        """
        execution_program = request.program
        if request.auto_seed:
            try:
                # TaskComposer mutates Cartesian waypoint seeds. Tesseract's
                # polymorphic copy retains plugin instructions without making
                # serialization registration an execution prerequisite.
                execution_program = _copy_program(request.program)
            except (RuntimeError, TypeError, ValueError) as error:
                raise TesseractProgramCopyError("Native program copy failed before pipeline {!r}: {}.".format(request.pipeline, error)) from error
        try:
            native_result = self.composer.plan(
                robot,
                execution_program,
                pipeline=request.pipeline,
                profiles=request.profiles,
                auto_seed=request.auto_seed,
            )
        except (RuntimeError, ValueError) as error:
            raise TesseractPlanningFailedError(request.pipeline, str(error)) from error
        return TesseractPlanningResult.build(request, native_result)
