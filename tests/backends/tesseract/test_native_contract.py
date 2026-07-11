from pathlib import Path

import pytest
from tesseract_robotics.planning.composer import PlanningResult
from tesseract_robotics.tesseract_command_language import CompositeInstruction
from tesseract_robotics.tesseract_command_language import ProfileDictionary
from tesseract_robotics.tesseract_command_language import SetDigitalInstruction

from compas_fab.backends.tesseract.errors import EmptyTesseractProgramError
from compas_fab.backends.tesseract.errors import InvalidTesseractProgramError
from compas_fab.backends.tesseract.errors import InvalidTesseractPipelineError
from compas_fab.backends.tesseract.errors import InvalidTesseractProfilesError
from compas_fab.backends.tesseract.errors import MalformedTesseractNativeResultError
from compas_fab.backends.tesseract.errors import MissingTesseractOutputError
from compas_fab.backends.tesseract.errors import TesseractPlanningFailedError
from compas_fab.backends.tesseract.errors import TesseractProgramCopyError
from compas_fab.backends.tesseract.errors import UnknownTesseractOptionError
from compas_fab.backends.tesseract.native import TesseractPlanningRequest
from compas_fab.backends.tesseract.native import TesseractPlanningResult
from compas_fab.backends.tesseract.runtime import TesseractRuntime
from compas_fab.backends.tesseract import runtime as runtime_module


def _program():
    program = CompositeInstruction("DEFAULT")
    program.push_back(SetDigitalInstruction("do_test", 0, True))
    return program


def test_native_request_retains_exact_native_objects():
    program = _program()
    profiles = ProfileDictionary()

    request = TesseractPlanningRequest.build(
        program=program,
        pipeline="OMPLPipeline",
        profiles=profiles,
        auto_seed=False,
    )

    assert request.program is program
    assert request.profiles is profiles
    assert request.pipeline == "OMPLPipeline"
    assert request.auto_seed is False


def test_empty_native_program_fails_loudly():
    with pytest.raises(EmptyTesseractProgramError):
        TesseractPlanningRequest.build(
            program=CompositeInstruction("DEFAULT"),
            pipeline="OMPLPipeline",
            profiles=ProfileDictionary(),
            auto_seed=False,
        )


def test_native_request_factory_rejects_wrong_native_types():
    with pytest.raises(InvalidTesseractProgramError):
        TesseractPlanningRequest.build(object(), "Pipeline", ProfileDictionary(), False)
    with pytest.raises(InvalidTesseractPipelineError):
        TesseractPlanningRequest.build(_program(), object(), ProfileDictionary(), False)
    with pytest.raises(InvalidTesseractProfilesError):
        TesseractPlanningRequest.build(_program(), "Pipeline", object(), False)
    with pytest.raises(UnknownTesseractOptionError, match="auto_seed"):
        TesseractPlanningRequest.build(_program(), "Pipeline", ProfileDictionary(), 1)


def test_native_request_raw_constructor_cannot_bypass_invariants():
    with pytest.raises(EmptyTesseractProgramError):
        TesseractPlanningRequest(
            CompositeInstruction("DEFAULT"),
            "Pipeline",
            ProfileDictionary(),
            False,
        )


@pytest.mark.parametrize("pipeline", ["", "  "])
def test_empty_pipeline_fails_loudly(pipeline):
    with pytest.raises(InvalidTesseractPipelineError):
        TesseractPlanningRequest.build(
            program=_program(),
            pipeline=pipeline,
            profiles=ProfileDictionary(),
            auto_seed=False,
        )


def test_native_result_retains_exact_request_and_result():
    request = TesseractPlanningRequest.build(
        program=_program(),
        pipeline="OMPLPipeline",
        profiles=ProfileDictionary(),
        auto_seed=False,
    )
    native_result = PlanningResult(successful=True, raw_results=request.program)

    result = TesseractPlanningResult.build(request, native_result)

    assert result.request is request
    assert result.native_result is native_result
    assert result.raw_program is request.program


def test_failed_native_result_fails_loudly():
    request = TesseractPlanningRequest.build(
        program=_program(),
        pipeline="TrajOptPipeline",
        profiles=ProfileDictionary(),
        auto_seed=False,
    )
    native_result = PlanningResult(successful=False, message="solver failed")

    with pytest.raises(TesseractPlanningFailedError, match="TrajOptPipeline.*solver failed"):
        TesseractPlanningResult.build(request, native_result)


def test_native_result_raw_constructor_rejects_failure():
    request = TesseractPlanningRequest.build(
        _program(),
        "TrajOptPipeline",
        ProfileDictionary(),
        False,
    )
    native_result = PlanningResult(successful=False, message="solver failed")

    with pytest.raises(TesseractPlanningFailedError, match="solver failed"):
        TesseractPlanningResult(request, native_result, _program())


def test_native_result_raw_constructor_rejects_missing_output():
    request = TesseractPlanningRequest.build(
        _program(),
        "TrajOptPipeline",
        ProfileDictionary(),
        False,
    )
    native_result = PlanningResult(successful=True, raw_results=None)

    with pytest.raises(MissingTesseractOutputError):
        TesseractPlanningResult(request, native_result, _program())


def test_native_result_raw_constructor_requires_exact_raw_output():
    request = TesseractPlanningRequest.build(
        _program(),
        "TrajOptPipeline",
        ProfileDictionary(),
        False,
    )
    native_result = PlanningResult(successful=True, raw_results=request.program)

    with pytest.raises(MalformedTesseractNativeResultError, match="raw output"):
        TesseractPlanningResult(request, native_result, _program())


def test_native_result_factory_owns_wrong_type_error_model():
    request = TesseractPlanningRequest.build(
        _program(),
        "TrajOptPipeline",
        ProfileDictionary(),
        False,
    )
    native_result = PlanningResult(
        successful=True,
        raw_results=request.program,
    )

    with pytest.raises(MalformedTesseractNativeResultError, match="request"):
        TesseractPlanningResult.build(object(), native_result)
    with pytest.raises(MalformedTesseractNativeResultError, match="PlanningResult"):
        TesseractPlanningResult.build(request, object())


def test_native_execution_exception_retains_pipeline_diagnostic():
    class FailingComposer:
        def plan(self, robot, program, *, pipeline, profiles, auto_seed):
            raise RuntimeError("native execution failed")

    request = TesseractPlanningRequest.build(
        program=_program(),
        pipeline="TrajOptPipeline",
        profiles=ProfileDictionary(),
        auto_seed=False,
    )
    runtime = TesseractRuntime.build(FailingComposer())

    with pytest.raises(
        TesseractPlanningFailedError,
        match="TrajOptPipeline.*native execution failed",
    ):
        runtime.execute(object(), request)


def test_unseeded_native_execution_passes_exact_polymorphic_program():
    class CapturingComposer:
        program = None

        def plan(self, robot, program, *, pipeline, profiles, auto_seed):
            self.program = program
            return PlanningResult(successful=True, raw_results=program)

    composer = CapturingComposer()
    request = TesseractPlanningRequest.build(
        program=_program(),
        pipeline="CustomPluginPipeline",
        profiles=ProfileDictionary(),
        auto_seed=False,
    )

    result = TesseractRuntime.build(composer).execute(object(), request)

    assert composer.program is request.program
    assert result.raw_program is request.program


def test_seeded_native_execution_uses_polymorphic_copy_without_serialization():
    class MutatingComposer:
        program = None

        def plan(self, robot, program, *, pipeline, profiles, auto_seed):
            self.program = program
            program.setDescription("seeded execution")
            return PlanningResult(successful=True, raw_results=program)

    composer = MutatingComposer()
    request = TesseractPlanningRequest.build(
        program=_program(),
        pipeline="CustomPluginPipeline",
        profiles=ProfileDictionary(),
        auto_seed=True,
    )

    result = TesseractRuntime.build(composer).execute(object(), request)

    assert composer.program is not request.program
    assert result.raw_program is composer.program
    assert request.program.getDescription() != "seeded execution"
    assert "tesseract_serialization" not in Path(runtime_module.__file__).read_text(encoding="utf-8")


def test_native_program_copy_failure_is_named(monkeypatch):
    def fail_copy(program):
        raise RuntimeError("polymorphic copy failed")

    monkeypatch.setattr(runtime_module, "_copy_program", fail_copy)
    request = TesseractPlanningRequest.build(
        program=_program(),
        pipeline="TrajOptPipeline",
        profiles=ProfileDictionary(),
        auto_seed=True,
    )
    runtime = TesseractRuntime.build(object())

    with pytest.raises(TesseractProgramCopyError, match="polymorphic copy failed"):
        runtime.execute(object(), request)
