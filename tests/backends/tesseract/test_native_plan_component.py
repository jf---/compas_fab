from pathlib import Path
from types import ModuleType
from types import SimpleNamespace
import sys

import pytest
from attrs import evolve
from tesseract_robotics.planning.composer import PlanningResult
from tesseract_robotics.tesseract_command_language import CompositeInstruction
from tesseract_robotics.tesseract_command_language import ProfileDictionary
from tesseract_robotics.tesseract_command_language import SetDigitalInstruction

from compas_fab.backends.tesseract.errors import InvalidTesseractNativePlanError
from compas_fab.backends.tesseract.errors import TesseractPlanningFailedError
from compas_fab.backends.tesseract.native import TesseractPlanningResult
from compas_fab.backends.tesseract.native_plan import NativePlanCall
from compas_fab.backends.tesseract.planner import TesseractPlanner

COMPONENT = Path(__file__).parents[3] / "src" / "compas_fab" / "ghpython" / "components_cpython" / "Cf_TesseractNativePlan" / "code.py"


def _program(description=""):
    program = CompositeInstruction("DEFAULT")
    program.push_back(SetDigitalInstruction("do_test", 0, True))
    program.setDescription(description)
    return program


class CapturingPlanner(TesseractPlanner):
    def __init__(self):
        self.requests = []
        self.fail = False
        self.scene_revision = 0

    @property
    def native_scene_revision(self):
        return self.scene_revision

    def plan_native(self, request):
        self.requests.append(request)
        if self.fail:
            raise TesseractPlanningFailedError(request.pipeline, "solver failed")
        native = PlanningResult(
            successful=True,
            message="planned",
            raw_results=request.program,
        )
        return TesseractPlanningResult.build(request, native)


def _load_component(monkeypatch):
    sticky = {}
    errors = []
    grasshopper = ModuleType("Grasshopper")
    grasshopper.Kernel = SimpleNamespace(GH_ScriptInstance=object)
    compas_ghpython = ModuleType("compas_ghpython")
    compas_ghpython.create_id = lambda component, suffix: suffix
    compas_ghpython.error = lambda component, message: errors.append(message)
    scriptcontext = ModuleType("scriptcontext")
    scriptcontext.sticky = sticky
    monkeypatch.setitem(sys.modules, "Grasshopper", grasshopper)
    monkeypatch.setitem(sys.modules, "Rhino", ModuleType("Rhino"))
    monkeypatch.setitem(sys.modules, "System", ModuleType("System"))
    monkeypatch.setitem(sys.modules, "compas_ghpython", compas_ghpython)
    monkeypatch.setitem(sys.modules, "scriptcontext", scriptcontext)
    namespace = {
        "__name__": "test_tesseract_native_plan_component",
        "__file__": str(COMPONENT),
    }
    exec(
        compile(
            COMPONENT.read_text(encoding="utf-8"),
            str(COMPONENT),
            "exec",
        ),
        namespace,
    )
    namespace["ghenv"] = SimpleNamespace(Component=object())
    return namespace["TesseractNativePlanComponent"](), sticky, errors


def test_native_plan_call_retains_exact_request_inputs():
    planner = CapturingPlanner()
    program = _program()
    profiles = ProfileDictionary()

    call = NativePlanCall.build(
        planner,
        program,
        "DescartesFPipeline",
        profiles,
        True,
    )

    assert call.planner is planner
    assert call.request.program is program
    assert call.request.profiles is profiles
    assert call.request.pipeline == "DescartesFPipeline"
    assert call.request.auto_seed is True
    assert call.signature.planner_identity == id(planner)
    assert call.signature.scene_revision == 0
    assert call.signature.profile_identity == id(profiles)


def test_native_plan_call_executes_only_plan_native():
    planner = CapturingPlanner()
    call = NativePlanCall.build(
        planner,
        _program(),
        "DescartesFPipeline",
        ProfileDictionary(),
        False,
    )

    result = call.execute()

    assert result.request is call.request
    assert planner.requests == [call.request]


def test_native_plan_call_raw_constructor_cannot_bypass_signature():
    call = NativePlanCall.build(
        CapturingPlanner(),
        _program(),
        "DescartesFPipeline",
        ProfileDictionary(),
        False,
    )

    with pytest.raises(InvalidTesseractNativePlanError):
        evolve(call, planner=CapturingPlanner())


@pytest.mark.parametrize(
    ("planner", "program", "pipeline", "profiles", "auto_seed"),
    [
        (object(), _program(), "Pipeline", ProfileDictionary(), False),
        (CapturingPlanner(), object(), "Pipeline", ProfileDictionary(), False),
        (CapturingPlanner(), _program(), "", ProfileDictionary(), False),
        (CapturingPlanner(), _program(), "Pipeline", object(), False),
        (CapturingPlanner(), _program(), "Pipeline", ProfileDictionary(), 1),
    ],
)
def test_native_plan_call_rejects_inexact_inputs(
    planner,
    program,
    pipeline,
    profiles,
    auto_seed,
):
    with pytest.raises(InvalidTesseractNativePlanError):
        NativePlanCall.build(
            planner,
            program,
            pipeline,
            profiles,
            auto_seed,
        )


def test_compute_false_reuses_only_complete_matching_signature(monkeypatch):
    component, _, _ = _load_component(monkeypatch)
    planner = CapturingPlanner()
    program = _program()
    profiles = ProfileDictionary()

    first = component.RunScript(
        planner,
        program,
        "DescartesFPipeline",
        profiles,
        True,
        True,
    )
    cached = component.RunScript(
        planner,
        program,
        "DescartesFPipeline",
        profiles,
        True,
        False,
    )

    assert cached is first
    assert len(planner.requests) == 1


@pytest.mark.parametrize(
    "changed_input",
    ["planner", "program", "pipeline", "profiles", "auto_seed"],
)
def test_compute_false_clears_cache_when_any_input_changes(
    monkeypatch,
    changed_input,
):
    component, sticky, _ = _load_component(monkeypatch)
    planner = CapturingPlanner()
    program = _program()
    profiles = ProfileDictionary()
    component.RunScript(
        planner,
        program,
        "DescartesFPipeline",
        profiles,
        True,
        True,
    )
    values = {
        "planner": planner,
        "program": program,
        "pipeline": "DescartesFPipeline",
        "profiles": profiles,
        "auto_seed": True,
    }
    replacements = {
        "planner": CapturingPlanner(),
        "program": _program("different"),
        "pipeline": "DescartesDPipeline",
        "profiles": ProfileDictionary(),
        "auto_seed": False,
    }
    values[changed_input] = replacements[changed_input]

    result = component.RunScript(
        values["planner"],
        values["program"],
        values["pipeline"],
        values["profiles"],
        values["auto_seed"],
        False,
    )

    assert result is None
    assert sticky == {}


def test_in_place_program_mutation_invalidates_cached_result(monkeypatch):
    component, sticky, _ = _load_component(monkeypatch)
    planner = CapturingPlanner()
    program = _program()
    profiles = ProfileDictionary()
    component.RunScript(
        planner,
        program,
        "DescartesFPipeline",
        profiles,
        True,
        True,
    )

    program.setDescription("mutated")

    assert (
        component.RunScript(
            planner,
            program,
            "DescartesFPipeline",
            profiles,
            True,
            False,
        )
        is None
    )
    assert sticky == {}


def test_same_planner_scene_revision_invalidates_cached_result(monkeypatch):
    component, sticky, _ = _load_component(monkeypatch)
    planner = CapturingPlanner()
    program = _program()
    profiles = ProfileDictionary()
    component.RunScript(
        planner,
        program,
        "DescartesFPipeline",
        profiles,
        True,
        True,
    )

    planner.scene_revision += 1

    assert (
        component.RunScript(
            planner,
            program,
            "DescartesFPipeline",
            profiles,
            True,
            False,
        )
        is None
    )
    assert sticky == {}


def test_failed_recompute_clears_previous_success(monkeypatch):
    component, sticky, errors = _load_component(monkeypatch)
    planner = CapturingPlanner()
    program = _program()
    profiles = ProfileDictionary()
    component.RunScript(
        planner,
        program,
        "DescartesFPipeline",
        profiles,
        True,
        True,
    )
    planner.fail = True

    assert (
        component.RunScript(
            planner,
            program,
            "DescartesFPipeline",
            profiles,
            True,
            True,
        )
        is None
    )
    assert sticky == {}
    assert errors == ["Tesseract pipeline 'DescartesFPipeline' failed: solver failed"]
    planner.fail = False
    assert (
        component.RunScript(
            planner,
            program,
            "DescartesFPipeline",
            profiles,
            True,
            False,
        )
        is None
    )


def test_component_rejects_non_boolean_compute_without_planning(monkeypatch):
    component, sticky, errors = _load_component(monkeypatch)
    planner = CapturingPlanner()

    result = component.RunScript(
        planner,
        _program(),
        "DescartesFPipeline",
        ProfileDictionary(),
        False,
        1,
    )

    assert result is None
    assert planner.requests == []
    assert sticky == {}
    assert errors == ["compute must be bool, got int."]
