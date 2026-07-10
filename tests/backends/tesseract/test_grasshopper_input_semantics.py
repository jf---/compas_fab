from pathlib import Path
from types import ModuleType
from types import SimpleNamespace
import sys

from tesseract_robotics.planning import JointTarget
from tesseract_robotics.tesseract_command_language import ProfileDictionary

from compas_fab.ghpython.input_semantics import optional_connected_input

COMPONENTS = Path(__file__).parents[3] / "src" / "compas_fab" / "ghpython" / "components_cpython"


class FakeParameter:
    def __init__(self, name, connected):
        self.Name = name
        self.SourceCount = 1 if connected else 0


class FakeComponent:
    def __init__(self, connections):
        self.Params = SimpleNamespace(Input=[FakeParameter(name, connected) for name, connected in connections.items()])


def _load_component(monkeypatch, name, class_name, connections):
    errors = []
    host_component = FakeComponent(connections)
    grasshopper = ModuleType("Grasshopper")
    grasshopper.Kernel = SimpleNamespace(GH_ScriptInstance=object)
    compas_ghpython = ModuleType("compas_ghpython")
    compas_ghpython.error = lambda component, message: errors.append(message)
    monkeypatch.setitem(sys.modules, "Grasshopper", grasshopper)
    monkeypatch.setitem(sys.modules, "Rhino", ModuleType("Rhino"))
    monkeypatch.setitem(sys.modules, "System", ModuleType("System"))
    monkeypatch.setitem(sys.modules, "compas_ghpython", compas_ghpython)
    monkeypatch.setattr(
        "compas_fab.ghpython.ensure_value_list",
        lambda *args, **kwargs: None,
        raising=False,
    )
    source = COMPONENTS / name / "code.py"
    namespace = {"__name__": "test_" + name, "__file__": str(source)}
    exec(
        compile(source.read_text(encoding="utf-8"), str(source), "exec"),
        namespace,
    )
    namespace["ghenv"] = SimpleNamespace(Component=host_component)
    return namespace[class_name](), errors


def test_optional_connected_input_distinguishes_empty_from_absent():
    connected = FakeComponent({"values": True})
    unconnected = FakeComponent({"values": False})

    assert optional_connected_input(connected, "values", []) == []
    assert optional_connected_input(unconnected, "values", []) is None


def test_joint_target_connected_empty_names_fail(monkeypatch):
    component, errors = _load_component(
        monkeypatch,
        "Cf_TesseractJointTarget",
        "TesseractJointTargetComponent",
        {
            "positions": True,
            "joint_names": True,
            "move_type": False,
            "profile": False,
        },
    )

    target = component.RunScript([0.0], [], None, None)

    assert target is None
    assert errors == ["Target has 0 joint names for 1 positions."]


def test_joint_target_unconnected_empty_names_remain_absent(monkeypatch):
    component, errors = _load_component(
        monkeypatch,
        "Cf_TesseractJointTarget",
        "TesseractJointTargetComponent",
        {
            "positions": True,
            "joint_names": False,
            "move_type": False,
            "profile": False,
        },
    )

    target = component.RunScript([0.0], [], None, None)

    assert target.names is None
    assert errors == []


def test_state_target_connected_empty_dynamics_fail(monkeypatch):
    component, errors = _load_component(
        monkeypatch,
        "Cf_TesseractStateTarget",
        "TesseractStateTargetComponent",
        {
            "positions": True,
            "joint_names": False,
            "velocities": True,
            "accelerations": False,
            "time": False,
            "move_type": False,
            "profile": False,
        },
    )

    target = component.RunScript([0.0], [], [], [], None, None, None)

    assert target is None
    assert errors == ["Target velocities must be a non-empty finite one-dimensional vector."]


def test_descartes_connected_empty_profile_names_fail(monkeypatch):
    component, errors = _load_component(
        monkeypatch,
        "Cf_TesseractDescartesProfile",
        "TesseractDescartesProfileComponent",
        {
            "profile_names": True,
            "enable_collision": False,
            "enable_edge_collision": False,
            "num_threads": False,
            "sample_axis": False,
            "sample_resolution": False,
            "sample_min": False,
            "sample_max": False,
            "ik_solver": False,
            "use_redundant_joint_solutions": False,
            "move_profile": False,
        },
    )

    profiles = component.RunScript(
        [],
        False,
        False,
        0,
        [],
        0.0,
        0.0,
        0.0,
        "",
        False,
        None,
    )

    assert profiles is None
    assert errors == ["profile_names cannot be an empty connected sequence."]


def test_descartes_unconnected_falsey_values_preserve_native_defaults(
    monkeypatch,
):
    component, errors = _load_component(
        monkeypatch,
        "Cf_TesseractDescartesProfile",
        "TesseractDescartesProfileComponent",
        {
            "profile_names": False,
            "enable_collision": False,
            "enable_edge_collision": False,
            "num_threads": False,
            "sample_axis": False,
            "sample_resolution": False,
            "sample_min": False,
            "sample_max": False,
            "ik_solver": False,
            "use_redundant_joint_solutions": False,
            "move_profile": False,
        },
    )

    profiles = component.RunScript(
        [],
        False,
        False,
        0,
        [],
        0.0,
        0.0,
        0.0,
        "",
        False,
        None,
    )

    assert isinstance(profiles, ProfileDictionary)
    assert errors == []


def test_motion_program_connected_empty_default_is_invalid(
    monkeypatch,
    tesseract_robot,
):
    component, errors = _load_component(
        monkeypatch,
        "Cf_TesseractMotionProgram",
        "TesseractMotionProgramComponent",
        {
            "native_robot": True,
            "targets": True,
            "group_name": True,
            "tcp_frame": False,
            "working_frame": True,
            "profile": False,
        },
    )

    result = component.RunScript(
        tesseract_robot,
        [JointTarget([0.0])],
        "manipulator",
        "",
        "",
        "",
    )

    assert result == (None, None, None, None)
    assert errors == ["Motion program working frame must be an exact non-empty string."]
