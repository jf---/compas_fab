import json
from pathlib import Path
import struct
import sys
from types import ModuleType
from types import SimpleNamespace

import pytest

from compas_fab.backends.tesseract.errors import TesseractContactQueryError
from compas_fab.backends.tesseract.errors import TesseractKinematicsPluginError


COMPONENTS = Path(__file__).parents[3] / "src" / "compas_fab" / "ghpython" / "components_cpython"

COMPONENT_SPECS = (
    (
        "Cf_TesseractNativeForwardKinematics",
        "TesseractNativeForwardKinematicsComponent",
        ["planner", "robot_cell_state", "link_name", "group"],
        ["pose"],
        "forward_kinematics_native",
        "forward_kinematics",
    ),
    (
        "Cf_TesseractNativeInverseKinematics",
        "TesseractNativeInverseKinematicsComponent",
        ["planner", "target", "robot_cell_state", "group", "ik_solver_name"],
        ["result", "target_pose", "native_input", "group", "joint_names", "solutions"],
        "inverse_kinematics_native",
        "inverse_kinematics",
    ),
    (
        "Cf_TesseractNativeCollision",
        "TesseractNativeCollisionComponent",
        ["planner", "robot_cell_state", "request"],
        ["result", "native_map", "native_contacts", "colliding_contacts", "in_collision"],
        "check_collision_native",
        "check_collision",
    ),
)


def _load_component(monkeypatch, directory, class_name):
    errors = []
    grasshopper = ModuleType("Grasshopper")
    grasshopper.Kernel = SimpleNamespace(GH_ScriptInstance=object)
    compas_ghpython = ModuleType("compas_ghpython")
    compas_ghpython.error = lambda component, message: errors.append(message)
    monkeypatch.setitem(sys.modules, "Grasshopper", grasshopper)
    monkeypatch.setitem(sys.modules, "Rhino", ModuleType("Rhino"))
    monkeypatch.setitem(sys.modules, "System", ModuleType("System"))
    monkeypatch.setitem(sys.modules, "compas_ghpython", compas_ghpython)
    source = COMPONENTS / directory / "code.py"
    namespace = {"__name__": "test_" + directory, "__file__": str(source)}
    exec(compile(source.read_text(encoding="utf-8"), str(source), "exec"), namespace)
    namespace["ghenv"] = SimpleNamespace(Component=object())
    return namespace[class_name](), errors


def _png_size(path):
    content = path.read_bytes()
    assert content[:8] == b"\x89PNG\r\n\x1a\n"
    return struct.unpack(">II", content[16:24])


@pytest.mark.parametrize(
    ("directory", "class_name", "inputs", "outputs", "native_call", "conventional_call"),
    COMPONENT_SPECS,
)
def test_native_component_contract(directory, class_name, inputs, outputs, native_call, conventional_call):
    root = COMPONENTS / directory
    code = root.joinpath("code.py").read_text(encoding="utf-8")
    metadata = json.loads(root.joinpath("metadata.json").read_text(encoding="utf-8"))

    assert [item["name"] for item in metadata["ghpython"]["inputParameters"]] == inputs
    assert [item["name"] for item in metadata["ghpython"]["outputParameters"]] == outputs
    assert all(item.get("scriptParamAccess", 0) == 0 for item in metadata["ghpython"]["inputParameters"])
    assert "# r: tesseract-robotics-nanobind==0.35.0.7" in code
    assert "planner.{}(".format(native_call) in code
    assert "planner.{}(".format(conventional_call) not in code
    assert "except TesseractBackendError" in code
    assert "except Exception" not in code
    assert "list_to_tree" not in code
    assert len(code.splitlines()) < 100
    assert _png_size(root / "icon.png") == (24, 24)


def test_forward_kinematics_returns_exact_native_pose(monkeypatch):
    component, errors = _load_component(
        monkeypatch,
        "Cf_TesseractNativeForwardKinematics",
        "TesseractNativeForwardKinematicsComponent",
    )
    state = object()
    pose = object()

    class Planner:
        def forward_kinematics_native(self, received_state, received_link, received_group):
            assert received_state is state
            assert received_link == "tool0"
            assert received_group == "manipulator"
            return pose

    assert component.RunScript(Planner(), state, "tool0", "manipulator") is pose
    assert errors == []


def test_inverse_kinematics_preserves_exact_empty_native_result(monkeypatch):
    component, errors = _load_component(
        monkeypatch,
        "Cf_TesseractNativeInverseKinematics",
        "TesseractNativeInverseKinematicsComponent",
    )
    target = object()
    state = object()
    target_pose = object()
    native_input = object()
    solutions = []
    result = SimpleNamespace(
        target_pose=target_pose,
        native_input=native_input,
        group="manipulator",
        joint_names=("joint_1", "joint_2"),
        native_solutions=solutions,
    )

    class Planner:
        def inverse_kinematics_native(self, received_target, received_state, received_group, received_solver):
            assert received_target is target
            assert received_state is state
            assert received_group == "manipulator"
            assert received_solver == "KDLInvKinChainLMA"
            return result

    outputs = component.RunScript(Planner(), target, state, "manipulator", "KDLInvKinChainLMA")

    assert outputs == (result, target_pose, native_input, "manipulator", ["joint_1", "joint_2"], solutions)
    assert outputs[0] is result
    assert outputs[5] is solutions
    assert errors == []


def test_collision_preserves_all_contacts_before_filtering(monkeypatch):
    component, errors = _load_component(
        monkeypatch,
        "Cf_TesseractNativeCollision",
        "TesseractNativeCollisionComponent",
    )
    state = object()
    request = object()
    native_map = object()
    separated = SimpleNamespace(distance=0.01)
    touching = SimpleNamespace(distance=0.0)
    penetrating = SimpleNamespace(distance=-0.02)
    contacts = [separated, touching, penetrating]
    result = SimpleNamespace(native_map=native_map, native_results=contacts)

    class Planner:
        def check_collision_native(self, received_state, received_request):
            assert received_state is state
            assert received_request is request
            return result

    outputs = component.RunScript(Planner(), state, request)

    assert outputs == (result, native_map, contacts, [touching, penetrating], True)
    assert outputs[0] is result
    assert outputs[2] is contacts
    assert errors == []


@pytest.mark.parametrize(
    ("directory", "class_name", "arguments", "expected"),
    [
        (
            "Cf_TesseractNativeForwardKinematics",
            "TesseractNativeForwardKinematicsComponent",
            (None, object(), "tool0", "manipulator"),
            None,
        ),
        (
            "Cf_TesseractNativeInverseKinematics",
            "TesseractNativeInverseKinematicsComponent",
            (object(), None, object(), "manipulator", ""),
            (None, None, None, None, None, None),
        ),
        (
            "Cf_TesseractNativeCollision",
            "TesseractNativeCollisionComponent",
            (object(), object(), None),
            (None, None, None, None, None),
        ),
    ],
)
def test_missing_required_input_returns_only_none(monkeypatch, directory, class_name, arguments, expected):
    component, errors = _load_component(monkeypatch, directory, class_name)

    assert component.RunScript(*arguments) == expected
    assert errors == []


@pytest.mark.parametrize(
    ("directory", "class_name", "arguments", "exception", "expected"),
    [
        (
            "Cf_TesseractNativeForwardKinematics",
            "TesseractNativeForwardKinematicsComponent",
            (object(), object(), "tool0", "manipulator"),
            TesseractKinematicsPluginError("fk failed"),
            None,
        ),
        (
            "Cf_TesseractNativeInverseKinematics",
            "TesseractNativeInverseKinematicsComponent",
            (object(), object(), object(), "manipulator", ""),
            TesseractKinematicsPluginError("ik failed"),
            (None, None, None, None, None, None),
        ),
        (
            "Cf_TesseractNativeCollision",
            "TesseractNativeCollisionComponent",
            (object(), object(), object()),
            TesseractContactQueryError("collision failed"),
            (None, None, None, None, None),
        ),
    ],
)
def test_backend_error_is_reported(monkeypatch, directory, class_name, arguments, exception, expected):
    component, errors = _load_component(monkeypatch, directory, class_name)

    class Planner:
        def forward_kinematics_native(self, *args):
            raise exception

        def inverse_kinematics_native(self, *args):
            raise exception

        def check_collision_native(self, *args):
            raise exception

    received = (Planner(),) + arguments[1:]

    assert component.RunScript(*received) == expected
    assert errors == [str(exception)]
