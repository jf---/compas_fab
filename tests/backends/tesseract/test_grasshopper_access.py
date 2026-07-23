"""Grasshopper input access declarations and the StateTarget length guard.

Two independent guarantees are locked in here.

1. Access matrix. Rhino 8's ScriptEditor persists each script input's
   Grasshopper access in ``metadata.json`` under the ``scriptParamAccess`` key,
   which is the ``GH_ParamAccess`` enum (``0`` item, ``1`` list, ``2`` tree).
   Item access is the default and is expressed by the ABSENCE of the key. An
   ordered sequence (``targets``) or a fixed vector (``positions``,
   ``joint_names``, ``velocities``, ``accelerations``) MUST declare list access
   so Grasshopper runs ``RunScript`` once per branch -- one target or program
   per branch -- instead of lifting item-wise and splintering a single vector
   across leaves (a wrong toolpath). Scalars stay item so they tree-lift
   natively.

2. StateTarget length guard. A native ``StateTarget`` cannot mix a position
   vector with a velocity, acceleration, or name vector of a different length.
   The typed backend enforces every length at the ``state_target_from_native``
   boundary and the component surfaces the named ``TesseractBackendError`` via
   ``error(...)`` and returns ``None``. These tests drive ``RunScript``
   end-to-end so the guard is pinned at the component boundary, where a
   downstream regression would otherwise produce a silently wrong target.

Behavioral per-branch lifting -- that Grasshopper's solver actually slices
item/list/tree data and yields one target or program per branch -- is validated
for real by the rhinocode GH harness (Rhino 8.33 live interpreter, separate
track). This file pins the access declarations and the guard logic that harness
relies on; it drives a single ``RunScript`` call and does not itself slice trees.
"""

from pathlib import Path
from types import ModuleType
from types import SimpleNamespace
import json
import sys

import pytest

COMPONENTS = Path(__file__).parents[3] / "src" / "compas_fab" / "ghpython" / "components_cpython"

# GH_ParamAccess enum as persisted under "scriptParamAccess"; item is the
# default and is written as an absent key.
_GH_PARAM_ACCESS = {0: "item", 1: "list", 2: "tree"}
_ITEM = "item"
_LIST = "list"

# The exact access every authoring-component input must declare. Ordered
# sequences and fixed vectors are list; every scalar is item.
_ACCESS_MATRIX = {
    "Cf_TesseractJointTarget": {
        "positions": _LIST,
        "joint_names": _LIST,
        "move_type": _ITEM,
        "profile": _ITEM,
    },
    "Cf_TesseractStateTarget": {
        "positions": _LIST,
        "joint_names": _LIST,
        "velocities": _LIST,
        "accelerations": _LIST,
        "time": _ITEM,
        "move_type": _ITEM,
        "profile": _ITEM,
    },
    "Cf_TesseractMotionProgram": {
        "native_robot": _ITEM,
        "targets": _LIST,
        "group_name": _ITEM,
        "tcp_frame": _ITEM,
        "working_frame": _ITEM,
        "profile": _ITEM,
    },
    "Cf_TesseractPose": {
        "frame": _ITEM,
        "metres_per_user_unit": _ITEM,
        "working_frame": _ITEM,
    },
    "Cf_TesseractCartesianTarget": {
        "pose": _ITEM,
        "move_type": _ITEM,
        "profile": _ITEM,
    },
}

# The inputs whose list access is load-bearing: dropping the flag makes
# Grasshopper lift item-wise and silently splinter a vector or sequence.
_LIST_ACCESS_INPUTS = {
    "Cf_TesseractJointTarget": ["positions", "joint_names"],
    "Cf_TesseractStateTarget": ["positions", "joint_names", "velocities", "accelerations"],
    "Cf_TesseractMotionProgram": ["targets"],
}


def _metadata(component):
    return json.loads((COMPONENTS / component / "metadata.json").read_text(encoding="utf-8"))


def _input_access(parameter):
    return _GH_PARAM_ACCESS[parameter.get("scriptParamAccess", 0)]


@pytest.mark.parametrize("component", sorted(_ACCESS_MATRIX))
def test_authoring_component_declares_exact_input_access(component):
    parameters = {parameter["name"]: parameter for parameter in _metadata(component)["ghpython"]["inputParameters"]}
    expected = _ACCESS_MATRIX[component]

    # Every declared input is classified, so a newly added input cannot slip in
    # with an unreviewed default access.
    assert set(parameters) == set(expected)
    for name, access in expected.items():
        assert _input_access(parameters[name]) == access, (component, name)


def test_ordered_and_vector_inputs_keep_grasshopper_list_access():
    # Regression ratchet on the raw value: these ordered sequences and fixed
    # vectors must carry "scriptParamAccess": 1. If the flag is dropped the
    # access resolves to item and Grasshopper splinters the vector per leaf.
    for component, inputs in _LIST_ACCESS_INPUTS.items():
        parameters = {parameter["name"]: parameter for parameter in _metadata(component)["ghpython"]["inputParameters"]}
        for name in inputs:
            assert parameters[name].get("scriptParamAccess") == 1, (component, name)


def test_scalar_inputs_never_declare_list_or_tree_access():
    # The complementary ratchet: a scalar that gained list/tree access would
    # stop tree-lifting natively and mis-shape one target across a whole branch.
    for component, matrix in _ACCESS_MATRIX.items():
        parameters = {parameter["name"]: parameter for parameter in _metadata(component)["ghpython"]["inputParameters"]}
        for name, access in matrix.items():
            if access == _ITEM:
                assert "scriptParamAccess" not in parameters[name], (component, name)


class _FakeParameter:
    def __init__(self, name, connected):
        self.Name = name
        self.SourceCount = 1 if connected else 0
        self.PersistentDataCount = 0


class _FakeComponent:
    def __init__(self, connections):
        self.Params = SimpleNamespace(Input=[_FakeParameter(name, connected) for name, connected in connections.items()])


def _load_component(monkeypatch, name, class_name, connections):
    """Load a CPython component and capture its ``error(...)`` messages.

    Mirrors the harness in ``test_grasshopper_input_semantics.py``: Grasshopper,
    Rhino, and System are faked, ``compas_ghpython.error`` is redirected into a
    list, and ``ensure_value_list`` is neutralised because it mutates a live
    canvas value list that the fake host does not provide.
    """
    errors = []
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
    exec(compile(source.read_text(encoding="utf-8"), str(source), "exec"), namespace)
    namespace["ghenv"] = SimpleNamespace(Component=_FakeComponent(connections))
    return namespace[class_name](), errors


def _load_state_target(monkeypatch, connections):
    return _load_component(
        monkeypatch,
        "Cf_TesseractStateTarget",
        "TesseractStateTargetComponent",
        connections,
    )


def test_state_target_velocities_length_mismatch_is_rejected(monkeypatch):
    component, errors = _load_state_target(
        monkeypatch,
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

    target = component.RunScript([0.0, 0.0, 0.0], [], [1.0, 2.0], [], None, None, None)

    assert target is None
    assert errors == ["Target velocities length 2 does not match 3 positions."]


def test_state_target_accelerations_length_mismatch_is_rejected(monkeypatch):
    component, errors = _load_state_target(
        monkeypatch,
        {
            "positions": True,
            "joint_names": False,
            "velocities": False,
            "accelerations": True,
            "time": False,
            "move_type": False,
            "profile": False,
        },
    )

    target = component.RunScript([0.0, 0.0, 0.0], [], [], [1.0, 2.0], None, None, None)

    assert target is None
    assert errors == ["Target accelerations length 2 does not match 3 positions."]


def test_state_target_joint_names_length_mismatch_is_rejected(monkeypatch):
    component, errors = _load_state_target(
        monkeypatch,
        {
            "positions": True,
            "joint_names": True,
            "velocities": False,
            "accelerations": False,
            "time": False,
            "move_type": False,
            "profile": False,
        },
    )

    target = component.RunScript([0.0, 0.0, 0.0], ["a", "b"], [], [], None, None, None)

    assert target is None
    assert errors == ["Target has 2 joint names for 3 positions."]


def test_state_target_matched_length_dynamics_are_accepted(monkeypatch):
    component, errors = _load_state_target(
        monkeypatch,
        {
            "positions": True,
            "joint_names": True,
            "velocities": True,
            "accelerations": True,
            "time": False,
            "move_type": False,
            "profile": False,
        },
    )

    target = component.RunScript(
        [0.0, 0.0, 0.0],
        ["a", "b", "c"],
        [1.0, 2.0, 3.0],
        [4.0, 5.0, 6.0],
        None,
        None,
        None,
    )

    assert errors == []
    assert target is not None
    assert list(target.positions) == pytest.approx([0.0, 0.0, 0.0])
    assert list(target.velocities) == pytest.approx([1.0, 2.0, 3.0])
    assert list(target.accelerations) == pytest.approx([4.0, 5.0, 6.0])
    assert list(target.names) == ["a", "b", "c"]


def test_joint_target_builds_full_position_vector_under_list_access(monkeypatch):
    # List access delivers the whole joint vector as one branch, so the target
    # retains every position rather than one per lifted leaf.
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

    target = component.RunScript([0.1, 0.2, 0.3, 0.4, 0.5, 0.6], [], None, None)

    assert errors == []
    assert target is not None
    assert list(target.positions) == pytest.approx([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
