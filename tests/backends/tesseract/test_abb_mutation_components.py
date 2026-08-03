"""RunScript contract tests for the ABB MUTATION Grasshopper components.

These exercise the sync-direct write path each component's ``code.py`` presents to
Grasshopper: exec'd under a faithful fake host, ``RunScript`` is driven against a
canned ``FakeRws`` passed straight in as the ``controller``. No network is opened
and no owner/ledger/arming stack exists -- a component is a thin marshal that
calls one RWS method on the rising edge of a button.

The headline guarantee is FIRE-ONCE: :func:`compas_fab.ghpython.button_edge.rising_edge`
collapses a held or recomputed True to a single ``False -> True`` transition, so a
Grasshopper recompute with the button still pressed never re-commands the robot.
Also pinned: the None-controller no-op (and that it does not consume the edge),
the ``Cf_AbbIoWrite`` readback verification (a mismatch errors rather than being
believed), the ``Cf_AbbStart`` cycle/tasks defaulting, and controller-error
surfacing via ``error(...)``.
"""

from pathlib import Path
from types import ModuleType
from types import SimpleNamespace
import json
import sys

import pytest
from abb_robot_client.rws import ABBException

from compas_fab.ghpython.button_edge import rising_edge

COMPONENTS = Path(__file__).parents[3] / "src" / "compas_fab" / "ghpython" / "components_cpython"

_CLASS = {
    "Cf_AbbIoWrite": "AbbIoWriteComponent",
    "Cf_AbbStart": "AbbStartComponent",
    "Cf_AbbReset": "AbbResetComponent",
    "Cf_AbbStop": "AbbStopComponent",
}

# Canned controller reads the FakeRws answers with when nothing was written.
_DIGITAL_VALUE = 1
_ANALOG_VALUE = 3.5


# --- fake Grasshopper host -------------------------------------------------


class _FakeParameter:
    def __init__(self, name, connected):
        self.Name = name
        self.SourceCount = 1 if connected else 0
        self.PersistentDataCount = 0


class _FakeComponent:
    def __init__(self, connections):
        self.Params = SimpleNamespace(Input=[_FakeParameter(name, connected) for name, connected in connections.items()])


def _metadata(directory):
    return json.loads((COMPONENTS / directory / "metadata.json").read_text(encoding="utf-8"))


def _output_names(directory):
    return [parameter["name"] for parameter in _metadata(directory)["ghpython"]["outputParameters"]]


def _connections(directory, **states):
    """Model every declared input as wired unless a test unwires it."""
    names = [parameter["name"] for parameter in _metadata(directory)["ghpython"]["inputParameters"]]
    return {name: states.get(name, True) for name in names}


def _load(monkeypatch, directory, connections, sticky=None):
    """Exec a mutation component under a faithful fake host and capture ``error(...)``.

    Grasshopper, Rhino, System, and scriptcontext are faked because none exist
    outside Rhino; ``compas_ghpython`` is faked so ``error`` is recorded and
    ``create_id`` is deterministic; ``ensure_value_list``/``ensure_boolean_toggle``
    are neutralised because they mutate a live canvas. The REAL ``rising_edge`` runs
    -- fire-once is what these tests pin -- but ``button_edge.create_id`` is pointed
    at the same fake ``create_id`` so the host (which has no ``InstanceGuid``) still
    keys a stable, per-component edge slot. The one component instance and its
    sticky dict persist across ``RunScript`` calls, so edge state carries between
    the simulated solves.
    """
    errors = []
    sticky = {} if sticky is None else sticky
    fake_create_id = lambda component, name: "{}::{}".format(id(component), name)  # noqa: E731
    grasshopper = ModuleType("Grasshopper")
    grasshopper.Kernel = SimpleNamespace(GH_ScriptInstance=object)
    compas_ghpython = ModuleType("compas_ghpython")
    compas_ghpython.error = lambda component, message: errors.append(message)
    compas_ghpython.create_id = fake_create_id
    scriptcontext = ModuleType("scriptcontext")
    scriptcontext.sticky = sticky
    monkeypatch.setitem(sys.modules, "Grasshopper", grasshopper)
    monkeypatch.setitem(sys.modules, "Rhino", ModuleType("Rhino"))
    monkeypatch.setitem(sys.modules, "System", ModuleType("System"))
    monkeypatch.setitem(sys.modules, "compas_ghpython", compas_ghpython)
    monkeypatch.setitem(sys.modules, "scriptcontext", scriptcontext)
    monkeypatch.setattr("compas_fab.ghpython.ensure_value_list", lambda *args, **kwargs: None, raising=False)
    monkeypatch.setattr("compas_fab.ghpython.ensure_boolean_toggle", lambda *args, **kwargs: None, raising=False)
    # The real rising_edge keys via create_id; point it at the fake so the
    # InstanceGuid-less host still produces a stable per-component edge key.
    monkeypatch.setattr("compas_fab.ghpython.button_edge.create_id", fake_create_id, raising=False)
    source = COMPONENTS / directory / "code.py"
    namespace = {"__name__": "test_" + directory, "__file__": str(source)}
    exec(compile(source.read_text(encoding="utf-8"), str(source), "exec"), namespace)
    host = _FakeComponent(connections)
    namespace["ghenv"] = SimpleNamespace(Component=host)
    return namespace[_CLASS[directory]](), errors, sticky, host


# --- canned sync RWS session -----------------------------------------------


class FakeRws:
    """Canned sync ``RWS`` recording the reads and mutations a component drives.

    A single ``error`` (an ``ABBException``), if set, is raised by every method so
    a component's error surface can be pinned without a live controller. Written
    setpoints are echoed back by the matching getter (modelling a controller that
    stores what it is told) unless ``readback`` forces a fixed value, which lets a
    readback mismatch be provoked.
    """

    def __init__(self, error=None, readback=None):
        self._error = error
        self._readback = readback
        self.digital_reads = []
        self.analog_reads = []
        self.digital_writes = []
        self.analog_writes = []
        self.starts = []
        self.stops = 0
        self.resets = 0
        self.closed = False
        self._digital = {}
        self._analog = {}

    def _maybe_raise(self):
        if self._error is not None:
            raise self._error

    def get_controller_state(self):
        self._maybe_raise()
        return "motoron"

    def get_operation_mode(self):
        self._maybe_raise()
        return "AUTO"

    def get_execution_state(self):
        self._maybe_raise()
        return "running"

    def get_digital_io(self, signal, network="Local", unit="DRV_1"):
        self.digital_reads.append(signal)
        self._maybe_raise()
        if self._readback is not None:
            return self._readback
        return self._digital.get(signal, _DIGITAL_VALUE)

    def get_analog_io(self, signal, network="Local", unit="DRV_1"):
        self.analog_reads.append(signal)
        self._maybe_raise()
        if self._readback is not None:
            return self._readback
        return self._analog.get(signal, _ANALOG_VALUE)

    def set_digital_io(self, signal, value, network="Local", unit="DRV_1"):
        self.digital_writes.append((signal, value))
        self._maybe_raise()
        self._digital[signal] = value

    def set_analog_io(self, signal, value, network="Local", unit="DRV_1"):
        self.analog_writes.append((signal, value))
        self._maybe_raise()
        self._analog[signal] = value

    def start(self, cycle="asis", tasks=None):
        self.starts.append((cycle, tasks))
        self._maybe_raise()

    def stop(self):
        self.stops += 1
        self._maybe_raise()

    def resetpp(self):
        self.resets += 1
        self._maybe_raise()

    def close(self):
        self.closed = True


# --- button_edge unit tests (fire-once primitive, no host) -----------------


class _EdgeComponent:
    InstanceGuid = "edge-guid"


def test_rising_edge_fires_once_per_press():
    sticky = {}
    component = _EdgeComponent()

    assert rising_edge(component, True, sticky, "slot") is True  # rising
    assert rising_edge(component, True, sticky, "slot") is False  # steady True
    assert rising_edge(component, False, sticky, "slot") is False  # falling
    assert rising_edge(component, True, sticky, "slot") is True  # rising again


def test_rising_edge_slots_are_independent():
    sticky = {}
    component = _EdgeComponent()

    # Two buttons on one component keep separate, independently latched edges.
    assert rising_edge(component, True, sticky, "a") is True
    assert rising_edge(component, True, sticky, "b") is True
    assert rising_edge(component, True, sticky, "a") is False
    assert rising_edge(component, True, sticky, "b") is False


def test_rising_edge_coerces_truthy_value():
    sticky = {}
    component = _EdgeComponent()

    # A non-bool truthy input still latches; a falsey one never fires.
    assert rising_edge(component, 1, sticky, "slot") is True
    assert rising_edge(component, 1, sticky, "slot") is False
    assert rising_edge(component, 0, sticky, "slot") is False


# --- 1. Cf_AbbIoWrite (HEADLINE: fire-once) --------------------------------


def test_io_write_fires_once_per_press(monkeypatch):
    session = FakeRws()
    component, errors, _, _ = _load(monkeypatch, "Cf_AbbIoWrite", _connections("Cf_AbbIoWrite"))

    # Press: arm True on a fresh solve -> exactly one write, with readback echo.
    receipt, readback = component.RunScript(session, "DO_1", 1.0, False, True)
    assert errors == []
    assert receipt == "wrote DO_1=1"
    assert readback == 1
    assert session.digital_writes == [("DO_1", 1)]

    # Recompute with the button STILL held True -> NO second write.
    assert component.RunScript(session, "DO_1", 1.0, False, True) == (None, None)
    assert session.digital_writes == [("DO_1", 1)]

    # Release (False) then press again (True) -> the write fires once more.
    assert component.RunScript(session, "DO_1", 1.0, False, False) == (None, None)
    assert component.RunScript(session, "DO_1", 1.0, False, True) == ("wrote DO_1=1", 1)
    assert session.digital_writes == [("DO_1", 1), ("DO_1", 1)]
    assert errors == []


def test_io_write_analog_readback_match(monkeypatch):
    session = FakeRws()
    component, errors, _, _ = _load(monkeypatch, "Cf_AbbIoWrite", _connections("Cf_AbbIoWrite"))

    receipt, readback = component.RunScript(session, "AO_1", 3.5, True, True)

    assert errors == []
    # Analog write kept the float; digital was never touched.
    assert session.analog_writes == [("AO_1", 3.5)]
    assert session.digital_writes == []
    assert receipt == "wrote AO_1=3.5"
    assert readback == 3.5


def test_io_write_readback_mismatch_surfaces_error_and_returns_two_none(monkeypatch):
    # The controller echoes a different value than written; the write must not be
    # believed -- an error is raised and no receipt is emitted.
    session = FakeRws(readback=0)
    component, errors, _, _ = _load(monkeypatch, "Cf_AbbIoWrite", _connections("Cf_AbbIoWrite"))

    result = component.RunScript(session, "DO_1", 1.0, False, True)

    assert result == (None, None)
    # The write WAS attempted; only the belief that it took is withheld.
    assert session.digital_writes == [("DO_1", 1)]
    assert len(errors) == 1
    assert "Readback mismatch" in errors[0] and "wrote 1 read 0" in errors[0]


def test_io_write_none_controller_returns_two_none_without_error(monkeypatch):
    component, errors, _, _ = _load(monkeypatch, "Cf_AbbIoWrite", _connections("Cf_AbbIoWrite"))

    assert component.RunScript(None, "DO_1", 1.0, False, True) == (None, None)
    assert errors == []


def test_io_write_none_controller_does_not_consume_edge(monkeypatch):
    session = FakeRws()
    component, errors, _, _ = _load(monkeypatch, "Cf_AbbIoWrite", _connections("Cf_AbbIoWrite"))

    # arm held True but no controller yet -> no-op that must NOT consume the edge.
    assert component.RunScript(None, "DO_1", 1.0, False, True) == (None, None)
    # Controller now present, arm still held True -> the press still fires.
    receipt, readback = component.RunScript(session, "DO_1", 1.0, False, True)

    assert errors == []
    assert receipt == "wrote DO_1=1"
    assert session.digital_writes == [("DO_1", 1)]


def test_io_write_controller_error_surfaces_and_returns_two_none(monkeypatch):
    session = FakeRws(error=ABBException("write boom", -1))
    component, errors, _, _ = _load(monkeypatch, "Cf_AbbIoWrite", _connections("Cf_AbbIoWrite"))

    assert component.RunScript(session, "DO_1", 1.0, False, True) == (None, None)
    assert len(errors) == 1 and "write boom" in errors[0]


# --- 2. Cf_AbbStart --------------------------------------------------------


def test_start_fires_once_per_press(monkeypatch):
    session = FakeRws()
    component, errors, _, _ = _load(monkeypatch, "Cf_AbbStart", _connections("Cf_AbbStart"))

    assert component.RunScript(session, "once", ["T_ROB1"], True) == "started once"
    assert session.starts == [("once", ["T_ROB1"])]
    # Held True across a recompute -> no restart.
    assert component.RunScript(session, "once", ["T_ROB1"], True) is None
    assert session.starts == [("once", ["T_ROB1"])]
    # Release then press -> fires again.
    assert component.RunScript(session, "once", ["T_ROB1"], False) is None
    assert component.RunScript(session, "once", ["T_ROB1"], True) == "started once"
    assert len(session.starts) == 2
    assert errors == []


def test_start_unconnected_cycle_defaults_to_asis_and_omits_tasks(monkeypatch):
    session = FakeRws()
    component, errors, _, _ = _load(
        monkeypatch,
        "Cf_AbbStart",
        _connections("Cf_AbbStart", cycle=False, tasks=False),
    )

    # Non-default cycle/tasks supplied on unwired inputs must be ignored: cycle
    # falls back to asis and tasks are omitted so RWS applies its own default set.
    status = component.RunScript(session, "once", ["ignored"], True)

    assert errors == []
    assert status == "started asis"
    assert session.starts == [("asis", None)]


def test_start_none_controller_returns_none_without_error(monkeypatch):
    component, errors, _, _ = _load(monkeypatch, "Cf_AbbStart", _connections("Cf_AbbStart"))

    assert component.RunScript(None, "asis", ["T_ROB1"], True) is None
    assert errors == []


def test_start_controller_error_surfaces_and_returns_none(monkeypatch):
    session = FakeRws(error=ABBException("start boom", -1))
    component, errors, _, _ = _load(monkeypatch, "Cf_AbbStart", _connections("Cf_AbbStart", tasks=False))

    assert component.RunScript(session, "asis", None, True) is None
    assert len(errors) == 1 and "start boom" in errors[0]


# --- 3. Cf_AbbReset --------------------------------------------------------


def test_reset_fires_once_per_press(monkeypatch):
    session = FakeRws()
    component, errors, _, _ = _load(monkeypatch, "Cf_AbbReset", _connections("Cf_AbbReset"))

    assert component.RunScript(session, True) == "pp reset to main"
    assert session.resets == 1
    # Held True across a recompute -> no re-reset.
    assert component.RunScript(session, True) is None
    assert session.resets == 1
    # Release then press -> fires again.
    assert component.RunScript(session, False) is None
    assert component.RunScript(session, True) == "pp reset to main"
    assert session.resets == 2
    assert errors == []


def test_reset_none_controller_returns_none_without_error(monkeypatch):
    component, errors, _, _ = _load(monkeypatch, "Cf_AbbReset", _connections("Cf_AbbReset"))

    assert component.RunScript(None, True) is None
    assert errors == []


# --- 4. Cf_AbbStop (immediate; no arming, still edge-guarded) --------------


def test_stop_fires_once_per_press(monkeypatch):
    session = FakeRws()
    component, errors, _, _ = _load(monkeypatch, "Cf_AbbStop", _connections("Cf_AbbStop"))

    assert component.RunScript(session, True) == "stopped"
    assert session.stops == 1
    # Held True across a recompute -> no re-stop.
    assert component.RunScript(session, True) is None
    assert session.stops == 1
    # Release then press -> fires again.
    assert component.RunScript(session, False) is None
    assert component.RunScript(session, True) == "stopped"
    assert session.stops == 2
    assert errors == []


def test_stop_none_controller_returns_none_without_error(monkeypatch):
    component, errors, _, _ = _load(monkeypatch, "Cf_AbbStop", _connections("Cf_AbbStop"))

    assert component.RunScript(None, True) is None
    assert errors == []


# --- output-arity contract: declared outputs match each code None-shape ----

_NONE_SHAPE = {
    "Cf_AbbIoWrite": (None, None),
    "Cf_AbbStart": None,
    "Cf_AbbReset": None,
    "Cf_AbbStop": None,
}


@pytest.mark.parametrize("directory", sorted(_NONE_SHAPE))
def test_none_shape_matches_declared_output_arity(directory):
    expected = _NONE_SHAPE[directory]
    outputs = _output_names(directory)
    if expected is None:
        assert len(outputs) == 1
    else:
        assert isinstance(expected, tuple) and len(expected) == len(outputs)
