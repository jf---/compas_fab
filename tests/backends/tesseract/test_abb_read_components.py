"""RunScript contract tests for the ABB READ/observe Grasshopper components.

These exercise the thin-adapter surface the backend unit tests cannot reach: each
component's ``code.py`` is exec'd under a faithful fake Grasshopper host and
``RunScript`` is driven against the sync-direct ABB backend. No network is ever
opened -- ``connection.RWS`` is faked so session construction builds a canned
``FakeRws``, and the READ components call that session's methods directly:

* ``Cf_AbbController`` builds a sync ``RWS`` through ``cached_session``, caches it
  in sticky keyed to the component, reuses it across solves, applies the
  localhost/RW6 defaults, and never surfaces a secret in its summary.
* ``Cf_AbbControllerState`` / ``Cf_AbbIoRead`` call the session's read methods
  directly (no owner, no worker thread), honour the digital/analog toggle by
  Grasshopper connection state rather than Python truthiness, return the exact
  None-shape for a missing input with no error, and surface a controller error
  (an ``ABBException``) via ``error(...)``.
"""

from pathlib import Path
from types import ModuleType
from types import SimpleNamespace
import json
import sys

import pytest
from abb_robot_client.rws import ABBException

import compas_fab.backends.abb.connection as connection

COMPONENTS = Path(__file__).parents[3] / "src" / "compas_fab" / "ghpython" / "components_cpython"

_CLASS = {
    "Cf_AbbController": "AbbControllerComponent",
    "Cf_AbbControllerState": "AbbControllerStateComponent",
    "Cf_AbbIoRead": "AbbIoReadComponent",
}

# Canned controller reads the FakeRws answers with.
_CONTROLLER_STATE = "motoron"
_OPERATION_MODE = "AUTO"
_EXECUTION_STATE = "running"
_DIGITAL_VALUE = 1
_ANALOG_VALUE = 3.5

# The RobotStudio VC default password: it must NEVER appear in a summary. The
# summary is built from the safe addressing triple, never from resolved creds.
_VC_DEFAULT_PASSWORD = "robotics"


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
    """Exec a component under a faithful fake host and capture ``error(...)``.

    Grasshopper, Rhino, System, and scriptcontext are faked because none exist
    outside Rhino; ``compas_ghpython`` is faked so ``error`` is recorded and
    ``create_id`` is deterministic; ``ensure_value_list``/``ensure_boolean_toggle``
    are neutralised because they mutate a live canvas the fake host does not
    provide. Returns the component, the captured error list, the sticky cache,
    and the host (whose identity seeds ``create_id``).
    """
    errors = []
    sticky = {} if sticky is None else sticky
    grasshopper = ModuleType("Grasshopper")
    grasshopper.Kernel = SimpleNamespace(GH_ScriptInstance=object)
    compas_ghpython = ModuleType("compas_ghpython")
    compas_ghpython.error = lambda component, message: errors.append(message)
    compas_ghpython.create_id = lambda component, name: "{}::{}".format(id(component), name)
    scriptcontext = ModuleType("scriptcontext")
    scriptcontext.sticky = sticky
    monkeypatch.setitem(sys.modules, "Grasshopper", grasshopper)
    monkeypatch.setitem(sys.modules, "Rhino", ModuleType("Rhino"))
    monkeypatch.setitem(sys.modules, "System", ModuleType("System"))
    monkeypatch.setitem(sys.modules, "compas_ghpython", compas_ghpython)
    monkeypatch.setitem(sys.modules, "scriptcontext", scriptcontext)
    monkeypatch.setattr("compas_fab.ghpython.ensure_value_list", lambda *args, **kwargs: None, raising=False)
    monkeypatch.setattr("compas_fab.ghpython.ensure_boolean_toggle", lambda *args, **kwargs: None, raising=False)
    source = COMPONENTS / directory / "code.py"
    namespace = {"__name__": "test_" + directory, "__file__": str(source)}
    exec(compile(source.read_text(encoding="utf-8"), str(source), "exec"), namespace)
    host = _FakeComponent(connections)
    namespace["ghenv"] = SimpleNamespace(Component=host)
    return namespace[_CLASS[directory]](), errors, sticky, host


# --- canned sync RWS session -----------------------------------------------


class FakeRws:
    """Canned sync ``RWS`` recording the reads a component drives through it.

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
        return _CONTROLLER_STATE

    def get_operation_mode(self):
        self._maybe_raise()
        return _OPERATION_MODE

    def get_execution_state(self):
        self._maybe_raise()
        return _EXECUTION_STATE

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


def _fake_rws_ctor(built):
    """Return an ``RWS`` stand-in that records the addressing triple, opens no socket."""

    def ctor(base_url, username, password, version):
        rws = FakeRws()
        rws.base_url = base_url
        rws.username = username
        rws.password = password
        rws.version = version
        built.append(rws)
        return rws

    return ctor


# --- 1. Cf_AbbController (real cached_session; no network) ------------------


def test_controller_builds_and_caches_session(monkeypatch):
    built = []
    monkeypatch.setattr(connection, "RWS", _fake_rws_ctor(built))
    endpoint = "http://192.168.125.1:80"
    component, errors, sticky, host = _load(monkeypatch, "Cf_AbbController", _connections("Cf_AbbController"))

    controller, summary = component.RunScript(endpoint, "RW6", None)

    assert errors == []
    # The one built session is returned and cached under this component's slot.
    assert controller is built[0]
    key = "{}::abb_session".format(id(host))
    assert sticky[key][1] is controller
    # It was constructed against the connected endpoint and the VC defaults.
    assert controller.base_url == endpoint
    assert (controller.username, controller.password) == ("Default User", "robotics")
    # Summary carries endpoint/version/handle-name and never a resolved secret.
    assert endpoint in summary and "RW6" in summary and "default" in summary
    assert _VC_DEFAULT_PASSWORD not in summary


def test_controller_reuses_cached_session_across_solves(monkeypatch):
    built = []
    monkeypatch.setattr(connection, "RWS", _fake_rws_ctor(built))
    endpoint = "http://192.168.125.1:80"
    component, errors, _, _ = _load(monkeypatch, "Cf_AbbController", _connections("Cf_AbbController"))

    first, _ = component.RunScript(endpoint, "RW6", None)
    second, _ = component.RunScript(endpoint, "RW6", None)

    assert errors == []
    # Same config -> the very same session is returned, not a reconnect storm.
    assert second is first
    assert len(built) == 1


def test_controller_unconnected_endpoint_defaults_to_localhost(monkeypatch):
    built = []
    monkeypatch.setattr(connection, "RWS", _fake_rws_ctor(built))
    component, errors, _, _ = _load(
        monkeypatch,
        "Cf_AbbController",
        _connections("Cf_AbbController", endpoint=False),
    )

    # A non-default endpoint is supplied on an unwired input; connection state,
    # not the value, must select the localhost virtual-controller default.
    controller, summary = component.RunScript("http://ignored-because-unconnected:80", "RW6", None)

    assert errors == []
    assert controller.base_url == "http://127.0.0.1:80"
    assert "http://127.0.0.1:80" in summary


def test_controller_unknown_rw_version_surfaces_error_and_returns_two_none(monkeypatch):
    built = []
    monkeypatch.setattr(connection, "RWS", _fake_rws_ctor(built))
    component, errors, sticky, host = _load(monkeypatch, "Cf_AbbController", _connections("Cf_AbbController"))

    # A version outside RW6/RW7 drives the session factory to raise a named
    # AbbControllerError before any session is constructed.
    result = component.RunScript("http://192.168.125.1:80", "RW9", None)

    assert result == (None, None)
    assert len(errors) == 1 and "RW9" in errors[0]
    assert built == []
    # No half-built session leaks into the slot on a resolution failure.
    assert "{}::abb_session".format(id(host)) not in sticky


# --- 2. Cf_AbbControllerState (canned session, direct reads) ---------------


def test_state_reads_call_session_directly(monkeypatch):
    component, errors, _, _ = _load(monkeypatch, "Cf_AbbControllerState", _connections("Cf_AbbControllerState"))

    result = component.RunScript(FakeRws())

    assert errors == []
    assert result == (_CONTROLLER_STATE, _OPERATION_MODE, _EXECUTION_STATE)


def test_state_none_controller_returns_three_none_without_error(monkeypatch):
    component, errors, _, _ = _load(monkeypatch, "Cf_AbbControllerState", _connections("Cf_AbbControllerState"))

    assert component.RunScript(None) == (None, None, None)
    assert errors == []


def test_state_controller_error_surfaces_and_returns_three_none(monkeypatch):
    component, errors, _, _ = _load(monkeypatch, "Cf_AbbControllerState", _connections("Cf_AbbControllerState"))

    result = component.RunScript(FakeRws(error=ABBException("read boom", -1)))

    assert result == (None, None, None)
    assert len(errors) == 1 and "read boom" in errors[0]


# --- 3. Cf_AbbIoRead (canned session, direct reads) ------------------------


def test_io_read_digital_by_default(monkeypatch):
    session = FakeRws()
    component, errors, _, _ = _load(monkeypatch, "Cf_AbbIoRead", _connections("Cf_AbbIoRead"))

    value = component.RunScript(session, "DO_1", False)

    assert errors == []
    assert value == _DIGITAL_VALUE
    # The digital read was driven with the exact signal; analog was never touched.
    assert session.digital_reads == ["DO_1"]
    assert session.analog_reads == []


def test_io_read_analog_when_toggle_true(monkeypatch):
    session = FakeRws()
    component, errors, _, _ = _load(monkeypatch, "Cf_AbbIoRead", _connections("Cf_AbbIoRead"))

    value = component.RunScript(session, "AO_1", True)

    assert errors == []
    assert value == _ANALOG_VALUE
    assert session.analog_reads == ["AO_1"]
    assert session.digital_reads == []


def test_io_read_unconnected_analog_reads_digital(monkeypatch):
    session = FakeRws()
    component, errors, _, _ = _load(monkeypatch, "Cf_AbbIoRead", _connections("Cf_AbbIoRead", analog=False))

    # analog=True is supplied but the toggle is unwired; connection state, not
    # the value, must keep the read digital.
    value = component.RunScript(session, "DO_1", True)

    assert errors == []
    assert value == _DIGITAL_VALUE
    assert session.digital_reads == ["DO_1"]
    assert session.analog_reads == []


def test_io_read_none_controller_returns_none_without_error(monkeypatch):
    component, errors, _, _ = _load(monkeypatch, "Cf_AbbIoRead", _connections("Cf_AbbIoRead"))

    assert component.RunScript(None, "DO_1", False) is None
    assert errors == []


def test_io_read_blank_signal_returns_none_without_error(monkeypatch):
    session = FakeRws()
    component, errors, _, _ = _load(monkeypatch, "Cf_AbbIoRead", _connections("Cf_AbbIoRead"))

    assert component.RunScript(session, "", False) is None
    assert component.RunScript(session, "   ", False) is None
    assert errors == []
    # A blank signal never reaches the session.
    assert session.digital_reads == [] and session.analog_reads == []


def test_io_read_controller_error_surfaces_and_returns_none(monkeypatch):
    component, errors, _, _ = _load(monkeypatch, "Cf_AbbIoRead", _connections("Cf_AbbIoRead"))

    value = component.RunScript(FakeRws(error=ABBException("io boom", -1)), "DO_1", False)

    assert value is None
    assert len(errors) == 1 and "io boom" in errors[0]


# --- output-arity contract: declared outputs match each code None-shape ----

_NONE_SHAPE = {
    "Cf_AbbController": (None, None),
    "Cf_AbbControllerState": (None, None, None),
    "Cf_AbbIoRead": None,
}


@pytest.mark.parametrize("directory", sorted(_NONE_SHAPE))
def test_none_shape_matches_declared_output_arity(directory):
    expected = _NONE_SHAPE[directory]
    outputs = _output_names(directory)
    if expected is None:
        assert len(outputs) == 1
    else:
        assert isinstance(expected, tuple) and len(expected) == len(outputs)
