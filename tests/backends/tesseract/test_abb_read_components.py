"""RunScript contract tests for the ABB READ/observe Grasshopper components.

These exercise the thin-adapter surface the backend unit tests cannot reach:
each component's ``code.py`` is exec'd under a faithful fake Grasshopper host and
``RunScript`` is driven against the REAL ABB controller backend (the arming
primitives, the registry, and a real ``ControllerOwner``). No network is ever
opened -- ``acquire_owner`` is lazy, and the owner reads are served by a canned
``FakeSession`` -- so the wiring, not a live controller, is what is pinned:

* ``Cf_AbbController`` builds a content-addressed ``ControllerId``, acquires a
  sticky-cached owner keyed to the component, reuses it across solves, applies
  the localhost/RW6 defaults, and never surfaces a secret in its summary.
* ``Cf_AbbControllerState`` / ``Cf_AbbIoRead`` route their reads through
  ``owner.submit_read`` (proving the serialized-worker wiring), honour the
  digital/analog toggle by Grasshopper connection state rather than Python
  truthiness, return the exact None-shape for a missing input with no error, and
  surface a named ``AbbControllerError`` via ``error(...)``.
"""

from pathlib import Path
from types import ModuleType
from types import SimpleNamespace
import json
import sys

import pytest

from compas_fab.backends.abb.arming import ControllerId
from compas_fab.backends.abb.controller_owner import ControllerOwner
from compas_fab.backends.abb.errors import RwsCommandError

COMPONENTS = Path(__file__).parents[3] / "src" / "compas_fab" / "ghpython" / "components_cpython"

_CLASS = {
    "Cf_AbbController": "AbbControllerComponent",
    "Cf_AbbControllerState": "AbbControllerStateComponent",
    "Cf_AbbIoRead": "AbbIoReadComponent",
}

# The zero-config controller identity every owner-level test is scoped to.
_CONTROLLER_ID = ControllerId.build("http://127.0.0.1:80", "RW6", "default")

# Canned controller reads the FakeSession answers with.
_CONTROLLER_STATE = "motoron"
_OPERATION_MODE = "AUTO"
_EXECUTION_STATE = "running"
_DIGITAL_VALUE = 1
_ANALOG_VALUE = 3.5

# The RobotStudio VC default password: it must NEVER appear in a summary. The
# summary is built from the (safe) ControllerId, never from resolved credentials.
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


# --- canned session + owner lifecycle --------------------------------------


class FakeSession:
    """Canned ``ControllerSession`` recording the reads the owner drives through it.

    A single ``error`` (an ``AbbControllerError``), if set, is raised by every
    read so the component's named-error surface can be pinned. ``logout`` is
    recorded so an owner close is observable.
    """

    def __init__(self, error=None):
        self._error = error
        self.digital_signals = []
        self.analog_signals = []
        self.logged_out = False

    def _read(self, value):
        if self._error is not None:
            raise self._error
        return value

    def get_controller_state(self):
        return self._read(_CONTROLLER_STATE)

    def get_operation_mode(self):
        return self._read(_OPERATION_MODE)

    def get_execution_state(self):
        return self._read(_EXECUTION_STATE)

    def get_digital_io(self, signal, network="Local", unit="DRV_1"):
        self.digital_signals.append(signal)
        return self._read(_DIGITAL_VALUE)

    def get_analog_io(self, signal, network="Local", unit="DRV_1"):
        self.analog_signals.append(signal)
        return self._read(_ANALOG_VALUE)

    def logout(self):
        self.logged_out = True


@pytest.fixture
def make_owner():
    """Yield a factory that builds real owners over a canned session and closes them."""
    owners = []

    def _make(session):
        owner = ControllerOwner(_CONTROLLER_ID, lambda: session)
        owners.append(owner)
        return owner

    yield _make
    for owner in owners:
        owner.close()


@pytest.fixture
def owner_sticky():
    """Yield a sticky cache and close any owners a component acquired into it."""
    cache = {}
    yield cache
    for value in list(cache.values()):
        if isinstance(value, ControllerOwner):
            value.close()


# --- 1. Cf_AbbController (real id + registry; no network) ------------------


def test_controller_creates_and_caches_lazy_owner(monkeypatch, owner_sticky):
    endpoint = "http://192.168.125.1:80"
    component, errors, sticky, host = _load(monkeypatch, "Cf_AbbController", _connections("Cf_AbbController"), owner_sticky)

    controller, summary = component.RunScript(endpoint, "RW6", None)

    assert errors == []
    assert isinstance(controller, ControllerOwner)
    # The owner is the exact object cached under this component's slot.
    key = "{}::abb_controller".format(id(host))
    assert sticky[key] is controller
    # Its identity is the content address of the connected inputs; a None handle
    # is tagged "default" so the id is deterministic without naming a secret.
    assert controller.controller_id == ControllerId.build(endpoint, "RW6", "default")
    # Summary carries endpoint/version/handle-name and never a resolved secret.
    assert endpoint in summary and "RW6" in summary and "default" in summary
    assert _VC_DEFAULT_PASSWORD not in summary


def test_controller_reuses_cached_owner_across_solves(monkeypatch, owner_sticky):
    endpoint = "http://192.168.125.1:80"
    component, errors, sticky, _ = _load(monkeypatch, "Cf_AbbController", _connections("Cf_AbbController"), owner_sticky)

    first, _ = component.RunScript(endpoint, "RW6", None)
    second, _ = component.RunScript(endpoint, "RW6", None)

    assert errors == []
    # Same identity -> the very same owner is returned, not a reconnect storm.
    assert second is first
    assert second.controller_id == first.controller_id


def test_controller_unconnected_endpoint_defaults_to_localhost(monkeypatch, owner_sticky):
    component, errors, _, _ = _load(
        monkeypatch,
        "Cf_AbbController",
        _connections("Cf_AbbController", endpoint=False),
        owner_sticky,
    )

    # A non-default endpoint is supplied on an unwired input; connection state,
    # not the value, must select the localhost virtual-controller default.
    controller, summary = component.RunScript("http://ignored-because-unconnected:80", "RW6", None)

    assert errors == []
    assert controller.controller_id.endpoint == "http://127.0.0.1:80"
    assert "http://127.0.0.1:80" in summary


def test_controller_unknown_rw_version_surfaces_error_and_returns_two_none(monkeypatch, owner_sticky):
    component, errors, sticky, host = _load(monkeypatch, "Cf_AbbController", _connections("Cf_AbbController"), owner_sticky)

    # A version outside RW6/RW7 drives the session factory to raise a named
    # AbbControllerError before any owner is created.
    result = component.RunScript("http://192.168.125.1:80", "RW9", None)

    assert result == (None, None)
    assert len(errors) == 1 and "RW9" in errors[0]
    # No half-built owner leaks into the slot on a resolution failure.
    assert "{}::abb_controller".format(id(host)) not in sticky


# --- 2. Cf_AbbControllerState (real owner + canned session) ----------------


def test_state_reads_route_through_submit_read(monkeypatch, make_owner):
    session = FakeSession()
    owner = make_owner(session)
    component, errors, _, _ = _load(monkeypatch, "Cf_AbbControllerState", _connections("Cf_AbbControllerState"))

    result = component.RunScript(owner)

    assert errors == []
    assert result == (_CONTROLLER_STATE, _OPERATION_MODE, _EXECUTION_STATE)


def test_state_none_controller_returns_three_none_without_error(monkeypatch, make_owner):
    component, errors, _, _ = _load(monkeypatch, "Cf_AbbControllerState", _connections("Cf_AbbControllerState"))

    assert component.RunScript(None) == (None, None, None)
    assert errors == []


def test_state_backend_error_surfaces_and_returns_three_none(monkeypatch, make_owner):
    owner = make_owner(FakeSession(error=RwsCommandError("read boom")))
    component, errors, _, _ = _load(monkeypatch, "Cf_AbbControllerState", _connections("Cf_AbbControllerState"))

    result = component.RunScript(owner)

    assert result == (None, None, None)
    assert len(errors) == 1 and "read boom" in errors[0]


# --- 3. Cf_AbbIoRead (real owner + canned session) -------------------------


def test_io_read_digital_by_default(monkeypatch, make_owner):
    session = FakeSession()
    owner = make_owner(session)
    component, errors, _, _ = _load(monkeypatch, "Cf_AbbIoRead", _connections("Cf_AbbIoRead"))

    value = component.RunScript(owner, "DO_1", False)

    assert errors == []
    assert value == _DIGITAL_VALUE
    # The digital read was driven with the exact signal; analog was never touched.
    assert session.digital_signals == ["DO_1"]
    assert session.analog_signals == []


def test_io_read_analog_when_toggle_true(monkeypatch, make_owner):
    session = FakeSession()
    owner = make_owner(session)
    component, errors, _, _ = _load(monkeypatch, "Cf_AbbIoRead", _connections("Cf_AbbIoRead"))

    value = component.RunScript(owner, "AO_1", True)

    assert errors == []
    assert value == _ANALOG_VALUE
    assert session.analog_signals == ["AO_1"]
    assert session.digital_signals == []


def test_io_read_unconnected_analog_reads_digital(monkeypatch, make_owner):
    session = FakeSession()
    owner = make_owner(session)
    component, errors, _, _ = _load(monkeypatch, "Cf_AbbIoRead", _connections("Cf_AbbIoRead", analog=False))

    # analog=True is supplied but the toggle is unwired; connection state, not
    # the value, must keep the read digital.
    value = component.RunScript(owner, "DO_1", True)

    assert errors == []
    assert value == _DIGITAL_VALUE
    assert session.digital_signals == ["DO_1"]
    assert session.analog_signals == []


def test_io_read_none_controller_returns_none_without_error(monkeypatch, make_owner):
    component, errors, _, _ = _load(monkeypatch, "Cf_AbbIoRead", _connections("Cf_AbbIoRead"))

    assert component.RunScript(None, "DO_1", False) is None
    assert errors == []


def test_io_read_blank_signal_returns_none_without_error(monkeypatch, make_owner):
    session = FakeSession()
    owner = make_owner(session)
    component, errors, _, _ = _load(monkeypatch, "Cf_AbbIoRead", _connections("Cf_AbbIoRead"))

    assert component.RunScript(owner, "", False) is None
    assert component.RunScript(owner, "   ", False) is None
    assert errors == []
    # A blank signal never reaches the session, so the owner never connects.
    assert session.digital_signals == [] and session.analog_signals == []


def test_io_read_backend_error_surfaces_and_returns_none(monkeypatch, make_owner):
    owner = make_owner(FakeSession(error=RwsCommandError("io boom")))
    component, errors, _, _ = _load(monkeypatch, "Cf_AbbIoRead", _connections("Cf_AbbIoRead"))

    value = component.RunScript(owner, "DO_1", False)

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
