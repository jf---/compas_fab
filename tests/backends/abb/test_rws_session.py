"""Contract tests for the RWS -> ControllerSession adapter.

`abb_robot_client` is installed but no live controller is reachable, so a fake
RWS (recording calls, returning canned values, or raising the real
``ABBException``) drives every path. The factory is exercised hermetically by
monkeypatching ``RWS`` so no network call is made.
"""

from typing import List
from typing import Sequence
from typing import Tuple

import pytest
from abb_robot_client import RobotWareVersion
from abb_robot_client.rws import ABBException

import compas_fab.backends.abb.rws_session as rws_session
from compas_fab.backends.abb.errors import ControllerConnectionError
from compas_fab.backends.abb.errors import RwsCommandError
from compas_fab.backends.abb.rws_session import RwsControllerSession
from compas_fab.backends.abb.rws_session import build_rws_session_factory
from compas_fab.backends.abb.session import ControllerSession


class FakeRws:
    """Records delegated calls and returns canned values; can be made to fail."""

    def __init__(self, alive: bool = True) -> None:
        self.calls: List[Tuple[object, ...]] = []
        self._alive = alive

    def get_controller_state(self) -> str:
        self.calls.append(("get_controller_state",))
        return "motoron"

    def get_operation_mode(self) -> str:
        self.calls.append(("get_operation_mode",))
        if not self._alive:
            raise ABBException("controller unreachable", -1)
        return "AUTO"

    def get_execution_state(self) -> object:
        self.calls.append(("get_execution_state",))
        return "running"

    def get_digital_io(self, signal: str, network: str, unit: str) -> int:
        self.calls.append(("get_digital_io", signal, network, unit))
        return 1

    def set_digital_io(self, signal: str, value: int, network: str, unit: str) -> None:
        self.calls.append(("set_digital_io", signal, value, network, unit))

    def get_analog_io(self, signal: str, network: str, unit: str) -> float:
        self.calls.append(("get_analog_io", signal, network, unit))
        return 2.5

    def set_analog_io(self, signal: str, value: float, network: str, unit: str) -> None:
        self.calls.append(("set_analog_io", signal, value, network, unit))

    def start(self, cycle: str, tasks: Sequence[str]) -> None:
        self.calls.append(("start", cycle, tuple(tasks)))

    def stop(self) -> None:
        self.calls.append(("stop",))

    def resetpp(self) -> None:
        self.calls.append(("resetpp",))

    def close(self) -> None:
        self.calls.append(("close",))


def test_reads_delegate_and_return_client_values() -> None:
    fake = FakeRws()
    session = RwsControllerSession(fake)  # type: ignore[arg-type]

    assert session.get_controller_state() == "motoron"
    assert session.get_operation_mode() == "AUTO"
    assert session.get_execution_state() == "running"
    assert session.get_digital_io("DI1") == 1
    assert session.get_analog_io("AI1") == pytest.approx(2.5)


def test_io_defaults_mirror_the_rws_defaults() -> None:
    fake = FakeRws()
    session = RwsControllerSession(fake)  # type: ignore[arg-type]

    session.get_digital_io("DI1")
    session.set_digital_io("DO1", 1)

    assert ("get_digital_io", "DI1", "Local", "DRV_1") in fake.calls
    assert ("set_digital_io", "DO1", 1, "Local", "DRV_1") in fake.calls


def test_start_defaults_and_tasks_are_forwarded() -> None:
    fake = FakeRws()
    session = RwsControllerSession(fake)  # type: ignore[arg-type]

    session.start()
    session.start(cycle="once", tasks=["T_ROB1", "T_ROB2"])

    assert ("start", "asis", ("T_ROB1",)) in fake.calls
    assert ("start", "once", ("T_ROB1", "T_ROB2")) in fake.calls


def test_logout_closes_the_underlying_session() -> None:
    # The adapter must delegate logout to RWS.close so the requests.Session is
    # closed too, not merely logged out.
    fake = FakeRws()
    session = RwsControllerSession(fake)  # type: ignore[arg-type]

    session.logout()

    assert ("close",) in fake.calls


def test_is_alive_true_when_controller_answers() -> None:
    session = RwsControllerSession(FakeRws(alive=True))  # type: ignore[arg-type]
    assert session.is_alive() is True


def test_is_alive_false_when_controller_unreachable() -> None:
    session = RwsControllerSession(FakeRws(alive=False))  # type: ignore[arg-type]
    assert session.is_alive() is False


class _RaisingRws(FakeRws):
    def set_digital_io(self, signal: str, value: int, network: str, unit: str) -> None:
        raise ABBException("write refused", -1)


def test_client_abbexception_is_wrapped_as_rws_command_error() -> None:
    session = RwsControllerSession(_RaisingRws())  # type: ignore[arg-type]

    with pytest.raises(RwsCommandError) as info:
        session.set_digital_io("DO1", 1)
    # The named error carries the operation and chains the client cause.
    assert "set_digital_io" in str(info.value)
    assert isinstance(info.value.__cause__, ABBException)


def test_adapter_satisfies_the_controller_session_protocol() -> None:
    session = RwsControllerSession(FakeRws())  # type: ignore[arg-type]
    assert isinstance(session, ControllerSession)


def test_factory_builds_a_session_without_network(monkeypatch: pytest.MonkeyPatch) -> None:
    captured = {}

    def fake_rws_ctor(base_url: str, username: str, password: str, version: object) -> FakeRws:
        captured.update(base_url=base_url, username=username, password=password, version=version)
        return FakeRws()

    monkeypatch.setattr(rws_session, "RWS", fake_rws_ctor)
    factory = build_rws_session_factory("http://127.0.0.1:80", RobotWareVersion.RW6, "user", "pw")

    session = factory()

    assert isinstance(session, RwsControllerSession)
    assert captured == {"base_url": "http://127.0.0.1:80", "username": "user", "password": "pw", "version": RobotWareVersion.RW6}


def test_factory_wraps_construction_failure(monkeypatch: pytest.MonkeyPatch) -> None:
    def boom(base_url: str, username: str, password: str, version: object) -> FakeRws:
        raise ABBException("bad endpoint", -1)

    monkeypatch.setattr(rws_session, "RWS", boom)
    factory = build_rws_session_factory("http://nope", RobotWareVersion.RW7, "user", "pw")

    with pytest.raises(ControllerConnectionError):
        factory()
