"""Tests for sync RWS session construction and sticky caching. No network."""

from typing import List

import pytest
from abb_robot_client import RobotWareVersion
from abb_robot_client.rws import ABBException

import compas_fab.backends.abb.connection as connection
from compas_fab.backends.abb.connection import build_session
from compas_fab.backends.abb.connection import cached_session
from compas_fab.backends.abb.errors import ControllerConnectionError


class FakeRws:
    def __init__(self, base_url: str, username: str, password: str, version: object) -> None:
        self.base_url = base_url
        self.username = username
        self.password = password
        self.version = version
        self.closed = False

    def close(self) -> None:
        self.closed = True


def _record_ctor(built: List[FakeRws]):
    def ctor(base_url: str, username: str, password: str, version: object) -> FakeRws:
        rws = FakeRws(base_url, username, password, version)
        built.append(rws)
        return rws

    return ctor


def test_build_session_wires_version_and_default_credentials(monkeypatch: pytest.MonkeyPatch) -> None:
    built: List[FakeRws] = []
    monkeypatch.setattr(connection, "RWS", _record_ctor(built))

    session = build_session("http://127.0.0.1:80", "RW6", None)

    assert session is built[0]
    assert (session.base_url, session.username, session.password, session.version) == (
        "http://127.0.0.1:80",
        "Default User",
        "robotics",
        RobotWareVersion.RW6,
    )


def test_build_session_wraps_construction_failure(monkeypatch: pytest.MonkeyPatch) -> None:
    def boom(base_url: str, username: str, password: str, version: object) -> FakeRws:
        raise ABBException("bad endpoint", -1)

    monkeypatch.setattr(connection, "RWS", boom)

    with pytest.raises(ControllerConnectionError):
        build_session("http://nope", "RW7", None)


def test_cached_session_reuses_on_unchanged_config(monkeypatch: pytest.MonkeyPatch) -> None:
    built: List[FakeRws] = []
    monkeypatch.setattr(connection, "RWS", _record_ctor(built))
    sticky: dict = {}

    first = cached_session(sticky, "slot", "http://host", "RW6", None)
    second = cached_session(sticky, "slot", "http://host", "RW6", None)

    assert first is second
    assert len(built) == 1  # a recompute did not reconnect


def test_cached_session_rebuilds_and_closes_on_config_change(monkeypatch: pytest.MonkeyPatch) -> None:
    built: List[FakeRws] = []
    monkeypatch.setattr(connection, "RWS", _record_ctor(built))
    sticky: dict = {}

    old = cached_session(sticky, "slot", "http://host-a", "RW6", None)
    new = cached_session(sticky, "slot", "http://host-b", "RW6", None)

    assert old is not new
    assert old.closed is True  # the superseded session was logged out
    assert len(built) == 2
