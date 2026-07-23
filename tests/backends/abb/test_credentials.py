"""Tests for credential resolution and session-factory assembly.

The zero-config RobotStudio path, environment-backed handles for physical
controllers, RobotWare version mapping, and that secrets are resolved (not
stored) are all pinned here. No network is touched.
"""

from typing import Callable

import pytest
from abb_robot_client import RobotWareVersion

import compas_fab.backends.abb.credentials as credentials
from compas_fab.backends.abb.credentials import RobotStudioDefaultProvider
from compas_fab.backends.abb.credentials import build_controller_session_factory
from compas_fab.backends.abb.credentials import robotware_version
from compas_fab.backends.abb.errors import CredentialResolutionError
from compas_fab.backends.abb.errors import UnknownRobotWareVersionError


def test_absent_handle_resolves_to_robotstudio_defaults() -> None:
    provider = RobotStudioDefaultProvider()
    assert provider.resolve(None) == ("Default User", "robotics")
    assert provider.resolve("   ") == ("Default User", "robotics")


def test_named_handle_resolves_from_environment(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("ABB_CELL1_USERNAME", "operator")
    monkeypatch.setenv("ABB_CELL1_PASSWORD", "s3cret")

    assert RobotStudioDefaultProvider().resolve("cell1") == ("operator", "s3cret")


def test_named_handle_with_missing_env_raises(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv("ABB_CELL2_USERNAME", raising=False)
    monkeypatch.delenv("ABB_CELL2_PASSWORD", raising=False)

    with pytest.raises(CredentialResolutionError):
        RobotStudioDefaultProvider().resolve("cell2")


def test_robotware_version_maps_names_case_insensitively() -> None:
    assert robotware_version("RW6") is RobotWareVersion.RW6
    assert robotware_version("rw7") is RobotWareVersion.RW7


def test_robotware_version_rejects_unknown() -> None:
    with pytest.raises(UnknownRobotWareVersionError):
        robotware_version("RW5")


def test_factory_wires_resolved_version_and_credentials(monkeypatch: pytest.MonkeyPatch) -> None:
    captured = {}

    def fake_build(endpoint: str, version: object, username: str, password: str) -> Callable[[], object]:
        captured.update(endpoint=endpoint, version=version, username=username, password=password)
        return lambda: object()

    monkeypatch.setattr(credentials, "build_rws_session_factory", fake_build)

    factory = build_controller_session_factory("http://127.0.0.1:80", "RW6", None)

    assert callable(factory)
    assert captured == {
        "endpoint": "http://127.0.0.1:80",
        "version": RobotWareVersion.RW6,
        "username": "Default User",
        "password": "robotics",
    }


def test_factory_uses_a_supplied_provider(monkeypatch: pytest.MonkeyPatch) -> None:
    class FixedProvider:
        def resolve(self, handle: object) -> object:
            return ("alice", "pw")

    captured = {}

    def fake_build(endpoint: str, version: object, username: str, password: str) -> Callable[[], object]:
        captured.update(username=username, password=password)
        return lambda: object()

    monkeypatch.setattr(credentials, "build_rws_session_factory", fake_build)

    build_controller_session_factory("http://host", "RW7", "ignored", FixedProvider())

    assert captured == {"username": "alice", "password": "pw"}
