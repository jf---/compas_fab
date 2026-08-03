"""Tests for credential and RobotWare-version resolution."""

import pytest
from abb_robot_client import RobotWareVersion

from compas_fab.backends.abb.credentials import RobotStudioDefaultProvider
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
