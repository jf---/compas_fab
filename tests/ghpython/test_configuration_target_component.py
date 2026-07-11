import pytest
from compas_robots import Configuration

from compas_fab.backends.interfaces.planner_capabilities import ConfigurationTolerancePolicy
from compas_fab.ghpython.configuration_target_policy import InvalidConfigurationToleranceInputError
from compas_fab.ghpython.configuration_target_policy import InvalidConfigurationTolerancePolicyError
from compas_fab.robots import ConfigurationTarget
from tests.ghpython.component_harness import load_component


def component(monkeypatch, **connections):
    defaults = {
        "target_configuration": True,
        "tolerance_above": False,
        "tolerance_below": False,
        "tolerance_policy": False,
    }
    defaults.update(connections)
    loaded, _ = load_component(
        monkeypatch,
        "Cf_ConfigurationTarget",
        "ConfigurationTargetComponent",
        defaults,
    )
    return loaded


def test_unwired_empty_lists_keep_legacy_defaults(monkeypatch) -> None:
    configuration = Configuration.from_revolute_values([0.0])
    expected = ConfigurationTarget.generate_default_tolerances(configuration, 0.001, 0.017453292519943295)
    target = component(monkeypatch).RunScript(configuration, [], [], "")
    assert target.tolerance_above == expected[0]
    assert target.tolerance_below == expected[1]


def test_explicit_lists_and_connected_empty_shape(monkeypatch) -> None:
    configured = component(monkeypatch, tolerance_above=True, tolerance_below=True)
    target = configured.RunScript(Configuration.from_revolute_values([0.0]), [0.2], [0.3], "")
    assert target.tolerance_above == [0.2]
    assert target.tolerance_below == [0.3]
    policy_path = component(monkeypatch, tolerance_above=True, tolerance_below=True, tolerance_policy=True)
    with pytest.raises(InvalidConfigurationToleranceInputError):
        policy_path.RunScript(Configuration.from_revolute_values([0.0]), [], [0.3], "legacy_defaults")


def test_preserve_absent_and_planner_policy(monkeypatch) -> None:
    configured = component(monkeypatch, tolerance_policy=True)
    absent = configured.RunScript(Configuration.from_revolute_values([0.0]), [], [], "preserve_absent")
    assert absent.tolerance_above is None
    planner_policy = ConfigurationTolerancePolicy.PRESERVE_ABSENT
    from_planner = configured.RunScript(Configuration.from_revolute_values([0.0]), [], [], planner_policy)
    assert from_planner.tolerance_below is None


def test_unknown_mode_fails_named(monkeypatch) -> None:
    configured = component(monkeypatch, tolerance_policy=True)
    with pytest.raises(InvalidConfigurationTolerancePolicyError):
        configured.RunScript(Configuration.from_revolute_values([0.0]), [], [], "invented")
