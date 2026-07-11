from math import inf
from math import nan

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


def test_connected_policy_accepts_explicit_tolerance_lists(monkeypatch) -> None:
    configured = component(monkeypatch, tolerance_above=True, tolerance_below=True, tolerance_policy=True)

    target = configured.RunScript(
        Configuration.from_revolute_values([0.0]),
        [0.2],
        [0.3],
        "legacy_defaults",
    )

    assert target.tolerance_above == [0.2]
    assert target.tolerance_below == [0.3]


@pytest.mark.parametrize("invalid_tolerance", (-0.1, nan, inf, -inf))
@pytest.mark.parametrize("input_name", ("tolerance_above", "tolerance_below"))
def test_connected_policy_rejects_invalid_tolerances_through_run_script(
    monkeypatch,
    invalid_tolerance: float,
    input_name: str,
) -> None:
    configured = component(monkeypatch, tolerance_above=True, tolerance_below=True, tolerance_policy=True)
    above = [invalid_tolerance] if input_name == "tolerance_above" else [0.2]
    below = [invalid_tolerance] if input_name == "tolerance_below" else [0.3]

    with pytest.raises(InvalidConfigurationToleranceInputError):
        configured.RunScript(
            Configuration.from_revolute_values([0.0]),
            above,
            below,
            "legacy_defaults",
        )


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
