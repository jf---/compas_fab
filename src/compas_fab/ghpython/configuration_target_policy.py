from math import isfinite
from typing import List
from typing import Optional
from typing import Sequence
from typing import Tuple
from typing import Union

from compas_robots import Configuration  # type: ignore[import-untyped]

from compas_fab.backends.interfaces.planner_capabilities import ConfigurationTolerancePolicy
from compas_fab.robots import ConfigurationTarget

DEFAULT_PRISMATIC_TOLERANCE_METERS = 0.001  # Existing GH contract: 1 mm.
DEFAULT_REVOLUTE_TOLERANCE_RADIANS = 0.017453292519943295  # Existing GH contract: pi / 180.
PolicyInput = Optional[Union[str, ConfigurationTolerancePolicy]]


class InvalidConfigurationTolerancePolicyError(ValueError):
    """Raised when a tolerance policy is not a declared contract."""

    pass


class InvalidConfigurationToleranceInputError(ValueError):
    """Raised when connected tolerance values violate the target shape."""

    pass


def parse_configuration_tolerance_policy(value: PolicyInput) -> ConfigurationTolerancePolicy:
    """Parse a native Grasshopper value as an exact planner policy."""

    if type(value) is ConfigurationTolerancePolicy:
        return value
    if value is None:
        return ConfigurationTolerancePolicy.LEGACY_DEFAULTS
    if type(value) is not str:
        raise InvalidConfigurationTolerancePolicyError("Tolerance policy must be a declared policy or string.")
    try:
        return ConfigurationTolerancePolicy(value)
    except ValueError as error:
        raise InvalidConfigurationTolerancePolicyError("Unknown tolerance policy: {!r}.".format(value)) from error


def _validated(
    configuration: Configuration,
    values: Optional[Sequence[float]],
    name: str,
) -> Optional[List[float]]:
    if values is None:
        return None
    retained = list(values)
    if not retained or len(retained) != len(configuration.joint_values):
        raise InvalidConfigurationToleranceInputError("{} must match the non-empty joint vector.".format(name))
    if any(type(value) is not float or not isfinite(value) or value < 0.0 for value in retained):
        raise InvalidConfigurationToleranceInputError("{} must contain finite non-negative floats.".format(name))
    return retained


def resolve_configuration_tolerances(
    configuration: Configuration,
    above: Optional[Sequence[float]],
    below: Optional[Sequence[float]],
    policy: ConfigurationTolerancePolicy,
) -> Tuple[Optional[List[float]], Optional[List[float]]]:
    """Validate connected lists and resolve policy-controlled absence."""

    validated_above = _validated(configuration, above, "tolerance_above")
    validated_below = _validated(configuration, below, "tolerance_below")
    if type(policy) is not ConfigurationTolerancePolicy:
        raise InvalidConfigurationTolerancePolicyError("policy must be ConfigurationTolerancePolicy.")
    if policy is ConfigurationTolerancePolicy.PRESERVE_ABSENT:
        return validated_above, validated_below
    default_above, default_below = ConfigurationTarget.generate_default_tolerances(
        configuration,
        DEFAULT_PRISMATIC_TOLERANCE_METERS,
        DEFAULT_REVOLUTE_TOLERANCE_RADIANS,
    )
    return validated_above or default_above, validated_below or default_below
