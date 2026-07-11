from __future__ import annotations

from enum import Enum
from typing import Sequence
from typing import Tuple

from attrs import define

from .planner_errors import InvalidPlannerCapabilitiesError
from .planner_operation import PlannerOperation


class ConfigurationTolerancePolicy(Enum):
    LEGACY_DEFAULTS = "legacy_defaults"
    PRESERVE_ABSENT = "preserve_absent"


@define(frozen=True, slots=True)
class PlannerImplementationId:
    value: str

    @classmethod
    def build(cls, value: str) -> "PlannerImplementationId":
        return cls(value)

    def __attrs_post_init__(self) -> None:
        if type(self.value) is not str or not self.value:
            raise InvalidPlannerCapabilitiesError("Implementation ID must be non-empty str.")


@define(frozen=True, slots=True)
class PlannerCapabilities:
    implementation_id: PlannerImplementationId
    operations: Tuple[PlannerOperation, ...]
    configuration_tolerance_policy: ConfigurationTolerancePolicy

    @classmethod
    def build(
        cls,
        implementation_id: PlannerImplementationId,
        operations: Sequence[PlannerOperation],
        tolerance_policy: ConfigurationTolerancePolicy,
    ) -> "PlannerCapabilities":
        return cls(implementation_id, tuple(operations), tolerance_policy)

    def __attrs_post_init__(self) -> None:
        invalid = (
            type(self.implementation_id) is not PlannerImplementationId
            or type(self.operations) is not tuple
            or not self.operations
            or type(self.configuration_tolerance_policy) is not ConfigurationTolerancePolicy
        )
        if invalid:
            raise InvalidPlannerCapabilitiesError("Planner capabilities are inconsistent.")
        if any(type(operation) is not PlannerOperation for operation in self.operations):
            raise InvalidPlannerCapabilitiesError("Planner operations must be exact PlannerOperation values.")
        if len(set(self.operations)) != len(self.operations):
            raise InvalidPlannerCapabilitiesError("Planner operations must be unique.")
