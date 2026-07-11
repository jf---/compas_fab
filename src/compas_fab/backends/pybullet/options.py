from typing import Mapping
from typing import Optional

from compas_fab.backends.interfaces.planner_errors import PlannerCapabilityError
from compas_fab.backends.interfaces.planner_options import PlanMotionLegacyOptions
from compas_fab.backends.interfaces.planner_options import ResolvedPlannerOptions


class UnsupportedPlanMotionOptions:
    @classmethod
    def resolve(
        cls,
        legacy: PlanMotionLegacyOptions,
        native: Optional[Mapping[str, object]],
    ) -> ResolvedPlannerOptions:
        raise PlannerCapabilityError("Planner does not support plan_motion.")
