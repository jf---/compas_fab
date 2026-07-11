from typing import Protocol
from typing import Type

from .planner_capabilities import PlannerCapabilities
from .planner_capabilities import PlannerImplementationId
from .planner_options import PlanMotionOptionsAdapter


class PlannerContract(Protocol):
    implementation_id: PlannerImplementationId
    capabilities: PlannerCapabilities
    plan_motion_options: Type[PlanMotionOptionsAdapter]
