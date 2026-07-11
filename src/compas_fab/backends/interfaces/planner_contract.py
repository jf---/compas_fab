from typing import Protocol

from .planner_capabilities import PlannerCapabilities
from .planner_capabilities import PlannerImplementationId
from .planner_options import PlanMotionOptionsAdapter


class PlannerContract(Protocol):
    @property
    def implementation_id(self) -> PlannerImplementationId: ...

    @property
    def capabilities(self) -> PlannerCapabilities: ...

    @property
    def plan_motion_options(self) -> type[PlanMotionOptionsAdapter]: ...
