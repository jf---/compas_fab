"""
Internal implementation of the planner backend interface for MoveIt!

"""

from compas_fab.backends.interfaces.planner import PlannerInterface
from compas_fab.backends.interfaces.planner_capabilities import ConfigurationTolerancePolicy
from compas_fab.backends.interfaces.planner_capabilities import PlannerCapabilities
from compas_fab.backends.interfaces.planner_capabilities import PlannerImplementationId
from compas_fab.backends.interfaces.planner_operation import PlannerOperation
from compas_fab.backends.ros.backend_features import MoveItSetRobotCell
from compas_fab.backends.ros.backend_features import MoveItSetRobotCellState
from compas_fab.backends.ros.backend_features.move_it_check_collision import MoveItCheckCollision
from compas_fab.backends.ros.backend_features.move_it_forward_kinematics import MoveItForwardKinematics
from compas_fab.backends.ros.backend_features.move_it_inverse_kinematics import MoveItInverseKinematics
from compas_fab.backends.ros.backend_features.move_it_plan_cartesian_motion import MoveItPlanCartesianMotion
from compas_fab.backends.ros.backend_features.move_it_plan_motion import MoveItPlanMotion
from compas_fab.backends.ros.backend_features.move_it_planning_scene import MoveItPlanningScene
from compas_fab.backends.ros.backend_features.move_it_reset_planning_scene import MoveItResetPlanningScene
from compas_fab.backends.ros.client import RosClient
from compas_fab.backends.ros.options import MoveItPlanMotionOptions

__all__ = [
    "MoveItPlanner",
]


class MoveItPlanner(
    MoveItPlanningScene,
    MoveItSetRobotCell,
    MoveItSetRobotCellState,
    MoveItCheckCollision,
    MoveItForwardKinematics,
    MoveItInverseKinematics,
    MoveItPlanMotion,
    MoveItPlanCartesianMotion,
    MoveItResetPlanningScene,
    PlannerInterface,
):
    """Implement the planner backend interface based on MoveIt!"""

    implementation_id = PlannerImplementationId.build("compas_fab.moveit/v1")
    capabilities = PlannerCapabilities.build(
        implementation_id,
        tuple(PlannerOperation),
        ConfigurationTolerancePolicy.LEGACY_DEFAULTS,
    )
    plan_motion_options = MoveItPlanMotionOptions

    def __init__(self, client: RosClient) -> None:
        # Initialize all mixins
        super(MoveItPlanner, self).__init__()  # type: ignore[no-untyped-call]

        self._client = client
        self._current_rigid_body_hashes: dict[str, bytes] = {}
        self._current_tool_hashes: dict[str, bytes] = {}

        # Reset the planning scene in the backend to clear all objects left by previous runs
        self.reset_planning_scene()  # type: ignore[no-untyped-call]
