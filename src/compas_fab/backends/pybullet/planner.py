from typing import TYPE_CHECKING

from compas_fab.backends.interfaces.planner import PlannerInterface
from compas_fab.backends.interfaces.planner_capabilities import ConfigurationTolerancePolicy
from compas_fab.backends.interfaces.planner_capabilities import PlannerCapabilities
from compas_fab.backends.interfaces.planner_capabilities import PlannerImplementationId
from compas_fab.backends.interfaces.planner_operation import PlannerOperation
from compas_fab.backends.pybullet.backend_features import PyBulletCheckCollision
from compas_fab.backends.pybullet.backend_features import PyBulletForwardKinematics
from compas_fab.backends.pybullet.backend_features import PyBulletInverseKinematics
from compas_fab.backends.pybullet.backend_features import PyBulletPlanCartesianMotion
from compas_fab.backends.pybullet.backend_features import PyBulletSetRobotCell
from compas_fab.backends.pybullet.backend_features import PyBulletSetRobotCellState
from compas_fab.backends.pybullet.options import UnsupportedPlanMotionOptions

if TYPE_CHECKING:
    from compas_fab.backends import PyBulletClient

__all__ = [
    "PyBulletPlanner",
]


class PyBulletPlanner(
    PyBulletCheckCollision,
    PyBulletForwardKinematics,
    PyBulletInverseKinematics,
    PyBulletPlanCartesianMotion,
    PyBulletSetRobotCell,
    PyBulletSetRobotCellState,
    PlannerInterface,
):
    """Implement the planner backend interface for PyBullet."""

    implementation_id = PlannerImplementationId.build("compas_fab.pybullet/v1")
    capabilities = PlannerCapabilities.build(
        implementation_id,
        (
            PlannerOperation.INVERSE_KINEMATICS,
            PlannerOperation.PLAN_CARTESIAN_MOTION,
            PlannerOperation.CHECK_COLLISION,
        ),
        ConfigurationTolerancePolicy.LEGACY_DEFAULTS,
    )
    plan_motion_options = UnsupportedPlanMotionOptions

    def __init__(self, client: "PyBulletClient") -> None:
        # Initialize all mixins
        super(PyBulletPlanner, self).__init__()  # type: ignore[no-untyped-call]

        self._client: PyBulletClient = client
