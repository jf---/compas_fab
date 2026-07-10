"""COMPAS FAB planner facade over one capability-preserving runtime."""

from __future__ import annotations

from compas_fab.backends.interfaces.planner import PlannerInterface

from .backend_features.check_collision import TesseractCheckCollision
from .backend_features.forward_kinematics import TesseractForwardKinematics
from .backend_features.inverse_kinematics import TesseractInverseKinematics
from .backend_features.plan_motion import TesseractPlanMotion
from .backend_features.set_robot_cell import TesseractSetRobotCell
from .client import TesseractClient
from .native import TesseractPlanningRequest
from .native import TesseractPlanningResult


class TesseractPlanner(
    TesseractCheckCollision,
    TesseractForwardKinematics,
    TesseractInverseKinematics,
    TesseractPlanMotion,
    TesseractSetRobotCell,
    PlannerInterface,
):
    """Expose native Tesseract planning and existing COMPAS projections."""

    _client: TesseractClient

    def __init__(self, client: TesseractClient) -> None:
        object.__init__(self)
        self._client = client

    def plan_native(self, request: TesseractPlanningRequest) -> TesseractPlanningResult:
        """Execute an exact native request against an isolated environment clone."""
        robot = self._client.clone_robot()
        return self._client.runtime.execute(robot, request)
