from compas_fab.backends.interfaces.planner_contract import PlannerContract
from compas_fab.backends.interfaces.planner_capabilities import PlannerCapabilities
from compas_fab.backends.interfaces.planner_capabilities import PlannerImplementationId
from compas_fab.backends.interfaces.planner_options import PlanMotionOptionsAdapter
from compas_fab.backends.kinematics.planner import AnalyticalKinematicsPlanner
from compas_fab.backends.kinematics.planner import AnalyticalPyBulletPlanner
from compas_fab.backends.pybullet.planner import PyBulletPlanner
from compas_fab.backends.ros.planner import MoveItPlanner
from compas_fab.backends.tesseract.planner import TesseractPlanner


PlannerContractMembers = tuple[PlannerImplementationId, PlannerCapabilities, type[PlanMotionOptionsAdapter]]


def consume_planner(planner: PlannerContract) -> PlannerContractMembers:
    """Read every exact member required from a planner instance."""
    implementation_id: PlannerImplementationId = planner.implementation_id
    capabilities: PlannerCapabilities = planner.capabilities
    options_adapter: type[PlanMotionOptionsAdapter] = planner.plan_motion_options
    return implementation_id, capabilities, options_adapter


def check_all_planner_instances(
    analytical: AnalyticalKinematicsPlanner,
    analytical_pybullet: AnalyticalPyBulletPlanner,
    pybullet: PyBulletPlanner,
    moveit: MoveItPlanner,
    tesseract: TesseractPlanner,
) -> None:
    """Prove all five typed planner instances satisfy the consumer boundary."""
    consume_planner(analytical)
    consume_planner(analytical_pybullet)
    consume_planner(pybullet)
    consume_planner(moveit)
    consume_planner(tesseract)
