from compas_fab.backends.interfaces.planner_contract import PlannerContract
from compas_fab.backends.kinematics.planner import AnalyticalKinematicsPlanner
from compas_fab.backends.kinematics.planner import AnalyticalPyBulletPlanner
from compas_fab.backends.pybullet.planner import PyBulletPlanner
from compas_fab.backends.ros.planner import MoveItPlanner
from compas_fab.backends.tesseract.planner import TesseractPlanner


def accepts_planner_contract(planner: type[PlannerContract]) -> None:
    """Model the class-level contract consumed by Grasshopper components."""


accepts_planner_contract(AnalyticalKinematicsPlanner)
accepts_planner_contract(AnalyticalPyBulletPlanner)
accepts_planner_contract(PyBulletPlanner)
accepts_planner_contract(MoveItPlanner)
accepts_planner_contract(TesseractPlanner)
