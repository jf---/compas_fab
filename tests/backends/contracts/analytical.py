from collections.abc import Iterator
from contextlib import contextmanager
from pathlib import Path

from compas_fab.backends import AnalyticalKinematicsPlanner  # type: ignore[import-untyped]
from compas_fab.backends import UR5Kinematics
from compas_fab.robots import RobotCellLibrary  # type: ignore[import-untyped]

from .model import PlannerContractCase
from .model import PlannerContractHarness


@contextmanager
def open_analytical_harness(cache_root: Path) -> Iterator[PlannerContractHarness]:
    del cache_root
    cell, state = RobotCellLibrary.ur5(load_geometry=False)
    planner = AnalyticalKinematicsPlanner(UR5Kinematics())
    planner.set_robot_cell(cell, state)
    yield PlannerContractHarness.build(planner.client, planner, cell, state)


ANALYTICAL_CASE = PlannerContractCase.build(
    "analytical",
    open_analytical_harness,
    ValueError,
)
