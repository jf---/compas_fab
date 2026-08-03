from collections.abc import Iterator
from contextlib import contextmanager
from pathlib import Path

from compas_fab.backends import PlanningGroupNotExistsError  # type: ignore[import-untyped]
from compas_fab.backends import PyBulletClient
from compas_fab.backends import PyBulletPlanner
from compas_fab.robots import RobotCellLibrary  # type: ignore[import-untyped]

from .model import PlannerContractCase
from .model import PlannerContractHarness


@contextmanager
def open_pybullet_harness(cache_root: Path) -> Iterator[PlannerContractHarness]:
    del cache_root
    cell, state = RobotCellLibrary.ur5(load_geometry=True)
    with PyBulletClient(connection_type="direct") as client:
        planner = PyBulletPlanner(client)
        planner.set_robot_cell(cell, state)
        yield PlannerContractHarness.build(client, planner, cell, state)


PYBULLET_CASE = PlannerContractCase.build(
    "pybullet",
    open_pybullet_harness,
    PlanningGroupNotExistsError,
)
