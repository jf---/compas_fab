from collections.abc import Iterator
from contextlib import contextmanager
from pathlib import Path

import compas_fab  # type: ignore[import-untyped]

from compas_fab.backends.tesseract.artifact import CollisionMeshPolicy  # type: ignore[import-untyped]
from compas_fab.backends.tesseract.artifact import ContinuousContactManager
from compas_fab.backends.tesseract.artifact import DiscreteContactManager
from compas_fab.backends.tesseract.artifact import KdlInverseKinematics
from compas_fab.backends.tesseract.artifact_loader import ResourceRoot  # type: ignore[import-untyped]
from compas_fab.backends.tesseract.artifact_loader import RobotArtifactLoader
from compas_fab.backends.tesseract.client import TesseractClient  # type: ignore[import-untyped]
from compas_fab.backends.tesseract.compas_artifact import CompasRobotArtifactCompiler  # type: ignore[import-untyped]
from compas_fab.backends.tesseract.planner import TesseractPlanner  # type: ignore[import-untyped]
from compas_fab.robots import RobotCellLibrary  # type: ignore[import-untyped]

from .model import PlannerContractCase
from .model import PlannerContractHarness


@contextmanager
def open_tesseract_harness(cache_root: Path) -> Iterator[PlannerContractHarness]:
    cell, state = RobotCellLibrary.ur5(load_geometry=False)
    resource_root = Path(compas_fab.get("robot_library/ur5_robot"))
    loader = RobotArtifactLoader.build(
        resource_root / "urdf" / "robot_description.urdf",
        resource_root / "robot_description_semantic.srdf",
        [ResourceRoot.build(resource_root)],
    )
    artifact = CompasRobotArtifactCompiler.build(
        loader=loader,
        collision_mesh_policy=CollisionMeshPolicy.CONVEX_HULL,
        groups=["manipulator"],
        inverse_kinematics=KdlInverseKinematics.LMA,
        discrete_contact_manager=DiscreteContactManager.BULLET_BVH,
        continuous_contact_manager=ContinuousContactManager.BULLET_CAST_BVH,
    ).compile(cell)
    with TesseractClient(artifact, cache_root=cache_root) as client:
        planner = TesseractPlanner(client)
        planner.set_robot_cell(cell, state)
        yield PlannerContractHarness.build(client, planner, cell, state)


TESSERACT_CASE = PlannerContractCase.build(
    "tesseract",
    open_tesseract_harness,
    KeyError,
)
