from pathlib import Path

import compas_fab
import pytest
from compas.data import json_dumps
from compas_robots import Configuration
from compas_robots.model import Joint

from compas_fab.backends.tesseract.client import TesseractClient
from compas_fab.backends.tesseract.artifact import CollisionMeshPolicy
from compas_fab.backends.tesseract.artifact import ContinuousContactManager
from compas_fab.backends.tesseract.artifact import DiscreteContactManager
from compas_fab.backends.tesseract.artifact import KdlInverseKinematics
from compas_fab.backends.tesseract.artifact_loader import ResourceRoot
from compas_fab.backends.tesseract.artifact_loader import RobotArtifactLoader
from compas_fab.backends.tesseract.compas_artifact import CompasRobotArtifactCompiler
from compas_fab.backends.tesseract.errors import UnknownTesseractOptionError
from compas_fab.backends.tesseract.errors import MissingContactManagerPluginError
from compas_fab.backends.tesseract.errors import UnsupportedTesseractToleranceError
from compas_fab.backends.tesseract.planner import TesseractPlanner
from compas_fab.backends.tesseract.conversions import native_result_from_trajectory
from compas_fab.robots import ConfigurationTarget
from compas_fab.robots.robot_library import RobotCellLibrary


def _target(value, tolerance=None):
    configuration = Configuration([value], [Joint.REVOLUTE], ["joint1"])
    tolerances = [tolerance] if tolerance is not None else None
    return ConfigurationTarget(configuration, tolerances, tolerances)


def test_plan_motion_executes_existing_compas_interface(
    tesseract_artifact,
    one_joint_cell,
    one_joint_state,
    tmp_path,
):
    target = _target(0.5)
    target_before = json_dumps(target)
    state_before = json_dumps(one_joint_state)

    with TesseractClient(tesseract_artifact, cache_root=tmp_path) as client:
        planner = TesseractPlanner(client)
        planner.set_robot_cell(one_joint_cell)

        trajectory = planner.plan_motion(target, one_joint_state)

    assert trajectory.joint_names == ["joint1"]
    assert trajectory.points
    assert trajectory.points[-1].joint_values[0] == pytest.approx(0.5)
    assert trajectory.start_state == one_joint_state
    assert native_result_from_trajectory(trajectory).native_result.successful
    assert json_dumps(target) == target_before
    assert json_dumps(one_joint_state) == state_before


def test_plan_motion_native_retains_complete_tesseract_result(
    tesseract_artifact,
    one_joint_cell,
    one_joint_state,
    tmp_path,
):
    with TesseractClient(tesseract_artifact, cache_root=tmp_path) as client:
        planner = TesseractPlanner(client)
        planner.set_robot_cell(one_joint_cell)

        result = planner.plan_motion_native(_target(0.25), one_joint_state)

    assert result.native_result.successful
    assert result.raw_program is result.native_result.raw_results
    assert len(result.raw_program) > 0


def test_plan_motion_rejects_unknown_options(
    tesseract_artifact,
    one_joint_cell,
    one_joint_state,
    tmp_path,
):
    with TesseractClient(tesseract_artifact, cache_root=tmp_path) as client:
        planner = TesseractPlanner(client)
        planner.set_robot_cell(one_joint_cell)

        with pytest.raises(UnknownTesseractOptionError, match="mystery"):
            planner.plan_motion(_target(0.5), one_joint_state, options={"mystery": True})


def test_plan_motion_rejects_unrepresentable_joint_tolerance(
    tesseract_artifact,
    one_joint_cell,
    one_joint_state,
    tmp_path,
):
    with TesseractClient(tesseract_artifact, cache_root=tmp_path) as client:
        planner = TesseractPlanner(client)
        planner.set_robot_cell(one_joint_cell)

        with pytest.raises(UnsupportedTesseractToleranceError):
            planner.plan_motion(_target(0.5, tolerance=0.01), one_joint_state)


def test_conventional_planning_requires_its_contact_managers_at_consuming_boundary(
    kdl_artifact,
    one_joint_cell,
    one_joint_state,
    tmp_path,
):
    with TesseractClient(kdl_artifact, cache_root=tmp_path) as client:
        planner = TesseractPlanner(client)
        planner.set_robot_cell(one_joint_cell)

        with pytest.raises(MissingContactManagerPluginError):
            planner.plan_motion(_target(0.5), one_joint_state)


def test_plan_motion_executes_exact_bundled_ur5_artifact(tmp_path):
    robot_cell, start_state = RobotCellLibrary.ur5(load_geometry=False)
    start_state.robot_configuration = robot_cell.get_configuration_from_group_state("manipulator", "up")
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
    ).compile(robot_cell)
    goal_configuration = start_state.robot_configuration.copy()
    goal_configuration["shoulder_pan_joint"] = 0.1

    with TesseractClient(artifact, cache_root=tmp_path) as client:
        planner = TesseractPlanner(client)
        planner.set_robot_cell(robot_cell, start_state)
        trajectory = planner.plan_motion(
            ConfigurationTarget(goal_configuration),
            start_state,
            group="manipulator",
        )

    assert trajectory.points[-1].joint_values[0] == pytest.approx(0.1)
