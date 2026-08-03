from math import pi
from pathlib import Path

import numpy as np
import pytest
from tesseract_robotics.tesseract_motion_planners_descartes import DescartesDefaultMoveProfileD


EXAMPLES = Path(__file__).parents[3] / "docs" / "backends" / "tesseract" / "files"
CONVENTIONAL_EXAMPLE = EXAMPLES / "01_compas_plan_motion.py"
NATIVE_EXAMPLE = EXAMPLES / "02_native_program.py"
RAPID_EXAMPLE = EXAMPLES / "03_rapid_emitter.py"
COMPONENT_WORKFLOW_EXAMPLE = EXAMPLES / "04_native_component_workflow.py"


def test_examples_exercise_convex_hull_loading_default():
    for example in (CONVENTIONAL_EXAMPLE, NATIVE_EXAMPLE):
        source = example.read_text(encoding="utf-8")
        assert "collision_mesh_policy=" not in source


def test_native_example_uses_tesseract_axis_symmetry():
    source = NATIVE_EXAMPLE.read_text(encoding="utf-8")

    assert 'pipeline="DescartesFPipeline"' in source
    assert "TOOL_Z_AXIS = (0.0, 0.0, 1.0)" in source
    assert "TOOL_AXIS_SAMPLE_STEP = Radians(radians(1.0))" in source
    assert "TOOL_AXIS_SAMPLE_MIN = Radians(-pi)" in source
    assert "TOOL_AXIS_SAMPLE_MAX = Radians(pi)" in source
    assert "sample_axis=TOOL_Z_AXIS" in source
    assert "sample_resolution=TOOL_AXIS_SAMPLE_STEP" in source
    assert "sample_min=TOOL_AXIS_SAMPLE_MIN" in source
    assert "sample_max=TOOL_AXIS_SAMPLE_MAX" in source
    assert "use_redundant_joint_solutions=True" in source


def test_rapid_example_keeps_planning_and_emission_independent():
    source = RAPID_EXAMPLE.read_text(encoding="utf-8")

    assert 'Robot.from_tesseract_support("abb_irb2400")' in source
    assert 'pipeline="DescartesFPipeline"' in source
    assert "TOOL_Z_AXIS = (0.0, 0.0, 1.0)" in source
    assert "TOOL_AXIS_SAMPLE_STEP = Radians(radians(1.0))" in source
    assert "TOOL_AXIS_SAMPLE_MIN = Radians(-pi)" in source
    assert "TOOL_AXIS_SAMPLE_MAX = Radians(pi)" in source
    assert "sample_axis=TOOL_Z_AXIS" in source
    assert "sample_resolution=TOOL_AXIS_SAMPLE_STEP" in source
    assert "sample_min=TOOL_AXIS_SAMPLE_MIN" in source
    assert "sample_max=TOOL_AXIS_SAMPLE_MAX" in source
    assert "use_redundant_joint_solutions=True" in source
    assert "authored_program = program.to_composite_instruction" in source
    assert "TesseractRapidEmitter.emit(" in source
    assert "result.raw_results" not in source
    assert ".write(" not in source


def test_released_descartes_defaults_are_known_and_overridden_explicitly():
    profile = DescartesDefaultMoveProfileD()

    assert profile.target_pose_fixed is True
    np.testing.assert_array_equal(profile.target_pose_sample_axis, [0.0, 0.0, 1.0])
    assert profile.target_pose_sample_resolution == pytest.approx(pi / 2.0)
    assert profile.target_pose_sample_min == pytest.approx(-pi)
    assert profile.target_pose_sample_max == pytest.approx(pi / 2.0)
    assert profile.use_redundant_joint_solutions is False


def test_component_workflow_uses_every_native_factory_and_axis_redundancy():
    source = COMPONENT_WORKFLOW_EXAMPLE.read_text(encoding="utf-8")

    for symbol in (
        "WorkingFrameUserUnits.build",
        "pose_from_working_frame",
        "cartesian_target_from_native",
        "build_motion_program",
        "build_descartes_profiles",
        "TesseractPlanningRequest.build",
        "TesseractNativeResultView.build",
    ):
        assert symbol in source
    assert 'Robot.from_tesseract_support("abb_irb2400")' in source
    assert 'pipeline="DescartesFPipeline"' in source
    assert "TOOL_Z_AXIS = (0.0, 0.0, 1.0)" in source
    assert "TOOL_AXIS_SAMPLE_STEP = Radians(radians(1.0))" in source
    assert "TOOL_AXIS_SAMPLE_MIN = Radians(-pi)" in source
    assert "TOOL_AXIS_SAMPLE_MAX = Radians(pi)" in source
    assert "sample_axis=TOOL_Z_AXIS" in source
    assert "sample_resolution=TOOL_AXIS_SAMPLE_STEP" in source
    assert "sample_min=TOOL_AXIS_SAMPLE_MIN" in source
    assert "sample_max=TOOL_AXIS_SAMPLE_MAX" in source
    assert "use_redundant_joint_solutions=True" in source
    assert "robot_frame_from_isometry" in source
    assert "JointTrajectory" not in source
