from __future__ import annotations

from compas.data import json_dumps
from compas.geometry import Frame
from compas_robots import Configuration
from compas_robots.model import Joint
import pytest
from tesseract_robotics.planning import Pose

from compas_fab.backends.tesseract.client import TesseractClient
from compas_fab.backends.tesseract.errors import InvalidTesseractNativeScaleError
from compas_fab.backends.tesseract.errors import UnknownTesseractOptionError
from compas_fab.backends.tesseract.errors import UnsupportedTesseractToleranceError
from compas_fab.backends.tesseract.planner import TesseractPlanner
from compas_fab.robots import FrameTarget
from compas_fab.robots import RobotCellState
from compas_fab.robots import TargetMode


# KDL LMA's default angular convergence is looser than COMPAS TOL.angular.
KDL_LMA_TEST_ANGLE_TOLERANCE = 1e-4


def _state(value: float, base_frame: Frame | None = None) -> RobotCellState:
    configuration = Configuration([value], [Joint.REVOLUTE], ["joint1"])
    return RobotCellState(base_frame, configuration)


def test_forward_kinematics_retains_native_pose_and_projects_world_units(
    tesseract_artifact,
    one_joint_cell,
    tmp_path,
):
    state = _state(0.4, Frame([2.0, 3.0, 4.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]))
    state_before = json_dumps(state)

    with TesseractClient(tesseract_artifact, cache_root=tmp_path) as client:
        planner = TesseractPlanner(client)
        planner.set_robot_cell(one_joint_cell)

        native_pose = planner.forward_kinematics_native(state, "tip", "manipulator")
        frame_mm = planner.forward_kinematics(
            state,
            TargetMode.ROBOT,
            "manipulator",
            native_scale=0.001,
        )

    assert isinstance(native_pose, Pose)
    assert native_pose.translation.tolist() == pytest.approx([0.0, 0.0, 1.0])
    assert list(frame_mm.point) == pytest.approx([2000.0, 3000.0, 5000.0])
    assert json_dumps(state) == state_before


@pytest.mark.parametrize("invalid_scale", [0.0, -1.0, float("nan"), "millimetres"])
def test_forward_kinematics_rejects_invalid_native_scale(
    tesseract_artifact,
    one_joint_cell,
    tmp_path,
    invalid_scale,
):
    with TesseractClient(tesseract_artifact, cache_root=tmp_path) as client:
        planner = TesseractPlanner(client)
        planner.set_robot_cell(one_joint_cell)

        with pytest.raises(InvalidTesseractNativeScaleError):
            planner.forward_kinematics(
                _state(0.0),
                TargetMode.ROBOT,
                native_scale=invalid_scale,
            )


def test_inverse_kinematics_retains_all_native_solutions_before_projection(
    tesseract_artifact,
    one_joint_cell,
    tmp_path,
):
    target_state = _state(0.4, Frame([2.0, 3.0, 4.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]))
    seed_state = _state(0.0, target_state.robot_base_frame)

    with TesseractClient(tesseract_artifact, cache_root=tmp_path) as client:
        planner = TesseractPlanner(client)
        planner.set_robot_cell(one_joint_cell)
        target_frame = planner.forward_kinematics(
            target_state,
            TargetMode.ROBOT,
            "manipulator",
        )
        target = FrameTarget(target_frame, TargetMode.ROBOT)

        native_solutions = planner.inverse_kinematics_native(
            target,
            seed_state,
            "manipulator",
        )
        configuration = planner.inverse_kinematics(
            target,
            seed_state,
            "manipulator",
        )
        assert client.robot_cell_state == seed_state

    assert native_solutions
    assert native_solutions[0][0] == pytest.approx(0.4, abs=KDL_LMA_TEST_ANGLE_TOLERANCE)
    assert configuration.joint_names == ["joint1"]
    assert configuration.joint_values[0] == pytest.approx(0.4, abs=KDL_LMA_TEST_ANGLE_TOLERANCE)


def test_inverse_kinematics_rejects_unrepresented_tolerance_and_unknown_options(
    tesseract_artifact,
    one_joint_cell,
    tmp_path,
):
    state = _state(0.0)
    target = FrameTarget(
        Frame([0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]),
        TargetMode.ROBOT,
        tolerance_position=0.01,
    )
    with TesseractClient(tesseract_artifact, cache_root=tmp_path) as client:
        planner = TesseractPlanner(client)
        planner.set_robot_cell(one_joint_cell)

        with pytest.raises(UnsupportedTesseractToleranceError):
            planner.inverse_kinematics(target, state)
        with pytest.raises(UnknownTesseractOptionError, match="mystery"):
            planner.inverse_kinematics(
                FrameTarget(target.target_frame, TargetMode.ROBOT),
                state,
                options={"mystery": True},
            )
