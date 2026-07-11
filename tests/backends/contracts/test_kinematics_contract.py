from compas.data import json_dumps
import pytest

from compas_fab.robots import FrameTarget
from compas_fab.robots import TargetMode

from .assertions import assert_frames_close
from .model import PlannerContractCase
from .model import PlannerContractHarness


def test_fk_and_ik_leave_input_state_unchanged(
    kinematics_harness: PlannerContractHarness,
):
    state = kinematics_harness.robot_cell_state
    before = json_dumps(state)
    group = kinematics_harness.robot_cell.main_group_name
    frame = kinematics_harness.planner.forward_kinematics(
        state,
        TargetMode.ROBOT,
        group=group,
    )
    solutions = list(
        kinematics_harness.planner.iter_inverse_kinematics(
            FrameTarget(frame, TargetMode.ROBOT),
            state,
            group=group,
        )
    )

    assert solutions
    assert json_dumps(state) == before


def test_fk_ik_round_trip_preserves_group_order(
    kinematics_harness: PlannerContractHarness,
):
    cell = kinematics_harness.robot_cell
    state = kinematics_harness.robot_cell_state
    group = cell.main_group_name
    expected = kinematics_harness.planner.forward_kinematics(
        state,
        TargetMode.ROBOT,
        group=group,
    )
    solutions = list(
        kinematics_harness.planner.iter_inverse_kinematics(
            FrameTarget(expected, TargetMode.ROBOT),
            state,
            group=group,
        )
    )

    assert solutions
    assert all(solution.joint_names == cell.get_configurable_joint_names(group) for solution in solutions)
    reconstructed = state.copy()
    for solution in solutions:
        reconstructed.robot_configuration = solution
        actual = kinematics_harness.planner.forward_kinematics(
            reconstructed,
            TargetMode.ROBOT,
            group=group,
        )
        try:
            assert_frames_close(actual, expected)
            break
        except AssertionError:
            continue
    else:
        pytest.fail("No projected IK solution reconstructs the backend-local FK target.")


def test_unknown_group_fails_with_backend_error(
    kinematics_case: PlannerContractCase,
    kinematics_harness: PlannerContractHarness,
):
    with pytest.raises(kinematics_case.unknown_group_error):
        kinematics_harness.planner.forward_kinematics(
            kinematics_harness.robot_cell_state,
            TargetMode.ROBOT,
            group="missing_contract_group",
        )
