from compas_fab.robots import TargetMode

from .assertions import assert_frames_close
from .model import PlannerContractHarness


def test_link_fk_matches_robot_target_fk(link_fk_harness: PlannerContractHarness):
    cell = link_fk_harness.robot_cell
    state = link_fk_harness.robot_cell_state
    group = cell.main_group_name
    end_effector = cell.get_end_effector_link_name(group)

    expected = link_fk_harness.planner.forward_kinematics(
        state,
        TargetMode.ROBOT,
        group=group,
    )
    actual = link_fk_harness.planner.forward_kinematics_to_link(
        state,
        end_effector,
    )

    assert_frames_close(actual, expected)
