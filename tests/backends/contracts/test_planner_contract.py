from .model import PlannerContractHarness


def test_planner_owns_exact_client(kinematics_harness: PlannerContractHarness):
    assert kinematics_harness.planner.client is kinematics_harness.client


def test_planner_exposes_installed_cell(kinematics_harness: PlannerContractHarness):
    installed = kinematics_harness.planner.robot_cell
    assert installed is not None
    assert installed.structural_signature() == kinematics_harness.robot_cell.structural_signature()
