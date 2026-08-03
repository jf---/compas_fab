from contextlib import nullcontext

import pytest

from compas_fab.backends.interfaces import ClientInterface  # type: ignore[import-untyped]
from compas_fab.backends.interfaces import PlannerInterface
from compas_fab.robots import RobotCellLibrary  # type: ignore[import-untyped]

from .model import InvalidPlannerContractCaseError
from .model import InvalidPlannerContractHarnessError
from .model import PlannerContractCase
from .model import PlannerContractHarness


def test_harness_retains_exact_consumer_boundary() -> None:
    cell, state = RobotCellLibrary.ur5(load_geometry=False)
    client = ClientInterface()
    planner = PlannerInterface(client)

    harness = PlannerContractHarness.build(client, planner, cell, state)

    assert harness.client is client
    assert harness.planner is planner
    assert harness.robot_cell is cell
    assert harness.robot_cell_state is state


def test_harness_rejects_planner_owned_by_another_client() -> None:
    cell, state = RobotCellLibrary.ur5(load_geometry=False)
    with pytest.raises(InvalidPlannerContractHarnessError, match="planner.client"):
        PlannerContractHarness.build(
            ClientInterface(),
            PlannerInterface(ClientInterface()),
            cell,
            state,
        )


def test_case_validates_name_factory_and_error_type() -> None:
    cell, state = RobotCellLibrary.ur5(load_geometry=False)
    client = ClientInterface()
    harness = PlannerContractHarness.build(
        client,
        PlannerInterface(client),
        cell,
        state,
    )
    case = PlannerContractCase.build(
        "test",
        lambda cache_root: nullcontext(harness),
        ValueError,
    )

    assert case.name == "test"
    assert case.unknown_group_error is ValueError

    with pytest.raises(InvalidPlannerContractCaseError):
        PlannerContractCase.build("", lambda cache_root: nullcontext(harness), ValueError)
