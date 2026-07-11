from __future__ import annotations

from contextlib import AbstractContextManager
from pathlib import Path
from typing import Callable
from typing import cast

from attrs import define

from compas_fab.backends.interfaces import ClientInterface  # type: ignore[import-untyped]
from compas_fab.backends.interfaces import PlannerInterface
from compas_fab.robots import RobotCell  # type: ignore[import-untyped]
from compas_fab.robots import RobotCellState


class InvalidPlannerContractHarnessError(TypeError):
    """A test harness does not retain one exact consumer boundary."""


class InvalidPlannerContractCaseError(TypeError):
    """A backend contract case is malformed."""


def _validated_harness(
    client: object,
    planner: object,
    robot_cell: object,
    robot_cell_state: object,
) -> tuple[ClientInterface, PlannerInterface, RobotCell, RobotCellState]:
    if not isinstance(client, ClientInterface):
        raise InvalidPlannerContractHarnessError("Harness client must be ClientInterface.")
    if not isinstance(planner, PlannerInterface):
        raise InvalidPlannerContractHarnessError("Harness planner must be PlannerInterface.")
    if planner.client is not client:
        raise InvalidPlannerContractHarnessError("Harness planner.client must be the exact client.")
    if not isinstance(robot_cell, RobotCell):
        raise InvalidPlannerContractHarnessError("Harness robot_cell must be RobotCell.")
    if not isinstance(robot_cell_state, RobotCellState):
        raise InvalidPlannerContractHarnessError("Harness robot_cell_state must be RobotCellState.")
    return client, planner, robot_cell, robot_cell_state


@define(frozen=True, slots=True)
class PlannerContractHarness:
    client: ClientInterface
    planner: PlannerInterface
    robot_cell: RobotCell
    robot_cell_state: RobotCellState

    def __attrs_post_init__(self) -> None:
        _validated_harness(
            self.client,
            self.planner,
            self.robot_cell,
            self.robot_cell_state,
        )

    @classmethod
    def build(
        cls,
        client: object,
        planner: object,
        robot_cell: object,
        robot_cell_state: object,
    ) -> PlannerContractHarness:
        return cls(*_validated_harness(client, planner, robot_cell, robot_cell_state))


ContractFactory = Callable[[Path], AbstractContextManager[PlannerContractHarness]]


def _validated_case(
    name: object,
    factory: object,
    unknown_group_error: object,
) -> tuple[str, ContractFactory, type[Exception]]:
    if not isinstance(name, str) or not name.strip():
        raise InvalidPlannerContractCaseError("Contract case name must be non-empty.")
    if not callable(factory):
        raise InvalidPlannerContractCaseError("Contract case factory must be callable.")
    if not isinstance(unknown_group_error, type) or not issubclass(unknown_group_error, Exception):
        raise InvalidPlannerContractCaseError("Contract case error must be an Exception type.")
    return name, cast(ContractFactory, factory), unknown_group_error


@define(frozen=True, slots=True)
class PlannerContractCase:
    name: str
    factory: ContractFactory
    unknown_group_error: type[Exception]

    def __attrs_post_init__(self) -> None:
        _validated_case(self.name, self.factory, self.unknown_group_error)

    @classmethod
    def build(
        cls,
        name: object,
        factory: object,
        unknown_group_error: object,
    ) -> PlannerContractCase:
        return cls(*_validated_case(name, factory, unknown_group_error))

    def open(self, cache_root: Path) -> AbstractContextManager[PlannerContractHarness]:
        return self.factory(cache_root)
