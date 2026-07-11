from collections.abc import Iterator
from pathlib import Path
from typing import cast

import pytest

from .analytical import ANALYTICAL_CASE
from .model import PlannerContractCase
from .model import PlannerContractHarness
from .pybullet import PYBULLET_CASE
from .tesseract import TESSERACT_CASE

KINEMATICS_CASES = (
    ANALYTICAL_CASE,
    PYBULLET_CASE,
    TESSERACT_CASE,
)
LINK_FK_CASES = (
    PYBULLET_CASE,
    TESSERACT_CASE,
)


@pytest.fixture(params=KINEMATICS_CASES, ids=lambda case: case.name)
def kinematics_case(request: pytest.FixtureRequest) -> PlannerContractCase:
    return cast(PlannerContractCase, request.param)


@pytest.fixture
def kinematics_harness(
    kinematics_case: PlannerContractCase,
    tmp_path: Path,
) -> Iterator[PlannerContractHarness]:
    with kinematics_case.open(tmp_path) as harness:
        yield harness


@pytest.fixture(params=LINK_FK_CASES, ids=lambda case: case.name)
def link_fk_case(request: pytest.FixtureRequest) -> PlannerContractCase:
    return cast(PlannerContractCase, request.param)


@pytest.fixture
def link_fk_harness(
    link_fk_case: PlannerContractCase,
    tmp_path: Path,
) -> Iterator[PlannerContractHarness]:
    with link_fk_case.open(tmp_path) as harness:
        yield harness
