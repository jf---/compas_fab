# Backend Contract Tests Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a test-only consumer-contract suite that verifies the genuinely shared COMPAS planner behavior of Analytical Kinematics, PyBullet, and Tesseract without weakening any backend-native contract.

**Architecture:** Frozen test primitives describe a context-managed planner/client/cell/state harness and explicit backend case. Capability-specific fixture matrices collect only supported cases, so unsupported behavior is absent rather than skipped. Each backend owns one factory module; consumer contract modules call only public COMPAS planner methods.

**Tech Stack:** Python 3.12 and 3.9, pytest + pytest-xdist + pytest-testmon, attrs, COMPAS tolerance, Analytical Kinematics, PyBullet, `tesseract-robotics-nanobind==0.35.0.6`, pixi.

## Global Constraints

- No production package files change; phase 1 is test infrastructure only.
- Tesseract-native tests remain authoritative and unchanged.
- Shared contracts verify COMPAS projections only and never compare one backend numerically against another.
- Unsupported capabilities are excluded through explicit case matrices; never add `skip`, `skipif`, or `xfail`.
- Never add optional imports, `HAS_*` flags, fallback behavior, or broad exception assertions.
- Use named `compas.tolerance.Tolerance`; no bare assertion tolerances.
- Python 3.9 compatibility is mandatory because Rhino 8 is a supported consumer.
- Use pixi exclusively. Every pytest command includes `-n auto`; affected tests also run through pytest-testmon.
- Existing backend tests remain in place. Deletion or migration requires separate approval.

---

### Task 1: Typed Contract Model and Frame Assertion

**Files:**
- Create: `tests/backends/contracts/model.py`
- Create: `tests/backends/contracts/assertions.py`
- Create: `tests/backends/contracts/test_model.py`

**Interfaces:**
- Consumes: `ClientInterface`, `PlannerInterface`, `RobotCell`, `RobotCellState`, `Frame`.
- Produces: `PlannerContractHarness.build(...)`, `PlannerContractCase.build(...)`, `PlannerContractCase.open(cache_root)`, `assert_frames_close(actual, expected)`.

- [ ] **Step 1: Write failing model contracts**

Create `tests/backends/contracts/test_model.py`:

```python
from contextlib import nullcontext

import pytest

from compas_fab.backends.interfaces import ClientInterface
from compas_fab.backends.interfaces import PlannerInterface
from compas_fab.robots import RobotCellLibrary

from .model import InvalidPlannerContractCaseError
from .model import InvalidPlannerContractHarnessError
from .model import PlannerContractCase
from .model import PlannerContractHarness


def test_harness_retains_exact_consumer_boundary():
    cell, state = RobotCellLibrary.ur5(load_geometry=False)
    client = ClientInterface()
    planner = PlannerInterface(client)

    harness = PlannerContractHarness.build(client, planner, cell, state)

    assert harness.client is client
    assert harness.planner is planner
    assert harness.robot_cell is cell
    assert harness.robot_cell_state is state


def test_harness_rejects_planner_owned_by_another_client():
    cell, state = RobotCellLibrary.ur5(load_geometry=False)
    with pytest.raises(InvalidPlannerContractHarnessError, match="planner.client"):
        PlannerContractHarness.build(
            ClientInterface(),
            PlannerInterface(ClientInterface()),
            cell,
            state,
        )


def test_case_validates_name_factory_and_error_type():
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
```

- [ ] **Step 2: Verify RED**

Run:

```bash
pixi run -e default pytest tests/backends/contracts/test_model.py -n auto -q
```

Expected: collection fails because `tests.backends.contracts.model` does not exist.

- [ ] **Step 3: Implement bypass-safe frozen primitives**

Create `tests/backends/contracts/model.py`:

```python
from __future__ import annotations

from contextlib import AbstractContextManager
from pathlib import Path
from typing import Callable
from typing import cast

from attrs import define

from compas_fab.backends.interfaces import ClientInterface
from compas_fab.backends.interfaces import PlannerInterface
from compas_fab.robots import RobotCell
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
```

Create `tests/backends/contracts/assertions.py`:

```python
from compas.geometry import Frame
from compas.geometry import axis_angle_from_quaternion
from compas.tolerance import Tolerance


# Shared projections must agree within 1 mm and 0.01 rad. This is deliberately
# looser than Tesseract's native tests because PyBullet IK is iterative.
CONTRACT_TOL = Tolerance(
    unit="M",
    absolute=1e-3,
    relative=1e-6,
    angular=1e-2,
)


def assert_frames_close(actual: Frame, expected: Frame) -> None:
    position_error = actual.point.distance_to_point(expected.point)
    delta = expected.to_local_coordinates(actual)
    _, orientation_error = axis_angle_from_quaternion(delta.quaternion)
    assert CONTRACT_TOL.is_zero(position_error)
    assert CONTRACT_TOL.is_angle_zero(orientation_error)
```

- [ ] **Step 4: Verify GREEN and strict typing**

Run:

```bash
pixi run -e default ruff format tests/backends/contracts
pixi run -e default ruff check tests/backends/contracts
pixi run -e default mypy --strict tests/backends/contracts/model.py tests/backends/contracts/assertions.py
pixi run -e default pytest tests/backends/contracts/test_model.py -n auto -q
```

Expected: Ruff and mypy report no issues; 3 tests pass.

- [ ] **Step 5: Commit**

```bash
git add tests/backends/contracts/model.py tests/backends/contracts/assertions.py tests/backends/contracts/test_model.py
GIT_AUTHOR_NAME='Jelle Feringa' GIT_AUTHOR_EMAIL='jelleferinga@gmail.com' GIT_COMMITTER_NAME='Jelle Feringa' GIT_COMMITTER_EMAIL='jelleferinga@gmail.com' git commit -m 'test: add planner contract model'
```

### Task 2: Analytical Harness and Core Consumer Contracts

**Files:**
- Create: `tests/backends/contracts/analytical.py`
- Create: `tests/backends/contracts/conftest.py`
- Create: `tests/backends/contracts/test_planner_contract.py`
- Create: `tests/backends/contracts/test_kinematics_contract.py`

**Interfaces:**
- Consumes: `PlannerContractHarness`, `PlannerContractCase`, `assert_frames_close` from Task 1.
- Produces: `ANALYTICAL_CASE`, `kinematics_harness` fixture, backend-neutral planner/FK/IK assertions.

- [ ] **Step 1: Write contracts against the missing fixture**

Create `tests/backends/contracts/test_planner_contract.py`:

```python
from .model import PlannerContractHarness


def test_planner_owns_exact_client(kinematics_harness: PlannerContractHarness):
    assert kinematics_harness.planner.client is kinematics_harness.client


def test_planner_exposes_installed_cell(kinematics_harness: PlannerContractHarness):
    installed = kinematics_harness.planner.robot_cell
    assert installed is not None
    assert installed.structural_signature() == kinematics_harness.robot_cell.structural_signature()
```

Create `tests/backends/contracts/test_kinematics_contract.py`:

```python
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
```

- [ ] **Step 2: Verify RED**

Run:

```bash
pixi run -e default pytest tests/backends/contracts/test_planner_contract.py tests/backends/contracts/test_kinematics_contract.py -n auto -q
```

Expected: fixture lookup fails for `kinematics_harness` and `kinematics_case`.

- [ ] **Step 3: Add the Analytical case and explicit fixture matrix**

Create `tests/backends/contracts/analytical.py`:

```python
from collections.abc import Iterator
from contextlib import contextmanager
from pathlib import Path

from compas_fab.backends import AnalyticalKinematicsPlanner
from compas_fab.backends import UR5Kinematics
from compas_fab.robots import RobotCellLibrary

from .model import PlannerContractCase
from .model import PlannerContractHarness


@contextmanager
def open_analytical_harness(cache_root: Path) -> Iterator[PlannerContractHarness]:
    del cache_root
    cell, state = RobotCellLibrary.ur5(load_geometry=False)
    planner = AnalyticalKinematicsPlanner(UR5Kinematics())
    planner.set_robot_cell(cell, state)
    yield PlannerContractHarness.build(planner.client, planner, cell, state)


ANALYTICAL_CASE = PlannerContractCase.build(
    "analytical",
    open_analytical_harness,
    ValueError,
)
```

Create `tests/backends/contracts/conftest.py`:

```python
from collections.abc import Iterator
from pathlib import Path
from typing import cast

import pytest

from .analytical import ANALYTICAL_CASE
from .model import PlannerContractCase
from .model import PlannerContractHarness

KINEMATICS_CASES = (ANALYTICAL_CASE,)


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
```

- [ ] **Step 4: Verify GREEN**

Run:

```bash
pixi run -e default ruff format tests/backends/contracts
pixi run -e default ruff check tests/backends/contracts
pixi run -e default mypy --strict tests/backends/contracts/model.py tests/backends/contracts/assertions.py tests/backends/contracts/analytical.py
pixi run -e default pytest tests/backends/contracts -n auto -q
```

Expected: all model and Analytical contract tests pass.

- [ ] **Step 5: Commit**

```bash
git add tests/backends/contracts/analytical.py tests/backends/contracts/conftest.py tests/backends/contracts/test_planner_contract.py tests/backends/contracts/test_kinematics_contract.py
GIT_AUTHOR_NAME='Jelle Feringa' GIT_AUTHOR_EMAIL='jelleferinga@gmail.com' GIT_COMMITTER_NAME='Jelle Feringa' GIT_COMMITTER_EMAIL='jelleferinga@gmail.com' git commit -m 'test: add analytical planner contracts'
```

### Task 3: PyBullet Contract Case and Link FK

**Files:**
- Create: `tests/backends/contracts/pybullet.py`
- Create: `tests/backends/contracts/test_link_fk_contract.py`
- Modify: `tests/backends/contracts/conftest.py`

**Interfaces:**
- Consumes: Task 1 primitives and Task 2 fixture matrix.
- Produces: `PYBULLET_CASE`, PyBullet participation in `kinematics_harness`, and `link_fk_harness`.

- [ ] **Step 1: Add failing PyBullet case and link contract imports**

Create `tests/backends/contracts/test_link_fk_contract.py`:

```python
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
```

Add this import to `tests/backends/contracts/conftest.py` before defining matrices:

```python
from .pybullet import PYBULLET_CASE
```

Change the matrices to:

```python
KINEMATICS_CASES = (ANALYTICAL_CASE, PYBULLET_CASE)
LINK_FK_CASES = (PYBULLET_CASE,)
```

Add fixtures:

```python
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
```

- [ ] **Step 2: Verify RED**

Run:

```bash
pixi run -e default pytest tests/backends/contracts -n auto -q
```

Expected: collection fails because `tests.backends.contracts.pybullet` does not exist.

- [ ] **Step 3: Implement direct PyBullet lifetime factory**

Create `tests/backends/contracts/pybullet.py`:

```python
from collections.abc import Iterator
from contextlib import contextmanager
from pathlib import Path

from compas_fab.backends import PlanningGroupNotExistsError
from compas_fab.backends import PyBulletClient
from compas_fab.backends import PyBulletPlanner
from compas_fab.robots import RobotCellLibrary

from .model import PlannerContractCase
from .model import PlannerContractHarness


@contextmanager
def open_pybullet_harness(cache_root: Path) -> Iterator[PlannerContractHarness]:
    del cache_root
    cell, state = RobotCellLibrary.ur5(load_geometry=True)
    with PyBulletClient(connection_type="direct") as client:
        planner = PyBulletPlanner(client)
        planner.set_robot_cell(cell, state)
        yield PlannerContractHarness.build(client, planner, cell, state)


PYBULLET_CASE = PlannerContractCase.build(
    "pybullet",
    open_pybullet_harness,
    PlanningGroupNotExistsError,
)
```

- [ ] **Step 4: Verify GREEN and existing PyBullet tests**

Run:

```bash
pixi run -e default ruff format tests/backends/contracts
pixi run -e default ruff check tests/backends/contracts
pixi run -e default mypy --strict tests/backends/contracts/model.py tests/backends/contracts/assertions.py tests/backends/contracts/analytical.py tests/backends/contracts/pybullet.py
pixi run -e default pytest tests/backends/contracts tests/backends/pybullet tests/backends/kinematics -n auto -q
```

Expected: contract, PyBullet, and Analytical suites pass without skips.

- [ ] **Step 5: Commit**

```bash
git add tests/backends/contracts/pybullet.py tests/backends/contracts/conftest.py tests/backends/contracts/test_link_fk_contract.py
GIT_AUTHOR_NAME='Jelle Feringa' GIT_AUTHOR_EMAIL='jelleferinga@gmail.com' GIT_COMMITTER_NAME='Jelle Feringa' GIT_COMMITTER_EMAIL='jelleferinga@gmail.com' git commit -m 'test: add PyBullet planner contracts'
```

### Task 4: Tesseract Contract Case

**Files:**
- Create: `tests/backends/contracts/tesseract.py`
- Modify: `tests/backends/contracts/conftest.py`

**Interfaces:**
- Consumes: Task 1 primitives, Task 2 core contracts, Task 3 link contract.
- Produces: `TESSERACT_CASE` in both supported matrices without changing native Tesseract tests.

- [ ] **Step 1: Add the missing Tesseract case to explicit matrices**

Add this import to `tests/backends/contracts/conftest.py`:

```python
from .tesseract import TESSERACT_CASE
```

Change matrices to:

```python
KINEMATICS_CASES = (
    ANALYTICAL_CASE,
    PYBULLET_CASE,
    TESSERACT_CASE,
)
LINK_FK_CASES = (
    PYBULLET_CASE,
    TESSERACT_CASE,
)
```

- [ ] **Step 2: Verify RED**

Run:

```bash
pixi run -e default pytest tests/backends/contracts -n auto -q
```

Expected: collection fails because `tests.backends.contracts.tesseract` does not exist.

- [ ] **Step 3: Implement exact UR5 nanobind artifact factory**

Create `tests/backends/contracts/tesseract.py`:

```python
from collections.abc import Iterator
from contextlib import contextmanager
from pathlib import Path

import compas_fab

from compas_fab.backends.tesseract.artifact import CollisionMeshPolicy
from compas_fab.backends.tesseract.artifact import ContinuousContactManager
from compas_fab.backends.tesseract.artifact import DiscreteContactManager
from compas_fab.backends.tesseract.artifact import KdlInverseKinematics
from compas_fab.backends.tesseract.artifact_loader import ResourceRoot
from compas_fab.backends.tesseract.artifact_loader import RobotArtifactLoader
from compas_fab.backends.tesseract.client import TesseractClient
from compas_fab.backends.tesseract.compas_artifact import CompasRobotArtifactCompiler
from compas_fab.backends.tesseract.planner import TesseractPlanner
from compas_fab.robots import RobotCellLibrary

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
```

`KeyError` is the exact current conventional API behavior: semantic group lookup
occurs before Tesseract's native kinematics error wrapper. This contract records
that behavior without changing production code in the test-infrastructure phase.

- [ ] **Step 4: Verify GREEN across Python 3.12 and Rhino 3.9**

Run:

```bash
pixi run -e default ruff format tests/backends/contracts
pixi run -e default ruff check tests/backends/contracts
pixi run -e default mypy --strict tests/backends/contracts/model.py tests/backends/contracts/assertions.py tests/backends/contracts/analytical.py tests/backends/contracts/pybullet.py tests/backends/contracts/tesseract.py
pixi run -e default pytest tests/backends/contracts tests/backends/tesseract -n auto -q
pixi run -e rhino39 pytest tests/backends/contracts tests/backends/tesseract -n auto -q
```

Expected: all contract and native Tesseract tests pass in both environments.

- [ ] **Step 5: Commit**

```bash
git add tests/backends/contracts/tesseract.py tests/backends/contracts/conftest.py
GIT_AUTHOR_NAME='Jelle Feringa' GIT_AUTHOR_EMAIL='jelleferinga@gmail.com' GIT_COMMITTER_NAME='Jelle Feringa' GIT_COMMITTER_EMAIL='jelleferinga@gmail.com' git commit -m 'test: add Tesseract planner contracts'
```

### Task 5: Integration Ratchet

**Files:**
- Verify only; no source changes expected.

**Interfaces:**
- Consumes: all contract cases and unchanged backend suites.
- Produces: evidence that the new consumer contracts introduce no regressions or hidden capability skips.

- [ ] **Step 1: Run affected tests through testmon**

```bash
pixi run -e default pytest --testmon --testmon-noselect --dist=loadscope --no-loadscope-reorder tests/backends/contracts tests/backends/kinematics tests/backends/pybullet tests/backends/tesseract -n auto -q
```

Expected: all collected tests pass; the contract suite reports no skips.

- [ ] **Step 2: Run every backend suite**

```bash
pixi run -e default pytest tests/backends -n auto -q
```

Expected: all local tests pass; only the 21 pre-existing opt-in ROS live-stack tests skip.

- [ ] **Step 3: Run the complete repository suite**

```bash
pixi run -e default pytest -n auto -q
```

Expected: full suite passes; only pre-existing opt-in ROS tests skip.

- [ ] **Step 4: Confirm clean branch state and authored commits**

```bash
git status --short
git log -6 --format='%h %an <%ae> | %cn <%ce> | %s'
```

Expected: worktree clean; every new commit is authored and committed by `Jelle Feringa <jelleferinga@gmail.com>`.
