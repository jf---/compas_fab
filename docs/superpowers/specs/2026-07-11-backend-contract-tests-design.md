# Backend Contract Tests Design

## Objective

Add a test-only consumer-contract suite for the COMPAS-facing planner boundary shared by Analytical Kinematics, PyBullet, and Tesseract. The suite removes duplicated behavioral assertions without deleting or weakening backend-specific tests. Tesseract-native objects, ordering, profiles, collision results, and failure modes remain governed exclusively by the Tesseract suite.

## Approaches

### Selected: explicit consumer-contract cases

Each contract declares the backend cases that genuinely implement it. A typed test harness owns planner/client lifetime and a UR5 `RobotCell` plus matching `RobotCellState`. Tests call only public COMPAS planner methods. Capability-specific fixture lists prevent unsupported backends from being collected; there are no skips, `xfail` markers, conditional imports, or fallback behavior.

### Rejected: copy assertions into every backend directory

Copying FK/IK and cell-state scenarios preserves backend isolation but duplicates fixtures, tolerance policy, and assertions. The current suites already demonstrate this drift.

### Rejected: differential backend oracle

Comparing Tesseract directly against PyBullet, ROS, or Analytical results would make another backend authoritative and risk reducing Tesseract to their supported subset. Cross-backend numerical comparison may be useful diagnostically, but it is not a contract.

## Scope

Phase 1 covers:

- planner/client ownership and installed-cell visibility;
- input-state immutability across FK and IK;
- reachable FK-to-IK round trips in `TargetMode.ROBOT`;
- exact group joint-name order in projected configurations;
- failure on unknown planning groups, with each case declaring its named backend error;
- link-specific FK for PyBullet and Tesseract only.

Phase 1 does not cover:

- native Tesseract requests, results, programs, profiles, or collision data;
- tools, rigid bodies, target modes `TOOL`/`WORKPIECE`, or scene updates;
- `PointAxisTarget`, Cartesian motion, or configuration motion planning;
- ROS messages, actions, transport, live-stack setup, or documentation examples;
- numerical equality between different solvers.

Those exclusions are capability boundaries, not skipped tests. They receive later contract cases only after every listed backend represents the behavior exactly.

## Architecture

`tests/backends/contracts/model.py` defines two frozen, bypass-safe test primitives:

- `PlannerContractHarness`: exact `PlannerInterface`, `RobotCell`, and `RobotCellState` values owned by one context-managed backend setup;
- `PlannerContractCase`: a stable case name, context-manager factory, and exact unknown-group exception type.

Each backend setup has one file and one responsibility:

- `analytical.py` builds `AnalyticalKinematicsPlanner(UR5Kinematics())` without geometry;
- `pybullet.py` owns a direct `PyBulletClient` and `PyBulletPlanner` with the geometry required by PyBullet;
- `tesseract.py` compiles the repository UR5 URDF/SRDF into an exact KDL nanobind artifact, owns `TesseractClient`, and exposes `TesseractPlanner`.

`conftest.py` exposes two explicit fixture matrices:

- `kinematics_harness`: Analytical, PyBullet, Tesseract;
- `link_fk_harness`: PyBullet, Tesseract.

Contract files are split by consumer responsibility:

- `test_planner_contract.py`: ownership and installed-cell visibility;
- `test_kinematics_contract.py`: immutability, FK/IK round trip, group order, unknown groups;
- `test_link_fk_contract.py`: link-specific FK.

Existing backend tests remain in place. Migration or deletion requires separate approval after the new contracts prove stable.

## Numerical Policy

The suite derives each IK target from the same backend's FK result. It therefore verifies internal interface coherence without asserting that different solvers choose identical frames or branches. Frame comparison uses one named `compas.tolerance.Tolerance` instance with explicit metre and radian rationale; no bare tolerance literals appear at assertion sites.

Joint configurations are compared by exact joint-name order and backend-local FK reconstruction. The suite does not assume a fixed number or order of IK solutions.

## Errors and Lifetime

Factories are context managers. Client cleanup is structural: PyBullet and Tesseract clients close even when a contract fails. Harness and case raw constructors validate exact types and non-empty names.

Unknown groups must fail loudly. The case declares the exact exception class currently promised by that backend; the contract never uses `pytest.raises(Exception)`.

## Verification

Every task follows red-green-refactor:

1. add a contract that fails because its harness or fixture is missing;
2. run `pytest -n auto` and confirm the expected failure;
3. add the minimum test-only harness code;
4. run `pytest --testmon --testmon-noselect --dist=loadscope --no-loadscope-reorder -n auto`;
5. run all three existing backend suites and the full repository suite.

Strict mypy covers all contract harness modules. Ruff covers contract and existing backend tests. No production package files change in phase 1.
