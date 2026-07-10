# Tesseract Backend Phase 1 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. Do not dispatch subagents unless Jelle explicitly requests delegation.

**Goal:** Deliver a macOS-first, Rhino-installable COMPAS FAB backend that proves the complete Tesseract planning path without reducing Tesseract to COMPAS FAB's existing capability set.

**Architecture:** Existing COMPAS FAB calls compile into native Tesseract inputs and share one execution runtime with a lossless native entry point. Content-addressed robot artifacts initialize immutable Tesseract environments; every request plans against a clone. COMPAS `JointTrajectory` values are projections of a retained native result, never replacements for it.

**Tech Stack:** Python 3.9/3.12, COMPAS 2, COMPAS Robots 1, `tesseract-robotics-nanobind` 0.35, Task Composer, pytest/xdist/testmon, mypy strict, Ruff, Rhino 8 CPython Grasshopper components, `compas-actions.ghpython_components`, Pixi.

## Global Constraints

- COMPAS FAB may improve Tesseract ergonomics but must never cap, discard, approximate, silently default, or substitute a Tesseract capability.
- Phase 1 uses `tesseract_python_nanobind`; Cartesian lowering, `tesseract_concurrent_trajopt`, mobile/VKC, and S3 process planning are deferred. Existing Windows CI compiles Grasshopper source bundles.
- Rhino/Windows and macOS support Python 3.9; released Linux nanobind wheels start at Python 3.10. The macOS development environment uses Python 3.12 and the released ABI3 nanobind wheel.
- `tesseract-robotics-nanobind` is a hard dependency. No conditional imports, `HAS_*` flags, or fallback behavior.
- When capabilities overlap, native Tesseract is always the implementation. PyBullet, analytical kinematics, MoveIt, and COMPAS approximations are never fallback execution paths.
- Unknown options, unsupported targets, failed environment initialization, failed planning, and lossy projection each raise a dedicated named exception.
- New types use frozen slotted attrs classes compatible with Python 3.9, validated `build(...)` factories, explicit frame/unit names, and `mypy --strict`.
- Every production behavior starts with a failing test and is verified with `pytest -n auto`; affected tests also run with `pytest --testmon -n auto`.
- One responsibility per file. No new `__all__` declarations.
- No commits until Jelle explicitly requests one.

## Capability Preservation Contract

| Boundary | Required behavior |
| --- | --- |
| Native input | Accept the exact native `CompositeInstruction`, `ProfileDictionary`, pipeline name, and seeding policy. |
| Runtime | Execute the requested Task Composer pipeline without planner substitution or hidden interpolation. |
| Native output | Retain the complete nanobind planning result and raw output `CompositeInstruction`. |
| COMPAS projection | Derive `JointTrajectory` without mutating, clamping, reordering, or resampling the native output. |
| Failure | Raise a named exception carrying the pipeline and native diagnostic; never return an empty trajectory as success. |
| Environment | Build from content-addressed URDF/SRDF/resources and clone before applying request state. |
| Grasshopper | Offer simple COMPAS inputs while preserving a native advanced request/result path. |

## File Map

- `src/compas_fab/backends/tesseract/errors.py`: named Tesseract backend failures.
- `src/compas_fab/backends/tesseract/identity.py`: reproducible `BuildIdentity`.
- `src/compas_fab/backends/tesseract/native.py`: validated native request/result values.
- `src/compas_fab/backends/tesseract/runtime.py`: Task Composer lifetime and execution only.
- `src/compas_fab/backends/tesseract/artifact.py`: immutable URDF/SRDF/resource artifact construction.
- `src/compas_fab/backends/tesseract/environment.py`: initialized environment ownership and cloning.
- `src/compas_fab/backends/tesseract/conversions.py`: COMPAS/Tesseract frames, targets, and trajectories.
- `src/compas_fab/backends/tesseract/client.py`: local client lifecycle and stable backend state.
- `src/compas_fab/backends/tesseract/planner.py`: existing `PlannerInterface` composition.
- `src/compas_fab/backends/tesseract/backend_features/`: one existing COMPAS feature per file.
- `src/compas_fab/ghpython/components_cpython/Cf_TesseractRobotArtifact/`: exact artifact component source.
- `src/compas_fab/ghpython/components_cpython/Cf_TesseractPlanner/`: native client/planner component source.
- `tests/backends/tesseract/`: unit, contract, and real nanobind integration tests.

---

### Task 1: Reproducible Development and Import Gate

**Files:**
- Modify: `requirements.txt`
- Modify: `requirements-dev.txt`
- Modify: `pyproject.toml`
- Test: `tests/backends/tesseract/test_install_contract.py`

**Interfaces:**
- Produces: hard runtime import `tesseract_robotics`; Pixi tasks `test`, `testmon`, `lint`, `format`, and `typecheck-tesseract`.

- [x] **Step 1: Write the failing dependency contract**

```python
from importlib.metadata import version


def test_nanobind_runtime_is_installed_as_hard_dependency():
    import tesseract_robotics

    assert tesseract_robotics is not None
    assert version("tesseract-robotics-nanobind").startswith("0.35.")
```

- [x] **Step 2: Run RED**

Run: `pixi run test tests/backends/tesseract/test_install_contract.py`

Expected: FAIL because the repository has no Pixi workspace or nanobind dependency.

- [x] **Step 3: Add the hard dependency and Pixi tasks**

Add `tesseract-robotics-nanobind >=0.35.0.6,<0.36` to `requirements.txt`, pytest-xdist/testmon/mypy to development requirements, and `[tool.pixi.*]` tables in `pyproject.toml`. Public metadata and the macOS environment both resolve the released 0.35.0.6 build; Pixi declares the wheel's macOS 14 ARM64 system requirement.

- [x] **Step 4: Run GREEN and baseline**

Run: `pixi run test tests/backends/tesseract/test_install_contract.py`

Expected: PASS.

Run: `pixi run test`

Expected: existing suite passes before backend production code is added.

### Task 2: Content-Addressed Build Identity

**Files:**
- Create: `src/compas_fab/backends/tesseract/errors.py`
- Create: `src/compas_fab/backends/tesseract/identity.py`
- Create: `src/compas_fab/backends/tesseract/__init__.py`
- Test: `tests/backends/tesseract/test_identity.py`

**Interfaces:**
- Produces: `BuildIdentity.build(urdf: str, srdf: str, resources: Mapping[str, bytes]) -> BuildIdentity`.
- Produces: `BuildIdentity.digest`, `schema_version`, `compas_fab_version`, and `tesseract_version`.

- [x] **Step 1: Write RED tests**

```python
def test_identity_is_order_independent_for_resources():
    left = BuildIdentity.build("<robot/>", "<robot/>", {"b": b"2", "a": b"1"})
    right = BuildIdentity.build("<robot/>", "<robot/>", {"a": b"1", "b": b"2"})
    assert left == right


def test_identity_changes_when_any_input_changes():
    original = BuildIdentity.build("<robot/>", "<robot/>", {})
    changed = BuildIdentity.build("<robot name='changed'/>", "<robot/>", {})
    assert original.digest != changed.digest
```

- [x] **Step 2: Run RED**

Run: `pixi run test tests/backends/tesseract/test_identity.py`

Expected: FAIL because `BuildIdentity` does not exist.

- [x] **Step 3: Implement the minimal identity**

Use `hashlib.sha256`, length-prefixed UTF-8 fields, sorted resource URLs, and `importlib.metadata.version`. `BuildIdentity.build` rejects empty URDF/SRDF through `EmptyRobotDescriptionError`.

- [x] **Step 4: Run GREEN**

Run: `pixi run test tests/backends/tesseract/test_identity.py`

Expected: PASS.

### Task 3: Lossless Native Request and Result Boundary

**Files:**
- Create: `src/compas_fab/backends/tesseract/native.py`
- Create: `src/compas_fab/backends/tesseract/runtime.py`
- Test: `tests/backends/tesseract/test_native_contract.py`
- Test: `tests/backends/tesseract/test_native_runtime_integration.py`

**Interfaces:**
- Produces: `TesseractPlanningRequest.build(program, pipeline, profiles, auto_seed) -> TesseractPlanningRequest`.
- Produces: `TesseractPlanningResult.build(request, native_result) -> TesseractPlanningResult`.
- Produces: `TesseractRuntime.execute(robot, request) -> TesseractPlanningResult`.

- [x] **Step 1: Write RED identity-preservation tests**

```python
def test_native_request_retains_exact_native_objects(native_program, native_profiles):
    request = TesseractPlanningRequest.build(
        program=native_program,
        pipeline="OMPLPipeline",
        profiles=native_profiles,
        auto_seed=False,
    )
    assert request.program is native_program
    assert request.profiles is native_profiles
    assert request.auto_seed is False
```

- [x] **Step 2: Run RED, implement request validation, run GREEN**

Run: `pixi run test tests/backends/tesseract/test_native_contract.py`

Expected RED: missing type. Expected GREEN: exact objects retained; empty pipeline and empty program raise named errors.

- [x] **Step 3: Write the real integration RED test**

Construct `Robot.from_tesseract_support("abb_irb2400")`, a two-state native `MotionProgram`, and a native profile dictionary. Execute a real `FreespacePipeline`; assert the returned native result is retained by identity, is successful, and contains a non-empty raw `CompositeInstruction`.

- [x] **Step 4: Implement the runtime**

`TesseractRuntime` owns one warmed `TaskComposer`. It calls exactly `composer.plan(robot, request.program, pipeline=request.pipeline, profiles=request.profiles, auto_seed=request.auto_seed)`. Failure raises `TesseractPlanningFailedError` with pipeline and native message. No exception is converted to success.

- [x] **Step 5: Run GREEN**

Run: `pixi run test tests/backends/tesseract/test_native_runtime_integration.py`

Expected: PASS with a real native trajectory and raw output.

### Task 4: Exact Robot Artifact

**Files:**
- Create: `src/compas_fab/backends/tesseract/artifact.py`
- Test: `tests/backends/tesseract/test_artifact.py`

**Interfaces:**
- Produces: `RobotResource.build(url: str, content: bytes) -> RobotResource`.
- Produces: `RobotArtifact.build(urdf: str, srdf: str, resources: Mapping[str, bytes]) -> RobotArtifact`.
- Consumes: `BuildIdentity.build(...)`.

- [x] **Step 1: Write artifact RED tests**

Build from exact fixture descriptions; assert URDF round-trips through `RobotModel.from_urdf_string`, SRDF round-trips through `RobotSemantics.from_srdf_string`, resource order is canonical, source mappings cannot mutate the artifact, and the identity covers both descriptions and every resource byte.

- [x] **Step 2: Implement artifact and run GREEN**

`RobotArtifact` is frozen/slotted and stores exact, immutable resource entries. It never regenerates URDF/SRDF from the lossy `RobotModel`/`RobotSemantics` projections. Missing exact source descriptions raise `EmptyRobotDescriptionError`; invalid resources raise `InvalidRobotResourceError`.

Run: `pixi run test tests/backends/tesseract/test_artifact.py`

Expected: PASS.

### Task 5: Environment Initialization and Clone Isolation

**Files:**
- Create: `src/compas_fab/backends/tesseract/environment.py`
- Create: `src/compas_fab/backends/tesseract/materialization.py`
- Test: `tests/backends/tesseract/test_environment.py`

**Interfaces:**
- Produces: `TesseractEnvironment.build(artifact: RobotArtifact) -> TesseractEnvironment`.
- Produces: `TesseractEnvironment.clone_robot() -> tesseract_robotics.planning.Robot`.

- [x] **Step 1: Write RED environment contract tests**

Use a mesh-free two-joint URDF/SRDF fixture. Assert Tesseract initializes it, exposes the semantic group, and two clones do not share joint state. Assert bad XML raises `TesseractEnvironmentInitializationError` with the artifact digest.

- [x] **Step 2: Implement a content-addressed resource materializer and locator**

Materialize exact resource bytes beneath the artifact digest using `pathlib`, preserving safe package-relative hierarchy so mesh sidecars remain resolvable. Subclass nanobind `ResourceLocator`; return `SimpleLocatedResource` for exact resource URLs and fail with `UnknownRobotResourceError` for missing URLs. This filesystem contract is required for Tesseract octree and point-cloud loaders, which call `Resource.getFilePath()`. Retain the locator and materialization root for at least as long as the environment.

- [x] **Step 3: Implement initialization and run GREEN**

Initialize `Environment.initFromUrdfSrdf(artifact.urdf, artifact.srdf, locator)`, retain a `Robot`, and clone the native environment before every request.

Run: `pixi run test tests/backends/tesseract/test_environment.py`

Expected: PASS.

### Task 6: Native-to-COMPAS Trajectory Projection

**Files:**
- Create: `src/compas_fab/backends/tesseract/conversions.py`
- Test: `tests/backends/tesseract/test_trajectory_projection.py`

**Interfaces:**
- Produces: `joint_trajectory_from_result(result: TesseractPlanningResult, joint_types: Mapping[str, int]) -> JointTrajectory`.

- [x] **Step 1: Write RED projection tests**

Assert names, positions, velocities, accelerations, and monotonically increasing times survive projection without mutation or resampling. Assert inconsistent native joint ordering, absent joint types, missing native dynamics, and decreasing time each raise distinct named errors.

- [x] **Step 2: Implement exact projection and run GREEN**

Use native state waypoint names as the sole column-order authority. Require explicit COMPAS joint types instead of assuming revolute joints. Convert Tesseract seconds to COMPAS nanosecond `Duration` and retain the original float times in trajectory attributes. Retain the native result on the producing in-memory trajectory behind the typed `native_result_from_trajectory` accessor; exclude it from portable COMPAS serialization.

Run: `pixi run test tests/backends/tesseract/test_trajectory_projection.py`

Expected: PASS.

### Task 7: Existing COMPAS Planner Vertical Slice

**Files:**
- Create: `src/compas_fab/backends/tesseract/client.py`
- Create: `src/compas_fab/backends/tesseract/planner.py`
- Create: `src/compas_fab/backends/tesseract/backend_features/set_robot_cell.py`
- Create: `src/compas_fab/backends/tesseract/backend_features/plan_motion.py`
- Test: `tests/backends/tesseract/test_client.py`
- Test: `tests/backends/tesseract/test_plan_motion.py`

**Interfaces:**
- Produces: `TesseractClient`, `TesseractPlanner`.
- Produces: `TesseractPlanner.plan_native(request) -> TesseractPlanningResult`.
- Produces existing: `plan_motion(...) -> JointTrajectory`.

- [x] **Step 1: Write RED client lifecycle tests**

Assert `set_robot_cell` stores an input copy, builds one identity-keyed environment, and replacing the cell replaces the environment. Assert request-state application happens only on a clone.

- [x] **Step 2: Implement client and run GREEN**

The client owns artifact/environment/runtime by their distinct lifetimes. `connect()` warms Task Composer; `disconnect()` releases runtime references; `is_connected` reflects actual runtime availability.

- [x] **Step 3: Write RED `plan_motion` integration tests**

Use `ConfigurationTarget` with a real start state. Assert requested pipeline/profile/seed options map exactly to the native request, the native result is available through `plan_native`, and the COMPAS call returns the exact projected trajectory. Unknown option keys raise `UnknownTesseractOptionError`.

- [x] **Step 4: Implement `plan_motion` and run GREEN**

Compile only target types that can be represented losslessly. Unsupported target classes raise `UnsupportedTesseractTargetError`; they are never approximated.

- [x] **Step 5: Defer conventional Cartesian lowering without fallback**

The user-approved baseline establishes configuration-space planning first. Exact Cartesian programs remain available through `plan_native`; conventional `FrameWaypoints` and `PointAxisWaypoints` lowering is a later stage and must preserve every constraint before it is added.

### Task 8: Kinematics and Collision Baseline

**Files:**
- Create: `src/compas_fab/backends/tesseract/backend_features/forward_kinematics.py`
- Create: `src/compas_fab/backends/tesseract/backend_features/inverse_kinematics.py`
- Create: `src/compas_fab/backends/tesseract/backend_features/check_collision.py`
- Test: `tests/backends/tesseract/test_kinematics_integration.py`
- Test: `tests/backends/tesseract/test_collision_integration.py`

**Interfaces:**
- Produces existing COMPAS planner FK, IK iterator, and collision calls.

- [x] **Step 1: Write and verify failing FK/IK round-trip tests**

Use a known non-singular joint state; project FK to a framed COMPAS result, solve IK with the same explicit seed, and verify FK of the returned solution with `compas.tolerance.TOL` angular and linear predicates.

- [x] **Step 2: Implement FK/IK and run GREEN**

Resolve working frame and TCP from semantics. Never select a different link silently. No-solution raises `TesseractInverseKinematicsError`.

- [x] **Step 3: Write and verify failing collision tests**

Assert a known-free state returns normally and a known-colliding state raises `TesseractCollisionError` carrying contact pairs and distances.

- [x] **Step 4: Implement collision query and run GREEN**

Reuse the environment's discrete manager within its owning snapshot; never share a manager across mutable environments.

### Task 9: Grasshopper Component and Build Contract

**Files:**
- Create: `src/compas_fab/ghpython/components_cpython/Cf_TesseractRobotArtifact/code.py`
- Create: `src/compas_fab/ghpython/components_cpython/Cf_TesseractRobotArtifact/metadata.json`
- Create: `src/compas_fab/ghpython/components_cpython/Cf_TesseractPlanner/code.py`
- Create: `src/compas_fab/ghpython/components_cpython/Cf_TesseractPlanner/metadata.json`
- Test: `tests/backends/tesseract/test_grasshopper_components.py`
- Modify: `.github/workflows/release.yml` only if the existing component task does not discover the new folder automatically.

**Interfaces:**
- Produces exact artifact and planner/native-robot component source discovered by the existing Windows componentizer job.

- [x] **Step 1: Write RED component metadata/source tests**

Assert valid JSON metadata, required Rhino dependency directives, stable input/output names, no conditional import, and direct construction of `TesseractClient`.

- [x] **Step 2: Implement the component and run GREEN**

Separate artifact and planner nodes keep immutable build inputs apart from native runtime lifetime. The planner node caches one client per robot artifact identity and disposes replaced clients. Every Grasshopper callback/cache transition is commented as event-driven application logic.

- [ ] **Step 3: Build the real component**

Run in the existing `build-cpython-components` Windows CI job: `invoke build-cpython-ghuser-components`

Expected: componentizer creates both `.ghuser` artifacts and reports no metadata errors. macOS validates source/metadata contracts but does not run the Mono/System.Drawing componentizer.

### Task 10: Documentation, Type Gate, and Full Verification

**Files:**
- Create: `docs/backends/tesseract.md`
- Modify: `docs/backends/index.md`
- Create: `docs/backends/tesseract/files/01_native_program.py`
- Create: `docs/backends/tesseract/files/02_compas_plan_motion.py`
- Modify: `mkdocs.yml`

**Interfaces:**
- Documents both the ergonomic COMPAS projection and the capability-preserving native path.

- [x] **Step 1: Document installation and both APIs**

Use MkDocs Markdown and Google-style symbol documentation. State macOS/Rhino status precisely; do not claim Windows verification.

- [x] **Step 2: Run focused verification**

Run: `pixi run testmon`

Expected: all affected tests pass with `-n auto`.

- [x] **Step 3: Run quality gates**

Run: `pixi run format`

Run: `pixi run lint`

Run: `pixi run typecheck-tesseract`

Expected: Ruff clean and `mypy --strict` clean for the new backend.

- [ ] **Step 4: Run full contracts**

Run: `pixi run test`

Run: existing Windows `build-cpython-components` CI job

Expected: full suite and Windows componentizer pass. Record test counts and CI artifact path before claiming Phase 1 complete.
