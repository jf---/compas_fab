# Collision Default and Axis Symmetry Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Default robot loading to convex-hull collision meshes and demonstrate native local-TCP-Z redundancy with released Tesseract nanobind 0.35.0.6.

**Architecture:** Loading conveniences own the convex-hull default while the low-level `RobotArtifact` constructor remains explicit. The native example passes exact Cartesian instructions and Descartes profiles through `TesseractPlanningRequest`; COMPAS FAB neither samples rotations nor chooses IK branches.

**Tech Stack:** Python 3.9/3.12, attrs, COMPAS FAB, tesseract-robotics-nanobind 0.35.0.6, pytest-xdist/testmon, Ruff, mypy strict, MkDocs.

## Global Constraints

- `tesseract-robotics-nanobind>=0.35.0.6,<0.36`; never install or import `tesseract_python`.
- Python 3.9 compatibility is mandatory for Rhino 8.
- Tesseract remains the planner and redundancy sampler; no COMPAS fallback or manual yaw enumeration.
- `RobotArtifact.from_compas_urdf(...)` retains its required mesh-policy argument.
- Explicit `CollisionMeshPolicy.PRESERVE` remains exact and supported.
- Run pytest with `-n auto`; use `--testmon` after changes.
- Do not commit without Jelle's explicit instruction.

---

### Task 1: Loading Defaults

**Files:**
- Modify: `tests/backends/tesseract/test_artifact_loader.py`
- Modify: `tests/backends/tesseract/test_compas_artifact.py`
- Modify: `src/compas_fab/backends/tesseract/artifact_loader.py`
- Modify: `src/compas_fab/backends/tesseract/compas_artifact.py`

**Interfaces:**
- Consumes: `CollisionMeshPolicy.CONVEX_HULL`, `RobotArtifact.from_compas_urdf(...)`.
- Produces: `RobotArtifactLoader.load(collision_mesh_policy=CollisionMeshPolicy.CONVEX_HULL)` and `CompasRobotArtifactCompiler.build(..., collision_mesh_policy=CollisionMeshPolicy.CONVEX_HULL)`.

- [x] **Step 1: Write loader default and explicit-preserve tests**

```python
def test_loader_defaults_collision_meshes_to_convex_hulls(tmp_path):
    urdf_path = tmp_path / "robot.urdf"
    srdf_path = tmp_path / "robot.srdf"
    urdf_path.write_text('<robot name="primitive"><link name="base"/></robot>', encoding="utf-8")
    srdf_path.write_text('<robot name="primitive"/>', encoding="utf-8")

    artifact = RobotArtifactLoader.build(urdf_path, srdf_path, []).load()

    assert 'tesseract:make_convex="true"' in artifact.urdf
```

Keep the existing explicit `PRESERVE` test and assert `tesseract:make_convex="false"` there.

- [x] **Step 2: Write compiler default test**

Build `CompasRobotArtifactCompiler` without `collision_mesh_policy` and assert both `compiler.collision_mesh_policy is CollisionMeshPolicy.CONVEX_HULL` and the compiled URDF contains `tesseract:make_convex="true"`.

- [x] **Step 3: Verify RED**

Run: `pixi run pytest tests/backends/tesseract/test_artifact_loader.py tests/backends/tesseract/test_compas_artifact.py -n auto -q`

Expected: loader call raises `TypeError`; compiler call raises `TypeError`.

- [x] **Step 4: Implement minimal defaults**

```python
def load(
    self,
    collision_mesh_policy: CollisionMeshPolicy = CollisionMeshPolicy.CONVEX_HULL,
) -> RobotArtifact:
```

Order the compiler factory's required plugin arguments before the defaulted policy so keyword callers remain unchanged:

```python
def build(
    cls,
    loader: RobotArtifactLoader,
    groups: Sequence[str],
    inverse_kinematics: KdlInverseKinematics,
    discrete_contact_manager: DiscreteContactManager,
    continuous_contact_manager: ContinuousContactManager,
    collision_mesh_policy: CollisionMeshPolicy = CollisionMeshPolicy.CONVEX_HULL,
) -> CompasRobotArtifactCompiler:
```

- [x] **Step 5: Verify GREEN**

Run: `pixi run pytest tests/backends/tesseract/test_artifact_loader.py tests/backends/tesseract/test_compas_artifact.py -n auto -q`

Expected: all tests pass.

### Task 2: Grasshopper and Documentation Contract

**Files:**
- Modify: `tests/backends/tesseract/test_grasshopper_components.py`
- Modify: `src/compas_fab/ghpython/components_cpython/Cf_TesseractRobotArtifact/code.py`
- Modify: `src/compas_fab/ghpython/components_cpython/Cf_TesseractRobotArtifact/metadata.json`
- Modify: `docs/backends/tesseract.md`
- Modify: `docs/developer/grasshopper.md`

**Interfaces:**
- Consumes: loading defaults from Task 1.
- Produces: a visible Grasshopper policy input whose default is `convex_hull`, plus exact-mesh opt-in documentation.

- [x] **Step 1: Write failing Grasshopper default assertions**

```python
assert 'default="convex_hull"' in code
assert '"convex_hull",\n                    "collision mesh policy"' in code
mesh_input = next(item for item in metadata["ghpython"]["inputParameters"] if item["name"] == "collision_mesh_policy")
assert "defaults to convex_hull" in mesh_input["description"]
```

- [x] **Step 2: Verify RED**

Run: `pixi run pytest tests/backends/tesseract/test_grasshopper_components.py -n auto -q`

Expected: assertions report the current `preserve` defaults.

- [x] **Step 3: Change only default selection text and values**

Set both component defaults to `convex_hull`. Document that omitted policy uses convex hull and explicit `CollisionMeshPolicy.PRESERVE` retains exact triangle collision meshes. Remove claims that no documented default exists.

- [x] **Step 4: Verify GREEN**

Run: `pixi run pytest tests/backends/tesseract/test_grasshopper_components.py -n auto -q`

Expected: all tests pass.

### Task 3: Native Axis-Symmetric Example

**Files:**
- Create: `tests/backends/tesseract/test_documented_examples.py`
- Modify: `docs/backends/tesseract/files/01_compas_plan_motion.py`
- Modify: `docs/backends/tesseract/files/02_native_program.py`
- Modify: `docs/backends/tesseract.md`

**Interfaces:**
- Consumes: `TesseractPlanner.plan_native`, `TesseractPlanningRequest.build`, native `MotionProgram`, `CartesianTarget`, and `create_descartes_pipeline_profiles` from 0.35.0.6.
- Produces: an executable UR5 native example using `DescartesFPipeline`, local TCP Z sampling, a named 30-degree angular step, and native redundant joint solutions.

- [x] **Step 1: Write failing source-contract test**

```python
def test_native_example_uses_tesseract_axis_symmetry():
    source = NATIVE_EXAMPLE.read_text(encoding="utf-8")
    assert "DescartesFPipeline" in source
    assert "TOOL_Z_AXIS = (0.0, 0.0, 1.0)" in source
    assert "TOOL_AXIS_SAMPLE_STEP = radians(30.0)" in source
    assert "sample_axis=TOOL_Z_AXIS" in source
    assert "sample_resolution=TOOL_AXIS_SAMPLE_STEP" in source
    assert "use_redundant_joint_solutions=True" in source
```

Also assert neither example passes `collision_mesh_policy=` to `CompasRobotArtifactCompiler.build`.

- [x] **Step 2: Verify RED**

Run: `pixi run pytest tests/backends/tesseract/test_documented_examples.py -n auto -q`

Expected: the native example still contains `FreespacePipeline` and explicit mesh-policy arguments.

- [x] **Step 3: Implement the exact native program**

Use the stored UR5 `up` configuration and a second known joint configuration to derive two reachable `Pose` values with native `Robot.fk`. Build a two-waypoint Cartesian `MotionProgram`, configure profiles exactly as follows, and submit it with `auto_seed=True`:

```python
TOOL_Z_AXIS = (0.0, 0.0, 1.0)
TOOL_AXIS_SAMPLE_STEP = radians(30.0)

profiles = create_descartes_pipeline_profiles(
    sample_axis=TOOL_Z_AXIS,
    sample_resolution=TOOL_AXIS_SAMPLE_STEP,
    use_redundant_joint_solutions=True,
)
request = TesseractPlanningRequest.build(
    program=program,
    pipeline="DescartesFPipeline",
    profiles=profiles,
    auto_seed=True,
)
```

Remove explicit default mesh-policy imports and arguments from both examples.

- [x] **Step 4: Verify source contract and execute both examples**

Run: `pixi run pytest tests/backends/tesseract/test_documented_examples.py -n auto -q`

Run: `pixi run python docs/backends/tesseract/files/01_compas_plan_motion.py`

Run: `pixi run python docs/backends/tesseract/files/02_native_program.py`

Expected: tests pass; both processes exit zero; the native result reports successful Descartes planning.

### Task 3A: Descartes OpenMP Runtime Contract

**Files:**
- Modify: `tests/backends/tesseract/test_install_contract.py`
- Modify: `pyproject.toml`
- Modify: `pixi.lock`
- Modify: `docs/backends/tesseract.md`

**Interfaces:**
- Consumes: the released macOS nanobind wheel's bundled `libomp.dylib` and conda-forge BLAS variants.
- Produces: pthreads OpenBLAS in every locked environment/platform, leaving Descartes as the only initialized OpenMP runtime.

- [x] **Step 1: Reproduce and trace OMP Error #15**

`DYLD_PRINT_LIBRARIES=1` showed conda OpenBLAS loading `.pixi/envs/default/lib/libomp.dylib` before `libdescartes_light.dylib` loaded the wheel's `.dylibs/libomp.dylib`.

- [x] **Step 2: Add a failing manifest/lock contract test**

Require `libopenblas = { version = ">=0.3.30,<0.4", build = "*pthreads*" }`, require at least one locked OpenBLAS package, and require every locked OpenBLAS locator to contain `-pthreads_`.

- [x] **Step 3: Pin pthreads OpenBLAS and regenerate with Pixi**

Add the manifest constraint and run `pixi install`. Do not set `KMP_DUPLICATE_LIB_OK`.

- [x] **Step 4: Verify the root-cause fix**

The contract test passes, `otool -L .pixi/envs/default/lib/libopenblas.0.dylib` has no `libomp` dependency, and the real native UR5 Descartes example exits zero with `Planning successful`.

### Task 4: Regression Gates

**Files:**
- Verify all modified files above.

**Interfaces:**
- Consumes: Tasks 1-3.
- Produces: formatted, typed, documented, regression-tested branch state.

- [x] **Step 1: Run affected tests through testmon**

Run: `pixi run pytest tests/backends/tesseract --testmon -n auto -q`

Expected: all selected tests pass with no skips or xfails.

- [x] **Step 2: Run scoped lint, format check, and strict typing**

Run the repository's `lint-tesseract`, format-check, and `typecheck` Pixi tasks. Expected: zero errors.

- [x] **Step 3: Run backend and full suites**

Run: `pixi run pytest tests/backends/tesseract -n auto -q`

Run: `pixi run pytest -n auto -q`

Expected: all Tesseract tests pass; full-suite skips are limited to the repository's existing opt-in ROS tests.

- [x] **Step 4: Build documentation and inspect repository state**

Run: `pixi run mkdocs build --strict` if the repository supports strict mode; otherwise run the existing MkDocs task and record its known plugin warning. Then run `git diff --check` and `git status --short`.

Expected: documentation builds, `git diff --check` is silent, and no commit is created.
