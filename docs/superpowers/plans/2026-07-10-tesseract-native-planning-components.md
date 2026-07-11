# Tesseract Native Planning Components Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Ship eight compiled Grasshopper components that author, plan, and inspect exact Tesseract nanobind programs without reducing them to COMPAS FAB planning types.

**Architecture:** Five focused backend modules own pose, target, program, profile, and result contracts; a sixth module content-addresses mutable native programs for cache invalidation. Grasshopper nodes remain thin adapters over those factories, while native `MotionProgram`, `CompositeInstruction`, `ProfileDictionary`, `TesseractPlanningRequest`, and `TesseractPlanningResult` stay directly accessible. Windows CI compiles and requires all twelve Tesseract user objects.

**Tech Stack:** Python 3.9/3.12, attrs, NumPy, COMPAS `Frame`, tesseract-robotics-nanobind 0.35.0.6, native Tesseract serialization, Grasshopper CPython componentizer, pytest-xdist/testmon, Ruff, mypy strict, MkDocs.

## Global Constraints

- Require `tesseract-robotics-nanobind==0.35.0.6`; never install or import `tesseract_python` or `tesseract-python`.
- Python 3.9 is mandatory for Rhino 8; macOS ARM64/Python 3.12 remains the primary runtime.
- Exact native values are the graph currency. COMPAS FAB never replaces them with a smaller target, profile, program, or trajectory model.
- `Tesseract Pose` is the only geometry/unit conversion node and always requires explicit finite positive metres per user unit.
- All three native target types and all three native `MoveType` values remain available.
- `MotionProgram.add_target(...)` preserves authored move types; never route targets through convenience methods that overwrite them.
- Unconnected Descartes options remain `None`; native 0.35.0.6 defaults are never restated or silently changed.
- `Tesseract Native Plan` calls only `TesseractPlanner.plan_native(...)` with an exact required `ProfileDictionary`.
- Missing native dynamics remain `None`; never manufacture zero velocity, acceleration, or time values.
- No fallback, retry, conditional import, `HAS_*`, skip, xfail, generic exception catch, filesystem write, or conventional planner path.
- Run every pytest command with `-n auto`; run `pytest --testmon -n auto` after code changes.
- Use direct imports and keep `src/compas_fab/backends/tesseract/__init__.py` minimal; do not add `__all__`.
- Commit permission is active for this tranche. Use concise commits authored and committed by `Jelle Feringa <jelleferinga@gmail.com>`; never push without a separate instruction.
- Mobile/VKC remains last. Scene commands, process planning, `tesseract_3s_slicer`, and `tesseract_concurrent_trajopt` remain outside this tranche.

## File Responsibility Map

- `native_pose.py`: explicit user-unit `Frame` to native metre `Pose` only.
- `native_targets.py`: exact Cartesian, joint, and state target construction only.
- `native_program_builder.py`: validate a robot/group/frame/target sequence and build `MotionProgram` plus `CompositeInstruction`.
- `descartes_profiles.py`: validate every optional Descartes factory argument and forward it unchanged.
- `native_program_identity.py`: versioned SHA-256 over native binary `CompositeInstruction` serialization.
- `native_result_view.py`: lossless native result inspection without COMPAS trajectory projection.
- Each `Cf_Tesseract*` directory: one Grasshopper node with source, metadata, and one 24x24 icon.

---

### Task 1: Explicit Native Pose Boundary

**Files:**
- Modify: `src/compas_fab/backends/tesseract/errors.py`
- Create: `src/compas_fab/backends/tesseract/native_pose.py`
- Create: `tests/backends/tesseract/test_native_pose.py`

**Interfaces:**
- Consumes: `compas.geometry.Frame`, explicit user-unit scale.
- Produces: `pose_from_user_frame(frame: Frame, metres_per_user_unit: object) -> tesseract_robotics.planning.Pose`.

- [ ] **Step 1: Write failing pose contract tests**

```python
import numpy as np
import pytest
from compas.geometry import Frame
from tesseract_robotics.planning import Pose

from compas_fab.backends.tesseract.errors import InvalidTesseractPoseError
from compas_fab.backends.tesseract.native_pose import pose_from_user_frame


def test_pose_converts_only_position_from_millimetres_to_metres():
    pose = pose_from_user_frame(Frame([1000, 2000, 3000], [0, 1, 0], [-1, 0, 0]), 0.001)
    assert isinstance(pose, Pose)
    np.testing.assert_allclose(pose.position, [1.0, 2.0, 3.0])
    np.testing.assert_allclose(pose.rotation_matrix, [[0, -1, 0], [1, 0, 0], [0, 0, 1]])


@pytest.mark.parametrize("scale", [None, True, 0.0, -1.0, float("inf"), float("nan")])
def test_pose_requires_explicit_finite_positive_scale(scale):
    with pytest.raises(InvalidTesseractPoseError):
        pose_from_user_frame(Frame.worldXY(), scale)


def test_pose_rejects_non_frame_and_non_finite_coordinates():
    with pytest.raises(InvalidTesseractPoseError):
        pose_from_user_frame(object(), 1.0)
    frame = Frame.worldXY()
    frame.point.x = float("nan")
    with pytest.raises(InvalidTesseractPoseError):
        pose_from_user_frame(frame, 1.0)
```

- [ ] **Step 2: Verify RED**

Run: `pixi run pytest tests/backends/tesseract/test_native_pose.py -n auto -q`

Expected: collection fails because `native_pose` and `InvalidTesseractPoseError` do not exist.

- [ ] **Step 3: Implement the typed conversion boundary**

Add the named error:

```python
class InvalidTesseractPoseError(TesseractBackendError):
    """A geometry/unit boundary cannot produce an exact native pose."""
```

Implement the factory with `Transformation.from_frame(frame)`, finite-matrix validation, `MetersPerUserUnit(float(scale))`, and `Pose.from_matrix_position(rotation, position * scale)`. Reject `bool` separately from numeric values. Do not call `meters_per_user_unit(None)` because that existing conventional helper intentionally defaults to metres.

```python
def pose_from_user_frame(frame: Frame, metres_per_user_unit: object) -> Pose:
    if not isinstance(frame, Frame):
        raise InvalidTesseractPoseError("Tesseract pose requires compas.geometry.Frame, got {}.".format(type(frame).__name__))
    scale = _explicit_scale(metres_per_user_unit)
    matrix = np.asarray(Transformation.from_frame(frame).matrix, dtype=np.float64)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise InvalidTesseractPoseError("Tesseract pose frame must contain finite coordinates and axes.")
    return Pose.from_matrix_position(matrix[:3, :3], matrix[:3, 3] * scale)
```

- [ ] **Step 4: Verify GREEN and type gate**

Run: `pixi run pytest tests/backends/tesseract/test_native_pose.py -n auto -q`

Run: `pixi run mypy --strict src/compas_fab/backends/tesseract/native_pose.py`

Expected: pose tests and strict typing pass.

- [ ] **Step 5: Commit**

```bash
git add src/compas_fab/backends/tesseract/errors.py src/compas_fab/backends/tesseract/native_pose.py tests/backends/tesseract/test_native_pose.py
git commit -m "feat: add native pose boundary"
```

### Task 2: Exact Native Target Factories

**Files:**
- Modify: `src/compas_fab/backends/tesseract/errors.py`
- Create: `src/compas_fab/backends/tesseract/native_targets.py`
- Create: `tests/backends/tesseract/test_native_targets.py`

**Interfaces:**
- Consumes: exact native `Pose`, native joint values, `MoveType`, exact profile names.
- Produces: `build_cartesian_target(...) -> CartesianTarget`, `build_joint_target(...) -> JointTarget`, and `build_state_target(...) -> StateTarget`.

- [ ] **Step 1: Write failing exact-type and move-type tests**

```python
import numpy as np
import pytest
from tesseract_robotics.planning import CartesianTarget, JointTarget, MoveType, Pose, StateTarget

from compas_fab.backends.tesseract.errors import InvalidTesseractTargetError
from compas_fab.backends.tesseract.native_targets import build_cartesian_target, build_joint_target, build_state_target


@pytest.mark.parametrize("move_type", list(MoveType))
def test_cartesian_target_retains_exact_pose_and_every_move_type(move_type):
    pose = Pose.from_xyz(0.1, 0.2, 0.3)
    target = build_cartesian_target(pose, move_type, "CARTESIAN")
    assert isinstance(target, CartesianTarget)
    assert target.pose is pose
    assert target.move_type is move_type
    assert target.profile == "CARTESIAN"


def test_joint_target_preserves_explicit_name_order():
    target = build_joint_target([1.0, 2.0], ["joint_b", "joint_a"], MoveType.FREESPACE, "DEFAULT")
    assert isinstance(target, JointTarget)
    np.testing.assert_array_equal(target.positions, [1.0, 2.0])
    assert target.names == ["joint_b", "joint_a"]


def test_state_target_keeps_unconnected_dynamics_absent():
    target = build_state_target([1.0], None, None, None, None, MoveType.LINEAR, "DEFAULT")
    assert isinstance(target, StateTarget)
    assert target.names is None
    assert target.velocities is None
    assert target.accelerations is None
    assert target.time is None
```

- [ ] **Step 2: Add failing validation cases**

```python
@pytest.mark.parametrize(
    "factory,args",
    [
        (build_cartesian_target, (object(), MoveType.FREESPACE, "DEFAULT")),
        (build_joint_target, ([0.0, float("nan")], None, MoveType.FREESPACE, "DEFAULT")),
        (build_joint_target, ([0.0], ["joint", "extra"], MoveType.FREESPACE, "DEFAULT")),
        (build_joint_target, ([0.0, 1.0], ["joint", "joint"], MoveType.FREESPACE, "DEFAULT")),
        (build_state_target, ([0.0], None, [0.0, 1.0], None, None, MoveType.FREESPACE, "DEFAULT")),
        (build_state_target, ([0.0], None, None, None, -1.0, MoveType.FREESPACE, "DEFAULT")),
    ],
)
def test_target_factories_fail_loudly(factory, args):
    with pytest.raises(InvalidTesseractTargetError):
        factory(*args)
```

- [ ] **Step 3: Verify RED**

Run: `pixi run pytest tests/backends/tesseract/test_native_targets.py -n auto -q`

Expected: collection fails because the target factory module and named error do not exist.

- [ ] **Step 4: Implement semantic joint-unit validation and exact factories**

Add `InvalidTesseractTargetError`. Define semantic `NewType` values `NativeJointPosition`, `NativeJointVelocity`, `NativeJointAcceleration`, and `NativeTimeSeconds`. Centralize finite one-dimensional array validation, exact non-empty profile validation, native `MoveType` validation, and optional unique non-empty joint names.

```python
def build_cartesian_target(pose: object, move_type: object, profile: object) -> CartesianTarget:
    if not isinstance(pose, Pose):
        raise InvalidTesseractTargetError("Cartesian target requires exact native Pose, got {}.".format(type(pose).__name__))
    return CartesianTarget(pose=pose, move_type=_move_type(move_type), profile=_profile(profile))


def build_joint_target(positions, names, move_type, profile) -> JointTarget:
    values = _finite_vector(positions, "positions")
    joint_names = _joint_names(names, values.size)
    return JointTarget(values, names=joint_names, move_type=_move_type(move_type), profile=_profile(profile))


def build_state_target(positions, names, velocities, accelerations, time, move_type, profile) -> StateTarget:
    values = _finite_vector(positions, "positions")
    joint_names = _joint_names(names, values.size)
    velocity_values = _optional_vector(velocities, "velocities", values.size)
    acceleration_values = _optional_vector(accelerations, "accelerations", values.size)
    seconds = _optional_non_negative_time(time)
    return StateTarget(values, names=joint_names, velocities=velocity_values, accelerations=acceleration_values, time=seconds, move_type=_move_type(move_type), profile=_profile(profile))
```

- [ ] **Step 5: Verify GREEN and strict typing**

Run: `pixi run pytest tests/backends/tesseract/test_native_targets.py -n auto -q`

Run: `pixi run mypy --strict src/compas_fab/backends/tesseract/native_targets.py`

Expected: target tests and type gate pass.

- [ ] **Step 6: Commit**

```bash
git add src/compas_fab/backends/tesseract/errors.py src/compas_fab/backends/tesseract/native_targets.py tests/backends/tesseract/test_native_targets.py
git commit -m "feat: add native target factories"
```

### Task 3: Native Motion Program Builder and Identity

**Files:**
- Modify: `src/compas_fab/backends/tesseract/errors.py`
- Create: `src/compas_fab/backends/tesseract/native_program_builder.py`
- Create: `src/compas_fab/backends/tesseract/native_program_identity.py`
- Modify: `tests/backends/tesseract/conftest.py`
- Create: `tests/backends/tesseract/test_native_program_builder.py`
- Create: `tests/backends/tesseract/test_native_program_identity.py`

**Interfaces:**
- Consumes: exact native `Robot`, ordered native target sequence, group/TCP/working-frame/profile names.
- Produces: frozen `NativeProgramBuild` with exact `motion_program`, exact `composite_instruction`, resolved `joint_names`, and resolved `tcp_frame`; `native_program_digest(program) -> NativeProgramIdentity`.

- [ ] **Step 1: Write failing builder tests against a real one-joint robot**

First add a lifetime-correct fixture; the client remains connected for the complete test and is always disconnected afterward:

```python
@pytest.fixture
def tesseract_robot(tesseract_artifact, tmp_path):
    client = TesseractClient(tesseract_artifact, cache_root=tmp_path)
    client.connect()
    try:
        yield client.clone_robot()
    finally:
        client.disconnect()
```

```python
from tesseract_robotics.planning import JointTarget, MotionProgram, MoveType
from tesseract_robotics.tesseract_command_language import CompositeInstruction

from compas_fab.backends.tesseract.native_program_builder import build_motion_program


def test_builder_resolves_group_joint_order_and_tcp(tesseract_robot):
    target = JointTarget([0.1], move_type=MoveType.LINEAR, profile="CUSTOM")
    built = build_motion_program(tesseract_robot, [target], "manipulator", None, "base", "PROGRAM")
    assert isinstance(built.motion_program, MotionProgram)
    assert isinstance(built.composite_instruction, CompositeInstruction)
    assert built.motion_program.targets[0] is target
    assert target.move_type is MoveType.LINEAR
    assert built.joint_names == ("joint1",)
    assert built.tcp_frame == "tip"


def test_builder_does_not_mutate_robot_or_targets(tesseract_robot):
    target = JointTarget([0.1], names=None, profile="DEFAULT")
    before = tesseract_robot.get_state(["joint1"]).joint_positions.copy()
    build_motion_program(tesseract_robot, [target], "manipulator", None, "base", "DEFAULT")
    assert target.names is None
    np.testing.assert_array_equal(tesseract_robot.get_state(["joint1"]).joint_positions, before)
```

- [ ] **Step 2: Add failing invariant tests**

```python
@pytest.mark.parametrize(
    "targets,group,tcp,working,profile",
    [([], "manipulator", None, "base", "DEFAULT"), ([object()], "manipulator", None, "base", "DEFAULT"), ([JointTarget([0.0])], "", None, "base", "DEFAULT"), ([JointTarget([0.0])], "manipulator", "missing", "base", "DEFAULT")],
)
def test_builder_rejects_invalid_program_inputs(tesseract_robot, targets, group, tcp, working, profile):
    with pytest.raises(InvalidTesseractMotionProgramError):
        build_motion_program(tesseract_robot, targets, group, tcp, working, profile)


def test_builder_rejects_explicit_names_outside_selected_group(tesseract_robot):
    with pytest.raises(InvalidTesseractMotionProgramError, match="joint order"):
        build_motion_program(tesseract_robot, [JointTarget([0.0], names=["other"])], "manipulator", None, "base", "DEFAULT")
```

- [ ] **Step 3: Write failing content identity tests**

```python
def test_native_program_digest_is_stable_and_versioned(native_composite):
    left = native_program_digest(native_composite)
    right = native_program_digest(native_composite)
    assert left == right
    assert len(left.digest) == 64
    assert left.schema_version == NATIVE_PROGRAM_IDENTITY_SCHEMA_VERSION
    assert left.tesseract_version == "0.35.0.6"


def test_native_program_digest_detects_in_place_mutation(native_composite):
    before = native_program_digest(native_composite)
    native_composite.setDescription("changed")
    assert native_program_digest(native_composite) != before
```

Define `native_composite` in the identity test module from the real fixture so it is not a synthetic polymorphic instruction:

```python
@pytest.fixture
def native_composite(tesseract_robot):
    return build_motion_program(
        tesseract_robot,
        [JointTarget([0.0], profile="DEFAULT")],
        "manipulator",
        None,
        "base",
        "DEFAULT",
    ).composite_instruction
```

- [ ] **Step 4: Verify RED**

Run: `pixi run pytest tests/backends/tesseract/test_native_program_builder.py tests/backends/tesseract/test_native_program_identity.py -n auto -q`

Expected: collection fails because the builder, identity, and named error do not exist.

- [ ] **Step 5: Implement the exact program builder**

Add `InvalidTesseractMotionProgramError`. Use `Robot.get_joint_names(group_name)` and `Robot.get_manipulator_info(group_name, tcp_frame, working_frame)` as the native authorities. Validate selected frames against `robot.get_link_names()`. Validate explicit `JointTarget.names` and `StateTarget.names` as exactly equal to native group order. Build with `MotionProgram(...).set_joint_names(joint_names)` and call `add_target(target)` for every exact target before `to_composite_instruction(joint_names, resolved_tcp)`.

```python
@define(frozen=True, slots=True)
class NativeProgramBuild:
    motion_program: MotionProgram
    composite_instruction: CompositeInstruction
    joint_names: tuple[str, ...]
    tcp_frame: str

    def __attrs_post_init__(self) -> None:
        _validate_built_program(self.motion_program, self.composite_instruction, self.joint_names, self.tcp_frame)


def build_motion_program(robot, targets, group_name, tcp_frame, working_frame, profile) -> NativeProgramBuild:
    native_robot = _robot(robot)
    group = _name(group_name, "group")
    working = _name(working_frame, "working frame")
    program_profile = _name(profile, "program profile")
    joint_names = tuple(native_robot.get_joint_names(group))
    resolved_tcp = native_robot.get_manipulator_info(group, tcp_frame or None, working).tcp_frame
    native_targets = _targets(targets, joint_names)
    program = MotionProgram(group, tcp_frame=resolved_tcp, working_frame=working, profile=program_profile).set_joint_names(list(joint_names))
    for target in native_targets:
        program.add_target(target)
    composite = program.to_composite_instruction(list(joint_names), resolved_tcp)
    return NativeProgramBuild(program, composite, joint_names, resolved_tcp)
```

- [ ] **Step 6: Implement versioned native binary identity**

Use `composite_instruction_to_binary` from `tesseract_robotics.tesseract_serialization`, a schema version string, installed COMPAS FAB and nanobind versions, length-prefixed fields, and SHA-256. Reject anything except exact `CompositeInstruction` with `InvalidTesseractMotionProgramError`.

```python
@define(frozen=True, slots=True)
class NativeProgramIdentity:
    digest: NativeProgramDigest
    schema_version: str
    compas_fab_version: str
    tesseract_version: str


def native_program_digest(program: object) -> NativeProgramIdentity:
    if not isinstance(program, CompositeInstruction):
        raise InvalidTesseractMotionProgramError("Native program identity requires CompositeInstruction.")
    payload = bytes(composite_instruction_to_binary(program))
    compas_version = version("compas-fab")
    tesseract_version = version("tesseract-robotics-nanobind")
    digest = hashlib.sha256(_identity_payload(payload, compas_version, tesseract_version)).hexdigest()
    return NativeProgramIdentity(NativeProgramDigest(digest), NATIVE_PROGRAM_IDENTITY_SCHEMA_VERSION, compas_version, tesseract_version)
```

- [ ] **Step 7: Verify GREEN and strict typing**

Run: `pixi run pytest tests/backends/tesseract/test_native_program_builder.py tests/backends/tesseract/test_native_program_identity.py -n auto -q`

Run: `pixi run mypy --strict src/compas_fab/backends/tesseract/native_program_builder.py src/compas_fab/backends/tesseract/native_program_identity.py`

Expected: program and identity contracts pass.

- [ ] **Step 8: Commit**

```bash
git add src/compas_fab/backends/tesseract/errors.py src/compas_fab/backends/tesseract/native_program_builder.py src/compas_fab/backends/tesseract/native_program_identity.py tests/backends/tesseract/conftest.py tests/backends/tesseract/test_native_program_builder.py tests/backends/tesseract/test_native_program_identity.py
git commit -m "feat: add native program builder"
```

### Task 4: Complete Descartes Profile Boundary

**Files:**
- Modify: `src/compas_fab/backends/tesseract/errors.py`
- Create: `src/compas_fab/backends/tesseract/descartes_profiles.py`
- Create: `tests/backends/tesseract/test_descartes_profiles.py`

**Interfaces:**
- Consumes: every argument of native `create_descartes_pipeline_profiles(...)`.
- Produces: `build_descartes_profiles(...) -> ProfileDictionary` without replacing omitted values.

- [ ] **Step 1: Write failing forwarding and default-preservation tests**

```python
def test_unconnected_options_are_forwarded_as_none(mocker):
    native = mocker.patch("compas_fab.backends.tesseract.descartes_profiles.create_descartes_pipeline_profiles", return_value=ProfileDictionary())
    profiles = build_descartes_profiles(None, None, None, None, None, None, None, None, None, None, None)
    assert isinstance(profiles, ProfileDictionary)
    native.assert_called_once_with(profile_names=None, enable_collision=None, enable_edge_collision=None, num_threads=None, sample_axis=None, sample_resolution=None, sample_min=None, sample_max=None, ik_solver=None, use_redundant_joint_solutions=None, move_profile=None)


def test_full_range_one_degree_redundancy_is_forwarded_exactly(mocker):
    native = mocker.patch("compas_fab.backends.tesseract.descartes_profiles.create_descartes_pipeline_profiles", return_value=ProfileDictionary())
    build_descartes_profiles(["DEFAULT"], True, False, 8, [0, 0, 1], radians(1), -pi, pi, "OPWInvKin", True, None)
    kwargs = native.call_args.kwargs
    np.testing.assert_array_equal(kwargs["sample_axis"], [0.0, 0.0, 1.0])
    assert kwargs["sample_resolution"] == pytest.approx(radians(1))
    assert kwargs["sample_min"] == pytest.approx(-pi)
    assert kwargs["sample_max"] == pytest.approx(pi)
    assert kwargs["use_redundant_joint_solutions"] is True
```

- [ ] **Step 2: Add failing invariant and custom-profile tests**

```python
@pytest.mark.parametrize("axis", [[0, 0], [0, 0, 0], [0, float("nan"), 1]])
def test_invalid_sample_axis_fails(axis):
    with pytest.raises(InvalidTesseractDescartesProfileError):
        build_descartes_profiles(None, None, None, None, axis, None, None, None, None, None, None)


def test_custom_move_profile_conflicts_fail_before_native_call():
    custom = DescartesDefaultMoveProfileD()
    with pytest.raises(InvalidTesseractDescartesProfileError, match="full override"):
        build_descartes_profiles(None, True, None, None, None, None, None, None, None, None, custom)
```

- [ ] **Step 3: Verify RED**

Run: `pixi run pytest tests/backends/tesseract/test_descartes_profiles.py -n auto -q`

Expected: collection fails because the boundary and named error do not exist.

- [ ] **Step 4: Implement exact validation and forwarding**

Add `InvalidTesseractDescartesProfileError`. Validate optional profile names, booleans, positive integer thread count, finite non-degenerate three-vector axis, positive finite resolution, finite ordered bounds, non-empty IK solver, redundant flag, and exact `DescartesMoveProfileD`. Do not normalize the axis length because magnitude belongs to the caller/native API. A custom move profile conflicts with every move-profile convenience argument but not `profile_names` or `num_threads`.

```python
def build_descartes_profiles(profile_names, enable_collision, enable_edge_collision, num_threads, sample_axis, sample_resolution, sample_min, sample_max, ik_solver, use_redundant_joint_solutions, move_profile) -> ProfileDictionary:
    names = _optional_profile_names(profile_names)
    collision = _optional_bool(enable_collision, "enable_collision")
    edge_collision = _optional_bool(enable_edge_collision, "enable_edge_collision")
    threads = _optional_positive_int(num_threads, "num_threads")
    axis = _optional_axis(sample_axis)
    resolution = _optional_positive_float(sample_resolution, "sample_resolution")
    lower = _optional_finite_float(sample_min, "sample_min")
    upper = _optional_finite_float(sample_max, "sample_max")
    _ordered_bounds(lower, upper)
    solver = _optional_name(ik_solver, "ik_solver")
    redundant = _optional_bool(use_redundant_joint_solutions, "use_redundant_joint_solutions")
    custom = _optional_move_profile(move_profile)
    _reject_custom_conflicts(custom, collision, edge_collision, axis, resolution, lower, upper, solver, redundant)
    return create_descartes_pipeline_profiles(profile_names=names, enable_collision=collision, enable_edge_collision=edge_collision, num_threads=threads, sample_axis=axis, sample_resolution=resolution, sample_min=lower, sample_max=upper, ik_solver=solver, use_redundant_joint_solutions=redundant, move_profile=custom)
```

- [ ] **Step 5: Lock released defaults with a real profile contract test**

Keep the existing runtime assertions for `DescartesDefaultMoveProfileD`: fixed pose `True`, dormant axis `[0, 0, 1]`, dormant resolution `pi / 2`, dormant range `[-pi, pi / 2]`, and redundant solutions `False`. Add one real call proving `build_descartes_profiles(None, ..., None)` succeeds without substituting values.

- [ ] **Step 6: Verify GREEN and strict typing**

Run: `pixi run pytest tests/backends/tesseract/test_descartes_profiles.py tests/backends/tesseract/test_documented_examples.py -n auto -q`

Run: `pixi run mypy --strict src/compas_fab/backends/tesseract/descartes_profiles.py`

Expected: forwarding, native-default, and type contracts pass.

- [ ] **Step 7: Commit**

```bash
git add src/compas_fab/backends/tesseract/errors.py src/compas_fab/backends/tesseract/descartes_profiles.py tests/backends/tesseract/test_descartes_profiles.py tests/backends/tesseract/test_documented_examples.py
git commit -m "feat: add Descartes profile boundary"
```

### Task 5: Lossless Native Result View

**Files:**
- Modify: `src/compas_fab/backends/tesseract/errors.py`
- Create: `src/compas_fab/backends/tesseract/native_result_view.py`
- Create: `tests/backends/tesseract/test_native_result_view.py`

**Interfaces:**
- Consumes: exact `TesseractPlanningResult`.
- Produces: frozen `TesseractNativeResultView.build(result)` retaining exact native objects plus validated tuple views.

- [ ] **Step 1: Write failing retention and absent-dynamics tests**

```python
def test_result_view_retains_every_exact_native_object(native_result_without_dynamics):
    view = TesseractNativeResultView.build(native_result_without_dynamics)
    assert view.result is native_result_without_dynamics
    assert view.request is native_result_without_dynamics.request
    assert view.native_result is native_result_without_dynamics.native_result
    assert view.raw_program is native_result_without_dynamics.raw_program
    assert view.trajectory_points == tuple(native_result_without_dynamics.native_result.trajectory)
    assert view.velocities is None
    assert view.accelerations is None
    assert view.times is None


def test_result_view_exposes_ordered_native_arrays(native_result_with_dynamics):
    view = TesseractNativeResultView.build(native_result_with_dynamics)
    assert view.joint_names == ("joint1", "joint2")
    assert view.positions == ((0.0, 1.0), (0.5, 1.5))
    assert view.velocities == ((0.0, 0.0), (0.1, 0.2))
    assert view.accelerations == ((0.0, 0.0), (0.01, 0.02))
    assert view.times == (0.0, 1.0)
```

Define both fixtures locally with exact `TesseractPlanningRequest`, native `PlanningResult`, and `TrajectoryPoint` values:

```python
def _result(points):
    program = CompositeInstruction("DEFAULT")
    program.push_back(SetDigitalInstruction("do_test", 0, True))
    request = TesseractPlanningRequest.build(program, "DescartesFPipeline", ProfileDictionary(), False)
    return TesseractPlanningResult.build(request, PlanningResult(successful=True, message="ok", trajectory=points, raw_results=program))


@pytest.fixture
def native_result_without_dynamics():
    return _result([TrajectoryPoint(["joint1"], np.array([0.0]))])


@pytest.fixture
def native_result_with_dynamics():
    return _result([
        TrajectoryPoint(["joint1", "joint2"], np.array([0.0, 1.0]), np.array([0.0, 0.0]), np.array([0.0, 0.0]), 0.0),
        TrajectoryPoint(["joint1", "joint2"], np.array([0.5, 1.5]), np.array([0.1, 0.2]), np.array([0.01, 0.02]), 1.0),
    ])
```

- [ ] **Step 2: Add failing malformed-result tests**

```python
def test_result_view_rejects_mixed_optional_field_presence():
    result = _result([
        TrajectoryPoint(["joint1"], np.array([0.0]), velocities=None),
        TrajectoryPoint(["joint1"], np.array([1.0]), velocities=np.array([0.1])),
    ])
    with pytest.raises(MalformedTesseractNativeResultError, match="velocities"):
        TesseractNativeResultView.build(result)


@pytest.mark.parametrize(
    "points",
    [
        [TrajectoryPoint(["joint1"], np.array([0.0])), TrajectoryPoint(["other"], np.array([1.0]))],
        [TrajectoryPoint(["joint1"], np.array([0.0, 1.0]))],
        [TrajectoryPoint(["joint1"], np.array([float("nan")]))],
        [TrajectoryPoint(["joint1"], np.array([0.0]), time=-1.0)],
        [TrajectoryPoint(["joint1"], np.array([0.0]), time=1.0), TrajectoryPoint(["joint1"], np.array([1.0]), time=0.5)],
    ],
)
def test_result_view_rejects_malformed_native_trajectory(points):
    with pytest.raises(MalformedTesseractNativeResultError):
        TesseractNativeResultView.build(_result(points))
```

- [ ] **Step 3: Verify RED**

Run: `pixi run pytest tests/backends/tesseract/test_native_result_view.py -n auto -q`

Expected: collection fails because the view and named error do not exist.

- [ ] **Step 4: Implement a bypass-safe frozen view**

Add `MalformedTesseractNativeResultError`. Define semantic `NewType` values for native joint positions, velocities, accelerations, and seconds. `__attrs_post_init__` must revalidate exact object relationships and all tuple shapes so a raw constructor cannot bypass `build`.

```python
@define(frozen=True, slots=True)
class TesseractNativeResultView:
    result: TesseractPlanningResult
    request: TesseractPlanningRequest
    native_result: PlanningResult
    raw_program: CompositeInstruction
    message: str
    trajectory_points: tuple[TrajectoryPoint, ...]
    joint_names: tuple[str, ...]
    positions: tuple[tuple[float, ...], ...]
    velocities: Optional[tuple[tuple[float, ...], ...]]
    accelerations: Optional[tuple[tuple[float, ...], ...]]
    times: Optional[tuple[float, ...]]

    @classmethod
    def build(cls, result: object) -> TesseractNativeResultView:
        native_result = _exact_result(result)
        points = tuple(native_result.native_result.trajectory)
        names, positions, velocities, accelerations, times = _validated_fields(points)
        return cls(native_result, native_result.request, native_result.native_result, native_result.raw_program, native_result.native_result.message, points, names, positions, velocities, accelerations, times)
```

An empty successful trajectory is valid for exact inspection: return empty tuple positions/names and `None` optional fields. Reject duplicate/empty names, changing order, non-finite values, shape mismatches, mixed optional presence, negative time, and decreasing time.

- [ ] **Step 5: Verify GREEN and strict typing**

Run: `pixi run pytest tests/backends/tesseract/test_native_result_view.py -n auto -q`

Run: `pixi run mypy --strict src/compas_fab/backends/tesseract/native_result_view.py`

Expected: exact retention and malformed-result contracts pass.

- [ ] **Step 6: Commit**

```bash
git add src/compas_fab/backends/tesseract/errors.py src/compas_fab/backends/tesseract/native_result_view.py tests/backends/tesseract/test_native_result_view.py
git commit -m "feat: add native result view"
```

### Task 6: Pose and Target Grasshopper Components

**Files:**
- Create: `src/compas_fab/ghpython/components_cpython/Cf_TesseractPose/{code.py,metadata.json,icon.svg,icon.png}`
- Create: `src/compas_fab/ghpython/components_cpython/Cf_TesseractCartesianTarget/{code.py,metadata.json,icon.svg,icon.png}`
- Create: `src/compas_fab/ghpython/components_cpython/Cf_TesseractJointTarget/{code.py,metadata.json,icon.svg,icon.png}`
- Create: `src/compas_fab/ghpython/components_cpython/Cf_TesseractStateTarget/{code.py,metadata.json,icon.svg,icon.png}`
- Modify: `tests/backends/tesseract/test_grasshopper_components.py`

**Interfaces:**
- Consumes: Task 1 and Task 2 factories.
- Produces: four compiled-node sources outputting exact native `Pose`, `CartesianTarget`, `JointTarget`, and `StateTarget`.

- [ ] **Step 1: Write failing source and metadata contracts**

```python
@pytest.mark.parametrize(
    "component,inputs,outputs,factory",
    [
        ("Cf_TesseractPose", ["frame", "metres_per_user_unit"], ["pose"], "pose_from_user_frame"),
        ("Cf_TesseractCartesianTarget", ["pose", "move_type", "profile"], ["target"], "build_cartesian_target"),
        ("Cf_TesseractJointTarget", ["positions", "joint_names", "move_type", "profile"], ["target"], "build_joint_target"),
        ("Cf_TesseractStateTarget", ["positions", "joint_names", "velocities", "accelerations", "time", "move_type", "profile"], ["target"], "build_state_target"),
    ],
)
def test_native_authoring_component_contract(component, inputs, outputs, factory):
    code, metadata = _component(component)
    assert [item["name"] for item in metadata["ghpython"]["inputParameters"]] == inputs
    assert [item["name"] for item in metadata["ghpython"]["outputParameters"]] == outputs
    assert factory in code
    assert "# r: tesseract-robotics-nanobind==0.35.0.6" in code
    assert "except TesseractBackendError" in code
    assert "except Exception" not in code
    assert _png_size(COMPONENTS / component / "icon.png") == (24, 24)
```

Also assert `ensure_value_list(..., "move_type", ["FREESPACE", "LINEAR", "CIRCULAR"], default="FREESPACE")` appears in all three target sources and the Cartesian source does not import any COMPAS/Rhino conversion.

- [ ] **Step 2: Verify RED**

Run: `pixi run pytest tests/backends/tesseract/test_grasshopper_components.py -n auto -q`

Expected: the four component directories are absent.

- [ ] **Step 3: Add Tesseract Pose source and exact metadata**

Use `plane_to_compas_frame` only when the input is not already a COMPAS `Frame`, then call the sole unit conversion factory. Do not infer Rhino document units.

```python
class TesseractPoseComponent(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, frame, metres_per_user_unit: float):
        if frame is None:
            return None
        try:
            compas_frame = frame if isinstance(frame, Frame) else plane_to_compas_frame(frame)
            return pose_from_user_frame(compas_frame, metres_per_user_unit)
        except TesseractBackendError as backend_error:
            error(ghenv.Component, str(backend_error))  # noqa: F821
            return None
```

Metadata must state that coordinates are relative to the downstream program working frame, the scale is explicit metres per Rhino/COMPAS user unit, and output is exact native `Pose` in metres.

- [ ] **Step 4: Add three target sources and exact metadata**

Each source resolves only exact enum text to `MoveType[selection]`, calls its backend factory, catches only `TesseractBackendError`, and returns `None` when required inputs are absent. The Cartesian node accepts only native `Pose`. Joint/state list inputs use list access; optional lists remain `None` when unconnected.

```python
_MOVE_TYPES = [move_type.name for move_type in MoveType]


def _selected_move_type(component, value):
    ensure_value_list(component, "move_type", _MOVE_TYPES, default="FREESPACE")
    try:
        return MoveType[(value or "FREESPACE").strip().upper()]
    except KeyError as enum_error:
        raise InvalidTesseractTargetError("Unknown native move type {!r}.".format(value)) from enum_error
```

Use exact defaults visible in metadata/source: profile `DEFAULT`, move type `FREESPACE`. Do not default positions, pose, joint names, velocities, accelerations, or time.

- [ ] **Step 5: Create distinct Tesseract-system icons**

Author each SVG with the established grey `#4c5561` and teal `#20b8a6` palette and a distinct glyph: coordinate triad for Pose, framed point for Cartesian, joint ticks for Joint, and timed joint ticks for State. Render each exact 24x24 PNG with the repository's available SVG renderer and verify `_png_size` in tests.

- [ ] **Step 6: Verify GREEN**

Run: `pixi run pytest tests/backends/tesseract/test_grasshopper_components.py -n auto -q`

Expected: all four source/metadata/icon contracts pass.

- [ ] **Step 7: Commit**

```bash
git add src/compas_fab/ghpython/components_cpython/Cf_TesseractPose src/compas_fab/ghpython/components_cpython/Cf_TesseractCartesianTarget src/compas_fab/ghpython/components_cpython/Cf_TesseractJointTarget src/compas_fab/ghpython/components_cpython/Cf_TesseractStateTarget tests/backends/tesseract/test_grasshopper_components.py
git commit -m "feat: add native target components"
```

### Task 7: Motion Program and Descartes Profile Components

**Files:**
- Create: `src/compas_fab/ghpython/components_cpython/Cf_TesseractMotionProgram/{code.py,metadata.json,icon.svg,icon.png}`
- Create: `src/compas_fab/ghpython/components_cpython/Cf_TesseractDescartesProfile/{code.py,metadata.json,icon.svg,icon.png}`
- Modify: `tests/backends/tesseract/test_grasshopper_components.py`

**Interfaces:**
- Consumes: `build_motion_program` and `build_descartes_profiles`.
- Produces: exact `MotionProgram`, `CompositeInstruction`, resolved group data, and exact `ProfileDictionary`.

- [ ] **Step 1: Write failing component contracts**

Assert this exact metadata:

```python
assert _io("Cf_TesseractMotionProgram") == (
    ["native_robot", "targets", "group_name", "tcp_frame", "working_frame", "profile"],
    ["motion_program", "program", "joint_names", "tcp_frame"],
)
assert _io("Cf_TesseractDescartesProfile") == (
    ["profile_names", "enable_collision", "enable_edge_collision", "num_threads", "sample_axis", "sample_resolution", "sample_min", "sample_max", "ik_solver", "use_redundant_joint_solutions", "move_profile"],
    ["profiles"],
)
```

Require `build_motion_program`, `build_descartes_profiles`, nanobind directive, no generic catch, no direct `create_descartes_pipeline_profiles` call in component code, no `.move_to(`, `.linear_to(`, or `.circular_to(` in program component code, and 24x24 icons.

- [ ] **Step 2: Verify RED**

Run: `pixi run pytest tests/backends/tesseract/test_grasshopper_components.py -n auto -q`

Expected: both new component directories are absent.

- [ ] **Step 3: Add the Motion Program component**

```python
class TesseractMotionProgramComponent(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, native_robot, targets, group_name: str, tcp_frame: str, working_frame: str, profile: str):
        if native_robot is None or not targets:
            return (None, None, None, None)
        try:
            built = build_motion_program(native_robot, targets, group_name, tcp_frame or None, working_frame or "base_link", profile or "DEFAULT")
        except TesseractBackendError as backend_error:
            error(ghenv.Component, str(backend_error))  # noqa: F821
            return (None, None, None, None)
        return (built.motion_program, built.composite_instruction, list(built.joint_names), built.tcp_frame)
```

Use list access for targets. Metadata must say `native_robot` is the isolated clone from Tesseract Planner, target order/move types are preserved, and `program` connects directly to Native Plan or RAPID.

- [ ] **Step 4: Add the full Descartes Profile component**

Pass all eleven inputs directly to `build_descartes_profiles`. Do not use `or` defaults because `False`, zero validation, and `None` have distinct meanings. Return `None` only after a `TesseractBackendError`. Metadata documents radians for sample angles, tool-local axis, native collision defaults when unconnected, and the full-profile override conflict.

- [ ] **Step 5: Create distinct icons and verify GREEN**

Use a connected instruction-chain glyph for Motion Program and an axis-ring/ladder glyph for Descartes Profile in the same palette. Render exact 24x24 PNGs.

Run: `pixi run pytest tests/backends/tesseract/test_grasshopper_components.py -n auto -q`

Expected: all six new authoring/profile components pass source, metadata, and icon contracts.

- [ ] **Step 6: Commit**

```bash
git add src/compas_fab/ghpython/components_cpython/Cf_TesseractMotionProgram src/compas_fab/ghpython/components_cpython/Cf_TesseractDescartesProfile tests/backends/tesseract/test_grasshopper_components.py
git commit -m "feat: add native program components"
```

### Task 8: Native Plan Cache and Result Components

**Files:**
- Create: `src/compas_fab/ghpython/components_cpython/Cf_TesseractNativePlan/{code.py,metadata.json,icon.svg,icon.png}`
- Create: `src/compas_fab/ghpython/components_cpython/Cf_TesseractNativeResult/{code.py,metadata.json,icon.svg,icon.png}`
- Create: `tests/backends/tesseract/test_native_plan_component.py`
- Modify: `tests/backends/tesseract/test_grasshopper_components.py`

**Interfaces:**
- Consumes: exact `TesseractPlanner`, `CompositeInstruction`, `ProfileDictionary`, and `TesseractPlanningResult`.
- Produces: cached complete native result and lossless result inspection outputs.

- [ ] **Step 1: Extract the plan component script class for host-independent cache tests**

Write tests that load `Cf_TesseractNativePlan/code.py` with small stubs for Grasshopper modules, `ghenv.Component`, `create_id`, and `scriptcontext.sticky`. Use a fake exact `TesseractPlanner` instance whose `plan_native` method captures `TesseractPlanningRequest` and returns a valid `TesseractPlanningResult`.

```python
def test_native_plan_calls_only_plan_native(component, planner, program, profiles):
    result = component.RunScript(planner, program, "DescartesFPipeline", profiles, True, True)
    assert result is planner.result
    assert planner.native_requests[0].program is program
    assert planner.native_requests[0].profiles is profiles
    assert planner.native_requests[0].auto_seed is True


def test_compute_false_reuses_only_complete_matching_signature(component, planner, program, profiles):
    first = component.RunScript(planner, program, "DescartesFPipeline", profiles, True, True)
    assert component.RunScript(planner, program, "DescartesFPipeline", profiles, True, False) is first
    assert len(planner.native_requests) == 1


def test_in_place_program_mutation_invalidates_cached_result(component, planner, program, profiles):
    component.RunScript(planner, program, "DescartesFPipeline", profiles, True, True)
    program.setDescription("mutated")
    assert component.RunScript(planner, program, "DescartesFPipeline", profiles, True, False) is None
```

- [ ] **Step 2: Add failure and every-input invalidation tests**

Parametrize changed planner object, changed program, changed pipeline, changed profile object, and changed `auto_seed`; each `compute=False` call must clear stale output. Make the fake planner raise `TesseractPlanningFailedError` on recompute, then assert a later `compute=False` returns `None` rather than the old success.

- [ ] **Step 3: Verify RED**

Run: `pixi run pytest tests/backends/tesseract/test_native_plan_component.py tests/backends/tesseract/test_grasshopper_components.py -n auto -q`

Expected: both component directories are absent.

- [ ] **Step 4: Implement exact native planning and observable cache signature**

```python
class TesseractNativePlanComponent(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, planner, program, pipeline: str, profiles, auto_seed: bool, compute: bool):
        key = create_id(ghenv.Component, "tesseract_native_plan")  # noqa: F821
        if planner is None or program is None or profiles is None:
            st.pop(key, None)
            return None
        try:
            native_planner = _exact_planner(planner)
            seed = _required_bool(auto_seed, "auto_seed")
            should_compute = _required_bool(compute, "compute")
            identity = native_program_digest(program)
            request = TesseractPlanningRequest.build(program, pipeline, profiles, seed)
            signature = (id(native_planner), identity.digest, request.pipeline, id(profiles), request.auto_seed)
            cached = st.get(key)
            if not should_compute:
                if cached is not None and cached[0] == signature:
                    return cached[1]
                st.pop(key, None)
                return None
            st.pop(key, None)
            result = native_planner.plan_native(request)
            st[key] = (signature, result)
            return result
        except TesseractBackendError as backend_error:
            st.pop(key, None)
            error(ghenv.Component, str(backend_error))  # noqa: F821
            return None
```

Validate `planner` as exact `TesseractPlanner` before use rather than relying on duck typing. Validate `compute` and `auto_seed` as bool in a small named helper so integer `1` is not silently accepted. Profiles remain required. Document that same-object external `ProfileDictionary` mutation is opaque in 0.35.0.6 and therefore requires `compute=True`.

- [ ] **Step 5: Implement the lossless result component**

The component calls `TesseractNativeResultView.build(result)` and returns this exact order:

```python
return (
    view.request,
    view.native_result,
    view.raw_program,
    view.message,
    list(view.trajectory_points),
    list(view.joint_names),
    list_to_tree([list(row) for row in view.positions]),
    None if view.velocities is None else list_to_tree([list(row) for row in view.velocities]),
    None if view.accelerations is None else list_to_tree([list(row) for row in view.accelerations]),
    None if view.times is None else list(view.times),
)
```

Metadata output names are exactly `request`, `native_result`, `raw_program`, `message`, `trajectory_points`, `joint_names`, `positions`, `velocities`, `accelerations`, `times`. State explicitly that absent dynamics produce `None`, not zero trees, and no `JointTrajectory` is created.

- [ ] **Step 6: Add source/metadata/icon contracts and render icons**

Require exact I/O names, nanobind directives, `plan_native`, `TesseractPlanningRequest.build`, `native_program_digest`, `TesseractNativeResultView.build`, `list_to_tree`, no conventional planner calls, no `JointTrajectory`, no generic catch, and exact 24x24 icons. Use a ladder/play glyph for Native Plan and an inspected trajectory-grid glyph for Native Result.

- [ ] **Step 7: Verify GREEN**

Run: `pixi run pytest tests/backends/tesseract/test_native_plan_component.py tests/backends/tesseract/test_grasshopper_components.py -n auto -q`

Expected: cache behavior, exact planning path, result projection, metadata, and icons pass.

- [ ] **Step 8: Commit**

```bash
git add src/compas_fab/ghpython/components_cpython/Cf_TesseractNativePlan src/compas_fab/ghpython/components_cpython/Cf_TesseractNativeResult tests/backends/tesseract/test_native_plan_component.py tests/backends/tesseract/test_grasshopper_components.py
git commit -m "feat: add native planning components"
```

### Task 9: Windows Compilation and Twelve-Artifact Gates

**Files:**
- Modify: `.github/workflows/build.yml`
- Modify: `.github/workflows/publish_yak.yml`
- Modify: `.github/workflows/release.yml`
- Modify: `docs/developer/grasshopper.md`
- Modify: `tests/backends/tesseract/test_grasshopper_components.py`

**Interfaces:**
- Consumes: all eight component directories from Tasks 6-8 plus the existing four Tesseract components.
- Produces: Windows/Rhino 8 Python 3.9 compilation and publication gates for exactly twelve `.ghuser` files.

- [ ] **Step 1: Expand the failing workflow contract**

```python
TESSERACT_USER_OBJECTS = (
    "Cf_TesseractRobotArtifact.ghuser",
    "Cf_TesseractPlanner.ghuser",
    "Cf_TesseractRapidProfile.ghuser",
    "Cf_TesseractRapid.ghuser",
    "Cf_TesseractPose.ghuser",
    "Cf_TesseractCartesianTarget.ghuser",
    "Cf_TesseractJointTarget.ghuser",
    "Cf_TesseractStateTarget.ghuser",
    "Cf_TesseractMotionProgram.ghuser",
    "Cf_TesseractDescartesProfile.ghuser",
    "Cf_TesseractNativePlan.ghuser",
    "Cf_TesseractNativeResult.ghuser",
)


def test_windows_ci_requires_all_tesseract_user_objects_before_upload():
    for workflow_name in ("build.yml", "publish_yak.yml", "release.yml"):
        workflow = WORKFLOWS.joinpath(workflow_name).read_text(encoding="utf-8")
        for user_object in TESSERACT_USER_OBJECTS:
            assert user_object in workflow
        assert "Test-Path" in workflow
```

Update the directive/icon loop to cover all twelve source directories.

- [ ] **Step 2: Verify RED**

Run: `pixi run pytest tests/backends/tesseract/test_grasshopper_components.py -n auto -q`

Expected: workflow assertions fail for the eight new `.ghuser` names.

- [ ] **Step 3: Extend all PowerShell gates atomically**

Use the exact twelve-name tuple above in build, Yak, and release workflows. Keep `prefix-dev/setup-pixi@v0.9.3`, `environments: rhino39`, `locked: true`, and `pixi run -e rhino39 build-gh-components`. Do not introduce pip or a second compilation path.

- [ ] **Step 4: Update Grasshopper developer documentation**

Replace the four-object statement with the exact twelve names grouped as runtime/artifact, RAPID, authoring, profile/planning, and result inspection. State that componentizer runs on Windows CI under locked Rhino Python 3.9 and the CI artifact is the installable source for the compiled user objects.

- [ ] **Step 5: Verify GREEN and workflow syntax**

Run: `pixi run pytest tests/backends/tesseract/test_grasshopper_components.py -n auto -q`

Run: `pixi run python -c "import yaml; [yaml.safe_load(open(path, encoding='utf-8')) for path in ('.github/workflows/build.yml', '.github/workflows/publish_yak.yml', '.github/workflows/release.yml')]"`

Expected: artifact gates and YAML parsing pass.

- [ ] **Step 6: Commit**

```bash
git add .github/workflows/build.yml .github/workflows/publish_yak.yml .github/workflows/release.yml docs/developer/grasshopper.md tests/backends/tesseract/test_grasshopper_components.py
git commit -m "ci: require native planning components"
```

### Task 10: ABB Redundancy Workflow and User Documentation

**Files:**
- Create: `docs/backends/tesseract/files/04_native_component_workflow.py`
- Modify: `docs/backends/tesseract.md`
- Modify: `tests/backends/tesseract/test_documented_examples.py`

**Interfaces:**
- Consumes: all backend factories, exact `Robot.from_tesseract_support("abb_irb2400")`, `TaskComposer`, and the native Descartes pipeline.
- Produces: one executable factory-to-planning-to-result example matching the Grasshopper graph.

- [ ] **Step 1: Write the failing documented-example contract**

```python
COMPONENT_WORKFLOW_EXAMPLE = EXAMPLES / "04_native_component_workflow.py"


def test_component_workflow_uses_every_native_factory_and_axis_redundancy():
    source = COMPONENT_WORKFLOW_EXAMPLE.read_text(encoding="utf-8")
    for symbol in ("pose_from_user_frame", "build_cartesian_target", "build_motion_program", "build_descartes_profiles", "TesseractPlanningRequest.build", "TesseractNativeResultView.build"):
        assert symbol in source
    assert 'Robot.from_tesseract_support("abb_irb2400")' in source
    assert 'pipeline="DescartesFPipeline"' in source
    assert "TOOL_Z_AXIS = (0.0, 0.0, 1.0)" in source
    assert "TOOL_AXIS_SAMPLE_STEP = Radians(radians(1.0))" in source
    assert "TOOL_AXIS_SAMPLE_MIN = Radians(-pi)" in source
    assert "TOOL_AXIS_SAMPLE_MAX = Radians(pi)" in source
    assert "use_redundant_joint_solutions=True" in source
    assert "JointTrajectory" not in source
```

- [ ] **Step 2: Verify RED**

Run: `pixi run pytest tests/backends/tesseract/test_documented_examples.py -n auto -q`

Expected: the fourth example is absent.

- [ ] **Step 3: Build the executable ABB workflow**

Derive reachable start/goal poses from ABB FK, convert their COMPAS frames through `pose_from_user_frame(..., 1.0)`, create exact Cartesian targets, and build with the real robot/group/TCP. Plan with the explicit local-tool-Z settings below; inspect through `TesseractNativeResultView`.

```python
Radians = NewType("Radians", float)
TOOL_Z_AXIS = (0.0, 0.0, 1.0)
TOOL_AXIS_SAMPLE_STEP = Radians(radians(1.0))
TOOL_AXIS_SAMPLE_MIN = Radians(-pi)
TOOL_AXIS_SAMPLE_MAX = Radians(pi)

profiles = build_descartes_profiles(
    ["DEFAULT"],
    True,
    False,
    None,
    TOOL_Z_AXIS,
    TOOL_AXIS_SAMPLE_STEP,
    TOOL_AXIS_SAMPLE_MIN,
    TOOL_AXIS_SAMPLE_MAX,
    None,
    True,
    None,
)
request = TesseractPlanningRequest.build(
    program=built.composite_instruction,
    pipeline="DescartesFPipeline",
    profiles=profiles,
    auto_seed=True,
)
result = TesseractPlanningResult.build(request, composer.plan(robot, request.program, pipeline=request.pipeline, profiles=request.profiles, auto_seed=request.auto_seed))
view = TesseractNativeResultView.build(result)
```

Convert each exact FK `Pose` through `robot_frame_from_isometry(fk_pose).value`; `Pose` is an `Isometry3d` subclass in 0.35.0.6. Pass that exact COMPAS frame to `pose_from_user_frame(frame, 1.0)` so the example genuinely exercises the component boundary without synthesizing or approximating target poses. Print only native message, point count, joint order, and optional-dynamics availability.

- [ ] **Step 4: Document the twelve-node Grasshopper graph**

Update `docs/backends/tesseract.md` with the exact graph:

```text
Plane/Frame -> Tesseract Pose -> native Target -> Tesseract Motion Program
                                                  |             |
                                                  v             +-> Tesseract RAPID
Tesseract Descartes Profile -> Tesseract Native Plan -> Tesseract Native Result
```

Describe every new node's exact native type, explicit unit boundary, move-type preservation, Descartes `None` semantics, compute/cache behavior, absent-dynamics behavior, and the opaque same-object `ProfileDictionary` mutation limitation. Replace “four components” and phase-boundary text with the twelve-component baseline and embed `04_native_component_workflow.py`.

- [ ] **Step 5: Run the real example and docs tests**

Run: `pixi run python docs/backends/tesseract/files/04_native_component_workflow.py`

Expected: native Descartes planning succeeds and prints a nonzero trajectory point count.

Run: `pixi run pytest tests/backends/tesseract/test_documented_examples.py -n auto -q`

Expected: all four documented workflows satisfy source contracts.

- [ ] **Step 6: Commit**

```bash
git add docs/backends/tesseract/files/04_native_component_workflow.py docs/backends/tesseract.md tests/backends/tesseract/test_documented_examples.py
git commit -m "docs: add native component workflow"
```

### Task 11: Cross-Platform Verification Ratchet

**Files:**
- Modify only files implicated by failures; never weaken reference tests.

**Interfaces:**
- Consumes: completed Tasks 1-10.
- Produces: verified Python 3.12/macOS backend and Python 3.9/Rhino-compatible source set.

- [ ] **Step 1: Format and lint changed Python**

Run: `pixi run ruff format src/compas_fab/backends/tesseract src/compas_fab/ghpython/components_cpython tests/backends/tesseract docs/backends/tesseract/files`

Run: `pixi run ruff check src/compas_fab/backends/tesseract src/compas_fab/ghpython/components_cpython tests/backends/tesseract docs/backends/tesseract/files`

Expected: Ruff makes no further changes and reports no errors.

- [ ] **Step 2: Run strict backend typing**

Run: `pixi run mypy --strict src/compas_fab/backends/tesseract`

Expected: no type errors.

- [ ] **Step 3: Run affected tests through testmon**

Run: `pixi run pytest --testmon -n auto -q`

Expected: every affected test passes.

- [ ] **Step 4: Run the complete Tesseract suite on Python 3.12**

Run: `pixi run pytest tests/backends/tesseract -n auto -q`

Expected: all Tesseract backend, native runtime, component, workflow, and RAPID tests pass.

- [ ] **Step 5: Run the complete Tesseract suite on Rhino Python 3.9**

Run: `pixi run -e rhino39 pytest tests/backends/tesseract -n auto -q`

Expected: the same Tesseract suite passes against the released CPython 3.9 wheel.

- [ ] **Step 6: Run the full repository suite**

Run: `pixi run pytest -n auto -q`

Expected: the full suite passes; only existing repository opt-in skips remain.

- [ ] **Step 7: Build MkDocs and validate the diff**

Run: `pixi run invoke docs --check-links`

Run: `git diff --check`

Expected: documentation builds, links resolve, and the diff has no whitespace errors.

- [ ] **Step 8: Inspect branch commits and status**

Run: `git log --oneline --decorate -15`

Run: `git status --short --branch`

Expected: implementation is split across the small commits above, the worktree is clean, and no push has occurred.
