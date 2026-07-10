# Tesseract

[Tesseract Robotics nanobind](https://github.com/tesseract-robotics/tesseract_nanobind) is the
preferred in-process backend when motion-planning capabilities overlap. COMPAS
FAB supplies ergonomic robot-cell and trajectory projections; Tesseract remains
the kinematics, collision, and motion-planning implementation.

## Capability contract

The backend has two layers that share one native runtime:

- Conventional methods such as `plan_motion`, `forward_kinematics`,
  `inverse_kinematics`, and `check_collision` accept COMPAS FAB values.
- `plan_native` accepts an exact Tesseract `CompositeInstruction`, pipeline
  name, `ProfileDictionary`, and seeding policy.

The native layer is not reduced to the conventional interface. A successful
native call retains the complete Tesseract planning result and raw output
program. A conventional `JointTrajectory` is a checked, point-for-point
projection of that retained result; it is never resampled or substituted.

!!! warning "No fallback planners"
    This backend never falls back to PyBullet, analytical kinematics, MoveIt,
    or a COMPAS interpolation. Unsupported conventional inputs fail loudly.
    Use `plan_native` whenever a Tesseract feature has no COMPAS equivalent.

## Install

The required distribution is `tesseract-robotics-nanobind`, which imports as
`tesseract_robotics`. The older `tesseract-python` package is not used.

```bash
pixi add --pypi compas_fab "tesseract-robotics-nanobind>=0.35.0.6,<0.36"
```

For this repository, `pixi install` resolves the released 0.35.0.6 macOS ARM64
wheel. The install contract checks the released version and verifies that the
inferior `tesseract-robotics` distribution is absent.

The macOS wheel bundles the OpenMP runtime used by Descartes. This workspace
pins conda-forge's pthreads OpenBLAS build so NumPy does not initialize a second
`libomp.dylib`; setting `KMP_DUPLICATE_LIB_OK` is not supported. The lockfile
enforces the pthreads build on macOS ARM64 and Windows for both Pixi environments.

The current repository runtime baseline is macOS 14+ ARM64 with Python 3.12. Rhino
8 component compilation is pinned to CPython 3.9, matching Rhino's embedded
interpreter. The Grasshopper source components include an explicit
`# r: tesseract-robotics-nanobind>=0.35.0.6,<0.36` directive. Windows CI
compiles the `.ghuser` objects and fails unless all twelve Tesseract user objects
are present before artifact upload or publication. The generic Linux Python 3.9
CI cell is excluded because released Linux nanobind wheels start at Python 3.10;
Rhino/Windows remains pinned to Python 3.9.

## Build the exact robot artifact

Tesseract consumes exact URDF, SRDF, meshes, plugin files, and mesh sidecars.
`RobotArtifactLoader` captures source text and resolves each
`package://<name>/...` URL beneath explicit resource roots. Every file in a
referenced package becomes part of the content-addressed artifact; unrelated
packages do not.

Standard COMPAS URDF/SRDF files do not select all Tesseract plugins.
`CompasRobotArtifactCompiler` adds explicit choices without overwriting an
existing source plugin configuration:

```python
from pathlib import Path

from compas_fab.backends.tesseract.artifact import CollisionMeshPolicy
from compas_fab.backends.tesseract.artifact import ContinuousContactManager
from compas_fab.backends.tesseract.artifact import DiscreteContactManager
from compas_fab.backends.tesseract.artifact import KdlInverseKinematics
from compas_fab.backends.tesseract.artifact_loader import ResourceRoot
from compas_fab.backends.tesseract.artifact_loader import RobotArtifactLoader
from compas_fab.backends.tesseract.compas_artifact import CompasRobotArtifactCompiler

loader = RobotArtifactLoader.build(
    Path("robot.urdf"),
    Path("robot.srdf"),
    [ResourceRoot.build(Path("robot_packages"))],
)
compiler = CompasRobotArtifactCompiler.build(
    loader=loader,
    collision_mesh_policy=CollisionMeshPolicy.PRESERVE,
    groups=["manipulator"],
    inverse_kinematics=KdlInverseKinematics.LMA,
    discrete_contact_manager=DiscreteContactManager.BULLET_BVH,
    continuous_contact_manager=ContinuousContactManager.BULLET_CAST_BVH,
)
artifact = compiler.compile(robot_cell)
print(artifact.identity.digest)
```

`RobotArtifactLoader.load()` and `CompasRobotArtifactCompiler.build()` default
to `CollisionMeshPolicy.CONVEX_HULL`. The Grasshopper input exposes the same
default visibly. Pass `CollisionMeshPolicy.PRESERVE` at either loading boundary
when exact triangle collision meshes are required. Both values remain distinct,
content-addressed artifact inputs; an explicit selection is never replaced.

On the bundled UR5 meshes, raw triangle collision processing can take more than
a minute, while the convex-hull artifact initializes locally in about twelve
seconds. The planning examples exercise the loading default directly.

## Conventional motion planning

The Phase 1 conventional motion path accepts `ConfigurationTarget` and executes
Tesseract's `FreespacePipeline`. Native pipeline controls remain available
through the `options` dictionary: `pipeline`, `profiles`, and `auto_seed`.
A custom pipeline requires its exact native `ProfileDictionary`.

```python
--8<-- "docs/backends/tesseract/files/01_compas_plan_motion.py"
```

The returned trajectory retains the exact native result that produced it. This
reference is intentionally excluded from portable COMPAS serialization:

```python
from compas_fab.backends.tesseract import native_result_from_trajectory

native_result = native_result_from_trajectory(trajectory)
```

## Native program and result

Use this path for arbitrary Tesseract instruction programs, profile sets, and
Task Composer pipelines. The request retains the exact program and profile
objects. With `auto_seed=False`, the exact caller program reaches Task Composer.
With `auto_seed=True`, the backend uses Tesseract's polymorphic instruction copy
before native seeding; serialization registration is never an execution
prerequisite. The result retains both the exact native planning result and its
raw output `CompositeInstruction`.

The example derives two reachable UR5 Cartesian poses, then lets
`DescartesFPipeline` sample rotations about the local TCP Z axis in 1-degree
increments. `use_redundant_joint_solutions=True` also retains Tesseract's native
joint-redundancy search. COMPAS FAB does not enumerate orientations or choose an
inverse-kinematics branch.

Released nanobind 0.35.0.6 keeps target poses fixed by default. Its dormant
sampling values are local Z, a 90-degree step, and a -180-to-90-degree range;
redundant joint solutions default off. The examples therefore override the
axis, 1-degree step, full -180-to-180-degree range, and redundancy flag
explicitly instead of inheriting any of those values.

```python
--8<-- "docs/backends/tesseract/files/02_native_program.py"
```

### Native component workflow

The same boundaries used by the Grasshopper nodes are ordinary Python
factories. This ABB IRB 2400 example derives reachable Cartesian poses through
native FK, crosses the explicit COMPAS-frame/metre boundary, and retains exact
native targets, program, profiles, request, result, and trajectory points.
`WorkingFrameUserUnits` carries coordinate-frame and unit semantics into that
boundary; joint/state component inputs likewise become distinct validated
position, velocity, acceleration, name, and time types before target creation.
Descartes owns the full local-tool-Z symmetry search at 1-degree resolution over
the explicit -180-to-180-degree range and includes redundant joint solutions.

```python
--8<-- "docs/backends/tesseract/files/04_native_component_workflow.py"
```

## RAPID emission

`TesseractRapidEmitter` delegates one exact native `CompositeInstruction` to
the RAPID emitter shipped by `tesseract-robotics-nanobind`. It does not accept
planning requests, planning results, or COMPAS trajectories, and it does not
walk instructions, convert units, or format RAPID itself. Native
`RapidEmitterError` subclasses therefore propagate unchanged.

The profile map binds exact Tesseract motion-profile names to native ABB
`RapidProfile` values. Emission returns an immutable `RapidProgram` containing
the exact input program reference, byte-identical native source, and a SHA-256
identity covering source, names, schema, COMPAS FAB version, and nanobind
version. `RapidProgram.write(Path)` is the only file-writing operation; neither
emission nor Grasshopper recomputation saves automatically.

Planning and code emission remain independent. This ABB IRB 2400 example lets
native Descartes sample the full -180-to-180-degree local-TCP-Z rotation range
in 1-degree steps and enumerate
redundant joint solutions, then emits the authored Cartesian
`CompositeInstruction` as `MoveL` statements. It does not reinterpret the
dense `StateWaypoint` planning result as controller code.

```python
--8<-- "docs/backends/tesseract/files/03_rapid_emitter.py"
```

## Kinematics and collision

- `forward_kinematics_native` returns the exact robot-relative native `Pose`.
- `inverse_kinematics_native` retains the native IK input and every native
  solution array in solver order.
- `check_collision_native` retains the exact `ContactRequest`, grouped
  `ContactResultMap`, and a non-destructive flattened result view.
- Their conventional counterparts project frames/configurations or raise named
  errors carrying the native result.

Every environment clone receives the complete active joint state before a
group-local solver or program runs. Omitted positioner or auxiliary joints are
errors, never assumed zero.

Phase 1 collision checking intentionally rejects tools and rigid bodies until
their exact native scene commands are implemented. It does not report a
robot-only query as a complete cell check.

## Grasshopper

Twelve components separate immutable inputs, exact native authoring/planning,
inspection, and pure code emission:

1. **Tesseract Robot Artifact** exposes resource roots, planning groups, mesh
   policy, KDL solver, and contact-manager selections. It outputs the full
   artifact and its SHA-256 identity.
2. **Tesseract Planner** caches a client by artifact identity. It outputs the
   conventional planner and an isolated native `Robot` clone.
3. **Tesseract Pose** is the only geometry/unit boundary. It requires explicit
   metres per Rhino/COMPAS user unit, validates a typed working-frame value,
   and outputs an exact native `Pose`.
4. **Tesseract Cartesian Target**, **Tesseract Joint Target**, and **Tesseract
   State Target** output exact native target types. All expose native
   `FREESPACE`, `LINEAR`, and `CIRCULAR` move types; optional state dynamics stay
   absent when unconnected. Connected values cross distinct typed native
   position, velocity, acceleration, name, and time boundaries.
5. **Tesseract Motion Program** uses the native robot to resolve group joint
   order and TCP, then outputs both exact `MotionProgram` and
   `CompositeInstruction` values without overwriting authored move types.
6. **Tesseract Descartes Profile** exposes every released native profile-factory
   argument. Unconnected options stay `None`, preserving Tesseract defaults.
7. **Tesseract Native Plan** requires exact pipeline and `ProfileDictionary`
   inputs and calls only `plan_native`.
8. **Tesseract Native Result** retains exact request, `PlanningResult`, raw
   program, and native points. Missing dynamics output `None`, never zeros.
9. **Tesseract RAPID Profile** and **Tesseract RAPID** bind native ABB variables
   and emit controller source from an authored exact `CompositeInstruction`.

```text
Plane/Frame -> Tesseract Pose -> native Target -> Tesseract Motion Program
                                                  |             |
                                                  v             +-> Tesseract RAPID
Tesseract Descartes Profile -> Tesseract Native Plan -> Tesseract Native Result
```

Changing any source resource or plugin selection changes the identity and
rebuilds the cached runtime. Direct native experiments receive a clone and
cannot mutate the planner's master environment. The clone receives the complete
stored robot state before it is returned. Warmup can target an exact pipeline
list or every pipeline in the retained Task Composer configuration; every
requested warmup failure raises a named error. Python users can also supply an
exact preconfigured native `TaskComposer` to `TesseractClient`.

Native Plan caches a success only while planner identity, native scene revision,
serialized program digest, pipeline, profile-object identity, and `auto_seed`
all match. Changed robot state, scene, inputs, or failed recomputation clear
stale output. Nanobind 0.35.0.6 exposes no way to enumerate or serialize a
`ProfileDictionary`; callers mutating the same dictionary object directly must
trigger `compute=True`.

## Current Phase 1 boundary

Implemented now:

- exact content-addressed URDF/SRDF/package resources;
- explicit KDL and Bullet/FCL plugin configuration;
- native request/result planning and conventional free-space planning;
- native-first FK, IK, and discrete collision checking;
- native RAPID emission with content-addressed source artifacts;
- twelve Grasshopper Tesseract component sources, with Windows Python 3.9
  compilation and artifact-presence gates in CI.

Deferred without fallback: Cartesian motion lowering, tools and rigid bodies in
the native scene, `tesseract_3s_slicer` process planning, and remaining advanced
planners including `tesseract_concurrent_trajopt`. Mobile/VKC remains the final
roadmap item.
