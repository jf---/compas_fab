# Tesseract Native Planning Components Design

## Objective

Make the native Tesseract planning workflow directly usable in Grasshopper without reducing it to COMPAS FAB's conventional planning types. The tranche adds eight composable components covering pose conversion, every native `MotionProgram` target type, exact program construction, Descartes profiles, native planning, and lossless result inspection.

Mobile/VKC remains the last roadmap item. This tranche does not add mobile planning, scene commands, process planning, conventional Cartesian lowering, or planner fallbacks.

## Native-first boundary

The graph's planning currency is the released `tesseract-robotics-nanobind` 0.35.0.6 API:

- `Pose`;
- `CartesianTarget`, `JointTarget`, and `StateTarget`;
- `MotionProgram` and `CompositeInstruction`;
- `ProfileDictionary`;
- `TesseractPlanningRequest` and `TesseractPlanningResult` retaining the native `PlanningResult`.

COMPAS FAB validates inputs and improves Grasshopper ergonomics but never replaces these values with a smaller action, target, trajectory, or profile model. Every native output remains connectable to Python components and nanobind calls.

## Python support boundaries

Grasshopper components delegate to focused, testable backend factories:

- `native_pose.py`: convert a validated COMPAS frame in explicit user units to native metre `Pose`.
- `native_targets.py`: construct all three exact native target types and validate move/profile/joint-state fields.
- `native_program_builder.py`: validate one robot/group/frame/target sequence and build both the native `MotionProgram` and exact `CompositeInstruction`.
- `descartes_profiles.py`: validate optional arguments and call native `create_descartes_pipeline_profiles(...)` without replacing omitted values.
- `native_result_view.py`: validate and expose native trajectory arrays without filling absent dynamics.

Factories own named error models and raw constructors cannot bypass invariants. Proposed named failures are `InvalidTesseractPoseError`, `InvalidTesseractTargetError`, `InvalidTesseractMotionProgramError`, `InvalidTesseractDescartesProfileError`, and `MalformedTesseractNativeResultError` under `TesseractBackendError`.

`MetersPerUserUnit` from the existing frame boundary remains the unit type. `Tesseract Pose` is the only node that converts geometry or length units. Native `Pose` coordinates are metres and are interpreted relative to the consuming program's explicit working-frame name; no target or planner node performs another transform or scale.

The Grasshopper boundary constructs `WorkingFrameUserUnits` before pose conversion. Joint and state nodes similarly construct non-interchangeable native position, velocity, acceleration, name, and time value types before exact target creation. Convenience Python factories remain available, but the component path is statically explicit about frame and quantity semantics.

## Components

### Tesseract Pose

Inputs:

- `frame`: Rhino Plane or COMPAS `Frame`;
- `metres_per_user_unit`: explicit finite positive scale.

Output:

- `pose`: exact native `Pose` in metres.

The Grasshopper source performs only Rhino Plane to COMPAS Frame conversion. The backend factory performs the typed scale and native pose conversion. No document-unit inference or default scale is allowed because that would hide unit semantics.

### Tesseract Cartesian Target

Inputs:

- exact native `Pose`;
- native move type: `FREESPACE`, `LINEAR`, or `CIRCULAR`;
- exact non-empty profile name.

Output: exact native `CartesianTarget`.

All native move types remain constructible. A later planner or emitter may reject a specific move type with its own native error; this component does not pre-emptively remove it.

### Tesseract Joint Target

Inputs:

- positions in native joint units;
- optional exact joint-name list;
- native move type;
- exact profile name.

Output: exact native `JointTarget` retaining the supplied order. When names are omitted, `Tesseract Motion Program` supplies the selected native group's joint order. Duplicate, empty, or shape-inconsistent explicit names fail loudly.

### Tesseract State Target

Inputs:

- positions in native joint units;
- optional exact joint-name list;
- optional velocities;
- optional accelerations;
- optional time in seconds;
- native move type;
- exact profile name.

Output: exact native `StateTarget`. Optional dynamics remain absent when unconnected; zeros are never manufactured. Every connected array must match the position shape, time must be finite and non-negative, and explicit names obey the Joint Target contract.

### Tesseract Motion Program

Inputs:

- exact native `Robot` clone;
- ordered list of exact native targets;
- exact group name;
- optional TCP frame name;
- working-frame name, defaulting visibly to native `base_link`;
- program profile name, defaulting visibly to native `DEFAULT`.

Outputs:

- exact native `MotionProgram`;
- exact native `CompositeInstruction`.

The factory reads native group joint order and resolves an omitted TCP through the native robot. It validates a non-empty target sequence, exact target types, unique group joints, frame/profile names, and explicit target joint names against the selected group. It does not mutate the robot or targets. Target order and move types are preserved exactly.

Construction uses native `MotionProgram.add_target(...)`; it does not route through `move_to(...)`, `linear_to(...)`, or `circular_to(...)`, because those convenience methods overwrite the target's already-authored move type.

### Tesseract Descartes Profile

Inputs mirror every argument accepted by native `create_descartes_pipeline_profiles(...)`:

- profile names;
- vertex and edge collision flags;
- solver thread count;
- sample axis, resolution, minimum, and maximum;
- IK solver name;
- redundant-joint-solution flag;
- optional exact native custom move profile.

Output: exact native `ProfileDictionary`.

Unconnected optional values remain `None`, preserving the released native defaults. The component never silently supplies the example's 1-degree/full-range settings. A connected sample resolution must be finite and positive; sample bounds must be finite and ordered; the axis must contain three finite values and be non-degenerate. A custom move profile cannot be combined with native convenience overrides, matching the native factory contract.

### Tesseract Native Plan

Inputs:

- exact `TesseractPlanner`;
- exact native `CompositeInstruction`;
- exact pipeline name;
- exact native `ProfileDictionary`;
- `auto_seed` boolean;
- `compute` boolean.

Output: complete `TesseractPlanningResult`.

The component builds `TesseractPlanningRequest` and calls `planner.plan_native(...)`; it never calls a conventional planning method. Profiles are required and are never inferred from the pipeline string. The component caches only a successful result paired with `(planner object identity, serialized program digest, pipeline, profile object identity, auto_seed)`. Native binary serialization makes in-place `CompositeInstruction` edits invalidate the cache. The released opaque `ProfileDictionary` API exposes neither enumeration nor serialization, so in-place mutation of the same dictionary cannot be observed; a caller using that advanced native path must set `compute=True`. The Descartes Profile component itself emits a new dictionary whenever any profile input changes.

With `compute=False`, the component returns a result only when the complete observable signature still matches; changed inputs clear stale output. Any failed recomputation clears the previous result before reporting the named error.

### Tesseract Native Result

Input: exact `TesseractPlanningResult`.

Outputs:

- exact request;
- exact native `PlanningResult`;
- exact raw `CompositeInstruction`;
- native message;
- exact native trajectory-point list;
- consistent joint-name order;
- position, velocity, acceleration, and time Grasshopper trees/lists.

The first five outputs preserve complete native access. Convenience arrays are inspection views only. Missing velocity, acceleration, or time remains `None`; the component does not create zeros or project a COMPAS `JointTrajectory`. Inconsistent joint order, non-finite values, or mismatched shapes raise a named malformed-result error.

## Data flow

```text
Rhino Plane / COMPAS Frame
        |
        v
Tesseract Pose -> native Target -> Tesseract Motion Program
                                      |              |
                                      |              +-> Tesseract RAPID
                                      v
Tesseract Descartes Profile -> Tesseract Native Plan -> Tesseract Native Result
```

The authored `CompositeInstruction` connects directly to RAPID emission. A dense planning result is inspected through `Tesseract Native Result`; it is not automatically reinterpreted as authored controller code.

## Grasshopper behavior

All eight components use the released `tesseract-robotics-nanobind>=0.35.0.6,<0.36` CPython directive and compile through the locked Windows/Rhino 8 Python 3.9 job. Icons extend the existing Tesseract grey/teal system.

Components catch adapter-owned `TesseractBackendError` failures only to report them on the Grasshopper component. No component catches generic exceptions, retries, substitutes values, writes files, or hides a native planning failure. Value-list controls expose exact native move types where appropriate.

Build, Yak, and release workflows require all twelve Tesseract `.ghuser` artifacts before upload or publication: the existing artifact, planner, RAPID profile, and RAPID nodes plus these eight components.

## Verification

- TDD covers every factory before its component source is added.
- Factory outputs are asserted to be the exact released native types and retain supplied native object/order identity where applicable.
- Pose conversion contract tests cover explicit metre and millimetre scales, invalid scales, quaternion orientation, and no hidden document-unit lookup.
- Target tests cover all three native target types, all native move types, missing optional dynamics, malformed shapes, and exact profile names.
- Program tests cover group-derived joint order/TCP, target ordering, empty programs, wrong native targets, and robot non-mutation.
- Descartes tests lock preservation of native defaults, explicit full-range 1-degree sampling, custom-profile conflicts, collision flags, and redundant-solution propagation.
- Plan cache tests prove exact signature reuse, invalidation on every observable input and in-place program mutation, no stale result after failure, and no conventional planner call.
- Result-view tests cover exact native-object retention, absent dynamics, shape/order validation, and Grasshopper tree contracts.
- A real ABB IRB 2400 axis-symmetric program flows through the same factories and native Descartes pipeline on macOS ARM64.
- Source/metadata/icon tests cover all inputs, outputs, dependency directives, error boundaries, and 24x24 icons.
- Windows workflow tests require all twelve compiled `.ghuser` artifacts.
- Ruff, strict mypy, affected testmon, Python 3.12, Rhino Python 3.9, the full suite, MkDocs, and `git diff --check` remain gates.

## Deferred scope

- FK, IK, and collision Grasshopper nodes are the next tranche after this one.
- Scene commands for tools, workpieces, and attachments follow kinematics/collision inspection.
- Process planning and `tesseract_3s_slicer` follow scene commands.
- Mobile/VKC is the final roadmap item.
- `tesseract_concurrent_trajopt` remains deferred until the baseline and earlier advanced workflows are established.
