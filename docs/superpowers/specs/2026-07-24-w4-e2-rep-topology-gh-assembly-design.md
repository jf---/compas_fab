# W4 E2 — REP topology + Grasshopper external-axis assembly, design

**Status:** amends `2026-07-24-w4-external-axes-design.md` (the E2 bullet under Phasing). **Date:** 2026-07-24

E2 is a **cell-loading + ergonomics** phase, not new planning or artifact code. The REP (robot-with-external-positioner) coordinated solve already works on the installed substrate the moment a REP cell is loaded; E1 proved the ROP twin. E2's real work is: get a full three-group REP cell *through* `set_robot_cell`, prove one coordinated REP toolpath, synthesize such cells from Grasshopper geometry, and make the external-axis slot mapping typed. Coordinated RAPID *semantics* (`\WObj:=`, `ActUnit`/`DeactUnit`, external-axis `speeddata`) stay in E3.

## Verified ground truth (why E2 is glue, not planning code)

- **The REP planning path is already topology-agnostic.** `plan_cartesian_motion.py` reads `ik_solver = client.artifact.default_inv_kin_solver(group)` from the loaded plugin-YAML `default` — ROP or REP, no branch. `DescartesFPipeline` enumerates whichever coupled solver the cell names.
- **The REP artifact emission is already generic.** `artifact.py::_coupled_plugin_yaml` emits `REPInvKinFactory` from `CoupledTopology.ROBOT_WITH_EXTERNAL_POSITIONER` with the identical machinery ROP uses; only the four link names, the topology, and the sample-resolution joints change.
- **The REP reference cell ships in `tesseract-robotics-nanobind==0.35.0.7`:** `abb_irb2400_external_positioner.{urdf,srdf}` under the tesseract support/urdf data dir. It is an **8-DOF, three-group** cell:
  - Robot base (`base_link`) is FIXED to `world`; a **sibling** positioner branch (`positioner_base_link` → `positioner_link_1` → `positioner_tool0`) carries the workpiece. The two branches meet only at `world`.
  - Positioner = a **2-DOF prismatic XY table**: `positioner_joint_1` (axis Y) and `positioner_joint_2` (axis X), each limit −1.0…1.0 m. Arm = `joint_1…joint_6` (revolute).
  - The **positioner joints are ordered first** in the coupled joint vector (the solver walks up the positioner branch to `world`, then down the robot branch to `tool0`).
  - Three SRDF groups: `manipulator` (`world`→`tool0`), `positioner` (`world`→`positioner_tool0`), `full_manipulator` (`positioner_tool0`→`tool0`, the coupled group). The shipped SRDF carries **no** `kinematics_plugin_config` (compas_fab injects it).
  - **Anomaly (do not key fixtures off it):** the REP SRDF root is mislabeled `name="abb_irb2400_on_positioner"` — identical to the ROP file. The cells are distinguished only by their chain base/tip links.

## The linchpin: `structural_validation` (#5) is now on the critical path

E1's ROP proof sidestepped `structural_validation` by stripping the cell to just `full_manipulator`, whose chain base (`positioner_base_link`) coincides with the native kinematic root. **REP breaks that coincidence structurally:** its `full_manipulator` chain is `positioner_tool0 → tool0`, whose base (`positioner_tool0`) is *not* the native root (`world` is — robot and positioner are siblings, not a serial chain). So loading any REP coupled group forces the reconciliation E1 deferred.

`structural_validation.py:70-73` compares `native_group.getBaseLinkName()` against `robot_cell.get_base_link_name(group)`; for a coupled or rigidly-mounted sub-group these differ (native kinematic root vs SRDF-chain base). The fix reconciles the two, **regression-guarding the ordinary single-group path** (a plain arm cell must still validate byte-for-byte as today). This unblocks the full three-group cell, which is the prerequisite for the REP proof — hence it lands first.

## Build order (dependency chain)

```
#5 structural_validation fix ──┐  linchpin: full 3-group cell loads through set_robot_cell
                                ▼
#1 REP coordination proof  ◄── full REP cell loads
   ├─ REPInvKin contract test — pin 8-DOF joint order/count (positioner joints first)
   └─ REP Cartesian proof — positioner reorients, TCP tracks the workpiece frame

#7 typed eax slot-map ─────────┐  independent, emitter-side
#3 eax inputs on targets ──────┤
cell_assembly.py synthesis ────┤  largest new surface
   ├─ #2 GH axis components + dedicated assembly component
   └─ #6 native_scene actuated joints + attached workpiece (own test, off proof path)
                                ▼
E2E round-trip contract: GH → synth cell → coordinated plan → RAPID eax re-parse
```

## Type surface (ETH baseline — frame + unit at the type level)

Two *distinct* external-axis types, separated by lifetime and consumer — not one half-populated god-type:

| Type | File | Lifetime / consumer |
|---|---|---|
| `ExternalAxis` *(exists)* | `external_axes.py` | **Derived from** a loaded cell — `role` / `unit` / `index`. Consumed by the emitter and the ROP/REP planner. |
| `ExternalAxisDescriptor` *(new)* | `cell_assembly.py` | **Authored** in Grasshopper to *synthesize* a cell — `name`, `role`, mount `Frame` (metres), unit `axis`, `lower`/`upper` limits, base + link meshes. Consumed by cell synthesis. |
| `ExternalAxisSlotMap` + `ControllerEaxSlot` *(new)* | `external_axes.py` | Typed axis-name → ABB `eax_a…eax_f` slot. `positional(...)` reproduces today's `external[0]→eax_a`; `build({name: slot})` maps a track to `eax_e`. Replaces the positional assumption in `rapid_emitter._emitter_external_axis_layout`. |

Rationale for the split: `ExternalAxis` has an `index` only after a cell is loaded and never carries geometry; `ExternalAxisDescriptor` carries mount frame + meshes and has no vector index until synthesis places it. One struct would be half-null in each direction — the god-object smell the project rules forbid. Two types, two `build()` validators, two consumers.

**Unit convention:** `ExternalAxisDescriptor.build(...)` accepts limits in the **role-natural author unit** (mm for `TRACK`, deg for `POSITIONER`), normalizes to SI (m / rad) for storage at the `build` boundary, and validates: unit-length `axis`, `lower ≤ upper`, all-finite, non-empty base and link meshes, non-empty name. The role→unit contract (`TRACK`→mm, `POSITIONER`→deg) remains the single source of truth already in `external_axes._ROLE_UNIT`.

## Cell synthesis (`cell_assembly.py`) — the largest new surface

`synthesize_coupled_cell(base_urdf, base_srdf, descriptors, topology) -> (urdf, srdf)`:

- **Input:** a plain N-axis robot description (a `manipulator` chain `base_link`→`tool0`) plus one or more `ExternalAxisDescriptor`s and a `CoupledTopology`.
- **URDF surgery:** for each descriptor, add a static base link and a moving link (each carrying its meshes as visuals + convex collisions), and a joint (PRISMATIC for `TRACK`, REVOLUTE for `POSITIONER`) carrying the descriptor's `axis`, SI limits, and an origin from the mount `Frame`.
- **Topology-specific wiring:**
  - `ROBOT_WITH_EXTERNAL_POSITIONER` (REP, from External *Rotational* Axis): add a `world` root, FIX the robot `base_link` to `world`, graft the positioner as a **sibling** branch to `positioner_tool0` (the workpiece mount).
  - `ROBOT_ON_POSITIONER` (ROP, from External *Linear* Axis): insert the positioner **serially below** `base_link` (`positioner_base_link`→`positioner_tool0`→[fixed]→`base_link`) so the track carries the robot.
- **SRDF rewrite:** emit the three groups (`manipulator`, `positioner`, `full_manipulator`) with the topology-correct base/tip links, leaving `kinematics_plugin_config` for `RobotArtifact.with_coupled_kinematics` to inject downstream.
- **Contract gate:** a synthesized abb_irb2400 + positioner cell must be **structurally equivalent** (link tree, joint types / axes / limits, the three groups) to the shipped `abb_irb2400_external_positioner` reference cell, and must load through `set_robot_cell`. Geometry is compared by structure, not mesh bytes (the reference ships real meshes; synthesis uses author meshes). This is the artifact-meets-reference gate.

## Error model (one named exception per failure mode, extend `errors.py`)

- `InvalidExternalAxisDescriptorError` — bad geometry, non-unit axis, `lower > upper`, non-finite limit, or empty mesh set.
- `CoupledCellAssemblyError` — synthesis failure: a synthesized link/joint name collides with the robot, the base description is malformed, or the topology wiring is unsatisfiable.
- `ExternalAxisSlotConflictError` — two axes map to one `eax` slot, or a slot is out of the `a…f` range.
- `UncoordinatedTargetError` — a coupled group's external DOF is present but a target omits its external-axis value.

`UnknownKinematicTopologyError` (exists) still covers unclassifiable coupled joints. The `structural_validation` fix reconciles the *known* native-root≠chain-base case; a residual, unexplained mismatch still raises the existing `RobotArtifactMismatchError`.

## Grasshopper surface (thin marshals + CI wiring)

- `Cf_TesseractExternalLinearAxis`, `Cf_TesseractExternalRotationalAxis` — each a one-call marshal (`plane`, `axis`/`direction`, `limits`, `meshes`, `name` → `ExternalAxisDescriptor.build(...)`).
- `Cf_TesseractRobotCell` (new dedicated assembly component) — robot description + `[descriptors]` + topology → `synthesize_coupled_cell` → `RobotArtifact.with_coupled_kinematics(...)`. Keeps `Cf_TesseractRobotArtifact` a pure loader.
- #3: eax inputs on the coordinated target component(s), carrying authored external values through to the emitter's `CoupledGroupLayout` path.
- Every new component ships four files — `code.py` (`# r: compas_fab>=2.0.1`, `# r: tesseract-robotics-nanobind==0.35.0.7`, class-based `GH_ScriptInstance`, `except TesseractBackendError`, `error(ghenv.Component, …)`), `metadata.json` (category `COMPAS FAB`, subcategory `Backends`, exposure `2`, input/output param lists matching the `RunScript` signature), a 24×24 `icon.png`, and an `icon.svg` source.
- Because the GH contract test hardcodes component names, each new component must additionally be wired into: `tests/backends/tesseract/test_grasshopper_components.py` (a new static assertion block), the module-level `TESSERACT_USER_OBJECTS` tuple, and the three CI workflows (`build.yml`, `publish_yak.yml`, `release.yml`) with `Test-Path` guards. GUIDs are auto-generated at build; none are authored.

## Testing & exit gates

- **REPInvKin contract test** (new, `test_coupled_kinematics.py` style) — `numJoints() == 8`, `getJointNames()` in coupled order (positioner joints first). No teardown scaffolding (`nb::keep_alive` shipped in 0.35.0.7; the ordered-`del` workaround was removed in `518bb16c`).
- **REP Cartesian proof** (new) — positioner XY-table travel exceeds a threshold well above the sample step (coordination happened) *and* `tool0` tracks the **moving** workpiece frame within tolerance. Because the target frame is workpiece-fixed (`full_manipulator` working frame is `positioner_tool0`), the expected `tool0` world position is computed by composing the positioner FK at each trajectory point, then compared with a raw numpy L2 norm against the same bound the ROP proof uses (`< 5e-3` m, position-only) — matching the existing proof discipline, not `compas.tolerance.TOL`.
- **cell_assembly contract test** — synthesized cell structurally equivalent to the shipped reference cell and loading through `set_robot_cell`.
- **structural_validation regression** — full three-group cell loads; the single-group path is un-regressed (guarded by an explicit before/after test).
- **GH static contracts** for the three new components + **E2E eax round-trip** extending `test_rapid_external_axes.py` (real `eax`, correct mm/deg units, byte-identical when `external_axes is None`).
- **Gates of record:** `pixi run mypy --strict src/compas_fab/backends/tesseract` clean · `pixi run ruff check` clean · `pixi run pytest tests/backends/tesseract -n auto` green (`-n auto` is the real keep_alive teardown stress test; compas_fab CI does not fire on the branch).
- **If a gate slips:** write `docs/superpowers/state/w4-e2-gate-<X>-analysis.md` (root cause + options) *before* patching.

## Deferred to E3 (the `\WObj` seam)

`MoveL … \WObj:=` as a positioner-driven coordinated mechanical unit · `ActUnit`/`DeactUnit` · external-axis `speeddata`. E2 stops at populated `eax` values and workpiece-relative target authoring; the emitted RAPID moves the positioner via its joint values, not yet via a coordinated work object.

## Decisions locked

1. **`\WObj` E2/E3 boundary → eax now, `\WObj` in E3.** E2 proves the kinematic coordination and populates `eax`; the coordinated-RAPID semantics stay E3, matching the W4 phasing and keeping E2 validated before RAPID polish layers on.
2. **GH assembly → dedicated component.** Thin axis components emit `ExternalAxisDescriptor`s; a dedicated `Cf_TesseractRobotCell` + `cell_assembly.py` performs synthesis. `Cf_TesseractRobotArtifact` stays a pure loader (one responsibility per component).
3. **REP proof decoupled from #6.** The proof uses coupled kinematics + workpiece-relative targets (no scene-attached rigid body); the `plan_cartesian_motion.py:113` tool/body gate stays shut on the proof path. Attached-workpiece collision handling (#6) lands with its own test, off the proof's critical path.
4. **Cell synthesis covers both topologies.** External Rotational Axis → REP (sibling positioner branch carrying the workpiece); External Linear Axis → ROP (serial track carrying the robot). Validates against E1's hand-authored ROP cell and the shipped REP reference cell.

## Out (unchanged)

Coordinated RAPID semantics (E3, above) · yak packaging · the live-Rhino kernel harness as a CI gate (`tests/rhino/` is local-only, license-gated; GH component *logic* is validated headlessly via the static contract test).
