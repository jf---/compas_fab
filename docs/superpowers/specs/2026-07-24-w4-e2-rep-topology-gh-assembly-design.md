# W4 E2 — REP topology + Grasshopper external-axis assembly, design

**Status:** amends `2026-07-24-w4-external-axes-design.md` (the E2 bullet under Phasing). Superseding detail on the REP coupled-group blocker + fix lives in `docs/superpowers/state/w4-e2-rep-coupled-group-analysis.md` (live-probe evidence). **Date:** 2026-07-24 (revised after empirical probing).

E2 is a **cell-loading + ergonomics** phase, not new planning or artifact code. The REP (robot-with-external-positioner) coordinated solve already works on the installed substrate the moment a REP cell is loaded; E1 proved the ROP twin. E2's real work is: get a full REP coupled cell *through* `set_robot_cell`, prove one coordinated REP toolpath, synthesize such cells from Grasshopper geometry, and carry authored external-axis values into the emitter. Coordinated RAPID *semantics* (`\WObj:=`, `ActUnit`/`DeactUnit`, external-axis `speeddata`) and the typed controller eax-slot map stay out of E2 (see Deferred).

## Verified ground truth (why E2 is glue, not planning code)

- **The REP planning path is already topology-agnostic.** `plan_cartesian_motion.py` reads `ik_solver = client.artifact.default_inv_kin_solver(group)` from the loaded plugin-YAML `default` — ROP or REP, no branch. `DescartesFPipeline` enumerates whichever coupled solver the cell names.
- **The REP artifact emission is already generic.** `artifact.py::_coupled_plugin_yaml` emits `REPInvKinFactory` from `CoupledTopology.ROBOT_WITH_EXTERNAL_POSITIONER` with the identical machinery ROP uses; only the four link names, the topology, and the sample-resolution joints change.
- **The REP reference cell ships in `tesseract-robotics-nanobind==0.35.0.7`:** `abb_irb2400_external_positioner.{urdf,srdf}`. It is an **8-DOF** cell — robot base (`base_link`) FIXED to `world`; a **sibling** positioner branch (`positioner_base_link → positioner_link_1 → positioner_tool0`, a 2-DOF prismatic XY table: `positioner_joint_1` axis Y, `positioner_joint_2` axis X, each limit −1.0…1.0 m) carries the workpiece. Three SRDF groups: `manipulator` (`world`→`tool0`), `positioner` (`world`→`positioner_tool0`), `full_manipulator` (the coupled group). The shipped SRDF carries **no** `kinematics_plugin_config` (compas_fab injects it). **Anomaly:** the REP SRDF root is mislabeled `name="abb_irb2400_on_positioner"` — do not key fixtures off it.

## The linchpin: the REP coupled group (#5) — reshaped after probing

E1's ROP proof sidestepped `structural_validation` by stripping the cell to just `full_manipulator`, a valid serial chain (`positioner_base_link→tool0`). **REP cannot be sidestepped that way**, and the blocker is deeper than the spec first assumed. Live probes established:

1. **COMPAS cannot even parse the shipped REP coupled group.** `full_manipulator` ships as `<chain base_link="positioner_tool0" tip_link="tool0">`, which crosses the `world` fork; `RobotSemantics.from_srdf_string` raises `Exception("No chain found…")` at `iter_link_chain` — *before* `structural_validation`. Redefining the group as an explicit `<joint>` list fixes parsing (and, as a bonus, canonicalizes the native coupled joint order to `[positioner_joint_1, positioner_joint_2, joint_1…joint_6]`).
2. **A joint-defined group is necessary but not sufficient.** Two residual mismatches survive that **no SRDF authoring can fix**: (a) `structural_validation:70-73` — native base `world` vs COMPAS joint-group base `positioner_base_link`; (b) `plan_cartesian_motion:133-134` — COMPAS reports tip `link_5`, but the coupled solve needs `tool0`, which COMPAS can *never* report for a joint group (a leaf is only ever a joint child). The `<link name="world"/>` patch is a dead-end (Tesseract rejects mixed-type groups). And `structural_validation:77`'s `iter_joint_chain` would also fail cross-fork.

**The fix (single sound approach):**

- **SRDF:** the coupled group is **joint-defined**. The reference-cell REP proof rewrites `full_manipulator` to a `<joint>` list on the COMPAS side (as E1's ROP proof already rewrites the SRDF); `cell_assembly` emits synthesized coupled groups as `<joint>` lists.
- **Backend sources coupled base/tip from the config, not from COMPAS.** A group is *coupled* iff `RobotArtifact.default_inv_kin_solver(group)` names a ROP/REP solver. For a coupled group:
  - **`structural_validation`** gains a coupled branch: validate joint **names + order + per-joint properties** by iterating the group's joints directly (native and COMPAS agree once joint-defined), confirm the native base is the kinematic root, but **do not** compare COMPAS's joint-group base and **do not** `iter_joint_chain` across the fork. Non-coupled groups keep the exact current path (regression-guarded).
  - **`plan_cartesian_motion`** sources the coupled group's frames from the config: `tcp_frame = manipulator_tip_link` (`tool0`), `working_frame = positioner_base_link` (a `world`-fixed link; the ROP precedent used the same link and produced TCP-accurate plans). Config links are read from the emitted kinematics-plugin YAML — the same authoritative bytes `default_inv_kin_solver` reads — so nothing drifts from the running solver. The REP proof is the oracle that pins the exact `working_frame` via TCP accuracy.

This is the prerequisite that unblocks the REP proof; it lands first.

## Build order (dependency chain)

```
#5 REP coupled-group fix ──┐  joint-defined SRDF + backend config-sourced base/tip
                            ▼
#1 REP coordination proof  ◄── full REP coupled cell loads through set_robot_cell
   ├─ REPInvKin contract test — pin 8-DOF joint order [pj1, pj2, j1..6]
   └─ REP Cartesian proof — positioner XY table coordinates; TCP tracks within tol

#3 eax inputs on targets ──────┐  independent; positional external[0]->eax_a (E1 path)
cell_assembly.py synthesis ────┤  largest new surface; emits joint-defined coupled groups
   ├─ #2 GH axis components + dedicated assembly component
   └─ #6 native_scene actuated joints + attached workpiece (own test, off proof path)
                                ▼
E2E round-trip contract: GH -> synth cell -> coordinated plan -> RAPID eax re-parse
```

## Type surface (ETH baseline — frame + unit at the type level)

Two *distinct* external-axis types, separated by lifetime and consumer — not one half-populated god-type:

| Type | File | Lifetime / consumer |
|---|---|---|
| `ExternalAxis` *(exists)* | `external_axes.py` | **Derived from** a loaded cell — `role` / `unit` / `index`. Consumed by the emitter and the ROP/REP planner. |
| `ExternalAxisDescriptor` *(new)* | `cell_assembly.py` | **Authored** in Grasshopper to *synthesize* a cell — `name`, `role`, mount `Frame` (metres), unit `axis`, `lower`/`upper` limits, base + link meshes. Consumed by cell synthesis. |

Rationale for the split: `ExternalAxis` has an `index` only after a cell is loaded and never carries geometry; `ExternalAxisDescriptor` carries mount frame + meshes and has no vector index until synthesis places it. One struct would be half-null in each direction — the god-object smell the project rules forbid.

**Unit convention:** `ExternalAxisDescriptor.build(...)` accepts limits in the **role-natural author unit** (mm for `TRACK`, deg for `POSITIONER`), normalizes to SI (m / rad) for storage at the `build` boundary, and validates: unit-length `axis`, `lower ≤ upper`, all-finite, non-empty base and link meshes, non-empty name. The role→unit contract already in `external_axes._ROLE_UNIT` stays the single source of truth.

## Cell synthesis (`cell_assembly.py`) — the largest new surface

`synthesize_coupled_cell(base_urdf, base_srdf, descriptors, topology) -> (urdf, srdf)`:

- **Input:** a plain N-axis robot description (a `manipulator` chain `base_link`→`tool0`) plus one or more `ExternalAxisDescriptor`s and a `CoupledTopology`.
- **URDF surgery:** for each descriptor, add a static base link and a moving link (each carrying its meshes as visuals + convex collisions), and a joint (PRISMATIC for `TRACK`, REVOLUTE for `POSITIONER`) carrying the descriptor's `axis`, SI limits, and an origin from the mount `Frame`.
- **Topology-specific wiring:**
  - `ROBOT_WITH_EXTERNAL_POSITIONER` (REP, from External *Rotational* Axis): add a `world` root, FIX the robot `base_link` to `world`, graft the positioner as a **sibling** branch to `positioner_tool0` (the workpiece mount).
  - `ROBOT_ON_POSITIONER` (ROP, from External *Linear* Axis): insert the positioner **serially below** `base_link` so the track carries the robot.
- **SRDF rewrite:** emit `manipulator`, `positioner`, and a **joint-defined** `full_manipulator` (never a cross-fork `<chain>`), leaving `kinematics_plugin_config` for `RobotArtifact.with_coupled_kinematics` to inject downstream.
- **Contract gate:** a synthesized abb_irb2400 + positioner cell must be **structurally equivalent** (link tree, joint types / axes / limits, the three groups — with `full_manipulator` joint-defined) to the shipped `abb_irb2400_external_positioner` reference cell (compared by structure, not mesh bytes), and must load through `set_robot_cell`. The artifact-meets-reference gate.

## Error model (one named exception per failure mode, extend `errors.py`)

- `InvalidExternalAxisDescriptorError` — bad geometry, non-unit axis, `lower > upper`, non-finite limit, or empty mesh set.
- `CoupledCellAssemblyError` — synthesis failure: a synthesized link/joint name collides with the robot, the base description is malformed, or the topology wiring is unsatisfiable.
- `UncoordinatedTargetError` — a coupled group's external DOF is present but a target omits its external-axis value.

`UnknownKinematicTopologyError` (exists) still covers unclassifiable coupled joints. The coupled-group `structural_validation` branch removes the base-link comparison for coupled groups; a residual, unexplained mismatch in a **non-coupled** group still raises the existing `RobotArtifactMismatchError`.

## Grasshopper surface (thin marshals + CI wiring)

- `Cf_TesseractExternalLinearAxis`, `Cf_TesseractExternalRotationalAxis` — each a one-call marshal (`plane`, `axis`/`direction`, `limits`, `meshes`, `name` → `ExternalAxisDescriptor.build(...)`).
- `Cf_TesseractRobotCell` (new dedicated assembly component) — robot description + `[descriptors]` + topology → `synthesize_coupled_cell` → `RobotArtifact.with_coupled_kinematics(...)`. Keeps `Cf_TesseractRobotArtifact` a pure loader.
- #3: eax inputs on the coordinated target authoring path — a coordinated `JointTarget`/`StateTarget` whose `positions`+`names` cover the full coupled group (6 arm + external) reaches the emitter, which splits arm vs. external **by name** (positional `external[0]→eax_a`, the E1 path; no slot map — that is deferred). The target factories stay eax-agnostic; #3 surfaces the external-joint values/names into the joint vectors and guards against an omitted external value (`UncoordinatedTargetError`).
- Every new component ships four files — `code.py` (`# r: compas_fab>=2.0.1`, `# r: tesseract-robotics-nanobind==0.35.0.7`, class-based `GH_ScriptInstance`, `except TesseractBackendError`, `error(ghenv.Component, …)`), `metadata.json` (category `COMPAS FAB`, subcategory `Backends`, exposure `2`, input/output param lists matching the `RunScript` signature), a 24×24 `icon.png`, and an `icon.svg` source.
- Because the GH contract test hardcodes component names, each new component must additionally be wired into: `tests/backends/tesseract/test_grasshopper_components.py` (a new static assertion block), the module-level `TESSERACT_USER_OBJECTS` tuple, and the three CI workflows (`build.yml`, `publish_yak.yml`, `release.yml`) with `Test-Path` guards. GUIDs are auto-generated at build; none are authored.

## Testing & exit gates

- **REPInvKin contract test** (new, `test_coupled_kinematics.py` style) — via `createInvKin("full_manipulator","REPInvKin",…)`: `numJoints() == 8`, `getJointNames() == [positioner_joint_1, positioner_joint_2, joint_1…joint_6]` (the joint-defined-group order, matching `positioner_sample_resolution`). No teardown scaffolding (`nb::keep_alive` shipped in 0.35.0.7; the ordered-`del` workaround was removed in `518bb16c`).
- **REP Cartesian proof** (new) — the positioner XY table coordinates (a positioner column travel exceeds a threshold well above the sample step) *and* `tool0` tracks the target within tolerance, measured on the native robot via `getKinematicGroup("full_manipulator","").calcFwdKin(...)["tool0"]`, raw numpy L2 `< 5e-3` m, position-only (the exact ROP proof discipline, not `compas.tolerance.TOL`). The proof pins the exact `working_frame`.
- **structural_validation** — full REP coupled cell loads (coupled branch), and an explicit before/after test proves the ordinary single-group / non-coupled path is un-regressed.
- **cell_assembly contract test** — synthesized cell structurally equivalent to the shipped reference cell (joint-defined coupled group) and loading through `set_robot_cell`.
- **GH static contracts** for the three new components + **E2E eax round-trip** extending `test_rapid_external_axes.py` (real `eax`, correct mm/deg units, byte-identical when `external_axes is None`).
- **Gates of record:** `pixi run mypy --strict src/compas_fab/backends/tesseract` clean · `pixi run ruff check` clean · `pixi run pytest tests/backends/tesseract -n auto` green (`-n auto` is the real keep_alive teardown stress test; compas_fab CI does not fire on the branch).
- **If a gate slips:** write `docs/superpowers/state/w4-e2-gate-<X>-analysis.md` (root cause + options) *before* patching.

## Deferred out of E2

- **`\WObj` seam (E3):** `MoveL … \WObj:=` as a positioner-driven coordinated mechanical unit · `ActUnit`/`DeactUnit` · external-axis `speeddata`. E2 stops at populated `eax` values (positional) and workpiece-relative target authoring; the emitted RAPID moves the positioner via its joint values, not yet via a coordinated work object.
- **Typed controller eax-slot map (was #7):** the native emitter's `ExternalAxisLayout` is positional with trailing-sentinel padding and **no slot index**; mapping a track to `eax_e` (leading gap) is unreachable through the typed API at 0.35.0.7 and needs a native leading-pad change in the tesseract binding repo, outside E2's compas_fab-only scope. The typed `ExternalAxisSlotMap` + its emission land together when that native support ships (tracked like E1's T5). E2 keeps E1's positional `external[0]→eax_a`.

## Decisions locked

1. **`\WObj` E2/E3 boundary → eax now, `\WObj` in E3.** E2 proves the kinematic coordination and populates `eax` positionally; the coordinated-RAPID semantics stay E3.
2. **GH assembly → dedicated component.** Thin axis components emit `ExternalAxisDescriptor`s; a dedicated `Cf_TesseractRobotCell` + `cell_assembly.py` performs synthesis. `Cf_TesseractRobotArtifact` stays a pure loader.
3. **REP proof decoupled from #6.** The proof uses coupled kinematics + world-fixed targets (no scene-attached rigid body); the `plan_cartesian_motion.py:113` tool/body gate stays shut on the proof path. Attached-workpiece collision handling (#6) lands with its own test, off the proof's critical path.
4. **Cell synthesis covers both topologies.** External Rotational Axis → REP (sibling positioner branch carrying the workpiece); External Linear Axis → ROP (serial track carrying the robot). Validates against E1's hand-authored ROP cell and the shipped REP reference cell.
5. **REP coupled group is joint-defined + backend config-sourced (#5).** The only formulation that meets the bar; joint-defined SRDF alone is insufficient (see the analysis doc). The single-group / non-coupled path stays un-regressed.
6. **Typed eax-slot map (#7) deferred out of E2.** Genuinely blocked on a native emitter change; deferred as a whole unit rather than shipped as a type that can't emit its headline case.

## Out (unchanged)

Coordinated RAPID semantics + the typed eax-slot map (both above) · yak packaging · the live-Rhino kernel harness as a CI gate (`tests/rhino/` is local-only, license-gated; GH component *logic* is validated headlessly via the static contract test).
