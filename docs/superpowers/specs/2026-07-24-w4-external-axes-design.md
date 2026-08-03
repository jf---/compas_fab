# W4 — external axes (world-class), design

**Status:** amends `2026-07-23-succinct-product-design.md` (W4). **Date:** 2026-07-24

External axes is the marquee W4 deliverable and the one permanent RobotComponents gap. The bar is **world-class**: robot + external axes solved as ONE coordinated kinematic system, both topologies, coordinated RAPID — not decoupled axis values.

## Verified ground truth

Coordinated external-axis motion is **achievable on the installed substrate; no native capability is missing** (proven live):

- `ROPInvKin` (Robot-On-Positioner = track **carries the robot**, moving base) and `REPInvKin` (Robot-with-External-Positioner = positioner **carries the workpiece**, TCP tracks it) ship in `tesseract-robotics-nanobind==0.35.0.6`, load from the reference cells `abb_irb2400_on_positioner.srdf` / `abb_irb2400_external_positioner.srdf` (present in default **and** `rhino39` envs), configured by `positioner_sample_resolution` (principled redundancy sampling over the external DOF).
- The IK / planning / structural-validation layers are already DOF-agnostic (no hard-coded 6). Feeding `ik_solver_name="ROPInvKin"` to `inverse_kinematics` returns coordinated 7-DOF solutions with zero code change.
- The whole tesseract stack is ours (`~/Code/CADCAM/tesseract_python_nanobind`), so the emitter is ours to fix.

The gap is **product-side glue + one emitter fix**, split across two repos.

## Two concerns, two repos

**A. Coordinated trajectory — `compas_fab_2` (glue on proven primitives):**
1. **ROP/REP plugin-YAML emission** — generalize `artifact.py`'s KDL-only `_kdl_plugin_yaml`/`with_kdl_kinematics` to emit `class: ROPInvKinFactory`/`REPInvKinFactory` with `manipulator`, `manipulator_reach`, `positioner`, `positioner_sample_resolution`. The YAML shape is already generic; KDL is one special case. Integration seam: `getKinematicGroup(group)` raises without a plugin config, so this YAML is mandatory.
2. **Combined-chain cell** — accept a URDF/SRDF whose SRDF carries the `full_manipulator` chain group (ROP: `positioner_base→tool0`; REP: positioner branch + `positioner_tool0→tool0`). `set_robot_cell.py`/`structural_validation.py` already parse it. (Phase E2 adds GH assembly of this cell; see phasing.)
3. **`plan_cartesian_motion` feature** — map `FrameTarget`s → native `WorkingFrameCartesianTarget` (exists) → **`DescartesFPipeline`** + `build_descartes_profiles(ik_solver=…)`. (The `CartesianPipeline` wrapper cannot carry `ik_solver`; only a Descartes profile's `manipulator_ik_solver` selects the coupled solver, and those profiles register for `DescartesFPipeline`.) The ROP/REP solver name is **derived** from the group's plugin config (T1 baked it as the group default), never a user option — a shared `native_coupled_ik_solver_name(CoupledTopology)` keeps emitter and planner from drifting. Redundancy is resolved by the ROP/REP IK enumerated inside each Descartes vertex — **not** the tool-axis `sample_axis` (a different thing). E1 = ROP + ROBOT target-mode, reusing the `plan_motion.py:74` attached-body gate verbatim; REP → E2.

**B. RAPID out with `eax` populated — `tesseract_python_nanobind` (emitter):**
The `ExternalAxis` model exists but is never populated. **This lands on the `emitters-multiplatform` worktree, not main** — that refactor deletes `dispatch.py` (→ `core/lowering.py` + `rapid/backend.py`) and already ships `core/units.py` (the SI→mm/deg chokepoint) and `core/errors.py::ExternalAxisError`; it anticipated this feature, so building on main would be thrown away. Fix: `lower()` splits the external DOF off the joint vector **by joint NAME** (the positioner joint is ordered *first*, so an index split would write arm values into eax slots) and populates `external_axis`; **keep** the `jointtarget` 6-joint gate (robax is exactly 6; externals are peeled off upstream, never a relaxed gate); convert **mm (prismatic) / deg (revolute)** via `core/units.py`, not `format_ext_axis` (which only formats + pads the eax tokens). External-axis metadata reaches the emitter as a typed `emit_rapid(…, *, external_axes=None)` kwarg — `None` is byte-identical to today. Golden test: emitted `robtarget`/`jointtarget` carry real `eax`, never `9E9`, correct units; existing goldens stay byte-identical when `external_axes is None`.

**Cross-cutting:** an **arm-vs-external joint classification** (which joints in a group are the 6 arm axes vs. track/positioner) — a new notion, absent today, needed to split the vector before emission. Lives at the `RobotCell`/semantics boundary.

## Type + error model (ETH baseline)

- External DOF carries its **topology** (`TRACK` linear / `POSITIONER` rotary) and **unit** (mm / deg) at the type level — not a bare float list. The classification is a typed mapping from group joint-name → role, derived once from the artifact.
- One named exception per failure mode: `UnknownKinematicTopologyError` (group has no ROP/REP-classifiable structure), `ExternalAxisUnitError` (topology↔unit mismatch), `UncoordinatedTargetError` (external DOF present but a target omits it). No bare `ValueError`.
- **Contract test at the emit boundary**: a coordinated trajectory → RAPID → re-parsed `eax` values round-trip within tolerance (mm/deg). This is the artifact-meets-downstream-tool gate.

## Phasing

- **E1 — coordinated core (artifact-supplied ROP cell). DONE + PROVEN.** Classification (T2, `61e4e34`), ROP plugin-YAML emission (T1, `63915da4`), `plan_cartesian_motion` (T3, `979da6c1` — `DescartesFPipeline` + `ROPInvKin`, solver read from the loaded plugin YAML `default` via `RobotArtifact.default_inv_kin_solver` + shared `native_coupled_ik_solver_name`), and the emitter `eax` fix (T4, **PR #127** on `emitters-multiplatform`). Coordination is demonstrated in a committed test: the positioner sweeps **1.0 m** to reach a 1.8 m span the arm alone can't, TCP to 1 mm. *World-class core, won.* **Remaining — T5, the end-to-end round-trip contract test** (coordinated plan → emitter → RAPID `eax` re-parse) — is **gated** on the `emitters-multiplatform` refactor + eax landing in compas_fab_2's env (the eax emitter is not installed there yet); deferred until that arrives via a release, not another local-wheel override.
- **E2 — Grasshopper assembly (RobotComponents parity).** `External Linear Axis` / `External Rotational Axis` components (plane, direction/axis, limits mm/deg, meshes) that synthesize the combined cell; `eax` inputs on targets; REP (moving workpiece + coordinated `MoveL … \WObj:=`). Extends `native_scene.py` beyond `JointType.FIXED` for actuated joints; resolves the `plan_motion.py:74` attached-body gap for positioner-mounted workpieces. Also lands two E1-surfaced items: the **sub-group `structural_validation` fix** (a rigidly-mounted arm sub-group's native kinematic root is `positioner_base_link`, not the SRDF-chain base `base_link`, so a full coupled cell with all three groups cannot yet load through `set_robot_cell` — E1 sidesteps this with a `full_manipulator`-only cell), and **controller-specific eax slot mapping** (E1 defaults to positional order external[0]→`eax_a`; real controllers may map a track to `eax_e`).
- **E3 — coordinated-RAPID polish.** `ActUnit`/`DeactUnit`, external-axis `speeddata`, `\WObj` coordination.

Each phase validates before the next (no hacksaw): E1's coordinated toolpath is proven before E2 layers ergonomics.

## Then (smaller W4 items)

- **Binary playback** = upload emitted RAPID `.mod` via RWS → `resetpp` → `start` (RobotComponents-parity; reuses the emitter + `Cf_AbbStart`/`Cf_AbbReset`). One upload component. Not `abb_motion_program_exec`.
- **Projection polish** = `ProjectionPolicy`/`ProjectionReport` (caller-declared required vs. allowed-missing fields; honest absence, not fabricated-zero effort) + a `Cf_TesseractProjectResult` component.

Out (unchanged): yak packaging.
