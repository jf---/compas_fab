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
3. **`plan_cartesian_motion` feature** — map `FrameTarget`s → native `WorkingFrameCartesianTarget` (exists) → CartesianPipeline + `build_descartes_profiles(..., ik_solver="ROPInvKin"/"REPInvKin")`. The redundancy is resolved by the ROP/REP IK enumerated inside each Descartes vertex — **not** by the tool-axis `sample_axis` (that samples tool rotation, a different thing).

**B. RAPID out with `eax` populated — `tesseract_python_nanobind` (emitter):**
The `ExternalAxis` model exists (`rapid_writer.py:149`) but is never populated. Fix: `dispatch.py:191/208` populate `external_axis` from the waypoint's external DOF; **split** the external DOF off the joint vector so `jointtarget` keeps its 6 slots and relax the `joints.size != 6` gate (`rapid_writer.py:201`); add **mm (prismatic) / deg (revolute)** conversion to `format_ext_axis` (`utils.py:87`, today unit-naive). Golden test: emitted `robtarget`/`jointtarget` carry real `eax`, never `9E9`, correct units. Reconcile with the in-flight `emitters-multiplatform` worktree before touching.

**Cross-cutting:** an **arm-vs-external joint classification** (which joints in a group are the 6 arm axes vs. track/positioner) — a new notion, absent today, needed to split the vector before emission. Lives at the `RobotCell`/semantics boundary.

## Type + error model (ETH baseline)

- External DOF carries its **topology** (`TRACK` linear / `POSITIONER` rotary) and **unit** (mm / deg) at the type level — not a bare float list. The classification is a typed mapping from group joint-name → role, derived once from the artifact.
- One named exception per failure mode: `UnknownKinematicTopologyError` (group has no ROP/REP-classifiable structure), `ExternalAxisUnitError` (topology↔unit mismatch), `UncoordinatedTargetError` (external DOF present but a target omits it). No bare `ValueError`.
- **Contract test at the emit boundary**: a coordinated trajectory → RAPID → re-parsed `eax` values round-trip within tolerance (mm/deg). This is the artifact-meets-downstream-tool gate.

## Phasing

- **E1 — coordinated core (artifact-supplied ROP cell).** Classification + ROP plugin-YAML emission + `plan_cartesian_motion` + emitter `eax` fix + the round-trip contract test. Proves the whole coordinated chain end-to-end on a known-good cell. *This is where world-class is won.*
- **E2 — Grasshopper assembly (RobotComponents parity).** `External Linear Axis` / `External Rotational Axis` components (plane, direction/axis, limits mm/deg, meshes) that synthesize the combined cell; `eax` inputs on targets; REP (moving workpiece + coordinated `MoveL … \WObj:=`). Extends `native_scene.py` beyond `JointType.FIXED` for actuated joints; resolves the `plan_motion.py:74` attached-body gap for positioner-mounted workpieces.
- **E3 — coordinated-RAPID polish.** `ActUnit`/`DeactUnit`, external-axis `speeddata`, `\WObj` coordination.

Each phase validates before the next (no hacksaw): E1's coordinated toolpath is proven before E2 layers ergonomics.

## Then (smaller W4 items)

- **Binary playback** = upload emitted RAPID `.mod` via RWS → `resetpp` → `start` (RobotComponents-parity; reuses the emitter + `Cf_AbbStart`/`Cf_AbbReset`). One upload component. Not `abb_motion_program_exec`.
- **Projection polish** = `ProjectionPolicy`/`ProjectionReport` (caller-declared required vs. allowed-missing fields; honest absence, not fabricated-zero effort) + a `Cf_TesseractProjectResult` component.

Out (unchanged): yak packaging.
