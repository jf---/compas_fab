# W4 E2 kickoff — REP topology + Grasshopper external-axis assembly

I'm starting **W4 E2 — REP topology + Grasshopper external-axis assembly** of the compas_fab Tesseract backend (`/Users/jelle/Code/CADCAM/compas_fab_2`, branch `feat/tesseract-backend`). Commit `66476df1` — **E1 done + proven, T5 landed, tesseract 0.35.0.7 consumed** — is the last hard checkpoint. PR #2 (`feat/tesseract-backend → master`) is open and green locally (622 tesseract tests pass).

## 1. What I want you to do
Use **superpowers:brainstorming** to lock the E2 design first — REP is a genuine *second topology*, not a config flag, and the `\WObj` coordination is a real E2/E3 boundary to scope. Then **superpowers:writing-plans → superpowers:subagent-driven-development** to execute. E1 was executed SDD-style (fresh implementer + task-reviewer per task); match it.

## 2. Read first, in dependency order
1. `docs/superpowers/specs/2026-07-24-w4-external-axes-design.md` — the W4 design. **E2 = the "Grasshopper assembly (RobotComponents parity)" bullet under Phasing.** E1 (done) is the coordinated core E2 layers ergonomics + the second topology onto.
2. `docs/superpowers/specs/2026-07-23-succinct-product-design.md` — the one-page product contract that supersedes the master spec: components are **thin marshals**, no frameworks (the replan weeded −10.7k lines of ceremony — do NOT resurrect it).
3. `/Users/jelle/.claude/CLAUDE.md` — load-bearing global rules (see §8 below for the ones that bite in E2).
4. `~/.claude/projects/-Users-jelle-Code-CADCAM-compas-fab-2/memory/MEMORY.md` and the files it points to — especially `w4-external-axis-world-class.md` (the bar: **both topologies, NOT mere parity**), `tesseract-backend-replan-decisions.md`, `tesseract-group-default-invkin-solver-name.md`.
5. **E1 source you consume + extend:**
   - `src/compas_fab/backends/tesseract/external_axes.py` — `ExternalAxis`, `ExternalAxisRole {TRACK, POSITIONER}`, `ExternalAxisUnit {MILLIMETRE, DEGREE}`, `CoupledGroupLayout`.
   - `src/compas_fab/backends/tesseract/artifact.py` — `CoupledTopology {ROBOT_ON_POSITIONER, ROBOT_WITH_EXTERNAL_POSITIONER}`, `CoupledKinematics`, `RobotArtifact.with_coupled_kinematics`, `native_coupled_ik_solver_name`, `default_inv_kin_solver`.
   - `src/compas_fab/backends/tesseract/backend_features/plan_cartesian_motion.py` — `TesseractPlanCartesianMotion` (DescartesFPipeline + the coupled solver read from the plugin YAML default); E2 exercises this with the REP cell.
   - `src/compas_fab/backends/tesseract/{native_scene.py, structural_validation.py, conversions.py, errors.py}` — the seams E2 modifies.
   - Contract tests to mirror: `tests/backends/tesseract/test_coupled_kinematics.py`, `test_plan_cartesian_motion_coupled.py`, `test_rapid_external_axes.py`, `test_grasshopper_components.py`.
6. **REP reference cell (already shipped in 0.35.0.7):** `abb_irb2400_external_positioner.{urdf,srdf}` under the tesseract data dir — resolve via
   `python -c "import tesseract_robotics,pathlib; print(pathlib.Path(tesseract_robotics.__file__).parent/'data'/'tesseract'/'support'/'urdf')"`.
   Positioner carries the **workpiece**; the coupled chain is the positioner branch + `positioner_tool0→tool0`. `REPInvKinFactory` ships in the binding.

## 3. Boundary contract (= your input — the shape E1 emits)
```python
class ExternalAxisRole(Enum): TRACK; POSITIONER          # track carries robot; positioner carries workpiece
class ExternalAxisUnit(Enum): MILLIMETRE; DEGREE          # TRACK->mm (prismatic), POSITIONER->deg (revolute)

@dataclass(frozen=True)
class ExternalAxis:
    name: str; role: ExternalAxisRole; unit: ExternalAxisUnit; index: int   # index in the coupled joint vector
    @classmethod
    def build(cls, name: str, role: ExternalAxisRole, index: int) -> "ExternalAxis": ...

@dataclass(frozen=True)
class CoupledGroupLayout:                                 # arm-vs-external split of a coordinated group
    arm_joint_names: tuple[str, ...]
    external_axes: tuple[ExternalAxis, ...]
    @classmethod
    def build(cls, robot_cell, coupled_group: str, manipulator_group: str) -> "CoupledGroupLayout": ...

@dataclass(frozen=True)
class CoupledKinematics:                                  # topology-tagged; REP => ROBOT_WITH_EXTERNAL_POSITIONER
    group: str; topology: CoupledTopology
    positioner_base_link: str; positioner_tip_link: str
    manipulator_base_link: str; manipulator_tip_link: str
    manipulator: OpwParameters; manipulator_reach: float
    positioner_sample_resolution: tuple[tuple[str, float], ...]
```

## 4. Deliverables (from the design doc's E2 bullet — cite it in the plan)
1. **REP coordinated toolpath** — `plan_cartesian_motion` for `ROBOT_WITH_EXTERNAL_POSITIONER` on `abb_irb2400_external_positioner`, proven like E1's ROP 1.0 m sweep: the positioner reorients the workpiece, the TCP tracks a workpiece-fixed frame, one coordinated solve. Contract test with the same tolerance discipline (TCP ≈ 1 mm).
2. **`External Linear Axis` / `External Rotational Axis` GH components** — inputs `plane, direction/axis, limits (mm/deg), meshes` → synthesize the combined URDF/SRDF cell that feeds `with_coupled_kinematics`. New `Cf_TesseractExternalLinearAxis` / `Cf_TesseractExternalRotationalAxis` under `src/compas_fab/ghpython/components_cpython/` — **thin marshals**, mirroring the 12 existing `Cf_Tesseract*` components (each has `code.py` + `metadata.json` + `icon.png` and the `# r: tesseract-robotics-nanobind==0.35.0.7` header — see `test_grasshopper_components.py`).
3. **`eax` inputs on target components** — coordinated targets carry external-axis values through to the emitter's `CoupledGroupLayout` path.
4. **Coordinated `MoveL … \WObj:=`** — REP emits work-object-relative motion. **This is the E2/E3 seam — scope it explicitly in brainstorming** (how much RAPID coordination lands here vs E3).
5. **Sub-group `structural_validation` fix** — a rigidly-mounted arm sub-group's native kinematic root is `positioner_base_link`, not the SRDF-chain base `base_link`. `structural_validation.py` (`get_base_link_name(group)` ~L39/L71) can't yet load a full three-group coupled cell through `set_robot_cell`; E1 sidesteps with a `full_manipulator`-only cell. **E2 must load the full cell.** Regression-guard the existing single-group path.
6. **`native_scene.py` actuated joints** — extend beyond `joint.type = JointType.FIXED` (L125) so positioner/track joints are actuated in the native scene. The attached-body handling for a positioner-mounted workpiece lives in `native_scene.py` + `conversions.py` (the design doc's `plan_motion.py:74` reference is **stale** — locate the current seam).
7. **Controller-specific eax slot mapping** — E1 defaults to positional order `external[0] → eax_a`; real controllers may map a track to `eax_e`. Introduce a **typed** mapping (not positional) in `external_axes.py`.

## 5. Exit gates (DERIVED — the design doc has no formal per-phase gates; confirm/adjust in brainstorming)
- REP coordinated toolpath proven on `abb_irb2400_external_positioner` (analogous to E1's ROP proof), TCP ≈ 1 mm, in a committed test.
- A full three-group coupled cell loads through `set_robot_cell` (structural_validation fix landed, single-group path un-regressed).
- GH end-to-end: `External Linear/Rotational Axis` → synthesized cell → coordinated plan → RAPID with real `eax`, round-tripped (extends the E1 `test_rapid_external_axes.py` contract).
- `pixi run mypy --strict src/compas_fab/backends/tesseract` clean; `pixi run ruff check` clean; `pixi run pytest tests/backends/tesseract -n auto` green.
- **If a gate slips, write `docs/superpowers/state/w4-e2-gate-<X>-analysis.md` (root cause + options) BEFORE patching.**

## 6. Decoupling rule
E2's cell-synthesis + REP planning + emit all run **headless** in the pixi `default` env — no Rhino needed. Validate GH component *logic* the way `test_grasshopper_components.py` does (assert on generated `code.py` + `metadata.json`, not a live solve). The live-Rhino kernel harness (`tests/rhino/`) needs a license, is local-only, and is **not a CI gate** — do not block E2 on it.

## 7. Load-bearing conventions
- **pixi EXCLUSIVELY** (no pip/conda/venv). The env already has released `tesseract-robotics-nanobind==0.35.0.7` (keep_alive + eax + REP). There is **no local wheel override anymore** — do not reintroduce one.
- ETH-baseline types: frame+unit at the type level; `Cls.build(...)` validates invariants; **one named exception per failure mode** (extend `errors.py` — e.g. `UnknownKinematicTopologyError`, `ExternalAxisUnitError`, `UncoordinatedTargetError`); never a bare `ValueError`.
- No `__all__` in `__init__.py`. No conditional imports / `try/except ImportError` / `HAS_*` / fallbacks — a dep is hard or absent. `compas.tolerance.TOL`/predicates over hand-rolled tolerances.
- **`@overload`** for any dual scalar-positional / arraylike calling convention (vector-quantity factories expose BOTH).
- `git mv` to move tracked files (repo morphology matters). Author **AND** committer = `Jelle Feringa <jelleferinga@gmail.com>`; no Co-Authored-By; never the string "🤖 Generated with Claude Code".
- Commits allowed without asking for this SDD flow (memory `commit-authorization-grant`); destructive ops still ask. Run pixi/pytest/ruff without asking.

## 8. Environment booby traps
- `pyproject.toml` pins **`autobahn==24.4.2`** (committed, commented) — autobahn 26.6.2 is a source-only sdist that fails to build on macOS/py3.12. Do NOT unpin; any re-resolve re-picks 26.6.2 and breaks the env.
- The coupled-solver teardown SIGSEGV is **fixed** (keep_alive in 0.35.0.7). New coupled tests need **no** ordered-`del` workaround (E1's was removed in `518bb16c`).
- **compas_fab CI does not run on the branch** (no PR checks fire). `pixi run pytest tests/backends/tesseract -n auto` is your gate of record — `-n auto` is mandatory and is the real keep_alive teardown stress test.
- Recover a nuked env with `pixi install` (resolves 0.35.0.7 + autobahn 24.4.2 from PyPI — no local wheel path).

## 9. Suggested file structure (mark modify vs create)
```
src/compas_fab/backends/tesseract/
  external_axes.py                          MODIFY  controller eax slot mapping (typed, not positional)
  native_scene.py                           MODIFY  actuated positioner/track joints (beyond JointType.FIXED @L125)
  structural_validation.py                  MODIFY  sub-group native root = positioner_base_link
  conversions.py                            MODIFY? positioner-mounted attached workpiece
  backend_features/plan_cartesian_motion.py MODIFY/VERIFY  REP path
  cell_assembly.py                          CREATE? External-axis -> combined URDF/SRDF synthesis
src/compas_fab/ghpython/components_cpython/
  Cf_TesseractExternalLinearAxis/           CREATE  code.py + metadata.json + icon.png (# r: ==0.35.0.7)
  Cf_TesseractExternalRotationalAxis/       CREATE
tests/backends/tesseract/
  test_plan_cartesian_motion_rep.py         CREATE  REP coordinated proof
  test_cell_assembly.py                     CREATE
  test_external_axis_components.py          CREATE  headless GH component contract
  test_structural_validation.py             MODIFY  full three-group cell loads
```

## 10. Risk + budget
No numeric budget in the design doc. Risks, highest first: (1) **`\WObj` coordination bleeds E2 into E3** — fix the seam in brainstorming; (2) **structural_validation sub-group-root fix ripples** into `set_robot_cell`/`native_scene` for ordinary single-group cells — regression-guard first; (3) **GH cell synthesis** (plane+axis+limits+mesh → URDF/SRDF) is the largest new surface — follow RobotComponents' external-axis semantics but emit a *Tesseract* cell.

## 11. Start
```
cat docs/superpowers/specs/2026-07-24-w4-external-axes-design.md
```
Then invoke **superpowers:brainstorming** to lock the E2 design — REP as a real topology, the E2/E3 `\WObj` boundary, and the GH cell-synthesis shape — before writing any code.

End of prompt.
