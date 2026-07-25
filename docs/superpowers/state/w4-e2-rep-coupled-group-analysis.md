# W4 E2 — REP coupled-group representation: root cause + fix

**Date:** 2026-07-24 · **Status:** planning-time discovery; reshapes the E2 design's deliverable #5 and #7. Evidence from live probes of the shipped `abb_irb2400_external_positioner` cell (tesseract-robotics-nanobind 0.35.0.7).

!!! warning "Partially superseded by [`w4-e2-rep-coordination-resolved.md`](w4-e2-rep-coordination-resolved.md) (2026-07-25)"
    The coupled **working frame** here (`positioner_base_link` / "a `world`-fixed link, not `positioner_tool0`", §The fix B and Consequences) is **wrong**. `REPInvKin` accepts **only** `positioner_tool0` (the moving positioner tip / attached-part frame) and rejects any `world`-fixed frame. The joint-defined-group requirement and config-sourced base/tip conclusions below still hold; the follow-up doc adds the joint-**order** fix, the working-frame correction, and the reference-workcell recipe.

The REP coupled group needs a **joint-defined SRDF group** *plus* a **backend that sources the coupled group's base/tip links from the `CoupledKinematics` config** (not from COMPAS's joint-group accessors), applied in both `structural_validation` and `plan_cartesian_motion`. An SRDF-only fix is provably impossible. This is the single sound approach; the spike ruled out the alternatives.

## Root cause

The REP reference cell is a Y-fork at `world`: the robot base (`base_link`) and the positioner (`positioner_base_link → positioner_tool0`) are **sibling branches**, joined only at `world`. The coupled planning group `full_manipulator` is shipped as `<chain base_link="positioner_tool0" tip_link="tool0">` — a path that crosses the fork.

- **COMPAS cannot parse it.** `RobotSemantics.from_srdf_string` → `_get_group_link_names` → `RobotModel.iter_link_chain("positioner_tool0","tool0")` raises `Exception("No chain found between the specified element")` — COMPAS's `iter_chain` is a tree-only walker and cannot route up-through-`world`-and-back-down. This fires **before** `structural_validation` runs, so the spec's framing (deliverable #5 = `structural_validation` base reconciliation) never even gets reached for REP. `manipulator` (`world→tool0`) and `positioner` (`world→positioner_tool0`) build fine; only the cross-fork `full_manipulator` fails.
- **The native coupled-group base is `world`, not `positioner_tool0`.** So the spec's "targets authored relative to `positioner_tool0` (the working frame)" is also wrong.

## Spike: joint-defined group is necessary but not sufficient

Rewriting `full_manipulator` from `<chain>` to an explicit `<joint>` list (`positioner_joint_1, positioner_joint_2, joint_1…joint_6`), everything else identical:

| Check | Result |
|---|---|
| COMPAS `RobotSemantics.from_srdf_string` builds the cell | ✅ succeeds (a `<joint>` group never calls `iter_link_chain`) |
| Native `getKinematicGroup("full_manipulator")` loads `REPInvKin`, `numJoints()==8` | ✅ unchanged |
| Native joint order | ✅ **improved** — `<joint>` group yields `[positioner_joint_1, positioner_joint_2, joint_1…6]` (matches `positioner_sample_resolution` order), vs the `<chain>` group's odd `[positioner_joint_2, positioner_joint_1, …]` |
| `structural_validation` joint-order check (`:66-68`) | ✅ passes (native and COMPAS both `[pj1, pj2, j1..6]`) |
| `structural_validation` base-link check (`:70-73`) | ❌ **RAISES** `RobotArtifactMismatchError: … base link differs: Tesseract 'world', COMPAS 'positioner_base_link'` |
| `plan_cartesian_motion` working/tcp frames (`:133-134`) | ❌ COMPAS gives base `positioner_base_link`, **tip `link_5`** — the solve needs tip `tool0` |

Two residual mismatches, **neither addressable in SRDF**:

1. **Base link.** COMPAS derives a joint-group's base as the parent of the first joint (`positioner_base_link`); the coupled group's true native root is `world`. The obvious SRDF patch — prepend `<link name="world"/>` to steer COMPAS — makes Tesseract reject the group at env init: `Group: 'full_manipulator' is empty or multiple types were provided!` (groups must be single-type: chain XOR joints XOR links XOR subgroups). Hard dead-end.
2. **Tip link.** COMPAS's `get_end_effector_link_name` = last joint's **parent** = `link_5`. `tool0` is a leaf — only ever a joint *child*, never a *parent* — so **no `<joint>` arrangement can make COMPAS report `tool0`**. Structurally unreachable.

And `structural_validation:77` (`model.iter_joint_chain(compas_base, compas_tip)`, reached only after the base check) would *also* fail for a coupled group — `iter_joint_chain(positioner_base_link, link_5)` is again a cross-branch walk.

## The fix (single sound approach)

**A. SRDF — coupled group is joint-defined.** The reference-cell REP proof rewrites `full_manipulator` to a `<joint>` list on the COMPAS side (as E1's ROP proof already rewrites the SRDF to strip sub-groups); `cell_assembly` emits synthesized coupled groups as `<joint>` lists. This is the necessary foundation and, as a bonus, canonicalizes the coupled joint order.

**B. Backend sources coupled base/tip from the config, not from COMPAS.** A group is *coupled* iff the artifact's default solver for it is a ROP/REP solver (`RobotArtifact.default_inv_kin_solver(group)` names `ROPInvKin`/`REPInvKin`). For a coupled group:

- **`structural_validation`** gains a coupled branch: validate joint **names + order + per-joint properties** by iterating the group's joints directly (they match), and confirm the native base is the kinematic root — but **do not** compare against COMPAS's joint-group base, and **do not** walk `iter_joint_chain` across the fork. Non-coupled groups keep the exact existing path (regression-guarded).
- **`plan_cartesian_motion`** sources `tcp_frame` (and, for cleanliness, `working_frame`) for a coupled group from the coupled config — `tcp_frame = manipulator_tip_link` (`tool0`), `working_frame = positioner_base_link` (a frame fixed to `world`; the ROP precedent used the same link and produced TCP-accurate plans). The REP proof is the oracle that pins the exact `working_frame` by asserting TCP accuracy.

The coupled config's links are read from the emitted kinematics-plugin YAML (the same authoritative bytes `default_inv_kin_solver` already reads), so nothing drifts from what the native solver runs.

## Deliverable #7 (typed eax slot map) — blocked on a native change

The native emitter's `ExternalAxisLayout` is **purely positional with trailing-sentinel padding and no slot index**: `external_axes[0]→eax_a, [1]→eax_b, …`, and `format_ext_axis` pads only trailing slots. Mapping a track to `eax_e` (leading `9E9` gap at `a–d`) — the exact case the spec names — is **impossible through the typed API at 0.35.0.7**. It requires a native leading-pad + slot-index change in the tesseract binding repo (the `emitters-multiplatform` line), which is outside E2's compas_fab-only scope. Dummy/placeholder specs can't stand in: `_split_external` requires the waypoint's joint-name set to exactly equal the layout's. So #7's headline capability cannot land in E2.

## Consequences for the design

- Deliverable #5 is reshaped: joint-defined SRDF + backend config-sourced coupled base/tip (bigger than "reconcile base links", but the only correct fix). The ROP-precedent single-group path stays un-regressed.
- The REP target-authoring story is corrected: the coupled working frame is the **positioner tip** `positioner_tool0` (the workpiece frame), authored workpiece-relative — full detail + the joint-order fix + the reference recipe are in the resolution doc linked in the banner above.
- `cell_assembly` must emit coupled groups as `<joint>` lists (drives the synthesis contract).
- Deliverable #7 is deferred until the native leading-pad emitter support ships (tracked like E1's T5, which was gated on the emitter refactor).
