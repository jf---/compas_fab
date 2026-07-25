# W4 E2 — REP coordination resolved: it works, and here is the recipe

**Date:** 2026-07-25 · **Status:** live-probe resolution of the REP coordinated-planning question. Supersedes the two open questions in [`w4-e2-rep-coupled-group-analysis.md`](w4-e2-rep-coupled-group-analysis.md) (the coupled **working frame** and whether Descartes can coordinate REP at all). Evidence from native probes of the shipped `abb_irb2400_external_positioner` cell (tesseract-robotics-nanobind 0.35.0.7) cross-checked against the reference `tesseract_ros_workcell` (twc) positioner workcell.

**REP coordinates through Descartes + `REPInvKin`, and the TCP tracks the workpiece to ~1 mm.** An intermediate spike concluded "REP cannot coordinate through Descartes" — that was **wrong**, caused by two non-fundamental issues: (1) a **positioner joint-order swap** introduced by the shipped cross-fork `<chain>` group, and (2) driving the plan with the **bare default Descartes profile** instead of the reference's custom evaluators. With the coupled group defined as a **`<joint>` list in positioner-forward order**, a forcing Cartesian path yields **`positioner travel 1.200 m` and `endpoint TCP-tracking error 0.0010 m`** (< the 5 mm bar). The reference workcell independently confirms the recipe.

## Evidence table

| Claim | Value | Source |
|---|---|---|
| Coupled solver is `REPInvKin` (positioner=KDL FK, manipulator=OPW IK) | `default_inv_kin_solver("full_manipulator") == "REPInvKin"` | `rep_probe8` + emitter `artifact.py:_coupled_plugin_yaml` |
| Only `positioner_tool0` is a valid working frame | `world` → `RuntimeError: Specified working frame (world) is not in the list of identified working frames. Available: [positioner_tool0, positioner_tool0]` | `rep_probe6`/`9` (solver), `rep_probe8` (pipeline: uncatchable C++ abort) |
| Cross-fork `<chain>` group reverses the positioner pair | scene order `[positioner_joint_2, positioner_joint_1, …]` vs KDL/config order `[positioner_joint_1, positioner_joint_2, …]` | `rep_probe1`/`11` |
| The reversal is the whole "non-tracking" story | chain group: **32 / 580** solutions track as-is, **580 / 580** after swapping pj1↔pj2 | `rep_probe11` |
| Joint-defined group fixes it | joint group: **580 / 580** track as-is; forcing plan `pj2 travel 1.200 m`, `endpoint track 0.0010 m` | `rep_probe11` |
| Chain group left unfixed | forcing plan `pj1 travel 1.200 m` (wrong axis), `endpoint track 2.1213 m` | `rep_probe11` |
| TCP tracks the *moving* workpiece | `tool0 == positioner_tool0(q) · target_local`, residual ≈ sample resolution (0.1 m → ~1 mm) | `rep_probe6`/`10` |
| Reference REP params | `manipulator_reach: 2.3`; each `positioner_sample_resolution` has `value` **+ `min` + `max`** (`±0.2`) | twc `workcell_positioner_plugins.yaml` |
| Reference working frame / TCP | `ManipulatorInfo("manipulator", working_frame="part_link", tcp="st_tool0")` — waypoints authored **in the part frame** | twc `raster_applicataion.h` |
| Reference coupled joint order | `[positioner_base_joint, positioner_joint_1, robot_joint_1..6]` (positioner forward-chain first) | twc `raster_applicataion.h` |
| Reference tames redundancy with custom Descartes evaluators | `target_pose_sampler`, `vertex_evaluator`, `edge_evaluator` + `joint_weights` | twc `twc_planning_server_node.cpp:createDescartesPlanProfile` |

## Root cause 1 — positioner joint-order swap

The REP cell is a **Y-fork at `world`**: the robot and the positioner are sibling branches. The shipped coupled group `full_manipulator = <chain base_link="positioner_tool0" tip_link="tool0">` traces **up** the positioner branch and **down** the robot branch.

```text
                              world  ← coupled-group base
                 ┌──────────────┴───────────────┐
   world_positioner_joint (fixed)        world_robot_joint (fixed)
                 │                              │
        positioner_base_link                base_link
                 │ positioner_joint_1 (prismatic Y)   │ joint_1 … joint_6 (revolute)
        positioner_link_1                    link_6
                 │ positioner_joint_2 (prismatic X)   │ joint_6-tool0 (fixed)
        positioner_tool0  ·······················→  tool0
   (coupled <chain positioner_tool0 → tool0> walks UP the positioner branch —
    reversing the prismatic pair — then DOWN the robot branch to tool0)
```

The `REPInvKin` positioner forward-kinematics is **KDL over `positioner_base_link → positioner_tool0`**, so the solver emits positioner values in **forward-chain order** `[positioner_joint_1, positioner_joint_2]` — matching the `positioner_sample_resolution` YAML order. But the `<chain>`-defined group's scene-graph `JointGroup`, having walked the positioner branch **upward**, orders the pair **reversed** `[positioner_joint_2, positioner_joint_1]`.

`kg.calcInvKin` returns solutions in **solver order**; `kg.calcFwdKin` and the trajectory read them in **group order**. When the two disagree, the two positioner columns are transposed — every solution *looks* off-target unless `pj1 == pj2` (which is why the surviving "tracking" solutions all sat on the diagonal).

!!! warning "This masqueraded as a fundamental limitation"
    With the swap in place, `calcInvKin` returns 580 solutions of which only 32 track, and Descartes' min-motion tie-break selects a non-tracking null-space drift (TCP frozen, wrong positioner axis sweeping). It reads exactly like "the positioner is redundant and Descartes can't coordinate it." It is not — swapping the two positioner columns turns **all 580** solutions into tracking solutions (`rep_probe11`).

**Fix:** define the coupled group as a `<joint>` list in positioner-forward order (`positioner_joint_1, positioner_joint_2, joint_1..joint_6`) so the scene-graph group order equals the KDL/solver order. Then `calcInvKin` and `calcFwdKin` agree, and Descartes coordinates.

## Root cause 2 — the default Descartes profile lacks the reference's evaluators

The redundant-solution set is **expected** for a coupled solver; the reference tames it with a custom `DescartesDefaultPlanProfile` (twc `createDescartesPlanProfile`, excerpted):

```cpp
// tool-axis (spindle spin) sampling
profile->target_pose_sampler = [](const Eigen::Isometry3d& p){ return sampleToolZAxis(p, M_PI_4); };

// keep only NUT/FUT arm configs, reject redundant wrist turns
profile->vertex_evaluator = [](const auto& prob){
    return std::make_shared<DescartesStateValidator>(prob.manip, "robot_base_link", "robot_tool0"); };

// config consistency along the path + weighted joint motion
Eigen::VectorXd joint_weights = Eigen::VectorXd::Ones(8);   // 2-axis positioner (rail case: weights[0]=0.5)
profile->edge_evaluator = [joint_weights](const auto& prob){
    auto e = std::make_shared<CompoundEdgeEvaluator<float>>();
    e->evaluators.push_back(std::make_shared<RobotConfigEdgeEvaluator<float>>(prob.manip, "robot_base_link", "robot_tool0"));
    e->evaluators.push_back(std::make_shared<WeightedEuclideanDistanceEdgeEvaluator<float>>(joint_weights));
    return e; };
```

These are the coordination-quality knobs: `vertex_evaluator` prunes bad/redundant arm configs, `edge_evaluator` holds the robot configuration consistent across the raster and weights the positioner-vs-arm trade-off (`joint_weights`). After the order fix, even the bare default profile *tracks* at the waypoint endpoints; the evaluators are what make a full raster robust (no config flips mid-stroke, controllable coordination bias).

## Python translation (`tesseract_robotics` 0.35.0.7)

The twc code targets an older tesseract C++ generation. The concepts port cleanly and the hooks **are** reachable from Python, but it is **not literally 1:1** — the API surface moved, verified by live introspection of the installed bindings:

| twc C++ (ROS1-era) | Python 0.35.0.7 equivalent | status |
|---|---|---|
| `profile->vertex_evaluator = std::function` | override `DescartesDefaultPlanProfileD.createStateEvaluator(move, manip_info, env)` | ✅ trampoline verified |
| `profile->edge_evaluator = std::function` | override `DescartesDefaultPlanProfileD.createEdgeEvaluator(move, manip_info, env)` | ✅ trampoline verified |
| `target_pose_sampler = sampleToolZAxis(p, π/4)` | fields `target_pose_fixed=False`, `target_pose_sample_axis=[0,0,1]`, `…_min=-π`, `…_max=π`, `…_resolution=π/4` | ✅ fields present |
| coupled-solver selection | `profile.manipulator_ik_solver = "REPInvKin"` | ✅ |
| `DescartesVertexEvaluator` / `descartes_light::EdgeEvaluator` base | `DescartesStateEvaluatorD` / `DescartesEdgeEvaluatorD`, override `evaluate(state)->(bool,cost)` | ✅ subclassable |
| `CompoundEdgeEvaluator` | **not bound** — fold both checks into one `evaluate` | ⚠ adapt |
| `getRobotConfig` / `getJointTurns` / `RobotConfig` | **not bound in any submodule** — reimplement in Python or rely on `use_redundant_joint_solutions=False` | ⚠ gap |

!!! note "Verified vs illustrative"
    The class names, `evaluate` signatures (`evaluate(self, start, end) -> tuple[bool, float]`, `DescartesStateD.values` → joint vector), subclass trampolines, and profile fields are **verified by live introspection**. The `WeightedEuclideanEdge` body is behaviourally 1:1 with the reference; the NUT/FUT + wrist-turn filter depends on helpers Tesseract does not bind in Python.

```python
import numpy as np
from tesseract_robotics.tesseract_motion_planners_descartes import (
    DescartesDefaultPlanProfileD, DescartesEdgeEvaluatorD, DescartesStateD,
)

class WeightedEuclideanEdge(DescartesEdgeEvaluatorD):
    """twc WeightedEuclideanDistanceEdgeEvaluator — weighted L1 joint cost (1:1)."""
    def __init__(self, weights):
        super().__init__()
        self._w = np.asarray(weights, dtype=float)

    def evaluate(self, start: DescartesStateD, end: DescartesStateD):
        cost = float(np.sum(self._w * np.abs(end.values - start.values)))
        return (True, cost)          # (is_valid, cost)


class CoordinatedPositionerProfile(DescartesDefaultPlanProfileD):
    """twc createDescartesPlanProfile(ROBOT_WITH_2AXIS_POSITIONER), adapted to 0.35.0.7."""
    def __init__(self, ik_solver="REPInvKin", n_dof=8):
        super().__init__()
        self.manipulator_ik_solver = ik_solver           # coupled REP solver
        self.allow_collision = True
        self.use_redundant_joint_solutions = False        # stand-in for the wrist-turn filter
        # tool-axis sampling == sampleToolZAxis(pose, pi/4)
        self.target_pose_fixed = False
        self.target_pose_sample_axis = np.array([0.0, 0.0, 1.0])
        self.target_pose_sample_min = -np.pi
        self.target_pose_sample_max = np.pi
        self.target_pose_sample_resolution = np.pi / 4.0
        self._weights = np.ones(n_dof)                    # Ones(8); rail case: weights[0]=0.5

    def createEdgeEvaluator(self, move_instruction, manip_info, env):
        return WeightedEuclideanEdge(self._weights)
    # createStateEvaluator: the NUT/FUT + redundant-turn filter (twc DescartesStateValidator)
    # needs getRobotConfig / getJointTurns, which are NOT bound at 0.35.0.7.
```

The one piece that does not port as a bound call — `getRobotConfig`/`getJointTurns` (the arm-configuration + redundant-turn guard against flips mid-raster) — is the remaining work for backend gap #4: either reimplement that classification in Python or get the helpers bound in the tesseract binding repo.

## Canonical REP recipe (from the reference workcell)

!!! note "CONFIRMED against the reference"
    - **Working frame = the workpiece riding the positioner** (`part_link`, a child of `positioner_tool0`), **TCP = the robot tool** (`st_tool0`); Cartesian waypoints are authored **in the part frame**. As the positioner moves the part, the world targets move and the arm coordinates to follow. This is why the solver rejects `world`/`positioner_base_link` as working frames — only the positioner-tip / part frame is coordinated.
    - **Coupled group joint order** `[positioner…forward-chain…, robot_joint_1..6]`.
    - **REP plugin:** `REPInvKinFactory`, `manipulator_reach: 2.3`, positioner `KDLFwdKinChainFactory`, manipulator `OPWInvKinFactory`; each `positioner_sample_resolution` entry carries `value` **+ `min` + `max`** (`±0.2` windows bound the ladder graph).
    - **Planner:** Descartes for the coordinated Cartesian seed (with the evaluators above), wrapped in a raster pipeline (Descartes → TrajOpt refine → OMPL freespace transitions).
    - **REP and ROP are separate workcells** (`workcell_positioner` vs `workcell_rail`), never combined. The abb three-group cell (`manipulator` / `positioner` / `full_manipulator`) is a test fixture.

## Correction to the prior analysis

!!! warning "Supersedes `w4-e2-rep-coupled-group-analysis.md`"
    That doc concluded the coupled working frame is `positioner_base_link` — "a `world`-fixed link, not `positioner_tool0`." That is **wrong**: `REPInvKin` accepts **only** `positioner_tool0` (the moving positioner tip / part frame) as its working frame, and rejects any `world`-fixed frame. Its other conclusions stand: COMPAS cannot parse the cross-fork `<chain>` (so the group must be `<joint>`-defined), and the backend must source the coupled group's base/tip from the config rather than COMPAS's joint-group accessors.

## Backend gaps to close (prioritized)

1. **Positioner joint order (correctness-critical).** Emit the coupled REP group as a `<joint>` list in positioner-forward / KDL order — not the shipped cross-fork `<chain>`. Proven to flip tracking from 2.12 m to 1 mm.
2. **Coupled working/TCP frames.** In `plan_cartesian_motion.py:133-134`, source `working_frame` = the positioner tip / attached-part frame and `tcp_frame` = `manipulator_tip_link` from the coupled config — not COMPAS's SRDF-derived base/tip (which give `positioner_base_link` / `link_5`).
3. **`min`/`max` sample bounds.** `CoupledKinematics` + `_coupled_plugin_yaml` (`artifact.py:213, 261, 649`) emit only `{name, value}`; add optional `min`/`max` per positioner sample to match the reference and bound the ladder graph.
4. **Descartes coordination evaluators.** `build_descartes_profiles` (`descartes_profiles.py`) builds a stock profile (sampling + collision + `use_redundant_joint_solutions`) and never installs the reference's `vertex`/`edge` evaluators. The Python hooks **are** reachable — subclass `DescartesDefaultPlanProfileD` and override `createStateEvaluator`/`createEdgeEvaluator` (+ `target_pose_sample_*` fields, `manipulator_ik_solver`) — see the translation above. The only true binding gap is `getRobotConfig`/`getJointTurns` (unbound at 0.35.0.7), needed for the NUT/FUT + wrist-turn guard; reimplement in Python or bind upstream.

## Reference paths

- twc REP plugin config: `…/tesseract_ros_workcell/twc_support/config/workcell_positioner_plugins.yaml`
- twc coupled group (chain): `…/twc_support/config/workcell_positioner.srdf`
- twc consumer (ManipulatorInfo + joint order): `…/twc_application/include/twc_application/raster_applicataion.h`
- twc Descartes evaluators + `joint_weights`: `…/twc_motion_planning/{include/twc_motion_planning/utils.h, src/twc_planning_server_node.cpp}`
