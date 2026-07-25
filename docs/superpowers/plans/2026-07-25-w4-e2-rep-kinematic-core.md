# W4 E2 — REP kinematic core Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Prove a coordinated REP (robot-with-external-positioner) Cartesian toolpath on the shipped `abb_irb2400_external_positioner` cell — the TCP tracking a workpiece-relative path to ~1 mm while the positioner coordinates — by landing the four backend changes that let a full three-group REP cell load and plan.

**Architecture:** REP coordination already works on the installed substrate; the gaps are glue. A coupled group is defined as a `<joint>` list (positioner-forward / KDL order) so COMPAS can parse it *and* the scene-graph joint order matches the solver (the silent swap that made REP look un-coordinatable). The backend learns to treat a group as *coupled* when its default solver is `ROPInvKin`/`REPInvKin`, and for such groups sources the working/TCP frames from the coupled config (not COMPAS's joint-group accessors) and validates structure without the serial-chain assumptions. This is the first of three E2 plans (kinematic core → GH cell assembly → eax targets + attached workpiece + E2E); the two later plans build on the coupled-cell loading this one delivers.

**Tech Stack:** Python 3.12, `attrs`, `compas` / `compas_robots`, `tesseract-robotics-nanobind==0.35.0.7` (KDL positioner FK + OPW arm IK + `REPInvKin` + `DescartesFPipeline`), `pytest` + `pytest-xdist`, `mypy --strict`, `ruff`, `pixi`.

## Global Constraints

- **pixi exclusively** — no pip/conda/venv. The `default` env already has `tesseract-robotics-nanobind==0.35.0.7`; **do not** unpin `autobahn==24.4.2` (a re-resolve re-picks 26.6.2 and breaks the env).
- **Gates of record (all must pass before a task is done):** `pixi run pytest tests/backends/tesseract -n auto` green (`-n auto` mandatory — it is the real keep_alive teardown stress test) · `pixi run mypy --strict src/compas_fab/backends/tesseract` clean · `pixi run ruff check` clean. Run `pixi run ruff format` before committing.
- **No teardown scaffolding** — `nb::keep_alive` shipped in 0.35.0.7; coupled tests need no ordered-`del` / `gc.collect()` workaround (rely on `with TesseractClient(...)`; a single in-block `del native_handles; gc.collect()` before the client closes is allowed only to drop native handles, as E1's ROP proof does).
- **ETH type + error model** — typed quantities guard the native boundary; `Cls.build(...)` validates invariants; one named exception per failure mode (no bare `ValueError`); no `__all__` in `__init__.py`; no conditional imports / `try/except ImportError` / `HAS_*` / fallbacks; `compas.tolerance.TOL` for tolerances **except** the coordinated-proof TCP check, which uses a raw numpy L2 norm against a named `_CARTESIAN_FOLLOW_TOL_M` constant to match E1's established ROP-proof discipline verbatim.
- **Commits** — author **and** committer `Jelle Feringa <jelleferinga@gmail.com>`; no `Co-Authored-By`; never the string "🤖 Generated with Claude Code". Commits are pre-authorized for this SDD flow (no need to ask); destructive ops still ask.
- **REP cell facts (verified live):** 8 coupled DOF; coupled joint order `[positioner_joint_1, positioner_joint_2, joint_1..joint_6]`; positioner is a 2-DOF prismatic XY table (`positioner_joint_1` axis Y, `positioner_joint_2` axis X, limits −1.0…1.0 m); REP working frame is `positioner_tool0` (the positioner tip / workpiece frame — `REPInvKin` rejects any other); the shipped SRDF's `full_manipulator` is a cross-fork `<chain>` COMPAS cannot parse and whose scene order is *reversed*. Full evidence: `docs/superpowers/state/w4-e2-rep-coordination-resolved.md`.

---

### Task 1: `REPInvKin` contract test (8-DOF coordinated solver loads)

Locks that our generic coupled-plugin emission produces a working 8-DOF `REPInvKin` solver on the reference REP cell, in positioner-forward joint order. Mirrors the ROP contract `test_emitted_yaml_loads_coordinated_seven_dof_solver`. No production code — REP emission is already topology-agnostic; this characterization test must pass on first run and guards against regressions.

**Files:**
- Create: `tests/backends/tesseract/test_coupled_kinematics_rep.py`

**Interfaces:**
- Consumes: `RobotArtifact.build`, `RobotArtifact.with_coupled_kinematics`, `CoupledKinematics.build`, `OpwParameters.build`, `CoupledTopology.ROBOT_WITH_EXTERNAL_POSITIONER`, `KINEMATICS_PLUGIN_URL` (all in `compas_fab.backends.tesseract.artifact`).
- Produces: the module-level `_reference_rep_config()` and `_IRB2400_OPW` reused verbatim by Tasks 3 and 4 (copy them into those files — the codebase inlines fixtures per test file; see the ROP tests).

- [ ] **Step 1: Write the contract test**

```python
"""Coordinated REP kinematics: our emitted plugin YAML loads an 8-DOF REPInvKin solver.

Mirrors the ROP contract in test_coupled_kinematics.py for the robot-with-external-
positioner topology on abb_irb2400_external_positioner: two prismatic positioner
joints plus six arm joints, as one coupled system, in positioner-forward order.
"""

from pathlib import Path

import tesseract_robotics
from tesseract_robotics import tesseract_kinematics
from tesseract_robotics.tesseract_common import GeneralResourceLocator
from tesseract_robotics.tesseract_environment import Environment

from compas_fab.backends.tesseract.artifact import KINEMATICS_PLUGIN_URL
from compas_fab.backends.tesseract.artifact import CoupledKinematics
from compas_fab.backends.tesseract.artifact import CoupledTopology
from compas_fab.backends.tesseract.artifact import OpwParameters
from compas_fab.backends.tesseract.artifact import RobotArtifact

_SUPPORT_URDF = Path(tesseract_robotics.__file__).parent / "data" / "tesseract" / "support" / "urdf"
_REFERENCE = "abb_irb2400_external_positioner"
_FULL_GROUP = "full_manipulator"
# Positioner-forward (KDL) order: the two positioner joints first, then the six arm joints.
_EXPECTED_JOINTS = ["positioner_joint_1", "positioner_joint_2", "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

# IRB2400 OPW constants, from tesseract's online_planning_example_plugins.yaml.
_IRB2400_OPW = dict(
    a1=0.100,
    a2=-0.135,
    b=0.00,
    c1=0.615,
    c2=0.705,
    c3=0.755,
    c4=0.086,
    offsets=[0.0, 0.0, -1.57079632679, 0.0, 0.0, 0.0],
    sign_corrections=[1, 1, 1, 1, 1, 1],
)


def _reference_rep_config() -> CoupledKinematics:
    return CoupledKinematics.build(
        group=_FULL_GROUP,
        topology=CoupledTopology.ROBOT_WITH_EXTERNAL_POSITIONER,
        positioner_base_link="positioner_base_link",
        positioner_tip_link="positioner_tool0",
        manipulator_base_link="base_link",
        manipulator_tip_link="tool0",
        manipulator=OpwParameters.build(**_IRB2400_OPW),
        manipulator_reach=2.55,
        positioner_sample_resolution=[("positioner_joint_1", 0.1), ("positioner_joint_2", 0.1)],
    )


def test_emitted_rep_yaml_loads_coordinated_eight_dof_solver():
    urdf = (_SUPPORT_URDF / f"{_REFERENCE}.urdf").read_text()
    srdf = (_SUPPORT_URDF / f"{_REFERENCE}.srdf").read_text()

    locator = GeneralResourceLocator()
    env = Environment()
    assert env.initFromUrdfSrdf(urdf, srdf, locator), "reference REP cell failed to initialize"

    artifact = RobotArtifact.build(urdf, srdf, {}).with_coupled_kinematics(_reference_rep_config())
    plugin_yaml = artifact.resource(KINEMATICS_PLUGIN_URL).content.decode("utf-8")

    factory = tesseract_kinematics.KinematicsPluginFactory(plugin_yaml, locator)
    scene_graph = env.getSceneGraph()
    scene_state = env.getState()
    solver = factory.createInvKin(_FULL_GROUP, "REPInvKin", scene_graph, scene_state)

    # A single coordinated solver spanning the two positioner joints + the six arm joints.
    # createInvKin honours the positioner_sample_resolution order -> positioner joints first.
    assert solver.numJoints() == 8
    assert list(solver.getJointNames()) == _EXPECTED_JOINTS
```

- [ ] **Step 2: Run the test — expect PASS**

Run: `pixi run pytest tests/backends/tesseract/test_coupled_kinematics_rep.py -v`
Expected: PASS (REP emission is already generic; this locks it). If it FAILS on `numJoints`/order, stop — the emitter regressed and that is a separate bug.

- [ ] **Step 3: Gates + commit**

Run: `pixi run ruff format tests/backends/tesseract/test_coupled_kinematics_rep.py && pixi run ruff check tests/backends/tesseract/test_coupled_kinematics_rep.py`
Then:
```bash
git add tests/backends/tesseract/test_coupled_kinematics_rep.py
git -c user.name="Jelle Feringa" -c user.email="jelleferinga@gmail.com" \
  commit -m "test: REP contract — emitted YAML loads 8-DOF REPInvKin solver"
```

---

### Task 2: `RobotArtifact` coupled-group helpers (`is_coupled_group`, `coupled_group_frames`)

Two read-only methods the structural validator and the Cartesian planner both need: whether a group is coordinated, and its topology-aware working/TCP frames read from the authoritative emitted plugin YAML.

**Files:**
- Modify: `src/compas_fab/backends/tesseract/artifact.py` (add a module constant after `native_coupled_ik_solver_name` ~L90; add two methods to `RobotArtifact` after `default_inv_kin_solver` ~L583)
- Create: `tests/backends/tesseract/test_artifact_coupled_frames.py`

**Interfaces:**
- Consumes: `RobotArtifact.default_inv_kin_solver`, `native_coupled_ik_solver_name`, `CoupledTopology`, `KINEMATICS_PLUGIN_URL`, `InvalidKinematicsConfigError`, `yaml` (all already imported in `artifact.py`).
- Produces:
  - `RobotArtifact.is_coupled_group(group: str) -> bool`
  - `RobotArtifact.coupled_group_frames(group: str) -> Optional[tuple[str, str]]` returning `(working_frame, tcp_frame)` — for REP `("positioner_tool0", "tool0")`, for ROP `("positioner_base_link", "tool0")`, `None` for a non-coupled group. Consumed by Task 3 (`set_robot_cell`) and Task 4 (`plan_cartesian_motion`).

- [ ] **Step 1: Write the failing tests**

```python
"""RobotArtifact coupled-group detection and topology-aware working/TCP frames."""

from pathlib import Path

import pytest
import tesseract_robotics

from compas_fab.backends.tesseract.artifact import CoupledKinematics
from compas_fab.backends.tesseract.artifact import CoupledTopology
from compas_fab.backends.tesseract.artifact import OpwParameters
from compas_fab.backends.tesseract.artifact import RobotArtifact

_SUPPORT_URDF = Path(tesseract_robotics.__file__).parent / "data" / "tesseract" / "support" / "urdf"
_REP = "abb_irb2400_external_positioner"
_ROP = "abb_irb2400_on_positioner"
_IRB2400_OPW = dict(
    a1=0.100, a2=-0.135, b=0.00, c1=0.615, c2=0.705, c3=0.755, c4=0.086,
    offsets=[0.0, 0.0, -1.57079632679, 0.0, 0.0, 0.0], sign_corrections=[1, 1, 1, 1, 1, 1],
)


def _rep_artifact() -> RobotArtifact:
    urdf = (_SUPPORT_URDF / f"{_REP}.urdf").read_text()
    srdf = (_SUPPORT_URDF / f"{_REP}.srdf").read_text()
    config = CoupledKinematics.build(
        group="full_manipulator", topology=CoupledTopology.ROBOT_WITH_EXTERNAL_POSITIONER,
        positioner_base_link="positioner_base_link", positioner_tip_link="positioner_tool0",
        manipulator_base_link="base_link", manipulator_tip_link="tool0",
        manipulator=OpwParameters.build(**_IRB2400_OPW), manipulator_reach=2.55,
        positioner_sample_resolution=[("positioner_joint_1", 0.1), ("positioner_joint_2", 0.1)],
    )
    return RobotArtifact.build(urdf, srdf, {}).with_coupled_kinematics(config)


def _rop_artifact() -> RobotArtifact:
    urdf = (_SUPPORT_URDF / f"{_ROP}.urdf").read_text()
    srdf = (_SUPPORT_URDF / f"{_ROP}.srdf").read_text()
    config = CoupledKinematics.build(
        group="full_manipulator", topology=CoupledTopology.ROBOT_ON_POSITIONER,
        positioner_base_link="positioner_base_link", positioner_tip_link="base_link",
        manipulator_base_link="base_link", manipulator_tip_link="tool0",
        manipulator=OpwParameters.build(**_IRB2400_OPW), manipulator_reach=2.55,
        positioner_sample_resolution=[("positioner_joint_1", 0.1)],
    )
    return RobotArtifact.build(urdf, srdf, {}).with_coupled_kinematics(config)


def test_is_coupled_group_true_for_rep_full_manipulator():
    assert _rep_artifact().is_coupled_group("full_manipulator") is True


def test_is_coupled_group_false_for_uncoupled_group():
    # 'manipulator' has no inv-kin plugin entry -> default solver is None -> not coupled.
    assert _rep_artifact().is_coupled_group("manipulator") is False


def test_coupled_group_frames_rep_uses_positioner_tip_and_tool0():
    assert _rep_artifact().coupled_group_frames("full_manipulator") == ("positioner_tool0", "tool0")


def test_coupled_group_frames_rop_uses_positioner_base_and_tool0():
    assert _rop_artifact().coupled_group_frames("full_manipulator") == ("positioner_base_link", "tool0")


def test_coupled_group_frames_none_for_uncoupled_group():
    assert _rep_artifact().coupled_group_frames("manipulator") is None
```

- [ ] **Step 2: Run — expect FAIL**

Run: `pixi run pytest tests/backends/tesseract/test_artifact_coupled_frames.py -v`
Expected: FAIL — `AttributeError: 'RobotArtifact' object has no attribute 'is_coupled_group'`.

- [ ] **Step 3: Add the module constant**

In `src/compas_fab/backends/tesseract/artifact.py`, immediately after the `native_coupled_ik_solver_name` function (ends ~L90), add:

```python
# The native coordinated (coupled) inverse-kinematics solver names, for classifying a
# planning group as coordinated. Derived from the topology enum so the set cannot drift.
_COUPLED_SOLVER_NAMES = frozenset(native_coupled_ik_solver_name(topology) for topology in CoupledTopology)
_REP_SOLVER_NAME = native_coupled_ik_solver_name(CoupledTopology.ROBOT_WITH_EXTERNAL_POSITIONER)
```

- [ ] **Step 4: Add the two methods**

In `src/compas_fab/backends/tesseract/artifact.py`, add to `RobotArtifact` right after `default_inv_kin_solver` (ends ~L583):

```python
    def is_coupled_group(self, group: str) -> bool:
        """Whether a group's default solver is a coordinated ROP/REP solver.

        Args:
            group: Exact SRDF planning-group name.

        Returns:
            True when the emitted kinematics plugin makes the group's default
            inverse-kinematics solver the coordinated ``ROPInvKin``/``REPInvKin``.
        """
        return self.default_inv_kin_solver(group) in _COUPLED_SOLVER_NAMES

    def coupled_group_frames(self, group: str) -> Optional[tuple[str, str]]:
        """Return a coordinated group's ``(working_frame, tcp_frame)`` link names.

        For a coordinated group the TCP is the manipulator tip and the working
        frame is the positioner link the coordinated target references: the
        positioner **tip** for a robot-with-external-positioner cell (the workpiece
        rides it and targets are authored relative to it -- ``REPInvKin`` accepts
        only this frame) and the positioner **base** for a robot-on-positioner cell
        (a fixed reference the moving robot base is measured against). The links are
        read from the emitted kinematics-plugin YAML -- the same authoritative bytes
        the native solver loads -- so they cannot drift. COMPAS's joint-group
        accessors cannot report these for a cross-branch coupled group.

        Args:
            group: Exact SRDF planning-group name.

        Returns:
            ``(working_frame, tcp_frame)`` for a coordinated group, else ``None``.

        Raises:
            InvalidKinematicsConfigError: The group is coordinated but its plugin
                config lacks the positioner/manipulator link names.
        """
        solver = self.default_inv_kin_solver(group)
        if solver not in _COUPLED_SOLVER_NAMES:
            return None
        document = yaml.safe_load(self.resource(KINEMATICS_PLUGIN_URL).content.decode("utf-8"))
        group_plugins = document.get("kinematic_plugins", {}).get("inv_kin_plugins", {}).get(group, {})
        plugin_config = group_plugins.get("plugins", {}).get(solver, {}).get("config", {})
        positioner = plugin_config.get("positioner", {}).get("config", {})
        manipulator = plugin_config.get("manipulator", {}).get("config", {})
        tcp_frame = manipulator.get("tip_link")
        working_frame = positioner.get("tip_link") if solver == _REP_SOLVER_NAME else positioner.get("base_link")
        if not isinstance(working_frame, str) or not isinstance(tcp_frame, str):
            raise InvalidKinematicsConfigError("Coupled group {!r} plugin config lacks positioner/manipulator link names.".format(group))
        return working_frame, tcp_frame
```

- [ ] **Step 5: Run — expect PASS**

Run: `pixi run pytest tests/backends/tesseract/test_artifact_coupled_frames.py -v`
Expected: PASS (all five).

- [ ] **Step 6: Gates + commit**

Run: `pixi run ruff format src/compas_fab/backends/tesseract/artifact.py tests/backends/tesseract/test_artifact_coupled_frames.py && pixi run ruff check src/compas_fab/backends/tesseract/artifact.py tests/backends/tesseract/test_artifact_coupled_frames.py && pixi run mypy --strict src/compas_fab/backends/tesseract`
Then:
```bash
git add src/compas_fab/backends/tesseract/artifact.py tests/backends/tesseract/test_artifact_coupled_frames.py
git -c user.name="Jelle Feringa" -c user.email="jelleferinga@gmail.com" \
  commit -m "feat: RobotArtifact coupled-group detection + topology-aware frames"
```

---

### Task 3: `structural_validation` coupled branch + `set_robot_cell` wiring

Let a full three-group REP cell load through `set_robot_cell`. A coordinated group spans two scene-graph branches: its native root (`world`) is not the COMPAS joint-group base and no serial chain walks it, so for such a group validate joint names/order + per-joint properties directly and skip the serial-chain base/chain checks. Non-coupled groups keep the exact current path (regression-guarded).

**Files:**
- Modify: `src/compas_fab/backends/tesseract/structural_validation.py` (add `coupled_groups` param + a coupled branch)
- Modify: `src/compas_fab/backends/tesseract/backend_features/set_robot_cell.py` (compute + pass `coupled_groups`)
- Create: `tests/backends/tesseract/test_structural_validation_coupled.py`

**Interfaces:**
- Consumes: `RobotArtifact.is_coupled_group` (Task 2); the joint-list SRDF rewrite (defined in the test, reused by Task 4).
- Produces: `validate_robot_cell_structure(native_robot, robot_cell, artifact_groups, coupled_groups=frozenset())` — the new optional `coupled_groups: frozenset[str]` parameter (defaulted, so existing callers/tests are unaffected).

- [ ] **Step 1: Write the failing test**

```python
"""A full three-group REP cell loads through set_robot_cell (coupled branch)."""

import xml.etree.ElementTree as ElementTree
from pathlib import Path

import tesseract_robotics
from compas_robots import Configuration
from compas_robots import RobotModel
from compas_robots.model import Joint

from compas_fab.backends.tesseract.artifact import CollisionMeshPolicy
from compas_fab.backends.tesseract.artifact import CoupledKinematics
from compas_fab.backends.tesseract.artifact import CoupledTopology
from compas_fab.backends.tesseract.artifact import OpwParameters
from compas_fab.backends.tesseract.artifact_loader import ResourceRoot
from compas_fab.backends.tesseract.artifact_loader import RobotArtifactLoader
from compas_fab.backends.tesseract.client import TesseractClient
from compas_fab.backends.tesseract.planner import TesseractPlanner
from compas_fab.robots import RobotCell
from compas_fab.robots import RobotCellState
from compas_fab.robots import RobotSemantics

_DATA = Path(tesseract_robotics.__file__).parent / "data" / "tesseract"
_URDF = _DATA / "support" / "urdf" / "abb_irb2400_external_positioner.urdf"
_SRDF = _DATA / "support" / "urdf" / "abb_irb2400_external_positioner.srdf"
_RESOURCE_ROOT = _DATA.parent
_FULL_GROUP = "full_manipulator"
_COUPLED_JOINTS = ["positioner_joint_1", "positioner_joint_2", "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
_COUPLED_TYPES = [Joint.PRISMATIC, Joint.PRISMATIC, Joint.REVOLUTE, Joint.REVOLUTE, Joint.REVOLUTE, Joint.REVOLUTE, Joint.REVOLUTE, Joint.REVOLUTE]
_IRB2400_OPW = dict(
    a1=0.100, a2=-0.135, b=0.00, c1=0.615, c2=0.705, c3=0.755, c4=0.086,
    offsets=[0.0, 0.0, -1.57079632679, 0.0, 0.0, 0.0], sign_corrections=[1, 1, 1, 1, 1, 1],
)


def _joint_list_srdf() -> str:
    """The shipped SRDF with full_manipulator rewritten to a positioner-forward <joint> list.

    Sidesteps COMPAS's cross-fork chain walker and aligns the scene-graph joint order
    with the KDL solver so calcInvKin and calcFwdKin agree. All three groups are kept.
    """
    root = ElementTree.fromstring(_SRDF.read_text())
    for group in root.findall("group"):
        if group.get("name") == _FULL_GROUP:
            for chain in list(group.findall("chain")):
                group.remove(chain)
            for joint_name in _COUPLED_JOINTS:
                ElementTree.SubElement(group, "joint", {"name": joint_name})
    return ElementTree.tostring(root, encoding="unicode")


def _rep_config() -> CoupledKinematics:
    return CoupledKinematics.build(
        group=_FULL_GROUP, topology=CoupledTopology.ROBOT_WITH_EXTERNAL_POSITIONER,
        positioner_base_link="positioner_base_link", positioner_tip_link="positioner_tool0",
        manipulator_base_link="base_link", manipulator_tip_link="tool0",
        manipulator=OpwParameters.build(**_IRB2400_OPW), manipulator_reach=2.55,
        positioner_sample_resolution=[("positioner_joint_1", 0.1), ("positioner_joint_2", 0.1)],
    )


def _rep_artifact(tmp_path):
    srdf_path = tmp_path / "rep_jointgroup.srdf"
    srdf_path.write_text(_joint_list_srdf())
    loader = RobotArtifactLoader.build(_URDF, srdf_path, [ResourceRoot.build(_RESOURCE_ROOT)])
    return loader.load(CollisionMeshPolicy.CONVEX_HULL).with_coupled_kinematics(_rep_config())


def _rep_cell() -> RobotCell:
    model = RobotModel.from_urdf_string(_URDF.read_text())
    semantics = RobotSemantics.from_srdf_string(_joint_list_srdf(), model)
    return RobotCell(model, semantics)


def test_full_three_group_rep_cell_loads_through_set_robot_cell(tmp_path):
    cell = _rep_cell()
    assert set(cell.group_names) == {"manipulator", "positioner", "full_manipulator"}
    state = RobotCellState.from_robot_cell(cell)
    state.robot_configuration = Configuration([0.0] * 8, _COUPLED_TYPES, _COUPLED_JOINTS)

    with TesseractClient(_rep_artifact(tmp_path), cache_root=tmp_path) as client:
        planner = TesseractPlanner(client)
        planner.set_robot_cell(cell, state)  # must not raise: coupled branch validates full_manipulator
```

- [ ] **Step 2: Run — expect FAIL**

Run: `pixi run pytest tests/backends/tesseract/test_structural_validation_coupled.py -v`
Expected: FAIL — `RobotArtifactMismatchError: Planning group 'full_manipulator' base link differs: Tesseract 'world', COMPAS 'positioner_base_link'` (the serial base-link check at `structural_validation.py:70-73`).

- [ ] **Step 3: Add the coupled branch to `structural_validation.py`**

Change the signature and add the coupled skip + branch. Replace the function body's group loop. First, the signature (L19-23):

```python
def validate_robot_cell_structure(
    native_robot: Robot,
    robot_cell: RobotCell,
    artifact_groups: dict[str, Optional[tuple[str, str]]],
    coupled_groups: frozenset[str] = frozenset(),
) -> None:
    """Validate every unit-bearing kinematic field consumed by the backend.

    A coordinated (coupled ROP/REP) group named in ``coupled_groups`` spans two
    scene-graph branches; its native kinematic root is not the COMPAS joint-group
    base and no single serial chain walks it, so for such a group the base-link and
    chain-walk checks that assume a serial group are skipped -- joint names, order,
    and per-joint properties are still validated. Non-coupled groups take the exact
    serial path.
    """
```

Then guard the SRDF-chain check (currently `if artifact_chain is not None:` ~L37) with the coupled flag, and insert the coupled branch after the joint-order check (~L68) before the native-base check (~L70). The resulting group loop reads:

```python
    for group in robot_cell.group_names:
        if group not in artifact_groups:
            raise RobotArtifactMismatchError("Exact Tesseract SRDF has no semantic group {!r}.".format(group))
        coupled = group in coupled_groups
        artifact_chain = artifact_groups[group]
        if artifact_chain is not None and not coupled:
            artifact_base, artifact_tip = artifact_chain
            compas_base = robot_cell.get_base_link_name(group)
            compas_tip = robot_cell.get_end_effector_link_name(group)
            if artifact_base != compas_base:
                raise RobotArtifactMismatchError(
                    "Planning group {!r} base link differs: exact SRDF {!r}, COMPAS {!r}.".format(group, artifact_base, compas_base)
                )
            if artifact_tip != compas_tip:
                raise RobotArtifactMismatchError(
                    "Planning group {!r} tip link differs: exact SRDF {!r}, COMPAS {!r}.".format(group, artifact_tip, compas_tip)
                )
        compas_joints = robot_cell.get_configurable_joints(group)
        if not compas_joints:
            continue
        try:
            native_group = native_robot.env.getJointGroup(group)
        except (KeyError, RuntimeError) as error:
            raise RobotArtifactMismatchError("Tesseract artifact has no planning group {!r}.".format(group)) from error

        compas_names = [joint.name for joint in compas_joints]
        native_names = list(native_group.getJointNames())
        if native_names != compas_names:
            raise RobotArtifactMismatchError("Planning group {!r} joint order differs: Tesseract {}, COMPAS {}.".format(group, native_names, compas_names))

        if coupled:
            # A coupled ROP/REP group spans two scene-graph branches: its native root
            # (world) is not the COMPAS joint-group base and no serial chain walks it.
            # Validate each configurable joint directly by name; the base-link and
            # chain-walk checks below assume a serial group and do not apply.
            for compas_joint in compas_joints:
                try:
                    native_joint = native_robot.env.getJoint(compas_joint.name)
                except (KeyError, RuntimeError) as error:
                    raise RobotArtifactMismatchError("Tesseract coupled group {!r} is missing joint {!r}.".format(group, compas_joint.name)) from error
                _validate_joint(group, native_joint, compas_joint)
            continue

        native_base = native_group.getBaseLinkName()
        compas_base = robot_cell.get_base_link_name(group)
        if native_base != compas_base:
            raise RobotArtifactMismatchError("Planning group {!r} base link differs: Tesseract {!r}, COMPAS {!r}.".format(group, native_base, compas_base))

        compas_tip = robot_cell.get_end_effector_link_name(group)
        try:
            compas_chain = list(model.iter_joint_chain(compas_base, compas_tip))
        except (KeyError, RuntimeError, ValueError) as error:
            raise RobotArtifactMismatchError(
                "COMPAS planning group {!r} chain cannot be resolved from {!r} to {!r}: {}.".format(group, compas_base, compas_tip, error)
            ) from error
        for compas_joint in compas_chain:
            try:
                native_joint = native_robot.env.getJoint(compas_joint.name)
            except (KeyError, RuntimeError) as error:
                raise RobotArtifactMismatchError("Tesseract planning group {!r} is missing chain joint {!r}.".format(group, compas_joint.name)) from error
            _validate_joint(group, native_joint, compas_joint)
```

(This is the existing body with three additions: the `coupled` flag, `and not coupled` on the SRDF-chain check, and the `if coupled:` branch + `continue`. Everything else — `_validate_joint` and the helpers below — is unchanged.)

- [ ] **Step 4: Wire `coupled_groups` in `set_robot_cell.py`**

In `src/compas_fab/backends/tesseract/backend_features/set_robot_cell.py`, replace the single `validate_robot_cell_structure(...)` call (L48) with:

```python
        coupled_groups = frozenset(group for group in robot_cell.group_names if client.artifact.is_coupled_group(group))
        validate_robot_cell_structure(native_robot, robot_cell, artifact_groups, coupled_groups)
```

- [ ] **Step 5: Run the new test + the regression suite — expect PASS**

Run: `pixi run pytest tests/backends/tesseract/test_structural_validation_coupled.py tests/backends/tesseract/test_client.py -v`
Expected: the new coupled-load test PASSES; **every** existing `test_client.py` `set_robot_cell` test (joint-mismatch, tip-mismatch, jointless-group, one-joint) stays green — they are non-coupled, so `coupled_groups` is empty and their path is byte-identical.

- [ ] **Step 6: Gates + commit**

Run: `pixi run ruff format src/compas_fab/backends/tesseract/structural_validation.py src/compas_fab/backends/tesseract/backend_features/set_robot_cell.py tests/backends/tesseract/test_structural_validation_coupled.py && pixi run ruff check src/compas_fab/backends/tesseract tests/backends/tesseract/test_structural_validation_coupled.py && pixi run mypy --strict src/compas_fab/backends/tesseract`
Then:
```bash
git add src/compas_fab/backends/tesseract/structural_validation.py src/compas_fab/backends/tesseract/backend_features/set_robot_cell.py tests/backends/tesseract/test_structural_validation_coupled.py
git -c user.name="Jelle Feringa" -c user.email="jelleferinga@gmail.com" \
  commit -m "feat: structural_validation coupled branch — full REP cell loads"
```

---

### Task 4: `plan_cartesian_motion` coupled frame sourcing + the REP coordination proof

The marquee. For a coordinated group, source the Cartesian working/TCP frames from the coupled config (Task 2) instead of COMPAS's joint-group accessors (which give the wrong `link_5` tip), then prove a coordinated REP toolpath on the reference cell: the TCP tracks a workpiece-relative path to ~1 mm while the positioner sweeps the correct axis.

**Files:**
- Modify: `src/compas_fab/backends/tesseract/backend_features/plan_cartesian_motion.py` (frame sourcing at L133-134)
- Create: `tests/backends/tesseract/test_plan_cartesian_motion_rep.py`

**Interfaces:**
- Consumes: `RobotArtifact.coupled_group_frames` (Task 2); the coupled cell loading (Task 3); the `_joint_list_srdf` / `_rep_config` / `_rep_artifact` / `_rep_cell` fixture idiom (copy from Task 3's test file — the codebase inlines fixtures per file).
- Produces: nothing consumed downstream; this task's deliverable is the proven coordinated trajectory.

- [ ] **Step 1: Write the failing proof**

```python
"""Coordinated REP (robot-with-external-positioner) Cartesian proof.

The load-bearing proof: a FrameWaypoints path authored in the workpiece frame
(positioner_tool0), stepping ~1.2 m in the workpiece X, is followed by coordinating
the prismatic XY positioner with the six arm joints -- the positioner moves the
workpiece under the tool so the TCP tracks the workpiece-relative path. The robot
base is fixed at the world origin, so world-fixed reach is not the mechanism: only a
genuine coordinated solve tracks the moving workpiece target. Verified live: the
positioner sweeps ~1.0 m on the correct (X) axis and the TCP tracks to ~1 mm.
"""

import gc
import xml.etree.ElementTree as ElementTree
from pathlib import Path

import numpy as np
import tesseract_robotics
from compas.geometry import Frame
from compas.geometry import Transformation
from compas_robots import Configuration
from compas_robots import RobotModel
from compas_robots.model import Joint

from compas_fab.backends.tesseract.artifact import CollisionMeshPolicy
from compas_fab.backends.tesseract.artifact import CoupledKinematics
from compas_fab.backends.tesseract.artifact import CoupledTopology
from compas_fab.backends.tesseract.artifact import OpwParameters
from compas_fab.backends.tesseract.artifact_loader import ResourceRoot
from compas_fab.backends.tesseract.artifact_loader import RobotArtifactLoader
from compas_fab.backends.tesseract.client import TesseractClient
from compas_fab.backends.tesseract.planner import TesseractPlanner
from compas_fab.robots import FrameWaypoints
from compas_fab.robots import RobotCell
from compas_fab.robots import RobotCellState
from compas_fab.robots import RobotSemantics
from compas_fab.robots import TargetMode

_DATA = Path(tesseract_robotics.__file__).parent / "data" / "tesseract"
_URDF = _DATA / "support" / "urdf" / "abb_irb2400_external_positioner.urdf"
_SRDF = _DATA / "support" / "urdf" / "abb_irb2400_external_positioner.srdf"
_RESOURCE_ROOT = _DATA.parent
_FULL_GROUP = "full_manipulator"
_COUPLED_JOINTS = ["positioner_joint_1", "positioner_joint_2", "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
_COUPLED_TYPES = [Joint.PRISMATIC, Joint.PRISMATIC, Joint.REVOLUTE, Joint.REVOLUTE, Joint.REVOLUTE, Joint.REVOLUTE, Joint.REVOLUTE, Joint.REVOLUTE]
_IRB2400_OPW = dict(
    a1=0.100, a2=-0.135, b=0.00, c1=0.615, c2=0.705, c3=0.755, c4=0.086,
    offsets=[0.0, 0.0, -1.57079632679, 0.0, 0.0, 0.0], sign_corrections=[1, 1, 1, 1, 1, 1],
)
# The positioner (X axis, positioner_joint_2) must sweep to keep the TCP on the
# stepping workpiece path; observed ~1.0 m. Assert well above the 0.1 m sample step.
_MIN_POSITIONER_TRAVEL_M = 0.5
# tool0 tracks the workpiece-relative target; observed ~1 mm, bounded by the 0.1 m
# positioner sample step + Descartes discretisation (raw numpy L2, matching the ROP proof).
_CARTESIAN_FOLLOW_TOL_M = 5e-3


def _joint_list_srdf() -> str:
    root = ElementTree.fromstring(_SRDF.read_text())
    for group in root.findall("group"):
        if group.get("name") == _FULL_GROUP:
            for chain in list(group.findall("chain")):
                group.remove(chain)
            for joint_name in _COUPLED_JOINTS:
                ElementTree.SubElement(group, "joint", {"name": joint_name})
    return ElementTree.tostring(root, encoding="unicode")


def _rep_config() -> CoupledKinematics:
    return CoupledKinematics.build(
        group=_FULL_GROUP, topology=CoupledTopology.ROBOT_WITH_EXTERNAL_POSITIONER,
        positioner_base_link="positioner_base_link", positioner_tip_link="positioner_tool0",
        manipulator_base_link="base_link", manipulator_tip_link="tool0",
        manipulator=OpwParameters.build(**_IRB2400_OPW), manipulator_reach=2.55,
        positioner_sample_resolution=[("positioner_joint_1", 0.1), ("positioner_joint_2", 0.1)],
    )


def _rep_artifact(tmp_path):
    srdf_path = tmp_path / "rep_jointgroup.srdf"
    srdf_path.write_text(_joint_list_srdf())
    loader = RobotArtifactLoader.build(_URDF, srdf_path, [ResourceRoot.build(_RESOURCE_ROOT)])
    return loader.load(CollisionMeshPolicy.CONVEX_HULL).with_coupled_kinematics(_rep_config())


def _rep_cell() -> RobotCell:
    model = RobotModel.from_urdf_string(_URDF.read_text())
    semantics = RobotSemantics.from_srdf_string(_joint_list_srdf(), model)
    return RobotCell(model, semantics)


def _tool_down_in_workpiece(dx: float) -> Frame:
    # A tool-down pose at workpiece-X = dx, authored relative to positioner_tool0.
    return Frame([dx, 0.0, -0.1], [1.0, 0.0, 0.0], [0.0, -1.0, 0.0])


def _start_state(cell: RobotCell) -> RobotCellState:
    state = RobotCellState.from_robot_cell(cell)
    state.robot_configuration = Configuration([0.0] * 8, _COUPLED_TYPES, _COUPLED_JOINTS)
    return state


def test_coordinated_rep_plan_tracks_the_moving_workpiece(tmp_path):
    cell = _rep_cell()
    start_state = _start_state(cell)
    frames = [_tool_down_in_workpiece(-0.6), _tool_down_in_workpiece(0.6)]
    waypoints = FrameWaypoints(frames, TargetMode.ROBOT)

    with TesseractClient(_rep_artifact(tmp_path), cache_root=tmp_path) as client:
        planner = TesseractPlanner(client)
        planner.set_robot_cell(cell, start_state)
        trajectory = planner.plan_cartesian_motion(waypoints, start_state, group=_FULL_GROUP)

        # tool0 and the (moving) workpiece frame at each endpoint, measured natively.
        robot = client.environment.clone_robot()
        kinematic_group = robot.env.getKinematicGroup(_FULL_GROUP, "")
        ixx = list(kinematic_group.getJointNames()).index("positioner_joint_2")
        endpoints = []
        for point, frame in ((trajectory.points[0], frames[0]), (trajectory.points[-1], frames[-1])):
            fk = kinematic_group.calcFwdKin(np.asarray(point.joint_values, dtype=float))
            tool0_world = np.asarray(fk["tool0"].matrix, dtype=float)
            workpiece_world = np.asarray(fk["positioner_tool0"].matrix, dtype=float)
            want_world = workpiece_world @ np.asarray(Transformation.from_frame(frame).matrix, dtype=float)
            endpoints.append(float(np.linalg.norm(tool0_world[:3, 3] - want_world[:3, 3])))
        positioner_x = [point.joint_values[ixx] for point in trajectory.points]
        del kinematic_group, robot
        gc.collect()

    # A coordinated 8-DOF trajectory in positioner-forward order.
    assert trajectory.joint_names == _COUPLED_JOINTS
    assert trajectory.attributes["tesseract_pipeline"] == "DescartesFPipeline"

    # The X positioner coordinated (swept to keep the TCP on the workpiece path).
    positioner_travel = max(positioner_x) - min(positioner_x)
    assert positioner_travel > _MIN_POSITIONER_TRAVEL_M, "positioner did not coordinate: X travel {:.4f} m".format(positioner_travel)

    # The TCP tracked the moving workpiece-relative target at each endpoint.
    for error in endpoints:
        assert error < _CARTESIAN_FOLLOW_TOL_M, "tool0 left the workpiece path by {:.4f} m".format(error)
```

- [ ] **Step 2: Run — expect FAIL**

Run: `pixi run pytest tests/backends/tesseract/test_plan_cartesian_motion_rep.py -v`
Expected: FAIL — the planner uses COMPAS's `get_end_effector_link_name("full_manipulator")` (→ `link_5`) as the TCP frame, so the native motion program targets the wrong link and the plan either raises or produces a wildly non-tracking trajectory (`tool0 left the workpiece path by …`).

- [ ] **Step 3: Source coupled frames in `plan_cartesian_motion.py`**

In `src/compas_fab/backends/tesseract/backend_features/plan_cartesian_motion.py`, replace the two frame-resolution lines (currently L133-134):

```python
        working_frame = robot_cell.get_base_link_name(group_name)
        tcp_frame = robot_cell.get_end_effector_link_name(group_name)
```

with:

```python
        coupled_frames = client.artifact.coupled_group_frames(group_name)
        if coupled_frames is not None:
            # A coordinated ROP/REP group: COMPAS's joint-group accessors cannot report
            # the working/TCP frames of a cross-branch group (they give the wrong tip),
            # so source them from the coupled config -- the authoritative emitted bytes.
            working_frame, tcp_frame = coupled_frames
        else:
            working_frame = robot_cell.get_base_link_name(group_name)
            tcp_frame = robot_cell.get_end_effector_link_name(group_name)
```

- [ ] **Step 4: Run — expect PASS**

Run: `pixi run pytest tests/backends/tesseract/test_plan_cartesian_motion_rep.py -v`
Expected: PASS — `joint_names == _COUPLED_JOINTS`, `positioner_travel ≈ 1.0 m > 0.5`, both endpoint tracking errors ≈ 0.001 m < 0.005.

- [ ] **Step 5: Full gates + commit**

Run: `pixi run ruff format src/compas_fab/backends/tesseract/backend_features/plan_cartesian_motion.py tests/backends/tesseract/test_plan_cartesian_motion_rep.py && pixi run ruff check src/compas_fab/backends/tesseract tests/backends/tesseract && pixi run mypy --strict src/compas_fab/backends/tesseract && pixi run pytest tests/backends/tesseract -n auto`
Expected: ruff clean, mypy clean, the whole tesseract suite green (the `-n auto` run is the keep_alive teardown stress test).
Then:
```bash
git add src/compas_fab/backends/tesseract/backend_features/plan_cartesian_motion.py tests/backends/tesseract/test_plan_cartesian_motion_rep.py
git -c user.name="Jelle Feringa" -c user.email="jelleferinga@gmail.com" \
  commit -m "feat: coordinated REP Cartesian proof — TCP tracks workpiece to 1mm"
```

---

## Self-Review

**Spec coverage** (against `2026-07-24-w4-e2-rep-topology-gh-assembly-design.md` + the resolution doc):
- Deliverable #5 (REP coupled-group load) → Task 3 (joint-list group via the fixture; `structural_validation` coupled branch) + the joint-list SRDF rewrite embedded in Tasks 3/4. ✓
- Deliverable #1 (REP coordinated toolpath proof) → Task 1 (REPInvKin contract) + Task 4 (coordinated Cartesian proof). ✓
- Coupled base/tip sourced from config, topology-aware working frame → Task 2 + Task 4. ✓
- Regression guard on the non-coupled path → Task 3 Step 5 (existing `test_client.py` suite). ✓
- **Deferred to later E2 plans (not this plan):** GH cell assembly + `cell_assembly.py` + the three GH components (Plan 2); `eax` inputs on targets, the attached-workpiece #6, the E2E round-trip, the `min`/`max` sample windows + Descartes evaluators quality follow-on (Plan 3 / documented follow-on). These depend on the coupled-cell loading this plan delivers. `#7` (typed eax slot map) is deferred out of E2 entirely.

**Placeholder scan:** none — every step has the exact file, command, expected output, and full code.

**Type consistency:** `is_coupled_group` / `coupled_group_frames` signatures in Task 2 match their consumers in Tasks 3 (`set_robot_cell`) and 4 (`plan_cartesian_motion`); `validate_robot_cell_structure`'s new `coupled_groups` parameter matches the single call site; `_COUPLED_JOINTS` order (`[positioner_joint_1, positioner_joint_2, joint_1..6]`) is identical across Tasks 1/3/4 and matches the live-verified scene order.

**Fixture duplication note:** the `_joint_list_srdf` / `_rep_config` / `_rep_artifact` / `_rep_cell` / `_IRB2400_OPW` idiom is intentionally inlined in Tasks 3 and 4 (and the config/OPW in Task 1), matching the existing per-file fixture convention of `test_coupled_kinematics.py` / `test_plan_cartesian_motion_coupled.py`. Do not extract a shared module unless the project later adopts one.

## Execution Handoff

**Plan complete and saved to `docs/superpowers/plans/2026-07-25-w4-e2-rep-kinematic-core.md`. Two execution options:**

**1. Subagent-Driven (recommended)** — a fresh subagent per task, two-stage review between tasks, fast iteration (matches how E1 was executed).

**2. Inline Execution** — execute the tasks in this session with checkpoints for review.

**Which approach?**
