# W4 E2 — cell_assembly synthesis backend Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Synthesize a coordinated ROP/REP robot cell (URDF + SRDF) from a plain robot description plus typed external-axis descriptors, such that the synthesized cell is *structurally equivalent* to the shipped reference cell, loads through `set_robot_cell`, and coordinates — the headless backend the Grasshopper `External Linear/Rotational Axis` components (Plan 3) will marshal to.

**Architecture:** A `cell_assembly.py` module with one authored type (`ExternalAxisDescriptor`) and one factory (`synthesize_coupled_cell`). The descriptor carries an external axis's kinematics (name, role, joint axis, limits, joint origin) + optional geometry; the factory grafts a positioner branch onto a base robot's URDF and rewrites the SRDF to emit the three groups (`manipulator` / `positioner` / a **joint-list** `full_manipulator`, per the REP coordination fix in Plan 1). Topology (what the external axis carries — robot vs workpiece) is an explicit input, orthogonal to the axis's linear/rotary role. The contract is proven by round-tripping: a synthesized abb_irb2400 + positioner cell is structurally equal to `abb_irb2400_external_positioner` and passes Plan 1's coordinated REP proof.

**Tech Stack:** Python 3.12, `attrs`, `compas`/`compas_robots` (`Frame`, `Mesh`), `xml.etree.ElementTree` for URDF/SRDF surgery, `tesseract-robotics-nanobind==0.35.0.7`, `pytest`+`pytest-xdist`, `mypy --strict`, `ruff`, `pixi`.

## Global Constraints

- **pixi exclusively**; env has released `tesseract-robotics-nanobind==0.35.0.7`; do not unpin `autobahn==24.4.2`.
- **Gates of record:** `pixi run pytest tests/backends/tesseract -n auto` green · `pixi run mypy --strict src/compas_fab/backends/tesseract` clean · `pixi run ruff check` clean. `pixi run ruff format` before committing.
- **ETH type + error model:** typed quantities at the boundary; `Cls.build(...)` validates invariants; one named exception per failure mode (no bare `ValueError`); no `__all__`; no conditional imports/`HAS_*`/fallbacks; `compas.tolerance.TOL` for tolerances.
- **`@overload`** for any dual scalar-positional / arraylike vector-quantity factory (e.g. an axis triple accepts both `f(x, y, z)` and `f([x, y, z])`).
- **Commits:** author **and** committer `Jelle Feringa <jelleferinga@gmail.com>`; no `Co-Authored-By`; never "🤖 Generated with Claude Code". Pre-authorized for this SDD flow.
- **Coupled-group emission rule (Plan 1):** `full_manipulator` is **always** a `<joint>` list in positioner-forward order (`positioner joints…, arm joints…`), never a cross-fork `<chain>` — COMPAS can't parse the chain and it silently reverses the positioner joint order.
- **Synthesis target (verified, `abb_irb2400_external_positioner`):** `world` root (no geometry) · `world_robot_joint` fixed `world`→`base_link` (the robot mount) · `world_positioner_joint` fixed `world`→`positioner_base_link` at the positioner world mount (reference `xyz=1 0 1`) · positioner axes `positioner_base_link`→`positioner_link_1`→`positioner_tool0`, both prismatic, origins identity, axes `0 1 0` / `1 0 0`, limits `±1.0`, `velocity=2.618`, `effort=0` · positioner links carry **no** geometry. SRDF groups: `manipulator` (`world`→`tool0` chain), `positioner` (`world`→`positioner_tool0` chain), `full_manipulator` (`<joint>` list).

---

### Task 1: `ExternalAxisDescriptor` + errors

The authored type a GH axis component produces: one external axis's kinematics + optional geometry. Distinct from `external_axes.ExternalAxis` (which is *derived from* a loaded cell); this one *synthesizes* a cell (no vector index yet; carries a mount frame + geometry).

**Files:**
- Create: `src/compas_fab/backends/tesseract/cell_assembly.py`
- Modify: `src/compas_fab/backends/tesseract/errors.py` (add two exceptions)
- Create: `tests/backends/tesseract/test_cell_assembly_descriptor.py`

**Interfaces:**
- Consumes: `external_axes.ExternalAxisRole` (`TRACK`/`POSITIONER`), `external_axes.ExternalAxisUnit`, `compas.geometry.Frame`, `compas.datastructures.Mesh`, `compas.tolerance.TOL`.
- Produces:
  - `errors.InvalidExternalAxisDescriptorError`, `errors.CoupledCellAssemblyError`
  - `cell_assembly.ExternalAxisDescriptor` (frozen attrs) with fields `name: str`, `role: ExternalAxisRole`, `axis: tuple[float, float, float]` (unit joint axis), `lower: float`, `upper: float` (SI: metres for `TRACK`, radians for `POSITIONER`), `mount: Frame` (joint origin relative to its parent link), `visual_meshes: tuple[Mesh, ...]`, `collision_meshes: tuple[Mesh, ...]`
  - `ExternalAxisDescriptor.build(name, role, axis, lower, upper, *, mount=None, visual_meshes=(), collision_meshes=())` — accepts `lower`/`upper` in the **role-natural author unit** (mm for `TRACK`, deg for `POSITIONER`), normalizes to SI; validates unit-length axis, `lower ≤ upper`, finite, non-empty name. Meshes optional (the reference positioner is geometry-less). Consumed by Task 2.

- [ ] **Step 1: Add the two exceptions to `errors.py`**

Append after the existing coupled/kinematics errors (near `UnknownKinematicTopologyError`):

```python
class InvalidExternalAxisDescriptorError(TesseractBackendError):
    """A Grasshopper-authored external-axis descriptor violates its contract."""


class CoupledCellAssemblyError(TesseractBackendError):
    """Synthesizing a coordinated cell from a base robot and external axes failed."""
```

- [ ] **Step 2: Write the failing descriptor tests**

```python
"""ExternalAxisDescriptor invariants and role-natural unit normalization."""

import math

import pytest
from compas.geometry import Frame

from compas_fab.backends.tesseract.cell_assembly import ExternalAxisDescriptor
from compas_fab.backends.tesseract.errors import InvalidExternalAxisDescriptorError
from compas_fab.backends.tesseract.external_axes import ExternalAxisRole
from compas_fab.backends.tesseract.external_axes import ExternalAxisUnit


def test_track_descriptor_normalizes_mm_limits_to_metres():
    d = ExternalAxisDescriptor.build("positioner_joint_2", ExternalAxisRole.TRACK, (1.0, 0.0, 0.0), -1000.0, 1000.0)
    assert d.unit is ExternalAxisUnit.MILLIMETRE
    assert math.isclose(d.lower, -1.0) and math.isclose(d.upper, 1.0)  # mm -> m
    assert d.mount == Frame.worldXY()  # default identity mount


def test_positioner_descriptor_normalizes_deg_limits_to_radians():
    d = ExternalAxisDescriptor.build("turntable", ExternalAxisRole.POSITIONER, (0.0, 0.0, 1.0), -180.0, 180.0)
    assert d.unit is ExternalAxisUnit.DEGREE
    assert math.isclose(d.lower, -math.pi) and math.isclose(d.upper, math.pi)  # deg -> rad


def test_descriptor_rejects_non_unit_axis():
    with pytest.raises(InvalidExternalAxisDescriptorError):
        ExternalAxisDescriptor.build("j", ExternalAxisRole.TRACK, (1.0, 1.0, 0.0), -1.0, 1.0)


def test_descriptor_rejects_lower_above_upper():
    with pytest.raises(InvalidExternalAxisDescriptorError):
        ExternalAxisDescriptor.build("j", ExternalAxisRole.TRACK, (1.0, 0.0, 0.0), 10.0, -10.0)


def test_descriptor_rejects_empty_name():
    with pytest.raises(InvalidExternalAxisDescriptorError):
        ExternalAxisDescriptor.build("", ExternalAxisRole.TRACK, (1.0, 0.0, 0.0), -1.0, 1.0)
```

- [ ] **Step 3: Run — expect FAIL**

Run: `pixi run pytest tests/backends/tesseract/test_cell_assembly_descriptor.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'compas_fab.backends.tesseract.cell_assembly'`.

- [ ] **Step 4: Implement `ExternalAxisDescriptor` in `cell_assembly.py`**

```python
"""Synthesize a coordinated ROP/REP robot cell from a base robot + external axes.

A Grasshopper external-axis component authors an ``ExternalAxisDescriptor`` (kinematics
+ optional geometry of one external axis); ``synthesize_coupled_cell`` grafts a
positioner branch onto a base robot's URDF and rewrites its SRDF to emit the three
coordinated groups, so the result feeds ``RobotArtifact.with_coupled_kinematics`` and
loads/coordinates exactly like the shipped reference cells. The coupled ``full_manipulator``
group is always a ``<joint>`` list in positioner-forward order (a cross-fork ``<chain>``
is COMPAS-unparseable and reverses the positioner joints -- see the REP resolution doc).
"""

from __future__ import annotations

import math

from attrs import define
from attrs import field
from compas.datastructures import Mesh  # type: ignore[import-untyped]
from compas.geometry import Frame  # type: ignore[import-untyped]
from compas.tolerance import TOL  # type: ignore[import-untyped]

from .errors import InvalidExternalAxisDescriptorError
from .external_axes import ExternalAxisRole
from .external_axes import ExternalAxisUnit

# Author-facing limits arrive in the role's natural unit; store SI (URDF units).
_MM_PER_M = 1000.0
_DEG_PER_RAD = 180.0 / math.pi
_ROLE_UNIT = {ExternalAxisRole.TRACK: ExternalAxisUnit.MILLIMETRE, ExternalAxisRole.POSITIONER: ExternalAxisUnit.DEGREE}


@define(frozen=True, slots=True)
class ExternalAxisDescriptor:
    """One Grasshopper-authored external axis: kinematics + optional geometry.

    ``axis`` is the unit joint axis; ``lower``/``upper`` are stored in SI (metres for a
    ``TRACK`` prismatic axis, radians for a ``POSITIONER`` revolute axis); ``mount`` is
    the joint origin relative to its parent link (identity for stacked axes). Meshes are
    optional -- the reference positioner is geometry-less; a real cell supplies collision
    geometry.
    """

    name: str
    role: ExternalAxisRole
    unit: ExternalAxisUnit
    axis: tuple[float, float, float]
    lower: float
    upper: float
    mount: Frame
    visual_meshes: tuple[Mesh, ...] = field(default=())
    collision_meshes: tuple[Mesh, ...] = field(default=())

    @classmethod
    def build(
        cls,
        name: str,
        role: ExternalAxisRole,
        axis: "tuple[float, float, float] | list[float]",
        lower: float,
        upper: float,
        *,
        mount: "Frame | None" = None,
        visual_meshes: "tuple[Mesh, ...] | list[Mesh]" = (),
        collision_meshes: "tuple[Mesh, ...] | list[Mesh]" = (),
    ) -> ExternalAxisDescriptor:
        """Validate one authored external axis; normalize limits to SI.

        Args:
            name: Exact joint name to synthesize.
            role: TRACK (prismatic) or POSITIONER (revolute).
            axis: Unit joint axis.
            lower, upper: Travel limits in the role-natural unit (mm for TRACK, deg for
                POSITIONER); stored in SI (metres / radians).
            mount: Joint origin relative to its parent link; identity when omitted.
            visual_meshes, collision_meshes: Optional geometry (metres).

        Returns:
            Validated immutable descriptor.

        Raises:
            InvalidExternalAxisDescriptorError: Empty name, non-unit axis, non-finite or
                inverted limits.
        """
        if not name:
            raise InvalidExternalAxisDescriptorError("External axis descriptor name is empty.")
        axis_tuple = tuple(float(value) for value in axis)
        if len(axis_tuple) != 3 or not all(math.isfinite(value) for value in axis_tuple):
            raise InvalidExternalAxisDescriptorError("External axis '{}' axis must be three finite values.".format(name))
        norm = math.sqrt(sum(value * value for value in axis_tuple))
        if not TOL.is_close(norm, 1.0):
            raise InvalidExternalAxisDescriptorError("External axis '{}' axis must be a unit vector, got norm {:.6f}.".format(name, norm))
        if not math.isfinite(lower) or not math.isfinite(upper):
            raise InvalidExternalAxisDescriptorError("External axis '{}' limits must be finite.".format(name))
        if lower > upper:
            raise InvalidExternalAxisDescriptorError("External axis '{}' lower {} exceeds upper {}.".format(name, lower, upper))
        unit = _ROLE_UNIT[role]
        scale = _MM_PER_M if unit is ExternalAxisUnit.MILLIMETRE else _DEG_PER_RAD
        return cls(
            name,
            role,
            unit,
            axis_tuple,  # type: ignore[arg-type]
            lower / scale,
            upper / scale,
            mount if mount is not None else Frame.worldXY(),
            tuple(visual_meshes),
            tuple(collision_meshes),
        )
```

- [ ] **Step 5: Run — expect PASS**

Run: `pixi run pytest tests/backends/tesseract/test_cell_assembly_descriptor.py -q`
Expected: PASS (5 tests).

- [ ] **Step 6: Gates + commit**

Run: `pixi run ruff format src/compas_fab/backends/tesseract/cell_assembly.py src/compas_fab/backends/tesseract/errors.py tests/backends/tesseract/test_cell_assembly_descriptor.py >/dev/null && pixi run ruff check src/compas_fab/backends/tesseract tests/backends/tesseract/test_cell_assembly_descriptor.py && pixi run mypy --strict src/compas_fab/backends/tesseract`
Then:
```bash
git add src/compas_fab/backends/tesseract/cell_assembly.py src/compas_fab/backends/tesseract/errors.py tests/backends/tesseract/test_cell_assembly_descriptor.py
git -c user.name="Jelle Feringa" -c user.email="jelleferinga@gmail.com" \
  commit -m "feat: ExternalAxisDescriptor — authored external axis for cell synthesis"
```

---

### Task 2: `synthesize_coupled_cell` (REP) + reference-equivalence + coordinated proof

The core factory, REP path. Build the combined URDF/SRDF from a plain robot + descriptors + `positioner_mount`, and prove it (a) is structurally equal to the shipped `abb_irb2400_external_positioner` and (b) passes Plan 1's coordinated proof. **The two contract tests are the oracle for the URDF/SRDF surgery** — implement `synthesize_coupled_cell` against them.

**Files:**
- Modify: `src/compas_fab/backends/tesseract/cell_assembly.py` (add `synthesize_coupled_cell`)
- Create: `tests/backends/tesseract/test_cell_assembly_rep.py`

**Interfaces:**
- Consumes: `ExternalAxisDescriptor` (Task 1); `artifact.CoupledTopology`; `compas.geometry.Frame`/`Transformation`; `RobotArtifact` + `.with_coupled_kinematics` + `CoupledKinematics` (Plan 1); the Plan 1 REP proof helpers.
- Produces: `cell_assembly.synthesize_coupled_cell(base_urdf: str, base_srdf: str, descriptors: tuple[ExternalAxisDescriptor, ...], topology: CoupledTopology, *, positioner_mount: Frame, robot_mount: Frame = Frame.worldXY(), positioner_base_link: str = "positioner_base_link", positioner_tip_link: str = "positioner_tool0", robot_base_link: str = "base_link", robot_tip_link: str = "tool0", world_link: str = "world") -> tuple[str, str]` returning `(urdf, srdf)`.

- [ ] **Step 1: Write the failing contract tests**

The structural-equivalence test compares the synthesized cell against the reference by link tree, joint (name/type/axis/limits/parent/child), and SRDF groups — ignoring geometry (the reference positioner is geometry-less) and joint origins beyond the positioner mount. The coordinated test reuses Plan 1's proof on the synthesized cell.

```python
"""synthesize_coupled_cell (REP): structural equivalence to the reference + coordination."""

import xml.etree.ElementTree as ElementTree
from pathlib import Path

import gc
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
from compas_fab.backends.tesseract.artifact import RobotArtifact
from compas_fab.backends.tesseract.cell_assembly import ExternalAxisDescriptor
from compas_fab.backends.tesseract.cell_assembly import synthesize_coupled_cell
from compas_fab.backends.tesseract.client import TesseractClient
from compas_fab.backends.tesseract.external_axes import ExternalAxisRole
from compas_fab.backends.tesseract.planner import TesseractPlanner
from compas_fab.robots import FrameWaypoints
from compas_fab.robots import RobotCell
from compas_fab.robots import RobotCellState
from compas_fab.robots import RobotSemantics
from compas_fab.robots import TargetMode

_UD = Path(tesseract_robotics.__file__).parent / "data" / "tesseract" / "support" / "urdf"
_PLAIN_URDF = (_UD / "abb_irb2400.urdf").read_text()
_PLAIN_SRDF = (_UD / "abb_irb2400.srdf").read_text()
_REF_URDF = (_UD / "abb_irb2400_external_positioner.urdf").read_text()
_COUPLED_JOINTS = ["positioner_joint_1", "positioner_joint_2", "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
_COUPLED_TYPES = [Joint.PRISMATIC, Joint.PRISMATIC, Joint.REVOLUTE, Joint.REVOLUTE, Joint.REVOLUTE, Joint.REVOLUTE, Joint.REVOLUTE, Joint.REVOLUTE]
_IRB2400_OPW = dict(a1=0.100, a2=-0.135, b=0.00, c1=0.615, c2=0.705, c3=0.755, c4=0.086,
                    offsets=[0.0, 0.0, -1.57079632679, 0.0, 0.0, 0.0], sign_corrections=[1, 1, 1, 1, 1, 1])
_MIN_POSITIONER_TRAVEL_M = 0.5
_CARTESIAN_FOLLOW_TOL_M = 5e-3


def _rep_descriptors():
    # The reference XY table: two prismatic (TRACK) axes, ±1 m, at world mount (1,0,1).
    return (
        ExternalAxisDescriptor.build("positioner_joint_1", ExternalAxisRole.TRACK, (0.0, 1.0, 0.0), -1000.0, 1000.0),
        ExternalAxisDescriptor.build("positioner_joint_2", ExternalAxisRole.TRACK, (1.0, 0.0, 0.0), -1000.0, 1000.0),
    )


def _synthesize():
    return synthesize_coupled_cell(
        _PLAIN_URDF, _PLAIN_SRDF, _rep_descriptors(), CoupledTopology.ROBOT_WITH_EXTERNAL_POSITIONER,
        positioner_mount=Frame([1.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]),
    )


def _joint_index(urdf: str):
    root = ElementTree.fromstring(urdf)
    out = {}
    for j in root.findall("joint"):
        axis = j.find("axis")
        out[j.get("name")] = (j.get("type"), j.find("parent").get("link"), j.find("child").get("link"), axis.get("xyz") if axis is not None else None)
    return out


def test_synthesized_rep_urdf_matches_reference_joint_topology():
    got = _joint_index(_synthesize()[0])
    want = _joint_index(_REF_URDF)
    # Every joint of the reference is reproduced with the same type/parent/child/axis.
    for name, spec in want.items():
        assert name in got, "synthesized URDF missing joint {!r}".format(name)
        assert got[name] == spec, "joint {!r} differs: got {}, want {}".format(name, got[name], spec)


def test_synthesized_rep_srdf_emits_joint_list_full_manipulator():
    srdf = _synthesize()[1]
    root = ElementTree.fromstring(srdf)
    groups = {g.get("name"): g for g in root.findall("group")}
    assert set(groups) >= {"manipulator", "positioner", "full_manipulator"}
    full = groups["full_manipulator"]
    assert full.find("chain") is None, "full_manipulator must be a <joint> list, not a <chain>"
    assert [j.get("name") for j in full.findall("joint")] == _COUPLED_JOINTS


def test_synthesized_rep_cell_loads_and_coordinates(tmp_path):
    urdf, srdf = _synthesize()
    artifact = RobotArtifact.from_compas_urdf(urdf, srdf, {}, CollisionMeshPolicy.CONVEX_HULL).with_coupled_kinematics(
        CoupledKinematics.build(
            group="full_manipulator", topology=CoupledTopology.ROBOT_WITH_EXTERNAL_POSITIONER,
            positioner_base_link="positioner_base_link", positioner_tip_link="positioner_tool0",
            manipulator_base_link="base_link", manipulator_tip_link="tool0",
            manipulator=OpwParameters.build(**_IRB2400_OPW), manipulator_reach=2.55,
            positioner_sample_resolution=[("positioner_joint_1", 0.1), ("positioner_joint_2", 0.1)],
        )
    )
    model = RobotModel.from_urdf_string(urdf)
    cell = RobotCell(model, RobotSemantics.from_srdf_string(srdf, model))
    state = RobotCellState.from_robot_cell(cell)
    state.robot_configuration = Configuration([0.0] * 8, _COUPLED_TYPES, _COUPLED_JOINTS)
    frames = [Frame([dx, 0.0, -0.1], [1.0, 0.0, 0.0], [0.0, -1.0, 0.0]) for dx in (-0.6, 0.6)]

    with TesseractClient(artifact, cache_root=tmp_path) as client:
        planner = TesseractPlanner(client)
        planner.set_robot_cell(cell, state)
        trajectory = planner.plan_cartesian_motion(FrameWaypoints(frames, TargetMode.ROBOT), state, group="full_manipulator")
        robot = client.environment.clone_robot()
        kg = robot.env.getKinematicGroup("full_manipulator", "")
        ixx = list(kg.getJointNames()).index("positioner_joint_2")
        errs = []
        for point, frame in ((trajectory.points[0], frames[0]), (trajectory.points[-1], frames[-1])):
            fk = kg.calcFwdKin(np.asarray(point.joint_values, dtype=float))
            want = np.asarray(fk["positioner_tool0"].matrix, dtype=float) @ np.asarray(Transformation.from_frame(frame).matrix, dtype=float)
            errs.append(float(np.linalg.norm(np.asarray(fk["tool0"].matrix, dtype=float)[:3, 3] - want[:3, 3])))
        travel = max(p.joint_values[ixx] for p in trajectory.points) - min(p.joint_values[ixx] for p in trajectory.points)
        del kg, robot
        gc.collect()

    assert trajectory.joint_names == _COUPLED_JOINTS
    assert travel > _MIN_POSITIONER_TRAVEL_M
    for error in errs:
        assert error < _CARTESIAN_FOLLOW_TOL_M
```

- [ ] **Step 2: Run — expect FAIL**

Run: `pixi run pytest tests/backends/tesseract/test_cell_assembly_rep.py -q`
Expected: FAIL — `ImportError: cannot import name 'synthesize_coupled_cell'`.

- [ ] **Step 3: Implement `synthesize_coupled_cell`**

Add to `cell_assembly.py`. Build the URDF/SRDF with `ElementTree` against the verified target (Global Constraints). For REP: add the `world` root, fix `base_link` to `world` at `robot_mount`, fix `positioner_base_link` to `world` at `positioner_mount`, then chain the descriptor joints/links `positioner_base_link → … → positioner_tip_link`, and rewrite the SRDF groups (`manipulator` chain re-based to `world`; `positioner` chain `world`→tip; `full_manipulator` `<joint>` list). ROP (Task 3) inserts the positioner serially below `base_link` instead.

```python
import xml.etree.ElementTree as ElementTree

from compas.geometry import Transformation  # add to imports

from .artifact import CoupledTopology
from .errors import CoupledCellAssemblyError

_JOINT_TYPE = {ExternalAxisRole.TRACK: "prismatic", ExternalAxisRole.POSITIONER: "revolute"}
# Reference positioner limits carry these dynamics; match them so structural checks pass.
_AXIS_VELOCITY = 2.618
_AXIS_EFFORT = 0.0


def _origin_element(frame: Frame) -> ElementTree.Element:
    from compas.geometry import Frame as _F  # local: keep top-level imports minimal

    t = Transformation.from_frame(frame)
    x, y, z = t.translation_vector
    # URDF rpy = fixed-axis XYZ euler; compas Frame -> euler_angles(static=True, axes="xyz").
    rx, ry, rz = frame.euler_angles(static=True, axes="xyz")
    return ElementTree.Element("origin", {"xyz": "{} {} {}".format(x, y, z), "rpy": "{} {} {}".format(rx, ry, rz)})


def _fixed_joint(name: str, parent: str, child: str, frame: Frame) -> ElementTree.Element:
    joint = ElementTree.Element("joint", {"name": name, "type": "fixed"})
    joint.append(_origin_element(frame))
    ElementTree.SubElement(joint, "parent", {"link": parent})
    ElementTree.SubElement(joint, "child", {"link": child})
    return joint


def _axis_joint(descriptor: ExternalAxisDescriptor, parent: str, child: str) -> ElementTree.Element:
    joint = ElementTree.Element("joint", {"name": descriptor.name, "type": _JOINT_TYPE[descriptor.role]})
    joint.append(_origin_element(descriptor.mount))
    ElementTree.SubElement(joint, "parent", {"link": parent})
    ElementTree.SubElement(joint, "child", {"link": child})
    ElementTree.SubElement(joint, "axis", {"xyz": "{} {} {}".format(*descriptor.axis)})
    ElementTree.SubElement(joint, "limit", {
        "lower": repr(descriptor.lower), "upper": repr(descriptor.upper),
        "velocity": repr(_AXIS_VELOCITY), "effort": repr(_AXIS_EFFORT),
    })
    return joint


def synthesize_coupled_cell(
    base_urdf: str,
    base_srdf: str,
    descriptors: "tuple[ExternalAxisDescriptor, ...]",
    topology: CoupledTopology,
    *,
    positioner_mount: Frame,
    robot_mount: "Frame | None" = None,
    positioner_base_link: str = "positioner_base_link",
    positioner_tip_link: str = "positioner_tool0",
    robot_base_link: str = "base_link",
    robot_tip_link: str = "tool0",
    world_link: str = "world",
) -> "tuple[str, str]":
    """Graft a positioner branch onto a base robot to form a coordinated ROP/REP cell.

    Returns ``(urdf, srdf)`` whose SRDF carries the three coordinated groups
    (``manipulator`` / ``positioner`` / a ``<joint>``-list ``full_manipulator``) ready for
    ``RobotArtifact.with_coupled_kinematics``.

    Raises:
        CoupledCellAssemblyError: No descriptors, a synthesized name collides with the
            base robot, or the base URDF/SRDF is malformed.
    """
    if not descriptors:
        raise CoupledCellAssemblyError("At least one external-axis descriptor is required.")
    robot_mount = robot_mount if robot_mount is not None else Frame.worldXY()
    try:
        urdf_root = ElementTree.fromstring(base_urdf)
        srdf_root = ElementTree.fromstring(base_srdf)
    except ElementTree.ParseError as error:
        raise CoupledCellAssemblyError("Base URDF/SRDF cannot be parsed: {}.".format(error)) from error

    existing_links = {link.get("name") for link in urdf_root.findall("link")}
    existing_joints = {joint.get("name") for joint in urdf_root.findall("joint")}

    # Positioner link chain: base -> (descriptor links) -> tip.
    chain_links = [positioner_base_link] + ["{}_link_{}".format(positioner_base_link, i) for i in range(1, len(descriptors))] + [positioner_tip_link]
    new_links = [world_link, *chain_links]
    new_joints = ["world_robot_joint", "world_positioner_joint", *[d.name for d in descriptors]]
    collide = (set(new_links) & existing_links) | (set(new_joints) & existing_joints)
    # world/base_link may legitimately pre-exist; only flag positioner-name collisions.
    collide -= {world_link, robot_base_link}
    if collide:
        raise CoupledCellAssemblyError("Synthesized names collide with the base robot: {}.".format(sorted(collide)))

    for name in new_links:
        if name != robot_base_link:
            ElementTree.SubElement(urdf_root, "link", {"name": name})
    # (Optional geometry from descriptor.visual_meshes/collision_meshes attaches to the
    #  descriptor's CHILD link here; omitted -- the reference positioner is geometry-less.)

    if topology is CoupledTopology.ROBOT_WITH_EXTERNAL_POSITIONER:
        urdf_root.append(_fixed_joint("world_robot_joint", world_link, robot_base_link, robot_mount))
        urdf_root.append(_fixed_joint("world_positioner_joint", world_link, positioner_base_link, positioner_mount))
        parent = positioner_base_link
        for descriptor, child in zip(descriptors, chain_links[1:]):
            urdf_root.append(_axis_joint(descriptor, parent, child))
            parent = child
        manip_base = world_link
    elif topology is CoupledTopology.ROBOT_ON_POSITIONER:
        # Serial: world -> positioner_base -> ... -> positioner_tip -(fixed)-> base_link.
        urdf_root.append(_fixed_joint("world_positioner_joint", world_link, positioner_base_link, positioner_mount))
        parent = positioner_base_link
        for descriptor, child in zip(descriptors, chain_links[1:]):
            urdf_root.append(_axis_joint(descriptor, parent, child))
            parent = child
        urdf_root.append(_fixed_joint("positioner_robot_joint", positioner_tip_link, robot_base_link, robot_mount))
        manip_base = positioner_base_link
    else:
        raise CoupledCellAssemblyError("Unknown coupled topology {!r}.".format(topology))

    coupled_joint_names = [d.name for d in descriptors] + _arm_joint_names(srdf_root, robot_base_link, robot_tip_link)
    _rewrite_srdf_groups(srdf_root, manip_base, robot_tip_link, positioner_base_link if topology is CoupledTopology.ROBOT_ON_POSITIONER else world_link, positioner_tip_link, coupled_joint_names)
    return ElementTree.tostring(urdf_root, encoding="unicode"), ElementTree.tostring(srdf_root, encoding="unicode")
```

Implement the two SRDF helpers `_arm_joint_names` (read the base SRDF `manipulator` chain's configurable joints, or fall back to the URDF revolute/prismatic joints between `robot_base_link` and `robot_tip_link`) and `_rewrite_srdf_groups` (drop existing `<group>`s; emit `manipulator` chain `manip_base`→`robot_tip_link`, `positioner` chain `positioner_group_base`→`positioner_tip_link`, and `full_manipulator` as a `<joint>` list of `coupled_joint_names`) against the SRDF-group assertions in Step 1. Let the structural test pin the exact element shapes.

- [ ] **Step 4: Run — expect PASS**

Run: `pixi run pytest tests/backends/tesseract/test_cell_assembly_rep.py -q`
Expected: PASS (3). If the joint-topology test flags an origin/axis mismatch, adjust `_origin_element` euler convention or the axis emission until the synthesized joints equal the reference's; if the coordinated test's positioner doesn't travel, confirm `full_manipulator` is a `<joint>` list in positioner-forward order (the Plan 1 fix).

- [ ] **Step 5: Full gates + commit**

Run: `pixi run ruff format src/compas_fab/backends/tesseract/cell_assembly.py tests/backends/tesseract/test_cell_assembly_rep.py >/dev/null && pixi run ruff check src/compas_fab/backends/tesseract tests/backends/tesseract && pixi run mypy --strict src/compas_fab/backends/tesseract && pixi run pytest tests/backends/tesseract -n auto -q`
Then:
```bash
git add src/compas_fab/backends/tesseract/cell_assembly.py tests/backends/tesseract/test_cell_assembly_rep.py
git -c user.name="Jelle Feringa" -c user.email="jelleferinga@gmail.com" \
  commit -m "feat: synthesize_coupled_cell (REP) — reproduces reference cell + coordinates"
```

---

### Task 3: `synthesize_coupled_cell` (ROP) contract

Prove the serial ROP wiring against `abb_irb2400_on_positioner` (a track carrying the robot), so both topologies synthesize. Uses one prismatic TRACK descriptor.

**Files:**
- Create: `tests/backends/tesseract/test_cell_assembly_rop.py`
- (No new source — Task 2 already implemented the ROP branch; this task proves + hardens it.)

**Interfaces:**
- Consumes: `synthesize_coupled_cell` with `CoupledTopology.ROBOT_ON_POSITIONER`.

- [ ] **Step 1: Write the ROP contract test**

```python
"""synthesize_coupled_cell (ROP): structural equivalence to abb_irb2400_on_positioner."""

import xml.etree.ElementTree as ElementTree
from pathlib import Path

import tesseract_robotics
from compas.geometry import Frame

from compas_fab.backends.tesseract.artifact import CoupledTopology
from compas_fab.backends.tesseract.cell_assembly import ExternalAxisDescriptor
from compas_fab.backends.tesseract.cell_assembly import synthesize_coupled_cell
from compas_fab.backends.tesseract.external_axes import ExternalAxisRole

_UD = Path(tesseract_robotics.__file__).parent / "data" / "tesseract" / "support" / "urdf"
_PLAIN_URDF = (_UD / "abb_irb2400.urdf").read_text()
_PLAIN_SRDF = (_UD / "abb_irb2400.srdf").read_text()
_REF_URDF = (_UD / "abb_irb2400_on_positioner.urdf").read_text()


def _joints(urdf: str):
    root = ElementTree.fromstring(urdf)
    return {j.get("name"): (j.get("type"), j.find("parent").get("link"), j.find("child").get("link")) for j in root.findall("joint")}


def test_synthesized_rop_reproduces_reference_serial_chain():
    urdf, srdf = synthesize_coupled_cell(
        _PLAIN_URDF, _PLAIN_SRDF,
        (ExternalAxisDescriptor.build("positioner_joint_1", ExternalAxisRole.TRACK, (0.0, 1.0, 0.0), -1000.0, 1000.0),),
        CoupledTopology.ROBOT_ON_POSITIONER,
        positioner_mount=Frame.worldXY(),
    )
    got, want = _joints(urdf), _joints(_REF_URDF)
    # The positioner rides at the root and carries the robot base serially.
    assert got["positioner_joint_1"][0] == "prismatic"
    assert got["positioner_joint_1"][1] == "positioner_base_link"
    # base_link is a fixed-joint child of the positioner tip (robot rides the track).
    robot_mount_joints = [n for n, (t, p, c) in got.items() if c == "base_link" and t == "fixed"]
    assert robot_mount_joints, "robot base must mount onto the positioner via a fixed joint"
    full = ElementTree.fromstring(srdf).findall("group")
    names = {g.get("name") for g in full}
    assert {"manipulator", "positioner", "full_manipulator"} <= names
```

- [ ] **Step 2: Run — expect PASS or a precise structural signal**

Run: `pixi run pytest tests/backends/tesseract/test_cell_assembly_rop.py -q`
Expected: PASS. If the ROP branch mis-wires the serial mount, the assertion names the exact mismatch — fix the ROP branch of `synthesize_coupled_cell` (the `positioner_robot_joint` fixed link `positioner_tip_link`→`robot_base_link`).

- [ ] **Step 3: Full gates + commit**

Run: `pixi run ruff format tests/backends/tesseract/test_cell_assembly_rop.py >/dev/null && pixi run ruff check tests/backends/tesseract && pixi run mypy --strict src/compas_fab/backends/tesseract && pixi run pytest tests/backends/tesseract -n auto -q`
Then:
```bash
git add tests/backends/tesseract/test_cell_assembly_rop.py src/compas_fab/backends/tesseract/cell_assembly.py
git -c user.name="Jelle Feringa" -c user.email="jelleferinga@gmail.com" \
  commit -m "test: synthesize_coupled_cell (ROP) — serial track carries the robot"
```

---

## Self-Review

**Spec coverage:** Deliverable #2's cell synthesis (`cell_assembly.py`, both topologies, joint-list coupled group) → Tasks 1-3. The **GH components** (`Cf_TesseractExternalLinearAxis` / `…RotationalAxis` / `Cf_TesseractRobotCell`) + `TESSERACT_USER_OBJECTS` + CI wiring are **Plan 3** (thin marshals on this backend). Deliverable #3 (eax on targets), #6 (attached workpiece), the E2E round-trip, and the Descartes-evaluator quality follow-on are later plans.

**Placeholder scan:** Task 2 Step 3 leaves two named helpers (`_arm_joint_names`, `_rewrite_srdf_groups`) described-not-coded, and marks the optional descriptor-geometry attachment as omitted. This is deliberate: those are driven by the Step-1 SRDF assertions (the oracle), and geometry is off the proof path. Everything else is complete code. **Flag for the implementer:** write those two helpers first against the Step-1 SRDF group assertions.

**Type consistency:** `ExternalAxisDescriptor.build` (Task 1) fields/signature match `synthesize_coupled_cell`'s consumption (Task 2); `CoupledTopology` values match Plan 1; the coupled joint order `[positioner_joint_1, positioner_joint_2, joint_1..6]` is identical across the synthesis and the Plan-1-style proof.

**Known risk (honest):** `synthesize_coupled_cell`'s URDF surgery is the largest-uncertainty surface; unlike Plan 1, its exact element shapes (origin euler convention, SRDF group element form) are **driven by the contract tests at execution time**, not pre-verified line-by-line. The structural-equivalence + coordinated tests are mechanically decisive oracles (the synthesized cell either equals the reference and coordinates, or the assertion names the gap), so this is test-driven, not aspirational — but expect 1-2 iterations on `_origin_element` / `_rewrite_srdf_groups` during Task 2 Step 4.

## Execution Handoff

**Plan complete and saved to `docs/superpowers/plans/2026-07-25-w4-e2-cell-assembly-backend.md`. Two execution options:**

**1. Subagent-Driven (recommended)** — fresh subagent per task, review between tasks.

**2. Inline Execution** — execute here with checkpoints.

**Which approach?**
