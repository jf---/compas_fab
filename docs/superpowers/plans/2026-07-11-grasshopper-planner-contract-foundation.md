# Grasshopper Planner Contract Foundation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Track steps with checkboxes.

**Goal:** Add canonical component identities, a stale-output reducer, exact static planner contracts/options, and one connection-aware Configuration Target policy port without replacing working paths.

**Architecture:** Focused sidecars own identity, output state, operations, capabilities, options, and tolerance policy. Five small planner classes publish immutable declarations; legacy clients and planner calls remain untouched. The existing Configuration Target takes its old branch when policy is unwired and a new explicit branch only when policy is connected.

**Tech Stack:** Python 3.9/3.12, attrs, SHA-256, Grasshopper CPython, pytest-xdist/testmon, strict mypy, Ruff, Pixi, MkDocs.

## Global Constraints

- Preserve planner calls, client lifetimes, component directory/name, and the first three Configuration Target ports in order. Add paths; delete no working code or tests.
- Identity hashes only explicit length-prefixed UTF-8/bytes/scalars. Never use reflection, `repr`, object address, path, time, or a caller-supplied digest.
- Raw attrs constructors validate every invariant. Every failure uses a named exception.
- Compute is one `False -> True` edge. Identity change while held high clears output without dispatch. Superseded publication fails.
- Planner support comes only from `PlannerContract`; no backend-name branch, probing, optional import, fallback, broad catch, skip, or xfail.
- Opaque native profiles remain exact and force `UNVERIFIABLE`/non-cacheable resolution. MoveIt rejects unknown or noncanonical scalar option keys/values, but accepts exact `path_constraints` and `trajectory_constraints` as opaque `UNVERIFIABLE`/non-cacheable fields.
- Unwired GH empty lists mean absent. Connected empty tolerance lists are invalid because they cannot match a non-empty joint vector.
- Python blocks must pass Ruff and use Python 3.9 syntax. No semicolon, compound one-line statement, magic timing value, or new `__all__`.
- Every pytest command uses Pixi and `-n auto`; final affected verification uses `--testmon -n auto`.
- Implement with `apply_patch`. Commit every task as `Jelle Feringa <jelleferinga@gmail.com>`; do not push.

## Deliberate Boundary

This plan does not claim `PlannerInputIdentity`. Real scene revision, client instance, and reconnect generation belong to `2026-07-11-grasshopper-plan-motion-correctness.md`. Cartesian, IK, and component manifest/`BuildIdentity` belong to `2026-07-11-grasshopper-cartesian-planning.md`, `2026-07-11-grasshopper-inverse-kinematics.md`, and `2026-07-11-grasshopper-component-manifest.md`. The existing exact native Tesseract path remains the ceiling.

---

### Task 1: Canonical Identity and Current-or-Absent State

**Files:**
- Create: `src/compas_fab/ghpython/component_identity.py`
- Create: `src/compas_fab/ghpython/current_output.py`
- Create: `tests/ghpython/test_component_state.py`

**Interfaces:**
- `ComponentInstanceId.build`, `CanonicalField.text/bytes`, `ComponentInputIdentity.build`.
- `CurrentOutputState[T].build/observe/publish/fail/current/clear` and `ComputeDecision`.

- [ ] **Step 1: Write the complete RED test**

```python
# tests/ghpython/test_component_state.py
from attrs import evolve
import pytest

from compas_fab.ghpython.component_identity import CanonicalField, ComponentInputIdentity, ComponentInstanceId, InvalidComponentIdentityError
from compas_fab.ghpython.current_output import ComputeDecision, CurrentOutputState, SupersededComponentOutputError


def identity(value: str) -> ComponentInputIdentity:
    return ComponentInputIdentity.build(
        ComponentInstanceId.build("node"),
        "test/v1",
        (CanonicalField.text("value", value),),
    )


def test_identity_is_ordered_length_prefixed_and_raw_safe() -> None:
    first = (CanonicalField.text("x", "a|b"), CanonicalField.bytes("y", b"c"))
    second = (CanonicalField.text("x", "a"), CanonicalField.bytes("y", b"b|c"))
    left = ComponentInputIdentity.build(ComponentInstanceId.build("node"), "s/v1", first)
    right = ComponentInputIdentity.build(ComponentInstanceId.build("node"), "s/v1", second)
    assert left.digest != right.digest
    with pytest.raises(InvalidComponentIdentityError):
        evolve(left, digest="0" * 64)


def test_rising_edge_change_failure_and_late_publication() -> None:
    state = CurrentOutputState[str].build()
    first = identity("first")
    second = identity("second")
    assert state.observe(first, False) is ComputeDecision.IDLE
    assert state.observe(first, True) is ComputeDecision.EXECUTE
    state.publish(first, "current")
    assert state.observe(first, True) is ComputeDecision.CURRENT
    assert state.observe(second, True) is ComputeDecision.CLEARED
    with pytest.raises(SupersededComponentOutputError):
        state.publish(first, "late")
    assert state.current(second) is None
    state.fail(second)
```

- [ ] **Step 2: Verify RED**

Run: `pixi run pytest tests/ghpython/test_component_state.py -n auto -q`. Expected: collection fails because both production modules are absent.

- [ ] **Step 3: Add the complete production files**

```python
# src/compas_fab/ghpython/component_identity.py
from __future__ import annotations

from hashlib import sha256
from typing import Sequence
from typing import Tuple

from attrs import define


class InvalidComponentIdentityError(ValueError):
    pass


def _part(value: bytes) -> bytes:
    return len(value).to_bytes(8, "big") + value


@define(frozen=True, slots=True)
class ComponentInstanceId:
    value: str

    @classmethod
    def build(cls, value: str) -> "ComponentInstanceId":
        return cls(value)

    def __attrs_post_init__(self) -> None:
        if type(self.value) is not str or not self.value:
            raise InvalidComponentIdentityError("Component instance ID must be non-empty str.")


@define(frozen=True, slots=True)
class CanonicalField:
    name: str
    payload: bytes

    @classmethod
    def text(cls, name: str, value: str) -> "CanonicalField":
        if type(value) is not str:
            raise InvalidComponentIdentityError("Text field value must be str.")
        return cls(name, value.encode("utf-8"))

    @classmethod
    def bytes(cls, name: str, value: bytes) -> "CanonicalField":
        return cls(name, bytes(value))

    def __attrs_post_init__(self) -> None:
        if type(self.name) is not str or not self.name or type(self.payload) is not bytes:
            raise InvalidComponentIdentityError("Canonical field requires a name and exact bytes.")


def _digest(component: ComponentInstanceId, schema: str, fields: Tuple[CanonicalField, ...]) -> str:
    payload = _part(component.value.encode("utf-8")) + _part(schema.encode("utf-8"))
    for field in fields:
        payload += _part(field.name.encode("utf-8")) + _part(field.payload)
    return sha256(payload).hexdigest()


@define(frozen=True, slots=True)
class ComponentInputIdentity:
    component: ComponentInstanceId
    schema: str
    fields: Tuple[CanonicalField, ...]
    digest: str

    @classmethod
    def build(cls, component: ComponentInstanceId, schema: str, fields: Sequence[CanonicalField]) -> "ComponentInputIdentity":
        retained = tuple(fields)
        return cls(component, schema, retained, _digest(component, schema, retained))

    def __attrs_post_init__(self) -> None:
        invalid = not isinstance(self.component, ComponentInstanceId) or type(self.schema) is not str or not self.schema
        invalid = invalid or any(not isinstance(field, CanonicalField) for field in self.fields)
        invalid = invalid or self.digest != _digest(self.component, self.schema, self.fields)
        if invalid:
            raise InvalidComponentIdentityError("Component input identity is inconsistent.")
```

```python
# src/compas_fab/ghpython/current_output.py
from __future__ import annotations

from enum import Enum
from typing import Generic
from typing import Optional
from typing import TypeVar

from compas_fab.ghpython.component_identity import ComponentInputIdentity

T = TypeVar("T")


class SupersededComponentOutputError(RuntimeError):
    pass


class ComputeDecision(Enum):
    IDLE = "idle"
    EXECUTE = "execute"
    CURRENT = "current"
    CLEARED = "cleared"


class CurrentOutputState(Generic[T]):
    def __init__(self) -> None:
        self._identity: Optional[ComponentInputIdentity] = None
        self._value: Optional[T] = None
        self._has_value = False
        self._compute_high = False

    @classmethod
    def build(cls) -> "CurrentOutputState[T]":
        return cls()

    def observe(self, identity: ComponentInputIdentity, compute: bool) -> ComputeDecision:
        if type(compute) is not bool:
            raise TypeError("compute must be bool.")
        if self._identity != identity:
            changed = self._identity is not None
            self._identity = identity
            self._value = None
            self._has_value = False
            self._compute_high = compute
            return ComputeDecision.CLEARED if changed else (ComputeDecision.EXECUTE if compute else ComputeDecision.IDLE)
        rising = compute and not self._compute_high
        self._compute_high = compute
        if rising:
            return ComputeDecision.EXECUTE
        return ComputeDecision.CURRENT if self._has_value else ComputeDecision.IDLE

    def publish(self, identity: ComponentInputIdentity, value: T) -> None:
        if identity != self._identity:
            raise SupersededComponentOutputError("Cannot publish superseded output.")
        self._value = value
        self._has_value = True

    def fail(self, identity: ComponentInputIdentity) -> None:
        if identity != self._identity:
            raise SupersededComponentOutputError("Cannot fail superseded output.")
        self._value = None
        self._has_value = False

    def current(self, identity: ComponentInputIdentity) -> Optional[T]:
        if identity == self._identity and self._has_value:
            return self._value
        return None

    def clear(self) -> None:
        self.__init__()
```

- [ ] **Step 4: Verify, type, lint, and commit**

Run: `pixi run pytest tests/ghpython/test_component_state.py -n auto -q`. Expected: `2 passed`.

Run: `pixi run mypy --strict src/compas_fab/ghpython/component_identity.py src/compas_fab/ghpython/current_output.py`. Expected: zero errors.

Run: `pixi run ruff check src/compas_fab/ghpython/component_identity.py src/compas_fab/ghpython/current_output.py tests/ghpython/test_component_state.py`. Expected: `All checks passed!`.

```bash
git add src/compas_fab/ghpython/component_identity.py src/compas_fab/ghpython/current_output.py tests/ghpython/test_component_state.py
GIT_AUTHOR_NAME='Jelle Feringa' GIT_AUTHOR_EMAIL='jelleferinga@gmail.com' GIT_COMMITTER_NAME='Jelle Feringa' GIT_COMMITTER_EMAIL='jelleferinga@gmail.com' git commit -m 'feat: add current output contract'
```

### Task 2: Exact Static Planner Contract and Options

**Files:**
- Create: `src/compas_fab/backends/interfaces/planner_operation.py`
- Create: `src/compas_fab/backends/interfaces/planner_errors.py`
- Create: `src/compas_fab/backends/interfaces/planner_capabilities.py`
- Create: `src/compas_fab/backends/interfaces/planner_options.py`
- Create: `src/compas_fab/backends/interfaces/planner_contract.py`
- Create: `src/compas_fab/backends/kinematics/options.py`
- Create: `src/compas_fab/backends/pybullet/options.py`
- Create: `src/compas_fab/backends/ros/options.py`
- Modify additively: `src/compas_fab/backends/ros/exceptions.py`
- Modify additively: `src/compas_fab/backends/tesseract/options.py`
- Modify declarations only: four existing backend `planner.py` files
- Create: `tests/backends/contracts/test_planner_capability_foundation.py`

**Interfaces:** `PlannerContract` requires `implementation_id`, `capabilities`, and `plan_motion_options`; adapters resolve `PlanMotionLegacyOptions` plus optional native values to `ResolvedPlannerOptions`.

- [ ] **Step 1: Write the complete new RED contract test**

```python
# tests/backends/contracts/test_planner_capability_foundation.py
from math import inf
from math import nan

from attrs import evolve
import pytest
from tesseract_robotics.planning.profiles import create_freespace_pipeline_profiles

from compas_fab.backends.interfaces.planner_capabilities import ConfigurationTolerancePolicy, PlannerCapabilities, PlannerImplementationId
from compas_fab.backends.interfaces.planner_errors import InvalidPlannerCapabilitiesError, InvalidPlannerOptionsError, PlannerCapabilityError
from compas_fab.backends.interfaces.planner_operation import PlannerOperation
from compas_fab.backends.interfaces.planner_options import OptionIdentityState, PlanMotionLegacyOptions, ResolvedPlannerOptions
from compas_fab.backends.kinematics.options import UnsupportedPlanMotionOptions as AnalyticalOptions
from compas_fab.backends.kinematics.planner import AnalyticalKinematicsPlanner
from compas_fab.backends.kinematics.planner import AnalyticalPyBulletPlanner
from compas_fab.backends.pybullet.options import UnsupportedPlanMotionOptions as PyBulletOptions
from compas_fab.backends.pybullet.planner import PyBulletPlanner
from compas_fab.backends.ros.exceptions import InvalidMoveItPlanMotionOptionsError
from compas_fab.backends.ros.options import MoveItPlanMotionOptions
from compas_fab.backends.ros.planner import MoveItPlanner
from compas_fab.backends.tesseract.options import TesseractPlanOptions
from compas_fab.backends.tesseract.planner import TesseractPlanner


def test_all_five_exact_declarations_and_adapters() -> None:
    expected = (
        (AnalyticalKinematicsPlanner, "compas_fab.analytical/v1", AnalyticalOptions, {PlannerOperation.INVERSE_KINEMATICS, PlannerOperation.PLAN_CARTESIAN_MOTION}, ConfigurationTolerancePolicy.LEGACY_DEFAULTS),
        (AnalyticalPyBulletPlanner, "compas_fab.analytical_pybullet/v1", AnalyticalOptions, {PlannerOperation.INVERSE_KINEMATICS, PlannerOperation.PLAN_CARTESIAN_MOTION, PlannerOperation.CHECK_COLLISION}, ConfigurationTolerancePolicy.LEGACY_DEFAULTS),
        (PyBulletPlanner, "compas_fab.pybullet/v1", PyBulletOptions, {PlannerOperation.INVERSE_KINEMATICS, PlannerOperation.PLAN_CARTESIAN_MOTION, PlannerOperation.CHECK_COLLISION}, ConfigurationTolerancePolicy.LEGACY_DEFAULTS),
        (MoveItPlanner, "compas_fab.moveit/v1", MoveItPlanMotionOptions, set(PlannerOperation), ConfigurationTolerancePolicy.LEGACY_DEFAULTS),
        (TesseractPlanner, "compas_fab.tesseract/v1", TesseractPlanOptions, {PlannerOperation.INVERSE_KINEMATICS, PlannerOperation.PLAN_MOTION, PlannerOperation.CHECK_COLLISION}, ConfigurationTolerancePolicy.PRESERVE_ABSENT),
    )
    for planner, implementation_id, adapter, operations, tolerance_policy in expected:
        assert planner.implementation_id.value == implementation_id
        assert planner.capabilities.implementation_id == planner.implementation_id
        assert planner.plan_motion_options is adapter
        assert set(planner.capabilities.operations) == operations
        assert planner.capabilities.configuration_tolerance_policy is tolerance_policy


def test_raw_capability_and_option_constructors_revalidate() -> None:
    capabilities = PlannerCapabilities.build(PlannerImplementationId.build("test/v1"), (PlannerOperation.PLAN_MOTION,), ConfigurationTolerancePolicy.LEGACY_DEFAULTS)
    with pytest.raises(InvalidPlannerCapabilitiesError):
        evolve(capabilities, configuration_tolerance_policy="legacy_defaults")
    for invalid_time in (nan, inf, -inf):
        with pytest.raises(InvalidPlannerOptionsError):
            PlanMotionLegacyOptions(None, None, invalid_time)
    verified = ResolvedPlannerOptions.verified({"planner_id": "RRTConnect"})
    with pytest.raises(InvalidPlannerOptionsError):
        evolve(verified, identity_state="verified")


@pytest.mark.parametrize("planner", (AnalyticalKinematicsPlanner, AnalyticalPyBulletPlanner, PyBulletPlanner))
def test_unsupported_free_motion_adapters_fail_named(planner) -> None:
    with pytest.raises(PlannerCapabilityError):
        planner.plan_motion_options.resolve(PlanMotionLegacyOptions.build(None, None, None), None)


def test_moveit_rejects_noncanonical_and_native_profiles_are_unverifiable() -> None:
    legacy = PlanMotionLegacyOptions.build("RRTConnect", 2, 1.0)
    assert MoveItPlanMotionOptions.resolve(legacy, None).cacheable
    with pytest.raises(InvalidMoveItPlanMotionOptionsError):
        MoveItPlanMotionOptions.resolve(PlanMotionLegacyOptions.build(None, None, None), {"planner_id": object()})
    with pytest.raises(InvalidPlannerOptionsError):
        ResolvedPlannerOptions.verified({"opaque": object()})
    profiles = create_freespace_pipeline_profiles()
    resolved = TesseractPlanOptions.resolve(
        PlanMotionLegacyOptions.build(None, None, None),
        {"profiles": profiles},
    )
    assert resolved.to_backend_options()["profiles"] is profiles
    assert resolved.identity_state is OptionIdentityState.UNVERIFIABLE
    assert not resolved.cacheable


def test_moveit_preserves_base_and_exact_opaque_constraints() -> None:
    empty = PlanMotionLegacyOptions.build(None, None, None)
    base = MoveItPlanMotionOptions.resolve(empty, {"base_link": "world"})
    assert base.to_backend_options()["base_link"] == "world"
    assert base.cacheable
    for name in ("path_constraints", "trajectory_constraints"):
        constraint_tuple = (object(),)
        resolved = MoveItPlanMotionOptions.resolve(empty, {name: constraint_tuple})
        assert resolved.to_backend_options()[name] is constraint_tuple
        assert resolved.identity_state is OptionIdentityState.UNVERIFIABLE
        assert not resolved.cacheable
```

- [ ] **Step 2: Verify RED**

Run: `pixi run pytest tests/backends/contracts/test_planner_capability_foundation.py -n auto -q`. Expected: collection fails because the new modules are absent.

- [ ] **Step 3: Add complete neutral contract files**

```python
# planner_operation.py
from enum import Enum


class PlannerOperation(Enum):
    INVERSE_KINEMATICS = "inverse_kinematics"
    PLAN_MOTION = "plan_motion"
    PLAN_CARTESIAN_MOTION = "plan_cartesian_motion"
    CHECK_COLLISION = "check_collision"
```

```python
# planner_errors.py
from compas_fab.backends.exceptions import BackendError


class PlannerContractError(BackendError):
    pass


class InvalidPlannerCapabilitiesError(PlannerContractError):
    pass


class PlannerCapabilityError(PlannerContractError):
    pass


class InvalidPlannerOptionsError(PlannerContractError):
    pass


class ConflictingPlannerOptionsError(PlannerContractError):
    pass
```

```python
# planner_capabilities.py
from __future__ import annotations

from enum import Enum
from typing import Sequence
from typing import Tuple

from attrs import define

from .planner_errors import InvalidPlannerCapabilitiesError
from .planner_operation import PlannerOperation


class ConfigurationTolerancePolicy(Enum):
    LEGACY_DEFAULTS = "legacy_defaults"
    PRESERVE_ABSENT = "preserve_absent"


@define(frozen=True, slots=True)
class PlannerImplementationId:
    value: str

    @classmethod
    def build(cls, value: str) -> "PlannerImplementationId":
        return cls(value)

    def __attrs_post_init__(self) -> None:
        if type(self.value) is not str or not self.value:
            raise InvalidPlannerCapabilitiesError("Implementation ID must be non-empty str.")


@define(frozen=True, slots=True)
class PlannerCapabilities:
    implementation_id: PlannerImplementationId
    operations: Tuple[PlannerOperation, ...]
    configuration_tolerance_policy: ConfigurationTolerancePolicy

    @classmethod
    def build(cls, implementation_id: PlannerImplementationId, operations: Sequence[PlannerOperation], tolerance_policy: ConfigurationTolerancePolicy) -> "PlannerCapabilities":
        return cls(implementation_id, tuple(operations), tolerance_policy)

    def __attrs_post_init__(self) -> None:
        invalid = type(self.implementation_id) is not PlannerImplementationId or not self.operations or len(set(self.operations)) != len(self.operations)
        invalid = invalid or any(type(operation) is not PlannerOperation for operation in self.operations)
        invalid = invalid or type(self.configuration_tolerance_policy) is not ConfigurationTolerancePolicy
        if invalid:
            raise InvalidPlannerCapabilitiesError("Planner capabilities are inconsistent.")
```

```python
# planner_options.py
from __future__ import annotations

from enum import Enum
from hashlib import sha256
from math import isfinite
from struct import pack
from typing import Dict
from typing import Mapping
from typing import Optional
from typing import Protocol
from typing import Tuple

from attrs import define

from .planner_errors import InvalidPlannerOptionsError


class OptionIdentityState(Enum):
    VERIFIED = "verified"
    UNVERIFIABLE = "unverifiable"


@define(frozen=True, slots=True)
class PlanMotionLegacyOptions:
    planner_id: Optional[str]
    num_planning_attempts: Optional[int]
    allowed_planning_time: Optional[float]

    @classmethod
    def build(cls, planner_id: Optional[str], attempts: Optional[int], allowed_time: Optional[float]) -> "PlanMotionLegacyOptions":
        return cls(planner_id, attempts, allowed_time)

    def __attrs_post_init__(self) -> None:
        if self.planner_id is not None and (type(self.planner_id) is not str or not self.planner_id):
            raise InvalidPlannerOptionsError("planner_id must be non-empty str or None.")
        if self.num_planning_attempts is not None and (type(self.num_planning_attempts) is not int or self.num_planning_attempts <= 0):
            raise InvalidPlannerOptionsError("num_planning_attempts must be positive int or None.")
        if self.allowed_planning_time is not None:
            invalid_time = type(self.allowed_planning_time) is not float or not isfinite(self.allowed_planning_time) or self.allowed_planning_time <= 0.0
            if invalid_time:
                raise InvalidPlannerOptionsError("allowed_planning_time must be finite positive float or None.")

    @property
    def connected(self) -> bool:
        return any(value is not None for value in (self.planner_id, self.num_planning_attempts, self.allowed_planning_time))


def _part(value: bytes) -> bytes:
    return len(value).to_bytes(8, "big") + value


def _scalar(value: object) -> bytes:
    if type(value) is str:
        return b"s" + value.encode("utf-8")
    if type(value) is bool:
        return b"b1" if value else b"b0"
    if type(value) is int:
        return b"i" + str(value).encode("ascii")
    if type(value) is float and isfinite(value):
        return b"f" + pack("!d", value)
    if value is None:
        return b"n"
    raise InvalidPlannerOptionsError("Verified planner options require canonical scalar values.")


def _digest(values: Tuple[Tuple[str, object], ...]) -> str:
    payload = b""
    for key, value in values:
        if type(key) is not str or not key:
            raise InvalidPlannerOptionsError("Option names must be non-empty str.")
        payload += _part(key.encode("utf-8")) + _part(_scalar(value))
    return sha256(payload).hexdigest()


@define(frozen=True, slots=True)
class ResolvedPlannerOptions:
    values: Tuple[Tuple[str, object], ...]
    identity_state: OptionIdentityState
    identity_digest: Optional[str]

    @classmethod
    def verified(cls, values: Mapping[str, object]) -> "ResolvedPlannerOptions":
        retained = tuple(sorted(values.items()))
        return cls(retained, OptionIdentityState.VERIFIED, _digest(retained))

    @classmethod
    def unverifiable(cls, values: Mapping[str, object]) -> "ResolvedPlannerOptions":
        return cls(tuple(values.items()), OptionIdentityState.UNVERIFIABLE, None)

    def __attrs_post_init__(self) -> None:
        if type(self.identity_state) is not OptionIdentityState:
            raise InvalidPlannerOptionsError("identity_state must be OptionIdentityState.")
        if len({key for key, _ in self.values}) != len(self.values):
            raise InvalidPlannerOptionsError("Planner option names must be unique.")
        if self.identity_state is OptionIdentityState.VERIFIED:
            if self.identity_digest != _digest(self.values):
                raise InvalidPlannerOptionsError("Verified option digest is inconsistent.")
        elif self.identity_digest is not None:
            raise InvalidPlannerOptionsError("Unverifiable options cannot carry a digest.")

    @property
    def cacheable(self) -> bool:
        return self.identity_state is OptionIdentityState.VERIFIED

    def to_backend_options(self) -> Dict[str, object]:
        return dict(self.values)


class PlanMotionOptionsAdapter(Protocol):
    @classmethod
    def resolve(cls, legacy: PlanMotionLegacyOptions, native: Optional[Mapping[str, object]]) -> ResolvedPlannerOptions: ...
```

```python
# planner_contract.py
from typing import Protocol
from typing import Type

from .planner_capabilities import PlannerCapabilities
from .planner_capabilities import PlannerImplementationId
from .planner_options import PlanMotionOptionsAdapter


class PlannerContract(Protocol):
    implementation_id: PlannerImplementationId
    capabilities: PlannerCapabilities
    plan_motion_options: Type[PlanMotionOptionsAdapter]
```

- [ ] **Step 4: Add complete backend adapters and named ROS error**

Both new unsupported adapter files contain this complete code:

```python
# kinematics/options.py and pybullet/options.py
from typing import Mapping
from typing import Optional

from compas_fab.backends.interfaces.planner_errors import PlannerCapabilityError
from compas_fab.backends.interfaces.planner_options import PlanMotionLegacyOptions
from compas_fab.backends.interfaces.planner_options import ResolvedPlannerOptions


class UnsupportedPlanMotionOptions:
    @classmethod
    def resolve(
        cls,
        legacy: PlanMotionLegacyOptions,
        native: Optional[Mapping[str, object]],
    ) -> ResolvedPlannerOptions:
        raise PlannerCapabilityError("Planner does not support plan_motion.")
```
Append this class to `ros/exceptions.py` without changing its existing classes:

```diff
-    def __init__(self, message, error_code):
+    def __init__(self, message: str, error_code: object) -> None:
-        super(RosError, self).__init__("Error code: " + str(error_code) + "; " + message)
+        super(RosError, self).__init__("Error code: " + str(error_code) + "; " + message)  # type: ignore[no-untyped-call]
-    def __init__(self, original_exception, response):
+    def __init__(self, original_exception: Exception, response: object) -> None:
-        super(RosValidationError, self).__init__(str(original_exception))
+        super(RosValidationError, self).__init__(str(original_exception))  # type: ignore[no-untyped-call]
+
class InvalidMoveItPlanMotionOptionsError(BackendError):
    """MoveIt Plan Motion options violate the declared typed contract."""
```

```python
# ros/options.py
from math import isfinite
from typing import Dict
from typing import Mapping
from typing import Optional
from typing import Tuple
from typing import cast

from attrs import define

from compas_fab.backends.interfaces.planner_errors import ConflictingPlannerOptionsError
from compas_fab.backends.interfaces.planner_options import PlanMotionLegacyOptions
from compas_fab.backends.interfaces.planner_options import ResolvedPlannerOptions
from compas_fab.backends.ros.exceptions import InvalidMoveItPlanMotionOptionsError


@define(frozen=True, slots=True)
class MoveItPlanMotionOptions:
    planner_id: Optional[str] = None
    num_planning_attempts: Optional[int] = None
    allowed_planning_time: Optional[float] = None
    max_velocity_scaling_factor: Optional[float] = None
    max_acceleration_scaling_factor: Optional[float] = None
    base_link: Optional[str] = None
    path_constraints: object = None
    trajectory_constraints: object = None

    NAMES = frozenset(
        (
            "planner_id",
            "num_planning_attempts",
            "allowed_planning_time",
            "max_velocity_scaling_factor",
            "max_acceleration_scaling_factor",
            "base_link",
            "path_constraints",
            "trajectory_constraints",
        )
    )

    @classmethod
    def build(cls, values: Mapping[str, object]) -> "MoveItPlanMotionOptions":
        unknown = set(values) - cls.NAMES
        if unknown:
            raise InvalidMoveItPlanMotionOptionsError("Unknown MoveIt Plan Motion option.")
        cls._validate(values)
        return cls(
            planner_id=cast(Optional[str], values.get("planner_id")),
            num_planning_attempts=cast(Optional[int], values.get("num_planning_attempts")),
            allowed_planning_time=cast(Optional[float], values.get("allowed_planning_time")),
            max_velocity_scaling_factor=cast(Optional[float], values.get("max_velocity_scaling_factor")),
            max_acceleration_scaling_factor=cast(Optional[float], values.get("max_acceleration_scaling_factor")),
            base_link=cast(Optional[str], values.get("base_link")),
            path_constraints=values.get("path_constraints"),
            trajectory_constraints=values.get("trajectory_constraints"),
        )

    @classmethod
    def resolve(cls, legacy: PlanMotionLegacyOptions, native: Optional[Mapping[str, object]]) -> ResolvedPlannerOptions:
        if native is not None and legacy.connected:
            raise ConflictingPlannerOptionsError("Native options conflict with connected legacy ports.")
        values = dict(native) if native is not None else {
                "planner_id": legacy.planner_id,
                "num_planning_attempts": legacy.num_planning_attempts,
                "allowed_planning_time": legacy.allowed_planning_time,
        }
        return cls.build({key: value for key, value in values.items() if value is not None}).as_resolved()

    def __attrs_post_init__(self) -> None:
        self._validate(self.to_backend_values())

    def to_backend_values(self) -> Dict[str, object]:
        pairs: Tuple[Tuple[str, object], ...] = (
            ("planner_id", self.planner_id),
            ("num_planning_attempts", self.num_planning_attempts),
            ("allowed_planning_time", self.allowed_planning_time),
            ("max_velocity_scaling_factor", self.max_velocity_scaling_factor),
            ("max_acceleration_scaling_factor", self.max_acceleration_scaling_factor),
            ("base_link", self.base_link),
            ("path_constraints", self.path_constraints),
            ("trajectory_constraints", self.trajectory_constraints),
        )
        return {name: value for name, value in pairs if value is not None}

    def as_resolved(self) -> ResolvedPlannerOptions:
        values = self.to_backend_values()
        if self.path_constraints is not None or self.trajectory_constraints is not None:
            return ResolvedPlannerOptions.unverifiable(values)
        return ResolvedPlannerOptions.verified(values)

    @classmethod
    def _validate(cls, values: Mapping[str, object]) -> None:
        planner_id = values.get("planner_id")
        base_link = values.get("base_link")
        attempts = values.get("num_planning_attempts")
        allowed_time = values.get("allowed_planning_time")
        if planner_id is not None and (type(planner_id) is not str or not planner_id):
            raise InvalidMoveItPlanMotionOptionsError("planner_id must be non-empty str.")
        if base_link is not None and (type(base_link) is not str or not base_link):
            raise InvalidMoveItPlanMotionOptionsError("base_link must be non-empty str.")
        if attempts is not None and (type(attempts) is not int or attempts <= 0):
            raise InvalidMoveItPlanMotionOptionsError("num_planning_attempts must be positive int.")
        if allowed_time is not None and (
            type(allowed_time) is not float or not isfinite(allowed_time) or allowed_time <= 0.0
        ):
            raise InvalidMoveItPlanMotionOptionsError("allowed_planning_time must be finite positive float.")
        for name in ("max_velocity_scaling_factor", "max_acceleration_scaling_factor"):
            value = values.get(name)
            if value is not None and (
                type(value) is not float or not isfinite(value) or not 0.0 <= value <= 1.0
            ):
                raise InvalidMoveItPlanMotionOptionsError("{} must be a finite float in [0, 1].".format(name))
```
Add these imports and methods to the existing `tesseract/options.py`; retain its constants, fields, `.strip()` validation, default/custom-profile rules, and every existing named-error import unchanged:

```diff
 from compas_fab.backends.interfaces.planner_errors import ConflictingPlannerOptionsError
 from compas_fab.backends.interfaces.planner_options import PlanMotionLegacyOptions
 from compas_fab.backends.interfaces.planner_options import ResolvedPlannerOptions
@@
 class TesseractPlanOptions:
+    @classmethod
+    def resolve(
+        cls,
+        legacy: PlanMotionLegacyOptions,
+        native: Optional[Mapping[str, object]],
+    ) -> ResolvedPlannerOptions:
+        if legacy.connected:
+            raise ConflictingPlannerOptionsError("Tesseract has no legacy Plan Motion option ports.")
+        options = cls.build(native)
+        return ResolvedPlannerOptions.unverifiable(
+            {
+                "pipeline": options.pipeline,
+                "profiles": options.profiles,
+                "auto_seed": options.auto_seed,
+            }
+        )
```

- [ ] **Step 5: Publish declarations with exact additive planner patches**

For each class, insert declarations after its existing class docstring. Add the three matching imports shown by the expressions. Make only these exact typing corrections: direct-import `AnalyticalKinematics`, add `# type: ignore[no-untyped-call]` to the three reported kinematics `super`/client calls, annotate PyBullet/ROS constructors `-> None`, and add targeted ignores to their existing untyped `super`/`reset_planning_scene` calls.

```diff
# AnalyticalKinematicsPlanner
implementation_id = PlannerImplementationId.build("compas_fab.analytical/v1")
capabilities = PlannerCapabilities.build(
    implementation_id,
    (PlannerOperation.INVERSE_KINEMATICS, PlannerOperation.PLAN_CARTESIAN_MOTION),
    ConfigurationTolerancePolicy.LEGACY_DEFAULTS,
)
plan_motion_options = UnsupportedPlanMotionOptions

# AnalyticalPyBulletPlanner
implementation_id = PlannerImplementationId.build("compas_fab.analytical_pybullet/v1")
capabilities = PlannerCapabilities.build(
    implementation_id,
    (PlannerOperation.INVERSE_KINEMATICS, PlannerOperation.PLAN_CARTESIAN_MOTION, PlannerOperation.CHECK_COLLISION),
    ConfigurationTolerancePolicy.LEGACY_DEFAULTS,
)
plan_motion_options = UnsupportedPlanMotionOptions

# PyBulletPlanner
implementation_id = PlannerImplementationId.build("compas_fab.pybullet/v1")
capabilities = PlannerCapabilities.build(
    implementation_id,
    (PlannerOperation.INVERSE_KINEMATICS, PlannerOperation.PLAN_CARTESIAN_MOTION, PlannerOperation.CHECK_COLLISION),
    ConfigurationTolerancePolicy.LEGACY_DEFAULTS,
)
plan_motion_options = UnsupportedPlanMotionOptions

# MoveItPlanner
implementation_id = PlannerImplementationId.build("compas_fab.moveit/v1")
capabilities = PlannerCapabilities.build(implementation_id, tuple(PlannerOperation), ConfigurationTolerancePolicy.LEGACY_DEFAULTS)
plan_motion_options = MoveItPlanMotionOptions

# TesseractPlanner
implementation_id = PlannerImplementationId.build("compas_fab.tesseract/v1")
capabilities = PlannerCapabilities.build(
    implementation_id,
    (PlannerOperation.INVERSE_KINEMATICS, PlannerOperation.PLAN_MOTION, PlannerOperation.CHECK_COLLISION),
    ConfigurationTolerancePolicy.PRESERVE_ABSENT,
)
plan_motion_options = TesseractPlanOptions
```
Apply these exact typing-only corrections in the same patch:

```diff
+# kinematics/planner.py
+from compas_fab.backends.interfaces.planner_capabilities import ConfigurationTolerancePolicy, PlannerCapabilities, PlannerImplementationId
+from compas_fab.backends.interfaces.planner_operation import PlannerOperation
+from compas_fab.backends.kinematics.options import UnsupportedPlanMotionOptions
-from compas_fab.backends.kinematics.solvers import AnalyticalKinematics
+from compas_fab.backends.kinematics.solvers.analytical_kinematics import AnalyticalKinematics
-    def __init__(self, kinematics_solver: AnalyticalKinematics, verbose: Optional[bool] = False):
+    def __init__(self, kinematics_solver: AnalyticalKinematics, verbose: Optional[bool] = False) -> None:
-        super(AnalyticalKinematicsPlanner, self).__init__()
+        super(AnalyticalKinematicsPlanner, self).__init__()  # type: ignore[no-untyped-call]
-        self._client = AnalyticalKinematicsClient(verbose=verbose)
+        self._client = AnalyticalKinematicsClient(verbose=verbose)  # type: ignore[no-untyped-call]
-    def __init__(self, client: PyBulletClient, kinematics_solver: AnalyticalKinematics):
+    def __init__(self, client: PyBulletClient, kinematics_solver: AnalyticalKinematics) -> None:
-        super(AnalyticalPyBulletPlanner, self).__init__()
+        super(AnalyticalPyBulletPlanner, self).__init__()  # type: ignore[no-untyped-call]
+# pybullet/planner.py
+from compas_fab.backends.interfaces.planner_capabilities import ConfigurationTolerancePolicy, PlannerCapabilities, PlannerImplementationId
+from compas_fab.backends.interfaces.planner_operation import PlannerOperation
+from compas_fab.backends.pybullet.options import UnsupportedPlanMotionOptions
-    def __init__(self, client):
+    def __init__(self, client: "PyBulletClient") -> None:
-        super(PyBulletPlanner, self).__init__()
+        super(PyBulletPlanner, self).__init__()  # type: ignore[no-untyped-call]
+# ros/planner.py
+from compas_fab.backends.interfaces.planner_capabilities import ConfigurationTolerancePolicy, PlannerCapabilities, PlannerImplementationId
+from compas_fab.backends.interfaces.planner_operation import PlannerOperation
+from compas_fab.backends.ros.client import RosClient
+from compas_fab.backends.ros.options import MoveItPlanMotionOptions
-    def __init__(self, client):
+    def __init__(self, client: RosClient) -> None:
-        super(MoveItPlanner, self).__init__()
+        super(MoveItPlanner, self).__init__()  # type: ignore[no-untyped-call]
-        self.reset_planning_scene()
+        self.reset_planning_scene()  # type: ignore[no-untyped-call]
+# tesseract/planner.py
+from compas_fab.backends.interfaces.planner_capabilities import ConfigurationTolerancePolicy, PlannerCapabilities, PlannerImplementationId
+from compas_fab.backends.interfaces.planner_operation import PlannerOperation
+from .options import TesseractPlanOptions
```

Do not edit `interfaces/planner.py`, any client, backend feature, existing contract test, or existing Tesseract option behavior.

- [ ] **Step 6: Verify, type, lint, and commit**

Run: `pixi run pytest tests/backends/contracts/test_planner_capability_foundation.py tests/backends/tesseract/test_plan_motion.py -n auto -q`. Expected: all tests pass.

Run this exact strict gate (the GH host script is intentionally absent and is tested through Task 3's harness):

```bash
pixi run mypy --strict src/compas_fab/backends/interfaces/planner_operation.py src/compas_fab/backends/interfaces/planner_errors.py src/compas_fab/backends/interfaces/planner_capabilities.py src/compas_fab/backends/interfaces/planner_options.py src/compas_fab/backends/interfaces/planner_contract.py src/compas_fab/backends/kinematics/options.py src/compas_fab/backends/pybullet/options.py src/compas_fab/backends/ros/exceptions.py src/compas_fab/backends/ros/options.py src/compas_fab/backends/tesseract/options.py src/compas_fab/backends/kinematics/planner.py src/compas_fab/backends/pybullet/planner.py src/compas_fab/backends/ros/planner.py src/compas_fab/backends/tesseract/planner.py
```

Expected: zero errors. If strict cleanup needs a client/backend-feature edit, stop and split.

Run: `pixi run ruff check src/compas_fab/backends/interfaces/planner_operation.py src/compas_fab/backends/interfaces/planner_errors.py src/compas_fab/backends/interfaces/planner_capabilities.py src/compas_fab/backends/interfaces/planner_options.py src/compas_fab/backends/interfaces/planner_contract.py src/compas_fab/backends/kinematics/options.py src/compas_fab/backends/pybullet/options.py src/compas_fab/backends/ros/exceptions.py src/compas_fab/backends/ros/options.py src/compas_fab/backends/tesseract/options.py src/compas_fab/backends/kinematics/planner.py src/compas_fab/backends/pybullet/planner.py src/compas_fab/backends/ros/planner.py src/compas_fab/backends/tesseract/planner.py tests/backends/contracts/test_planner_capability_foundation.py`. Expected: `All checks passed!`.

```bash
git add src/compas_fab/backends tests/backends/contracts/test_planner_capability_foundation.py
GIT_AUTHOR_NAME='Jelle Feringa' GIT_AUTHOR_EMAIL='jelleferinga@gmail.com' GIT_COMMITTER_NAME='Jelle Feringa' GIT_COMMITTER_EMAIL='jelleferinga@gmail.com' git commit -m 'feat: declare planner contracts'
```

### Task 3: Connection-Aware Configuration Target Policy

**Files:**
- Create: `src/compas_fab/ghpython/configuration_target_policy.py`
- Modify additively: `Cf_ConfigurationTarget/code.py`; append one object in `metadata.json`
- Create: `tests/ghpython/component_harness.py`
- Create: `tests/ghpython/test_configuration_target_component.py`
- Create: `docs/frontends/ghpython-configuration-target.md`
- Modify: `docs/frontends/ghpython.md`

**Interfaces:** `parse_configuration_tolerance_policy`; `resolve_configuration_tolerances`; real `RunScript(target_configuration, tolerance_above, tolerance_below, tolerance_policy)` loaded by the harness.

- [ ] **Step 1: Add the complete harness and RED RunScript tests**

```python
# tests/ghpython/component_harness.py
from pathlib import Path
import sys
from types import ModuleType
from types import SimpleNamespace
from typing import Dict
from typing import Tuple

from pytest import MonkeyPatch

COMPONENTS = Path(__file__).parents[2] / "src" / "compas_fab" / "ghpython" / "components_cpython"


class FakeParameter:
    def __init__(self, name: str, connected: bool) -> None:
        self.Name = name
        self.SourceCount = int(connected)
        self.PersistentDataCount = 0


class FakeHost:
    def __init__(self, connections: Dict[str, bool]) -> None:
        parameters = [FakeParameter(name, connected) for name, connected in connections.items()]
        self.Params = SimpleNamespace(Input=parameters)


def load_component(
    monkeypatch: MonkeyPatch,
    directory: str,
    class_name: str,
    connections: Dict[str, bool],
) -> Tuple[object, FakeHost]:
    host = FakeHost(connections)
    grasshopper = ModuleType("Grasshopper")
    grasshopper.Kernel = SimpleNamespace(GH_ScriptInstance=object)
    monkeypatch.setitem(sys.modules, "Grasshopper", grasshopper)
    monkeypatch.setitem(sys.modules, "Rhino", ModuleType("Rhino"))
    monkeypatch.setitem(sys.modules, "System", ModuleType("System"))
    source = COMPONENTS / directory / "code.py"
    namespace = {"__name__": "test_" + directory, "__file__": str(source)}
    exec(compile(source.read_text(encoding="utf-8"), str(source), "exec"), namespace)
    namespace["ghenv"] = SimpleNamespace(Component=host)
    return namespace[class_name](), host
```

```python
# tests/ghpython/test_configuration_target_component.py
import pytest
from compas_robots import Configuration

from compas_fab.backends.interfaces.planner_capabilities import ConfigurationTolerancePolicy
from compas_fab.ghpython.configuration_target_policy import InvalidConfigurationToleranceInputError
from compas_fab.ghpython.configuration_target_policy import InvalidConfigurationTolerancePolicyError
from compas_fab.robots import ConfigurationTarget
from tests.ghpython.component_harness import load_component


def component(monkeypatch, **connections):
    defaults = {
        "target_configuration": True,
        "tolerance_above": False,
        "tolerance_below": False,
        "tolerance_policy": False,
    }
    defaults.update(connections)
    loaded, _ = load_component(
        monkeypatch,
        "Cf_ConfigurationTarget",
        "ConfigurationTargetComponent",
        defaults,
    )
    return loaded


def test_unwired_empty_lists_keep_legacy_defaults(monkeypatch) -> None:
    configuration = Configuration.from_revolute_values([0.0])
    expected = ConfigurationTarget.generate_default_tolerances(configuration, 0.001, 0.017453292519943295)
    target = component(monkeypatch).RunScript(configuration, [], [], "")
    assert target.tolerance_above == expected[0]
    assert target.tolerance_below == expected[1]


def test_explicit_lists_and_connected_empty_shape(monkeypatch) -> None:
    configured = component(monkeypatch, tolerance_above=True, tolerance_below=True)
    target = configured.RunScript(Configuration.from_revolute_values([0.0]), [0.2], [0.3], "")
    assert target.tolerance_above == [0.2]
    assert target.tolerance_below == [0.3]
    policy_path = component(monkeypatch, tolerance_above=True, tolerance_below=True, tolerance_policy=True)
    with pytest.raises(InvalidConfigurationToleranceInputError):
        policy_path.RunScript(Configuration.from_revolute_values([0.0]), [], [0.3], "legacy_defaults")


def test_preserve_absent_and_planner_policy(monkeypatch) -> None:
    configured = component(monkeypatch, tolerance_policy=True)
    absent = configured.RunScript(Configuration.from_revolute_values([0.0]), [], [], "preserve_absent")
    assert absent.tolerance_above is None
    planner_policy = ConfigurationTolerancePolicy.PRESERVE_ABSENT
    from_planner = configured.RunScript(Configuration.from_revolute_values([0.0]), [], [], planner_policy)
    assert from_planner.tolerance_below is None


def test_unknown_mode_fails_named(monkeypatch) -> None:
    configured = component(monkeypatch, tolerance_policy=True)
    with pytest.raises(InvalidConfigurationTolerancePolicyError):
        configured.RunScript(Configuration.from_revolute_values([0.0]), [], [], "invented")
```

- [ ] **Step 2: Verify RED**

Run: `pixi run pytest tests/ghpython/test_configuration_target_component.py -n auto -q`. Expected: helper import fails.

- [ ] **Step 3: Add the complete pure policy helper**

```python
# src/compas_fab/ghpython/configuration_target_policy.py
from math import isfinite
from typing import List
from typing import Optional
from typing import Sequence
from typing import Tuple
from typing import Union

from compas_robots import Configuration  # type: ignore[import-untyped]

from compas_fab.backends.interfaces.planner_capabilities import ConfigurationTolerancePolicy
from compas_fab.robots import ConfigurationTarget

DEFAULT_PRISMATIC_TOLERANCE_METERS = 0.001  # Existing GH contract: 1 mm.
DEFAULT_REVOLUTE_TOLERANCE_RADIANS = 0.017453292519943295  # Existing GH contract: pi / 180.
PolicyInput = Optional[Union[str, ConfigurationTolerancePolicy]]


class InvalidConfigurationTolerancePolicyError(ValueError):
    pass


class InvalidConfigurationToleranceInputError(ValueError):
    pass


def parse_configuration_tolerance_policy(value: PolicyInput) -> ConfigurationTolerancePolicy:
    if type(value) is ConfigurationTolerancePolicy:
        return value
    if value is None:
        return ConfigurationTolerancePolicy.LEGACY_DEFAULTS
    if type(value) is not str:
        raise InvalidConfigurationTolerancePolicyError("Tolerance policy must be a declared policy or string.")
    try:
        return ConfigurationTolerancePolicy(value)
    except ValueError as error:
        raise InvalidConfigurationTolerancePolicyError("Unknown tolerance policy: {!r}.".format(value)) from error


def _validated(
    configuration: Configuration,
    values: Optional[Sequence[float]],
    name: str,
) -> Optional[List[float]]:
    if values is None:
        return None
    retained = list(values)
    if not retained or len(retained) != len(configuration.joint_values):
        raise InvalidConfigurationToleranceInputError("{} must match the non-empty joint vector.".format(name))
    if any(type(value) is not float or not isfinite(value) or value < 0.0 for value in retained):
        raise InvalidConfigurationToleranceInputError("{} must contain finite non-negative floats.".format(name))
    return retained


def resolve_configuration_tolerances(
    configuration: Configuration,
    above: Optional[Sequence[float]],
    below: Optional[Sequence[float]],
    policy: ConfigurationTolerancePolicy,
) -> Tuple[Optional[List[float]], Optional[List[float]]]:
    validated_above = _validated(configuration, above, "tolerance_above")
    validated_below = _validated(configuration, below, "tolerance_below")
    if type(policy) is not ConfigurationTolerancePolicy:
        raise InvalidConfigurationTolerancePolicyError("policy must be ConfigurationTolerancePolicy.")
    if policy is ConfigurationTolerancePolicy.PRESERVE_ABSENT:
        return validated_above, validated_below
    default_above, default_below = ConfigurationTarget.generate_default_tolerances(
        configuration,
        DEFAULT_PRISMATIC_TOLERANCE_METERS,
        DEFAULT_REVOLUTE_TOLERANCE_RADIANS,
    )
    return validated_above or default_above, validated_below or default_below
```

- [ ] **Step 4: Patch the existing component additively**

Keep its imports/docstring and add imports for `optional_connected_input`, the parser, and resolver. Apply the exact signature change. The unwired-policy branch below is the existing default generation and return unchanged; only the connected-policy branch uses new resolution.

```diff
+from compas_fab.ghpython.configuration_target_policy import parse_configuration_tolerance_policy
+from compas_fab.ghpython.configuration_target_policy import resolve_configuration_tolerances
+from compas_fab.ghpython.input_semantics import optional_connected_input
-    def RunScript(self, target_configuration, tolerance_above, tolerance_below):
+    def RunScript(self, target_configuration, tolerance_above, tolerance_below, tolerance_policy):
```

```diff
component = ghenv.Component  # noqa: F821
policy_input = optional_connected_input(component, "tolerance_policy", tolerance_policy)

if policy_input is None:
    default_above, default_below = ConfigurationTarget.generate_default_tolerances(
        target_configuration,
        self.DEFAULT_TOLERANCE_METERS,
        self.DEFAULT_TOLERANCE_RADIANS,
    )
    return ConfigurationTarget(
        target_configuration=target_configuration,
        tolerance_above=tolerance_above or default_above,
        tolerance_below=tolerance_below or default_below,
    )

above = optional_connected_input(component, "tolerance_above", tolerance_above)
below = optional_connected_input(component, "tolerance_below", tolerance_below)
policy = parse_configuration_tolerance_policy(policy_input)
resolved_above, resolved_below = resolve_configuration_tolerances(
    target_configuration,
    above,
    below,
    policy,
)
return ConfigurationTarget(
    target_configuration=target_configuration,
    tolerance_above=resolved_above,
    tolerance_below=resolved_below,
)
```

Append this exact fourth metadata input after `tolerance_below`:

```json
{
  "name": "tolerance_policy",
  "description": "Unwired keeps legacy defaults; legacy_defaults fills absence; preserve_absent delegates absence to the planner.",
  "typeHintID": "str"
}
```

- [ ] **Step 5: Add complete docs and exact index patch**

```markdown
# Configuration Target

`Configuration Target` uses native joint units: metres for prismatic joints and radians for revolute or continuous joints. Existing definitions keep the 0.001 m / pi-per-180 rad defaults because an unwired `tolerance_policy` takes the legacy branch.

The existing inputs remain first and unchanged. `tolerance_policy` is appended. When policy is connected, each connected tolerance list must contain one finite non-negative float per joint; a connected empty list is invalid. `legacy_defaults` fills absent lists. `preserve_absent` retains `None`, including when supplied directly from a planner's declared `configuration_tolerance_policy`.

Unknown policies and invalid list shapes fail with named local errors. No planner policy is guessed.
```

Append this exact line under the existing Grasshopper introduction in `docs/frontends/ghpython.md`:

```markdown
- [Configuration Target tolerance policy](ghpython-configuration-target.md)
```

- [ ] **Step 6: Run all gates, then commit**

Run: `pixi run pytest tests/ghpython/test_configuration_target_component.py -n auto -q`
Run: `pixi run -e rhino39 pytest tests/ghpython/test_component_state.py tests/ghpython/test_configuration_target_component.py tests/backends/contracts/test_planner_capability_foundation.py -n auto -q`
Expected: all direct RunScript and Python 3.9 contracts pass.
Run this exact command; the GH host script is excluded because `ghenv` is verified by the harness:

```bash
pixi run mypy --strict src/compas_fab/backends/interfaces/planner_operation.py src/compas_fab/backends/interfaces/planner_errors.py src/compas_fab/backends/interfaces/planner_capabilities.py src/compas_fab/backends/interfaces/planner_options.py src/compas_fab/backends/interfaces/planner_contract.py src/compas_fab/backends/kinematics/options.py src/compas_fab/backends/pybullet/options.py src/compas_fab/backends/ros/exceptions.py src/compas_fab/backends/ros/options.py src/compas_fab/backends/tesseract/options.py src/compas_fab/backends/kinematics/planner.py src/compas_fab/backends/pybullet/planner.py src/compas_fab/backends/ros/planner.py src/compas_fab/backends/tesseract/planner.py src/compas_fab/ghpython/component_identity.py src/compas_fab/ghpython/current_output.py src/compas_fab/ghpython/configuration_target_policy.py
```
Run: `pixi run ruff check src/compas_fab/ghpython/component_identity.py src/compas_fab/ghpython/current_output.py src/compas_fab/ghpython/configuration_target_policy.py src/compas_fab/ghpython/components_cpython/Cf_ConfigurationTarget/code.py tests/ghpython tests/backends/contracts/test_planner_capability_foundation.py`
Run: `pixi run mkdocs build --strict`
Run: `pixi run pytest -n auto`
Run: `pixi run pytest --testmon -n auto`. Expected: every command passes without skip/xfail or reference-test edits.
Run: `! rg -ni '[r]obotcomponents|[r]obot components|[r]obot-components' docs/superpowers/plans/2026-07-11-grasshopper-planner-contract-foundation.md src tests docs/frontends`. Expected: zero matches.
Run: `git diff --check 30adc9ff -- && git diff --stat 30adc9ff -- && git status --short`. Expected: clean fixed-base diff containing only planned additions and bounded modifications.
```bash
git add src/compas_fab/ghpython/configuration_target_policy.py src/compas_fab/ghpython/components_cpython/Cf_ConfigurationTarget tests/ghpython docs/frontends/ghpython-configuration-target.md docs/frontends/ghpython.md
GIT_AUTHOR_NAME='Jelle Feringa' GIT_AUTHOR_EMAIL='jelleferinga@gmail.com' GIT_COMMITTER_NAME='Jelle Feringa' GIT_COMMITTER_EMAIL='jelleferinga@gmail.com' git commit -m 'feat: add target tolerance policy'
```

## Self-Review Gate

- Coverage: identity/state; exact five-planner declarations/adapters; MoveIt rejection; exact uncacheable profiles; real RunScript semantics; raw/finite/shape tests; docs; Python 3.12/3.9; strict typed modules; Ruff/full/testmon.
- Deferrals name scene/reconnect identity, Plan Motion, Cartesian, IK, and manifest work. Existing options/tests/path/port order/defaults remain; no placeholder, fallback, broad catch, skip, xfail, unowned digest, manifest, or reconnect claim remains.
