# Grasshopper-Native Vectorization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Prove that existing Tesseract components process frame, pose, target, and program series through native Grasshopper item/list access.

**Architecture:** Grasshopper owns trees and matching. Components stay thin adapters to exact Tesseract functions; no production series framework is added.

**Tech Stack:** Rhino 8 CPython 3.9, Grasshopper component metadata, COMPAS, `tesseract-robotics-nanobind==0.35.0.6`, pytest, Pixi.

## Global Constraints

- Prefer existing code; add no production tree, identity, matching, or wrapper model.
- Standard GH matching applies to item inputs; list inputs consume one ordered branch.
- Keep exact native objects and defaults.
- Use Pixi and `pytest -n auto`; add no skip or xfail.

---

### Task 1: Pin Vectorized Component Access

**Files:**
- Modify: `tests/backends/tesseract/test_grasshopper_components.py`
- Create: `docs/frontends/ghpython-tesseract-series.md`
- Modify: `mkdocs.yml`

**Interfaces:**
- Consumes: existing component metadata.
- Produces: an executable item/list access contract and concise user example.

- [ ] **Step 1: Add the failing metadata test**

```python
def test_native_authoring_uses_grasshopper_vectorization():
    expected = {
        "Cf_TesseractPose": {"frame": 0, "metres_per_user_unit": 0, "working_frame": 0},
        "Cf_TesseractCartesianTarget": {"pose": 0, "move_type": 0, "profile": 0},
        "Cf_TesseractJointTarget": {"positions": 1, "joint_names": 1, "move_type": 0, "profile": 0},
        "Cf_TesseractStateTarget": {
            "positions": 1, "joint_names": 1, "velocities": 1,
            "accelerations": 1, "time": 0, "move_type": 0, "profile": 0,
        },
        "Cf_TesseractMotionProgram": {
            "native_robot": 0, "targets": 1, "group_name": 0,
            "tcp_frame": 0, "working_frame": 0, "profile": 0,
        },
    }
    for component, access in expected.items():
        _, metadata = _component(component)
        assert {
            item["name"]: item.get("scriptParamAccess", 0)
            for item in metadata["ghpython"]["inputParameters"]
        } == access
```

- [ ] **Step 2: Run the test**

Run: `pixi run pytest tests/backends/tesseract/test_grasshopper_components.py -n auto -q`

Expected: pass if current metadata already expresses the intended contract; any failure is fixed only in the incorrect metadata entry.

- [ ] **Step 3: Document the series flow**

Write one short page showing multi-branch `Frames → Poses → Targets → Programs`, standard item matching, atomic joint-vector branches, and the one-item convenience case. Add it beneath Grasshopper in `mkdocs.yml`.

- [ ] **Step 4: Verify and commit**

Run: `pixi run pytest tests/backends/tesseract/test_grasshopper_components.py tests/backends/tesseract/test_grasshopper_input_semantics.py -n auto -q`

Run: `pixi run mkdocs build --strict`

Commit: `test: pin vectorized GH access`

### Task 2: Keep Native Planning Inputs Stable

**Files:**
- Modify: `src/compas_fab/backends/tesseract/native_plan.py`
- Modify: `tests/backends/tesseract/test_native_plan_component.py`

**Interfaces:**
- Consumes: `NativePlanCall.signature`.
- Produces: `validate_inputs_before_execution()` and endpoint validation around `plan_native`.

- [ ] **Step 1: Add failing drift tests**

Test that changing the program or scene after `NativePlanCall.build` fails before the planner call. Test that a persistent program or scene change during a controlled planner call discards the result. Match `InvalidTesseractNativePlanError` messages rather than adding exception subclasses.

- [ ] **Step 2: Implement the minimal guard**

```python
def validate_inputs_before_execution(self) -> None:
    if self.signature != _signature(self.planner, self.request):
        raise InvalidTesseractNativePlanError("Native plan inputs changed before execution.")

def execute(self) -> TesseractPlanningResult:
    self.validate_inputs_before_execution()
    result = self.planner.plan_native(self.request)
    if self.signature != _signature(self.planner, self.request):
        raise InvalidTesseractNativePlanError("Native plan inputs changed during execution.")
    return result
```

Use exact integer checks (`type(value) is int`) in `NativePlanSignature` so booleans cannot masquerade as identities or revisions.

- [ ] **Step 3: Verify and commit**

Run: `pixi run pytest tests/backends/tesseract/test_native_plan_component.py -n auto -q`

Run: `pixi run pytest tests/ghpython tests/backends/tesseract -n auto -q`

Run: `pixi run ruff check src/compas_fab/backends/tesseract/native_plan.py tests/backends/tesseract/test_native_plan_component.py`

Commit: `fix: guard native plan inputs`
