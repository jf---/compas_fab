# Native Tesseract Kinematics GH Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add three itemwise Grasshopper components for exact native FK, IK, and collision.

**Architecture:** Each script calls one existing planner method. Grasshopper owns iteration; exact native result objects remain primary outputs.

**Tech Stack:** Rhino 8 CPython 3.9, COMPAS componentizer, Tesseract nanobind 0.35.0.6, pytest, Pixi.

## Global Constraints

- No backend changes, shared helper, wrapper type, or production tree model.
- Every input uses item access; every script is under 100 lines.
- Catch only `TesseractBackendError`; preserve exact native objects.
- Use Pixi and `pytest -n auto`; no skip or xfail.

---

### Task 1: Native Kinematics Components

**Files:**
- Create: `src/compas_fab/ghpython/components_cpython/Cf_TesseractNativeForwardKinematics/{code.py,metadata.json,icon.svg,icon.png}`
- Create: `src/compas_fab/ghpython/components_cpython/Cf_TesseractNativeInverseKinematics/{code.py,metadata.json,icon.svg,icon.png}`
- Create: `src/compas_fab/ghpython/components_cpython/Cf_TesseractNativeCollision/{code.py,metadata.json,icon.svg,icon.png}`
- Create: `tests/backends/tesseract/test_native_kinematics_components.py`
- Modify: `tests/backends/tesseract/test_grasshopper_components.py`
- Modify: `.github/workflows/{build.yml,publish_yak.yml,release.yml}`

**Interfaces:**
- FK: `(planner, robot_cell_state, link_name, group) -> pose`.
- IK: `(planner, target, robot_cell_state, group, ik_solver_name) -> result, target_pose, native_input, group, joint_names, solutions`.
- Collision: `(planner, robot_cell_state, request) -> result, native_map, native_contacts, colliding_contacts, in_collision`.

- [ ] **Step 1: Write failing metadata and execution tests**

Pin exact port order, item access, nanobind directive, 24×24 icons, scripts below 100 lines,
and absence of conventional planner calls or broad exceptions. Fake planners must assert input
object identity and return exact native-like objects. Test empty IK solutions and collision filtering
with the existing `is_collision_distance` predicate.

- [ ] **Step 2: Implement the three adapters**

Each `RunScript` returns all-`None` outputs when required inputs are absent, calls exactly one native
planner method inside `try`, reports `TesseractBackendError` through `compas_ghpython.error`, and
otherwise returns the exact wrapper plus readable fields. IK preserves native solution order;
collision preserves all contacts before filtering colliding contacts.

- [ ] **Step 3: Add and render icons**

Create simple 24×24 SVGs matching the existing Tesseract icon language and render PNGs with
`rsvg-convert`. Do not add a renderer dependency.

- [ ] **Step 4: Register Windows artifacts**

Add these exact files to the existing required-user-object inventory in tests and all three workflows:

```text
Cf_TesseractNativeForwardKinematics.ghuser
Cf_TesseractNativeInverseKinematics.ghuser
Cf_TesseractNativeCollision.ghuser
```

- [ ] **Step 5: Verify and commit**

Run: `pixi run pytest tests/backends/tesseract/test_native_kinematics_components.py tests/backends/tesseract/test_grasshopper_components.py -n auto -q`

Run: `pixi run pytest tests/ghpython tests/backends/tesseract -n auto -q`

Run: `pixi run ruff check tests/backends/tesseract/test_native_kinematics_components.py tests/backends/tesseract/test_grasshopper_components.py`

Run: `git diff --check`

Commit: `feat: add native GH kinematics`

### Task 2: Contact Request Authoring

**Files:**
- Modify: `src/compas_fab/backends/tesseract/collision.py`
- Create: `src/compas_fab/ghpython/components_cpython/Cf_TesseractContactRequest/{code.py,metadata.json,icon.svg,icon.png}`
- Modify: `tests/backends/tesseract/{test_collision.py,test_grasshopper_components.py}`
- Modify: `.github/workflows/{build.yml,publish_yak.yml,release.yml}`

**Interfaces:**
- `build_contact_request(test_type, calculate_distance, calculate_penetration, contact_limit) -> ContactRequest`.
- GH ports are four item inputs and one exact native request output.

- [ ] **Step 1: Write failing factory and component tests**

Cover `FIRST`, `CLOSEST`, `ALL`, and `LIMITED`; exact bool validation; non-negative exact
integer limit; false/zero preservation; native defaults when ports are unwired; nanobind pin;
item access; script under 100 lines; and backend errors reaching the component.

- [ ] **Step 2: Add one factory function and one thin component**

Add the validation function to existing `collision.py`, raising existing
`TesseractContactQueryError`. The component uses `optional_connected_input` and a value list;
it adds no class beyond `GH_ScriptInstance` and no helper module.

- [ ] **Step 3: Register, verify, and commit**

Add `Cf_TesseractContactRequest.ghuser` to test/workflow inventories and render its 24×24 icon.

Run: `pixi run pytest tests/backends/tesseract/test_collision.py tests/backends/tesseract/test_grasshopper_components.py -n auto -q`

Run: `pixi run pytest tests/ghpython tests/backends/tesseract -n auto -q`

Run: `pixi run ruff check src/compas_fab/backends/tesseract/collision.py tests/backends/tesseract/test_collision.py tests/backends/tesseract/test_grasshopper_components.py`

Commit: `feat: add GH contact requests`
