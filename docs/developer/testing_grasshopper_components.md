# Testing Grasshopper components

**BLUF.** A Grasshopper component is a *thin adapter* over a backend that is
already tested. So we test the adapter, not the backend: exec the component's
`code.py` under a faithful **fake Grasshopper host** and call `RunScript`
directly against the **real** Tesseract backend. Four guarantees are pinned for
every component — argument translation, connection-state handling, error
surfacing, output arity — in ordinary `pytest`, with **no Rhino and no mocks of
our own code**. Grasshopper's tree-lifting is the one thing this cannot see; that
is proven once, separately (see [Rhino harness](rhino_harness.md)).

The canonical suite is
[`tests/backends/tesseract/test_grasshopper_runscript_contract.py`](https://github.com/compas-dev/compas_fab) —
every snippet below is drawn from it.

## Why this works: thin adapter over a tested core

The real work lives in the backend — `pose_from_working_frame`,
`joint_target_from_native`, `build_motion_program`, the profile factories — and
is covered by the backend suite. A component only *marshals*: read GH inputs,
apply defaults for unwired inputs, call the backend once, surface errors, shape
an output tuple. That tiny marshalling surface is exactly what these tests pin.

| What can break in a component | Tested by |
|---|---|
| Wrong backend fn, wrong arg order, misplaced default | Layer 1 — this suite (equivalence vs a direct backend call) |
| Unwired input read as a value instead of "absent" | Layer 1 — connection-state toggling |
| Swallowed error / wrong `None`-shape on failure | Layer 1 — error-surfacing tests |
| `scriptParamAccess` / port names / icons | Layer 2 — static metadata assertions |
| GH feeding one vector per branch (tree-lifting) | Layer 3 — one real GH solve ([Rhino harness](rhino_harness.md)) |

The backend's own correctness is **not** re-tested here — that would duplicate
the backend suite. A thin adapter over a tested core barely needs testing itself;
you test the core, the wiring, and (once) the one contract neither can see.

## The mechanism: a faithful fake host

`RunScript` needs `Grasshopper`, `Rhino`, `System`, `scriptcontext`,
`ghpythonlib`, and `compas_ghpython` — none of which exist outside Rhino. The
harness fakes **only** those, and keeps everything else real:

| Faked (absent off-Rhino) | Kept real (the point of the test) |
|---|---|
| `Grasshopper`, `Rhino`, `System` | the whole `compas_fab.backends.tesseract` backend |
| `scriptcontext.sticky` (a plain dict) | the component's own `code.py` |
| `ghpythonlib.treehelpers.list_to_tree` | native `tesseract_robotics` objects |
| `compas_ghpython.error` (captured into a list) | the one-joint URDF/SRDF fixtures |

The load helper execs the source and injects a fake `ghenv`:

```python
grasshopper = ModuleType("Grasshopper")
grasshopper.Kernel = SimpleNamespace(GH_ScriptInstance=object)
compas_ghpython = ModuleType("compas_ghpython")
compas_ghpython.error = lambda component, message: errors.append(message)  # capture, don't print
...
exec(compile(source.read_text(), str(source), "exec"), namespace)
namespace["ghenv"] = SimpleNamespace(Component=_FakeComponent(connections))
component = namespace[class_name]()
```

The **crux is connection state.** Optional inputs are read through
`optional_connected_input`, which consults `Params.Input[].SourceCount` — the
wire, not Python truthiness. The fake parameter models exactly that, so a test
can unwire an input *independently of the value it passes*:

```python
class _FakeParameter:
    def __init__(self, name, connected):
        self.SourceCount = 1 if connected else 0   # this, not the value, is authority
        self.PersistentDataCount = 0
```

```text
 test ─▶ _load(fake Grasshopper/Rhino/… + FakeComponent(connections))
            │  exec code.py, inject ghenv
            ▼
      component.RunScript(real inputs)
            │  reads SourceCount for optional inputs
            ▼
      REAL compas_fab.backends.tesseract  ──▶ native object / raised TesseractBackendError
            │
            ▼
      assert against a direct backend call   +   assert captured error() + None-shape
```

## The four guarantees, from the real suite

### 1. Argument translation — equivalence, not string match

Assert the native object equals what the wrapped backend factory produces for the
same inputs. A wrong arg order or dropped translation fails *structurally*:

```python
target = component.RunScript(positions, names, "LINEAR", "welding")
expected = joint_target_from_native(
    NativeJointPositions.build(positions),
    NativeJointNames.build(names, len(positions)),
    move_type_from_name("LINEAR"),
    "welding",
)
_assert_joint_targets_equal(target, expected)
```

### 2. Connection-state — the wire selects the default, not the value

The gem of the whole approach: a **non-empty value on an unwired input** must be
ignored in favour of the documented default.

```python
def test_pose_runscript_unconnected_working_frame_defaults_to_base_link(monkeypatch):
    component, errors, _, _ = _load(
        monkeypatch, "Cf_TesseractPose", _connections("Cf_TesseractPose", working_frame=False)
    )
    # value is supplied but the port is unwired → "base_link", not the value.
    pose = component.RunScript(_USER_FRAME, 0.001, "ignored_because_unconnected")
    assert pose.working_frame == "base_link"
```

### 3. Error surfacing — `error()` called, exact `None`-shape returned

An invalid input drives the backend to raise; the adapter must surface it and
return its declared `None`-shape — a bare `None` for one output, an N-tuple for N:

```python
pose = component.RunScript(_USER_FRAME, 0.0, "flange")
assert pose is None
assert errors == ["metres_per_user_unit must be an explicit finite positive value, got 0.0."]

# MotionProgram declares four outputs → the failure shape is a 4-None tuple:
assert component.RunScript(tesseract_robot, [JointTarget([0.0])], "no_such_group", "", "base", "") == (None, None, None, None)
```

### 4. Output arity — declared vs returned

One parametrized test pins every component's success and failure arity against
its `metadata.json` `outputParameters`, so a component and its declared ports can
never silently disagree.

!!! tip "This is what makes it *intelligent*, not just green"
    Several tests are written to fail on a plausible mistake, not just to pass on
    the happy path: the Descartes test wires `sample_min`/`sample_max` to their
    **own** native slots so a swapped wiring inverts the comparison and is caught;
    the RAPID-profile test asserts each variable lands under its **own**
    type-tagged slot so a `tool`↔`wobj` swap surfaces. Adversarial inputs, real
    backend, structural assertions.

## Do not reinvent compas tooling

There is no GH-component *test* harness in `compas_ghpython`/`compas_rhino`, so
the fake host above is genuinely ours. But the *dev* workflow is compas's:
reload with `compas_rhino.unload_modules("compas_fab")`, hot-reload with
`compas_rhino.devtools.DevTools.enable_reloader()`, and defer canvas mutations
with `compas_ghpython.timer.update_component` — see [Rhino harness](rhino_harness.md).

## Adding a test for a new component

1. Add the component's directory → class name to the `_CLASS` map.
2. `_load(monkeypatch, "Cf_YourComponent", _connections("Cf_YourComponent"))`.
3. Assert `RunScript(...)` equals a direct call to the backend factory it wraps.
4. Toggle each optional input with `_connections(..., your_input=False)` and
   assert the documented default is applied.
5. Drive one invalid input and assert `errors == [<message>]` plus the exact
   `None`-shape.
6. If the component declares > 1 output, add it to `_GUARD_NONE_SHAPE`.

## Coverage limits (documented, not hidden)

`Cf_TesseractNativePlan`, `Cf_TesseractRobotArtifact`, and `Cf_TesseractPlanner`
run heavy backend paths (native compile, planning, a live client). They are
tested at the reachable wiring boundary — guard clauses, cache eviction via
`sticky`, the real client build and isolated-clone exposure — while deep
execution (an actual plan solve, a full artifact compile) is left to the backend
suite and the installed examples. The limits are stated in the module docstring
rather than papered over.
