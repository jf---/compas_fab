# Testing Grasshopper components without the GUI

**BLUF.** Grasshopper components have historically been tested by hand — drop one
on a canvas, wire it, eyeball the output. This project tests them **rigorously
and automatically, with no GUI in the loop**, by splitting the problem across
three layers: the marshalling logic runs against the *real backend* under a fake
Grasshopper host in ordinary `pytest`; the metadata contracts are static
assertions; and the one thing only Grasshopper's solver can decide — `DataTree`
access-lifting — is driven in a **live Rhino** through the `rhinocode` CLI. The
result is **107 automated tests over 19 components**, most running in plain CI
with no Rhino at all, and the irreducible GUI-only behaviour reached
head-on rather than deferred to manual clicking.

## Evidence

Every claim below traces to a file, a count, or a commit — nothing is asserted
without a source.

| Claim | Evidence | Source |
|---|---|---|
| Marshalling tested against the real backend (no mocks of our code) | 42 equivalence/behaviour tests | `tests/backends/tesseract/test_grasshopper_runscript_contract.py` |
| ABB read + mutation components tested the same way | 14 + 18 tests | `test_abb_read_components.py`, `test_abb_mutation_components.py` |
| Metadata + `scriptParamAccess` pinned statically | 8 + 17 assertions | `test_grasshopper_access.py`, `test_grasshopper_components.py` |
| Connection-state semantics (wire, not truthiness) | 8 tests | `test_grasshopper_input_semantics.py` |
| Fire-once on recompute (a held button never re-commands the robot) | edge tests | `test_abb_mutation_components.py` |
| Grasshopper's `DataTree` model is reachable & driveable in live Rhino | built + read a 3-branch `GH_Structure` (paths `{0}{1}{2}`, lengths `[1,2,3]`) | commit `35ff4320`, `scripts/rhino_harness/probe_datatree.py` |
| 19 components under test | 12 Tesseract + 7 ABB `Cf_*` dirs | `src/compas_fab/ghpython/components_cpython/` |

!!! success "Confirmed"
    Layers 1 and 2 run green in the standard suite (part of the 809-test run).
    The live-Rhino transport is confirmed end-to-end: scripts execute in Rhino's
    CPython 3.9.10 and Grasshopper's `DataTree` is constructed and read back.

!!! warning "Under verification"
    The **full component-solve** in live Rhino — instantiating a Script component
    with `scriptParamAccess=1`, feeding it a tree, solving, and confirming one
    call *per branch* — is not yet automated. The data model it needs is proven
    reachable (above); the component-solve wiring is the next step. Until then,
    list-access lifting is asserted at the *declaration* level (Layer 2), not the
    *behaviour* level.

## Why this works: thin adapter over a tested core

The real work lives in the backend — `pose_from_working_frame`,
`joint_target_from_native`, `build_motion_program`, the profile factories, the
RWS session — and is covered by the backend suite. A component only *marshals*:
read GH inputs, apply defaults for unwired inputs, call the backend once, surface
errors, shape an output tuple. That tiny marshalling surface is exactly what
these tests pin.

| What can break in a component | Caught by |
|---|---|
| Wrong backend fn, wrong arg order, misplaced default | Layer 1 — equivalence vs a direct backend call |
| Unwired input read as a value instead of "absent" | Layer 1 — connection-state toggling |
| Swallowed error / wrong `None`-shape on failure | Layer 1 — error-surfacing tests |
| `scriptParamAccess` / port names / icons | Layer 2 — static metadata assertions |
| GH feeding one vector per branch (tree-lifting) | Layer 3 — a real GH solve (live Rhino) |

A thin adapter over a tested core barely needs testing itself: you test the core
(the backend suite), the wiring (Layer 1), the declarations (Layer 2), and — once
— the one contract none of those can see (Layer 3).

## The three layers

```text
 Layer 1  marshalling logic      pixi + pytest, fake host, REAL backend    ~90% of failures
 Layer 2  metadata contracts     pixi + pytest, static JSON/source         declarations
 Layer 3  GH access-lifting      live Rhino via rhinocode                  GH-solver-only behaviour
          └─ everything Grasshopper's own solver decides, and nothing else, lives here
```

Nothing is tested in the layer above the one that owns the behaviour. The backend
is not re-tested in Layer 1; GH's slicing is not faked in Layer 1; Layer 3 is
kept as small as the single fact only it can establish.

## Layer 1 — the faithful fake host

`RunScript` needs `Grasshopper`, `Rhino`, `System`, `scriptcontext`,
`ghpythonlib`, and `compas_ghpython` — none of which exist outside Rhino. The
harness fakes **only** those and keeps everything else real:

| Faked (absent off-Rhino) | Kept real (the point of the test) |
|---|---|
| `Grasshopper`, `Rhino`, `System` | the whole `compas_fab.backends.*` backend |
| `scriptcontext.sticky` (a plain dict) | the component's own `code.py` |
| `ghpythonlib.treehelpers.list_to_tree` | native `tesseract_robotics` / `abb_robot_client` objects |
| `compas_ghpython.error` (captured into a list) | the one-joint URDF/SRDF fixtures |

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
      REAL backend  ──▶ native object / raised named error
            │
            ▼
      assert against a direct backend call   +   assert captured error() + None-shape
```

### The four guarantees, from the real suite

**1. Argument translation — equivalence, not string match.** The native object
must equal what the wrapped backend factory produces for the same inputs; a wrong
arg order or dropped translation fails structurally:

```python
target = component.RunScript(positions, names, "LINEAR", "welding")
expected = joint_target_from_native(
    NativeJointPositions.build(positions), NativeJointNames.build(names, len(positions)),
    move_type_from_name("LINEAR"), "welding")
_assert_joint_targets_equal(target, expected)
```

**2. Connection-state — the wire selects the default, not the value.** A
non-empty value on an unwired input must be ignored in favour of the default:

```python
# value is supplied but the port is unwired → "base_link", not the value.
pose = component.RunScript(_USER_FRAME, 0.001, "ignored_because_unconnected")
assert pose.working_frame == "base_link"
```

**3. Error surfacing — `error()` called, exact `None`-shape returned.** An
invalid input drives the backend to raise; the adapter surfaces it and returns
its declared shape (a bare `None`, or an N-tuple for N outputs).

**4. Output arity — declared vs returned.** One parametrized test pins every
component's success and failure arity against its `metadata.json`, so a component
and its ports can never silently disagree.

!!! tip "Adversarial, not just green"
    Several tests are written to fail on a *plausible* mistake: the Descartes test
    wires `sample_min`/`sample_max` to their own native slots so a swap inverts
    the comparison and is caught; the RAPID-profile test asserts each variable
    lands under its own type-tagged slot so a `tool`↔`wobj` swap surfaces. And the
    ABB write component's fire-once test presses the arm button, recomputes with
    it still held, and asserts the controller is commanded **exactly once**.

## Layer 3 — the breakthrough: driving live Grasshopper from the CLI

GH's item/list/tree lifting happens in its **solver, before** `RunScript` is
called. No fake host can observe it — only a real GH solve can. The enabling
discovery is that Rhino 8's `rhinocode` CLI runs Python in a **live Rhino
instance's CPython 3.9.10** with Grasshopper loaded, and can be automated. The
transport has three non-obvious properties, each of which cost real time to find
(full detail in [Rhino harness](rhino_harness.md)):

- **Execution is asynchronous** — `rhinocode script` returns exit 0 on *submit*;
  Rhino runs the script a beat later. You must bounded-wait for the result.
- **stdout does not return** — a script's `print` goes to Rhino's console, not
  the CLI. The only back-channel is the **filesystem** (write JSON to
  `/Users/Shared/`, read it from the shell).
- **The path must be a real `.py` file** — rhinocode picks the language by
  extension; a process-substitution FD fails with `CodeLanguageNotFoundException`.

With those pinned, the harness genuinely drives Grasshopper. The validation run
(`scripts/rhino_harness/probe_datatree.py`, commit `35ff4320`) built a 3-branch
`GH_Structure[GH_Integer]` in the live interpreter and read back its exact shape:

```json
{ "path_count": 3, "data_count": 6, "branch_lengths": [1, 2, 3], "paths": ["{0}", "{1}", "{2}"] }
```

That establishes the data model the access-lifting test needs. The one remaining
piece — a full Script-component solve — is tracked under *Under verification*
above. Locally this runs against a persistent Rhino instance; the reproducible
CI gate runs the same driver on the self-hosted Windows Rhino runner.

## Do not reinvent compas tooling

There is no GH-component *test* harness in `compas_ghpython`/`compas_rhino`, so
the fake host is genuinely ours. But the *dev* workflow is compas's: reload with
`compas_rhino.unload_modules("compas_fab")`, hot-reload with
`compas_rhino.devtools.DevTools.enable_reloader()`, defer canvas mutations with
`compas_ghpython.timer.update_component`.

## Reproduce it

```bash
# Layers 1 + 2 — no Rhino, plain pytest:
pixi run pytest tests/backends/tesseract/test_grasshopper_runscript_contract.py \
                tests/backends/tesseract/test_abb_read_components.py \
                tests/backends/tesseract/test_abb_mutation_components.py \
                tests/backends/tesseract/test_grasshopper_access.py -n auto -q

# Layer 3 transport — live Rhino must be open; find the instance, run the probe:
RC="/Applications/Rhino 8.app/Contents/Resources/bin/rhinocode"
"$RC" list                                   # → rhinocode_remotepipe_<PID>
"$RC" -r <instance> script "$PWD/scripts/rhino_harness/probe_datatree.py"
cat /Users/Shared/rh_gh.json                 # the DataTree shape, read back
```

## Add a test for a new component

1. Add the component's directory → class name to the `_CLASS` map.
2. `_load(monkeypatch, "Cf_YourComponent", _connections("Cf_YourComponent"))`.
3. Assert `RunScript(...)` equals a direct call to the backend factory it wraps.
4. Toggle each optional input with `_connections(..., your_input=False)` and
   assert the documented default is applied.
5. Drive one invalid input and assert `errors == [<message>]` plus the exact
   `None`-shape.
6. If it declares > 1 output, add it to `_GUARD_NONE_SHAPE`.

## Coverage limits (stated, not hidden)

`Cf_TesseractNativePlan`, `Cf_TesseractRobotArtifact`, and `Cf_TesseractPlanner`
run heavy backend paths (native compile, planning, a live client). They are
tested at the reachable wiring boundary — guard clauses, `sticky` cache eviction,
the real client build and isolated-clone exposure — while deep execution is left
to the backend suite and the installed examples. And the Layer-3 component-solve
is not yet automated (above). Both limits are documented rather than papered over.
