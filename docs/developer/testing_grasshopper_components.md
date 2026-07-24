# Testing Grasshopper components without the GUI

**BLUF.** Grasshopper components are tested in three layers, and it matters to be
exact about what each one actually does. Layers 1–2 — the large majority — run in
plain `pytest` with the Grasshopper kernel **faked**: they call `RunScript`
directly and pin the component's marshalling *logic*, but they exercise **nothing
Grasshopper's solver does**. Layer 3 (`tests/rhino/`, opt-in) runs a component in
a **real `GH_Document` solved by the actual Grasshopper kernel** via the
`rhinocode` CLI, and asserts the kernel-only behaviour — access-lifting — that
Layers 1–2 cannot see. Keep the split in mind: most of the suite covers *logic*;
a small live-Rhino suite covers *Grasshopper's execution*.

## Evidence

Every claim traces to a file, a count, or a commit.

| Claim | Evidence | Source |
|---|---|---|
| Marshalling logic tested against the real backend (kernel faked, no mocks of our code) | 42 tests | `tests/backends/tesseract/test_grasshopper_runscript_contract.py` |
| ABB read + mutation components, same fake-host method | 14 + 18 tests | `test_abb_read_components.py`, `test_abb_mutation_components.py` |
| Metadata + `scriptParamAccess` pinned statically | 8 + 17 assertions | `test_grasshopper_access.py`, `test_grasshopper_components.py` |
| Connection-state semantics (wire, not truthiness) | 8 tests | `test_grasshopper_input_semantics.py` |
| **Access-lifting proven under the ACTUAL kernel** — list access → one call per branch, item → one per item | 2 passing tests, real `GH_Document` solve | `tests/rhino/test_gh_access_lifting.py`, commit `ccd4e88f` |
| Plain RhinoCommon runs under the live interpreter (no GH) | 2 tests (exact 3-4-5 distance, analytic sphere volume) | `tests/rhino/test_rhino_python.py` |

!!! success "Confirmed"
    Layers 1–2 run green in the standard suite. The Layer-3 live-Rhino suite
    passes against an open Rhino 8: a GhPython component with a **list**-access
    input, fed a 3-branch tree, is solved by the kernel and returns one result per
    branch (`{0}=1 {1}=2 {2}=3`); the **item** variant returns one per item. That
    is Grasshopper's real solver running a real component, asserted in `pytest`.

!!! warning "The honest remaining gap"
    The Layer-3 test proves the kernel's *access-lifting mechanism* using a
    generic GhPython component. It does **not** yet run *our compiled* `Cf_*`
    components under the kernel — Rhino's Python ships an old `compas_fab` and
    lacks our branch/deps (the env skew in [Rhino harness](rhino_harness.md)). So:
    the behaviour our components rely on is proven; running *those specific
    components* end-to-end under the kernel is still gated by the env skew.

## How Grasshopper is invoked — and where it is not

Be precise about this, because it is easy to overclaim:

| Layer | How the component is invoked | Is the real GH kernel running it? |
|---|---|---|
| **1** | `exec` the `code.py`, instantiate the class, call `RunScript(...)` **as a plain Python method** under faked `Grasshopper`/`Rhino`/`scriptcontext` | **No.** The kernel is bypassed entirely — access-lifting, casting, data matching, param handling never happen. Only the marshalling logic inside `RunScript` is exercised. |
| **2** | Not invoked; `metadata.json` + source are read as data | No. |
| **3** | A component is instantiated in a real `GH_Document` and solved by Grasshopper's kernel, driven via `rhinocode` (`tests/rhino/`) | **Yes.** The kernel runs the component and lifts access; asserted in `pytest`. (Running our *compiled* `Cf_*` specifically is still env-skew-gated — see the warning above.) |

!!! note "Layer 1 is a monkeypatch — by design, and it is not the kernel"
    Every Layer-1 test calls `RunScript` **directly** with the kernel faked. That
    is deliberate: it isolates and pins the marshalling logic cheaply, in plain CI
    with no Rhino. But it covers the component's *logic*, **not** Grasshopper's
    *execution* of it — item/list/tree lifting, casting, and data matching happen
    in the kernel, which Layer 1 never touches. Those are covered separately by
    the Layer-3 live-Rhino suite above. Do not read Layer 1 as evidence the kernel
    runs the component correctly; that is Layer 3's job.

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

## Layer 3 — running a component under the real kernel

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

With those pinned, `tests/rhino/test_gh_access_lifting.py` does the real thing: a
`GH_Document`, a GhPython component whose code is `a = len(x)`, its input `x` set
to `GH_ParamAccess.list`, fed a 3-branch integer tree, and `doc.NewSolution(True)`
— Grasshopper's own solver. The output is read back and asserted:

```python
assert result["phase"] == "Computed"            # the kernel solved it
assert result["branches"] == [["1"], ["2"], ["3"]]  # list access → one call per branch, x = the branch
```

An item-access companion (`a = x + 100`) returns one result per item, proving the
contrast. `tests/rhino/test_rhino_python.py` does the same for plain RhinoCommon
(no GH). Locally these run against an open Rhino instance; the reproducible CI
gate runs the same `pytest` on the self-hosted Windows Rhino runner. The
remaining gap — running our *compiled* `Cf_*` components under the kernel — is the
env skew noted at the top, not the harness.

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

# Layer 3 — real kernel; Rhino 8 must be open. Opt-in: the default suite excludes
# tests/rhino (root conftest.pytest_ignore_collect), so pass the files explicitly:
pixi run pytest tests/rhino/test_gh_access_lifting.py tests/rhino/test_rhino_python.py -v
```

The Layer-3 fixture (`tests/rhino/conftest.py`) finds the instance via `rhinocode
list`, submits the driver, waits for its result file, and returns the JSON — and
**fails loudly** if no Rhino is running rather than skipping.

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
to the backend suite and the installed examples. And Layer 3 proves the kernel's
access-lifting with a generic component; running our *compiled* `Cf_*` components
under the kernel is still gated by the Rhino env skew. Both limits are stated
rather than papered over.
