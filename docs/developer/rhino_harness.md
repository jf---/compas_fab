# Driving Rhino 8 from the CLI (rhinocode harness)

**BLUF.** Rhino 8 ships a CLI, `rhinocode`, that runs Python **in a live Rhino
instance's real CPython 3.9.10 interpreter** — RhinoCommon and Grasshopper
loaded. That is what makes Grasshopper behaviour (item/list/tree access lifting,
real `DataTree` solving) testable outside the GUI. It is not obvious to drive,
so this page is the working recipe and the three gotchas that cost real time.

The harness scripts live in `scripts/rhino_harness/`.

## The working recipe

```bash
RHINOCODE="/Applications/Rhino 8.app/Contents/Resources/bin/rhinocode"

# 1. Find the running instance id (Rhino must already be open).
"$RHINOCODE" list
#        PID ID                             DOC   PATH
#      17707 rhinocode_remotepipe_17707

# 2. Submit a REAL .py file. Returns exit 0 immediately (submit, not finish).
"$RHINOCODE" -r rhinocode_remotepipe_17707 script /abs/path/to/probe.py

# 3. The script writes its result to a shared file; wait for it, then read.
timeout 20 bash -c 'until [ -f /Users/Shared/rh_result.json ]; do :; done'
cat /Users/Shared/rh_result.json
```

The script itself reports back by **writing a file**, never by returning:

```python
import json
with open("/Users/Shared/rh_result.json", "w") as handle:
    json.dump({"ok": True}, handle)
```

## The three gotchas

!!! warning "Execution is asynchronous — `exit 0` means *submitted*, not *done*"
    `rhinocode script` hands the script to Rhino's pipe server and returns
    immediately with exit code 0. Rhino runs it a beat later on its main
    thread. Checking the result file on the next line finds nothing — the
    script has not run yet. **Always bounded-wait** for the result
    (`until [ -f <file> ]`), never assume completion from the exit code.

!!! warning "stdout does not come back — use a result file"
    A script's `print(...)` goes to **Rhino's own console**, not to the CLI's
    stdout. The CLI stays silent even when the script ran perfectly. The only
    back-channel that reaches your shell is the **filesystem**. Write JSON to a
    world-writable path both sides can see — `/Users/Shared/` and `/private/tmp/`
    both work; Rhino is not sandboxed (`os.path.expanduser("~")` is the real
    `/Users/<you>`).

!!! warning "The path must be a real `.py` file"
    rhinocode picks the language from the file **extension**. A process
    substitution (`script <(echo '...')`) resolves to `/dev/fd/NN` with no
    extension and fails with
    `CodeLanguageNotFoundException: Can not determine language for ...`. Write a
    real `*.py` file to disk and pass its path.

!!! note "Each run is a fresh engine"
    Rhino runs each submitted script with `ResetEngine` — a clean `sys.modules`
    every time. State does not persist between `script` invocations. (For a warm
    loop, see *Script listener* below.)

## The Rhino 8 CPython environment

Probed via `scripts/rhino_harness/probe_env.py`:

| Item | Value |
|---|---|
| Interpreter | CPython **3.9.10** (Rhino product target) |
| `compas` | 2.15.1 |
| `numpy` | 2.0.2 |
| RhinoCommon / Grasshopper / System (.NET) | import cleanly |

!!! warning "Two version mismatches block running *our* backend as-is"
    - **compas_fab**: Rhino resolves `compas_fab 1.0.2` from a *different*
      checkout (`~/Code/CADCAM/compas_fab`) via an **editable-install metapath
      finder**. That finder **outranks `sys.path.insert(0, ".../compas_fab_2/src")`**
      — the prepend does not win, so importing our branch code needs the editable
      install repointed at `compas_fab_2` (or the finder evicted), not just a path
      tweak.
    - **tesseract_robotics**: Rhino ships **0.34.1.6**; this repo pins
      **0.35.0.6**. Behavioural tests that construct native targets must account
      for the delta (or install 0.35.0.6 into Rhino's CPython).

    Pure Grasshopper access-lifting tests (does `scriptParamAccess=1` feed one
    vector per branch?) exercise GH's solver and do **not** need our backend or
    tesseract — prefer those for W1's lifting proof; defer real-backend runs
    until the env is reconciled.

## Getting source into Rhino — `~/.rhinocode/python-3.pth`

Rhino's **official** persistent Python 3 module-path config. One filesystem path
per line; each is inserted into `sys.path` at interpreter startup (the file
accepts paths only, never executable `.pth` code). This is the robust "editable
install" without Rhino's package machinery — imports point straight at the Git
working tree, so edits are visible immediately. Restart Rhino once after editing.

```bash
mkdir -p ~/.rhinocode
target="$HOME/Code/CADCAM/compas_fab_2/src"
grep -qxF "$target" ~/.rhinocode/python-3.pth 2>/dev/null || echo "$target" >> ~/.rhinocode/python-3.pth
```

Equivalent UI: *ScriptEditor → Tools → Options → Python 3 → Module Search Paths*
(stored in the same file). Do **not** `pip install -e` into Rhino's generated
env — the path file is simpler and less fragile.

!!! warning "A PEP 660 editable install outranks `python-3.pth`"
    This machine already carries
    `~/.rhinocode/py39-rh8/lib/python3.9/site-packages/__editable__.compas_fab-1.0.2.pth`
    — an editable pointing at the **old** `~/Code/CADCAM/compas_fab` (v1.0.2).
    Its import-hook finder is consulted **before** `sys.path`, so a
    `python-3.pth` entry (or a runtime `sys.path.insert`) for `compas_fab_2/src`
    **loses** to it. Making branch code win means first removing that editable
    finder from the rh8 env. Which is precisely why you don't want to depend on
    running branch code inside Rhino — see below.

## Dependency strategy — keep the heavy stuff out of Rhino

Rhino's `# venv:` / `# r:` header directives are shared-interpreter package
folders, **not** real virtual environments: every script in one Rhino process
shares interpreter state and memory, so conflicting versions poison the process.
Worse, native/binary wheels (numpy, and critically **`tesseract_robotics`**) must
match Rhino's exact embedded CPython + Apple-Silicon/Intel + macOS target +
already-loaded native libs. That is the nightmare — and why Rhino ships
`tesseract_robotics 0.34.1.6` while this repo pins `0.35.0.6`.

The robust architecture, and ours: a **thin Rhino adapter over a pixi-tested
core.** Rhino-specific imports never enter the computational core; heavy/native
computation runs in the pixi env *outside* Rhino; the two communicate over an
explicit boundary (subprocess/socket/HTTP) only if they must.

!!! tip "This makes the version skew moot"
    The GH access-lifting test uses a **minimal, dependency-free** Script
    component — it exercises GH's solver slicing, not our backend — so it needs
    neither `compas_fab_2` nor `tesseract 0.35` inside Rhino. We never fight the
    editable-finder conflict or native-wheel matching. The `python-3.pth` route
    above is documented for when branch code genuinely must run in Rhino; our
    test strategy is built specifically so it doesn't have to.

## Reloading (interactive dev only)

Rhino's interpreter is long-lived and caches modules — editing source does not
reload it. The CLI harness sidesteps this entirely: each `rhinocode script` run
is a fresh engine (`ResetEngine` → fresh `sys.modules`). For interactive
ScriptEditor work, prefer unload-and-reimport (delete the package's `sys.modules`
entries, then `importlib.invalidate_caches()`) over `importlib.reload`, which
mishandles `from x import y`, live class instances, and registered handlers.
Anything that registers with Rhino (Eto windows, document handlers, timers)
needs explicit `start()`/`stop()` teardown; restarting Rhino is the only truthful
reset for those.

## Test boundary — keep backend logic out of Rhino

Maintaining a Rhino dev env is genuinely hard: the interpreter's package layout
is unconventional, editable-install finders outrank `sys.path`, checkouts split
across directories, and versions skew from the repo pins (see the warning
above). **Do not port backend logic into Rhino to test it.** Draw the line by
*what owns the behaviour*:

| Behaviour under test | Where it runs | Why |
|---|---|---|
| Native target/program construction, planning, RAPID emit, profiles, length guards, `optional_connected_input` | **pixi env** (our branch, tesseract 0.35, the existing suite) | Plain Python. Rhino adds nothing but fragility. |
| Component metadata: `scriptParamAccess`, port names, icons | **pixi env** static tests | It is JSON + source; assert it directly. |
| Grasshopper access-*lifting* (does `scriptParamAccess=1` feed one vector per branch → one target per branch?) | **Rhino** | This is GH's solver slicing a `DataTree` before `RunScript`. Only a real GH solve shows it. |

The Rhino test needs a **minimal, dependency-free Script component** — it proves
GH's lifting semantics and does not import our backend or tesseract at all. That
keeps a broken Rhino env from ever blocking backend work, and keeps the one
Rhino-only test small enough to survive the env's nightmare.

## Script listener (warm loop, future)

Per-call `script` submission pays submit latency plus the async wait every time.
A **listener** — one long-running script submitted once that watches a request
directory, executes each dropped script, and writes its response — turns the
harness into a synchronous-feeling loop and keeps imports warm across calls.
Not yet built; noted so we adopt it rather than re-submitting per probe.

## Why this matters

Grasshopper lifts a component over a `DataTree` in its **solver, before**
`RunScript` is called: item access → once per item, list access → once per
branch. That behaviour is GH's, not our Python's, so it can only be observed by
making a real GH solve and reading the wires. `rhinocode` is the automatable
door to that — locally on macOS against a running instance, and on the
self-hosted Windows runner for the reproducible CI gate (same driver script).
