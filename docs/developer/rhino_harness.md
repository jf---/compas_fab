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
