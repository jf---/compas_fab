# Using a local `tesseract-robotics-nanobind` build

**BLUF.** To run a *locally-built* tesseract binding (e.g. to test a fix before it
ships) in this pixi env: build a wheel **pinned to the released version**, add it to
`[tool.pixi.pypi-dependencies]`, pin `autobahn`, then `pixi install`. Three steps —
but two traps make it a slog if you don't know them. **Never `pixi run pip install`
a wheel** — pip silently no-ops over pixi/uv-managed packages (the `.so` keeps its
old date and you waste an afternoon thinking it worked).

## The procedure

```bash
# 1. Build a self-contained wheel in the BINDING repo, pinned to the version
#    compas_fab requires (a plain build yields 0.35.0.6.devN, which uv rejects
#    against compas_fab's `==0.35.0.6` pin). --environment is REQUIRED or pixi
#    just prints help and exits 0 doing nothing.
cd ~/Code/CADCAM/tesseract_python_nanobind
SETUPTOOLS_SCM_PRETEND_VERSION=0.35.0.6 pixi run --environment default build-wheel
#    -> dist/tesseract_robotics_nanobind-0.35.0.6-cp312-abi3-macosx_15_0_arm64.whl

# 2. In THIS repo's pyproject [tool.pixi.pypi-dependencies] (NOT `pixi add` — it
#    conflicts with compas_fab's dynamic project.dependencies), add:
#      tesseract-robotics-nanobind = { path = "<abs path to the wheel>" }
#      autobahn = "==24.4.2"        # see trap 2

# 3. Install and verify.
cd ~/Code/CADCAM/compas_fab_2
pixi install
```

Verify the override actually took — the loaded `.so` mtime should be **today**, and
a no-ordered-`del` teardown must exit 0 (not 139/SIGSEGV):

```bash
find .pixi/envs/default/lib/python3.12/site-packages/tesseract_robotics \
     -name "_tesseract_kinematics*.so" -exec ls -la {} \;   # mtime = today
```

## The two traps

!!! warning "Trap 1 — version pin"
    `compas_fab` pins `tesseract-robotics-nanobind==0.35.0.6` (transitively). A
    dev wheel `0.35.0.6.devN` is **rejected as unsatisfiable**. Build the wheel
    *as* `0.35.0.6` with `SETUPTOOLS_SCM_PRETEND_VERSION=0.35.0.6` — honest, since
    it is `0.35.0.6` plus the pending patch.

!!! warning "Trap 2 — autobahn re-resolve"
    Adding any pypi dep triggers a full uv re-resolve. On macOS that resolves
    `autobahn` to the source-only `26.6.2`, which fails to build and **aborts the
    install, leaving the env broken** (`compas_fab` uninstalled). Pin
    `autobahn = "==24.4.2"` (the pure-Python wheel the lock already used) to keep
    the resolve on rails.

!!! danger "These edits are LOCAL-ONLY — do not commit"
    The absolute wheel path and the `autobahn` pin belong only in your working
    tree. `pyproject.toml` and `pixi.lock` changes for this override **must not be
    committed** (they break CI and other machines). The durable fix is the upstream
    release; drop the override once it lands.
