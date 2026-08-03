"""Our ABB backend code running inside live Rhino — real deps, real sticky.

Unlike the fake-host Layer-1 tests (which mock `scriptcontext` and never load
`abb_robot_client`), this executes our actual `compas_fab.backends.abb` code in
Rhino's CPython: our branch on `sys.path`, the `jf---` fork of `abb_robot_client`
preferred over the `# r:`-installed PyPI build (for its `RobotWareVersion`), the
unused native backends stubbed, and the **real** `scriptcontext.sticky`. It builds
a real `RWS` session (no controller connection — construction is offline) and
proves the sticky cache reuses it, all in the live interpreter.

Opt-in (the `rhino` path is excluded from the default suite); needs Rhino 8 open.
"""

# The driver runs in Rhino. The `# r:` line makes Rhino install abb_robot_client's
# pure-Python deps; the fork src is then preferred for the real API.
_DRIVER = """
# r: abb-robot-client
import sys
import json
import types
import traceback

REPO_SRC = "/Users/jelle/Code/CADCAM/compas_fab_2/src"
FORK_SRC = "/Users/jelle/Code/Robotics/abb_robot_client/src"
UNUSED = ("compas_fab.backends.ros", "compas_fab.backends.pybullet", "compas_fab.backends.kinematics")

out = {}
try:
    for cached in [m for m in list(sys.modules) if m.split(".")[0] in ("compas_fab", "abb_robot_client")]:
        del sys.modules[cached]
    for path in (FORK_SRC, REPO_SRC):
        if path not in sys.path:
            sys.path.insert(0, path)
    for name in UNUSED:
        stub = types.ModuleType(name)
        stub.__getattr__ = lambda attr: type(attr, (), {})
        stub.__path__ = []
        sys.modules[name] = stub

    from scriptcontext import sticky
    from compas_fab.backends.abb.connection import cached_session
    import abb_robot_client

    slot = "rhino_test_abb_session"
    sticky.pop(slot, None)
    first = cached_session(sticky, slot, "http://127.0.0.1:80", "RW6", None)
    second = cached_session(sticky, slot, "http://127.0.0.1:80", "RW6", None)

    out["fork_in_use"] = "Code/Robotics" in abb_robot_client.__file__
    out["session_type"] = type(first).__name__
    out["reused"] = first is second
    out["ok"] = True
    sticky.pop(slot, None)
except Exception as exc:
    out["error"] = repr(exc)
    out["trace"] = traceback.format_exc()

with open(r"__RESULT_PATH__", "w") as handle:
    json.dump(out, handle)
"""


def test_abb_connection_builds_real_rws_in_rhino(rhino_run):
    result = rhino_run(_DRIVER, timeout=120.0)

    assert result.get("ok"), result
    assert result["fork_in_use"] is True  # our fork's RWS, not the PyPI build
    assert result["session_type"] == "RWS"  # a real abb_robot_client session, constructed offline
    assert result["reused"] is True  # the sticky cache reused it across calls (real scriptcontext.sticky)
