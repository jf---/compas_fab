"""Proof the GRASSHOPPER kernel — not a stub — solves our components.

The companion `.NET` proof (``test_rhino_python``) shows the interpreter is real
Rhino/.NET. This shows the *Grasshopper solver itself* is what runs a component:
a `GhPython` component whose code ``raise``s is added to a real ``GH_Document``
and solved. The kernel catches that Python exception during its solution and
re-emits it as its OWN ``GH_RuntimeMessage`` — prefixed ``"Solution exception:"``
and escalated to ``Error`` level. That wrapper is produced inside Grasshopper's
``SolveInstance`` try/catch; nothing but the real solver writes it, and we never
set the level ourselves. A monkeypatch could echo our ``ValueError`` text back —
it cannot invent Grasshopper's solver-level error handling.
"""

# Runs inside Rhino's CPython. Feeding ``x`` one item guarantees the (item-access)
# script body executes once, so the raise actually fires during the solution.
_DRIVER = """
import json
import traceback

import System

out = {}
try:
    import Grasshopper
    from Grasshopper.Kernel import GH_RuntimeMessageLevel
    from Grasshopper.Kernel.Data import GH_Path
    from Grasshopper.Kernel.Types import GH_Integer

    doc = Grasshopper.Kernel.GH_Document()
    server = Grasshopper.Instances.ComponentServer

    component = server.EmitObjectProxy(System.Guid("410755b1-224a-4c1e-a407-bf32fb45ea7e")).CreateInstance()
    component.Code = "raise ValueError('proof-of-grasshopper-kernel')"
    component.CreateAttributes()
    doc.AddObject(component, False)

    x_param = next(p for p in component.Params.Input if p.NickName == "x")
    source = Grasshopper.Kernel.Parameters.Param_Integer()
    source.CreateAttributes()
    doc.AddObject(source, False)
    source.PersistentData.Append(GH_Integer(1), GH_Path(0))
    x_param.AddSource(source)

    doc.Enabled = True
    doc.NewSolution(True)

    out["component_type"] = component.GetType().FullName
    out["proxy_count"] = server.ObjectProxies.Count
    out["phase"] = str(component.Phase)
    out["message_level"] = str(component.RuntimeMessageLevel)
    out["error_messages"] = [str(m) for m in component.RuntimeMessages(GH_RuntimeMessageLevel.Error)]
    out["ok"] = True
except Exception as exc:
    out["error"] = repr(exc)
    out["trace"] = traceback.format_exc()

with open(r"__RESULT_PATH__", "w") as handle:
    json.dump(out, handle)
"""


def test_kernel_surfaces_a_raise_as_grasshopper_error(rhino_run):
    result = rhino_run(_DRIVER)

    assert result.get("ok"), result
    # The kernel ran a full solution on the component...
    assert result["phase"] == "Computed"
    # ...instantiated from the real GhPython .NET type, out of a fully loaded
    # ComponentServer (a live Grasshopper carries thousands of proxies):
    assert result["component_type"] == "GhPython.Component.ZuiPythonComponent"
    assert result["proxy_count"] > 1000
    # ...and, catching our raise, escalated to Error and wrapped it in the
    # solver's own "Solution exception:" message — the unforgeable part.
    assert result["message_level"] == "Error"
    assert result["error_messages"] == ["Solution exception:proof-of-grasshopper-kernel"]
