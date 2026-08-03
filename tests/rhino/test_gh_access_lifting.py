"""Grasshopper access-lifting, proven under the ACTUAL kernel (not a monkeypatch).

A GhPython component is instantiated in a real ``GH_Document``, its input ``x``
set to a given ``GH_ParamAccess``, fed a 3-branch integer tree
(``{0}=[0] {1}=[0,1] {2}=[0,1,2]``), and solved by Grasshopper's own solver. The
output reveals how the kernel bound ``x`` per solve call:

* **list** access, ``a = len(x)`` -> each branch's output is that branch's length
  (``{0}=1 {1}=2 {2}=3``): the kernel called the component once per branch with
  ``x`` bound to the whole branch list.
* **item** access, ``a = x + 100`` -> the output mirrors the input item-for-item
  (``{0}=[100] {1}=[100,101] {2}=[100,101,102]``): the kernel called the
  component once per item with ``x`` bound to a scalar.

This is the behaviour the fake-host Layer-1 suite cannot see, because it bypasses
the kernel entirely.
"""

# The driver runs inside Rhino's CPython. It substitutes three tokens the fixture
# / test fill in: the result path, the component code, and the access mode.
_DRIVER = """
import json
import traceback

import System

out = {}
try:
    import Grasshopper
    from Grasshopper.Kernel import GH_ParamAccess
    from Grasshopper.Kernel.Data import GH_Path
    from Grasshopper.Kernel.Types import GH_Integer

    doc = Grasshopper.Kernel.GH_Document()
    server = Grasshopper.Instances.ComponentServer

    component = server.EmitObjectProxy(System.Guid("410755b1-224a-4c1e-a407-bf32fb45ea7e")).CreateInstance()
    component.Code = "__CODE__"
    component.CreateAttributes()
    doc.AddObject(component, False)

    x_param = next(p for p in component.Params.Input if p.NickName == "x")
    x_param.Access = GH_ParamAccess.__ACCESS__

    source = Grasshopper.Kernel.Parameters.Param_Integer()
    source.CreateAttributes()
    doc.AddObject(source, False)
    for branch in range(3):
        for value in range(branch + 1):
            source.PersistentData.Append(GH_Integer(value), GH_Path(branch))
    x_param.AddSource(source)

    doc.Enabled = True
    doc.NewSolution(True)

    a_data = next(p for p in component.Params.Output if p.NickName == "a").VolatileData
    out["phase"] = str(component.Phase)
    out["paths"] = [p.ToString() for p in a_data.Paths]
    out["branches"] = [[str(i) for i in a_data.get_Branch(p)] for p in a_data.Paths]
    out["ok"] = True
except Exception as exc:
    out["error"] = repr(exc)
    out["trace"] = traceback.format_exc()

with open(r"__RESULT_PATH__", "w") as handle:
    json.dump(out, handle)
"""


def _driver(code: str, access: str) -> str:
    return _DRIVER.replace("__CODE__", code).replace("__ACCESS__", access)


def test_list_access_lifts_once_per_branch(rhino_run):
    result = rhino_run(_driver("a = len(x)", "list"))

    assert result.get("ok"), result
    assert result["phase"] == "Computed"  # the kernel actually solved the component
    assert result["paths"] == ["{0}", "{1}", "{2}"]
    # Each branch's output == its length -> x was bound to the whole branch list.
    assert result["branches"] == [["1"], ["2"], ["3"]]


def test_item_access_runs_once_per_item(rhino_run):
    result = rhino_run(_driver("a = x + 100", "item"))

    assert result.get("ok"), result
    assert result["phase"] == "Computed"
    assert result["paths"] == ["{0}", "{1}", "{2}"]
    # Output mirrors the input item-for-item -> x was bound to a scalar per call.
    assert result["branches"] == [["100"], ["100", "101"], ["100", "101", "102"]]
