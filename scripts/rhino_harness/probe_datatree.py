"""Probe: drive Grasshopper's DataTree in live Rhino via rhinocode.

Builds a 3-branch GH_Structure and reports its shape, to prove the harness can
exercise the data model the access-lifting test needs. Result → shared channel.
"""
import json
import traceback

RESULT = "/Users/Shared/rh_gh.json"
out = {}
try:
    import Grasshopper  # type: ignore
    from Grasshopper.Kernel.Data import GH_Path  # type: ignore
    from Grasshopper.Kernel.Types import GH_Integer  # type: ignore

    structure = Grasshopper.Kernel.Data.GH_Structure[GH_Integer]()
    for branch_index in range(3):
        path = GH_Path(branch_index)
        for value in range(branch_index + 1):  # branch i has i+1 items
            structure.Append(GH_Integer(value), path)

    out["gh_version"] = str(Grasshopper.Versioning.Version)
    out["path_count"] = structure.PathCount
    out["data_count"] = structure.DataCount
    out["branch_lengths"] = [structure.get_Branch(p).Count for p in structure.Paths]
    out["paths"] = [p.ToString() for p in structure.Paths]
    out["ok"] = True
except Exception as exc:  # noqa: BLE001
    out["error"] = repr(exc)
    out["trace"] = traceback.format_exc()

with open(RESULT, "w") as handle:
    json.dump(out, handle, indent=2)
