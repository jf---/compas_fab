"""Probe Rhino 8 CPython: interpreter + which harness dependencies import.

Submitted via `rhinocode script`; writes its result JSON to the shared channel.
"""
import sys
import json

RESULT = "/Users/Shared/rh_result.json"

result = {"py": sys.version, "executable": sys.executable}
for module in ("Rhino", "Grasshopper", "System", "compas", "compas_fab", "tesseract_robotics", "numpy"):
    try:
        imported = __import__(module)
        result[module] = getattr(imported, "__version__", getattr(imported, "__file__", "imported"))
    except Exception as exc:  # noqa: BLE001
        result[module] = "ERR: " + repr(exc)

with open(RESULT, "w") as handle:
    json.dump(result, handle, indent=2)
