"""Probe: does our branch compas_fab backend run inside Rhino 8 CPython?

Injects the working-tree src ahead of the pip-installed compas_fab, then builds a
real native JointTarget against whatever tesseract_robotics Rhino ships. Writes
the outcome (or the exact failure) to the shared channel.
"""
import sys
import json
import traceback

REPO_SRC = "/Users/jelle/Code/CADCAM/compas_fab_2/src"
RESULT = "/Users/Shared/rh_backend.json"

sys.path.insert(0, REPO_SRC)
out = {}
try:
    import tesseract_robotics

    out["tesseract_version"] = getattr(tesseract_robotics, "__version__", "?")
    import compas_fab

    out["compas_fab_version"] = compas_fab.__version__
    out["compas_fab_file"] = compas_fab.__file__

    from compas_fab.backends.tesseract.native_quantities import NativeJointPositions
    from compas_fab.backends.tesseract.native_targets import joint_target_from_native
    from compas_fab.backends.tesseract.native_targets import move_type_from_name

    target = joint_target_from_native(
        NativeJointPositions.build([0.0, 1.0]),
        None,
        move_type_from_name("FREESPACE"),
        "DEFAULT",
    )
    out["joint_target_positions"] = list(target.positions)
    out["ok"] = True
except Exception as exc:  # noqa: BLE001
    out["error"] = repr(exc)
    out["trace"] = traceback.format_exc()

with open(RESULT, "w") as handle:
    json.dump(out, handle, indent=2)
