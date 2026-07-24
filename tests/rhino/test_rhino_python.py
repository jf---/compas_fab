"""Plain Rhino Python (RhinoCommon) under the live interpreter — no Grasshopper.

The same ``rhino_run`` fixture runs any Python in Rhino's CPython, so RhinoCommon
geometry code is exercised directly: no fake ``Rhino`` module, no `rhino3dm`
stand-in, the real ``RhinoCommon.dll``. Results are checked against exact and
analytic values.
"""

_DRIVER = """
import json
import traceback

out = {}
try:
    import Rhino
    from Rhino.Geometry import Point3d
    from Rhino.Geometry import Sphere
    from Rhino.Geometry import VolumeMassProperties

    out["rhino_version"] = str(Rhino.RhinoApp.Version)
    # Exact: a 3-4-5 right triangle in the xy-plane.
    out["distance"] = Point3d(1, 2, 3).DistanceTo(Point3d(4, 6, 3))
    # Analytic: a radius-2 sphere has volume 4/3 * pi * r**3.
    brep = Sphere(Point3d(0, 0, 0), 2.0).ToBrep()
    out["sphere_volume"] = VolumeMassProperties.Compute(brep).Volume
    out["ok"] = True
except Exception as exc:
    out["error"] = repr(exc)
    out["trace"] = traceback.format_exc()

with open(r"__RESULT_PATH__", "w") as handle:
    json.dump(out, handle)
"""


def test_rhinocommon_point_distance_is_exact(rhino_run):
    result = rhino_run(_DRIVER)

    assert result.get("ok"), result
    assert result["distance"] == 5.0  # 3-4-5, computed by RhinoCommon


def test_rhinocommon_sphere_volume_matches_analytic(rhino_run):
    import math

    result = rhino_run(_DRIVER)

    assert result.get("ok"), result
    expected = 4.0 / 3.0 * math.pi * 2.0**3
    assert abs(result["sphere_volume"] - expected) < 1e-6
