# r: compas_fab>=2.0.1
# r: tesseract-robotics-nanobind==0.35.0.6
"""Inspect a complete native planning result without projecting it to COMPAS.

Exact request, PlanningResult, raw program, and trajectory points remain exposed.
Convenience trees preserve missing dynamics as None rather than zeros.

COMPAS FAB v2.0.1
"""

import Grasshopper
import Rhino
import System
from compas_ghpython import error
from ghpythonlib.treehelpers import list_to_tree

from compas_fab.backends.tesseract.errors import TesseractBackendError
from compas_fab.backends.tesseract.native_result_view import TesseractNativeResultView


class TesseractNativeResultComponent(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, result):
        if result is None:
            return (None, None, None, None, None, None, None, None, None, None)

        try:
            view = TesseractNativeResultView.build(result)
        except TesseractBackendError as backend_error:
            error(ghenv.Component, str(backend_error))  # noqa: F821
            return (None, None, None, None, None, None, None, None, None, None)

        positions = list_to_tree([list(row) for row in view.positions])
        velocities = None if view.velocities is None else list_to_tree([list(row) for row in view.velocities])
        accelerations = None if view.accelerations is None else list_to_tree([list(row) for row in view.accelerations])
        times = None if view.times is None else list(view.times)
        return (
            view.request,
            view.native_result,
            view.raw_program,
            view.message,
            list(view.trajectory_points),
            list(view.joint_names),
            positions,
            velocities,
            accelerations,
            times,
        )
