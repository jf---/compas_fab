# r: compas_fab>=2.0.1
# r: tesseract-robotics-nanobind==0.35.0.7
"""Run one exact native Tesseract discrete-contact request.

COMPAS FAB v2.0.1
"""

import Grasshopper
import Rhino
import System
from compas_ghpython import error

from compas_fab.backends.tesseract.collision import is_collision_distance
from compas_fab.backends.tesseract.errors import TesseractBackendError

_EMPTY = (None, None, None, None, None)


class TesseractNativeCollisionComponent(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, planner, robot_cell_state, request):
        if planner is None or robot_cell_state is None or request is None:
            return _EMPTY

        try:
            result = planner.check_collision_native(robot_cell_state, request)
        except TesseractBackendError as backend_error:
            error(ghenv.Component, str(backend_error))  # noqa: F821
            return _EMPTY

        native_contacts = result.native_results
        colliding_contacts = [native_contacts[index] for index in range(len(native_contacts)) if is_collision_distance(float(native_contacts[index].distance))]
        return (
            result,
            result.native_map,
            native_contacts,
            colliding_contacts,
            bool(colliding_contacts),
        )
