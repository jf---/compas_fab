# r: compas_fab>=2.0.1
# r: tesseract-robotics-nanobind==0.35.0.6
"""Run exact native Tesseract forward kinematics.

COMPAS FAB v2.0.1
"""

import Grasshopper
import Rhino
import System
from compas_ghpython import error

from compas_fab.backends.tesseract.errors import TesseractBackendError


class TesseractNativeForwardKinematicsComponent(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, planner, robot_cell_state, link_name: str, group: str):
        if planner is None or robot_cell_state is None or link_name is None:
            return None

        try:
            return planner.forward_kinematics_native(
                robot_cell_state,
                link_name,
                group,
            )
        except TesseractBackendError as backend_error:
            error(ghenv.Component, str(backend_error))  # noqa: F821
            return None
