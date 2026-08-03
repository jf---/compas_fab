# r: compas_fab>=2.0.1
# r: tesseract-robotics-nanobind==0.35.0.7
"""Run exact native Tesseract inverse kinematics without projection.

COMPAS FAB v2.0.1
"""

import Grasshopper
import Rhino
import System
from compas_ghpython import error

from compas_fab.backends.tesseract.errors import TesseractBackendError

_EMPTY = (None, None, None, None, None, None)


class TesseractNativeInverseKinematicsComponent(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, planner, target, robot_cell_state, group: str, ik_solver_name: str):
        if planner is None or target is None or robot_cell_state is None:
            return _EMPTY

        try:
            result = planner.inverse_kinematics_native(
                target,
                robot_cell_state,
                group,
                ik_solver_name or "",
            )
        except TesseractBackendError as backend_error:
            error(ghenv.Component, str(backend_error))  # noqa: F821
            return _EMPTY

        return (
            result,
            result.target_pose,
            result.native_input,
            result.group,
            list(result.joint_names),
            result.native_solutions,
        )
