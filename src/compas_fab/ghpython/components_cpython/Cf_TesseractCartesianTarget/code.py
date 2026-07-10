# r: compas_fab>=2.0.1
# r: tesseract-robotics-nanobind>=0.35.0.6,<0.36
"""Build an exact native CartesianTarget from an exact native Pose.

No geometry conversion, scaling, frame transform, or move-type reduction occurs
at this boundary.

COMPAS FAB v2.0.1
"""

import Grasshopper
import Rhino
import System
from compas_ghpython import error

from compas_fab.backends.tesseract.errors import TesseractBackendError
from compas_fab.backends.tesseract.native_targets import build_cartesian_target
from compas_fab.backends.tesseract.native_targets import move_type_from_name
from compas_fab.ghpython import ensure_value_list

_MOVE_TYPES = ["FREESPACE", "LINEAR", "CIRCULAR"]


class TesseractCartesianTargetComponent(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, pose, move_type: str, profile: str):
        ensure_value_list(
            ghenv.Component,  # noqa: F821
            "move_type",
            _MOVE_TYPES,
            default="FREESPACE",
        )
        if pose is None:
            return None

        try:
            return build_cartesian_target(
                pose,
                move_type_from_name(move_type or "FREESPACE"),
                profile or "DEFAULT",
            )
        except TesseractBackendError as backend_error:
            error(ghenv.Component, str(backend_error))  # noqa: F821
            return None
