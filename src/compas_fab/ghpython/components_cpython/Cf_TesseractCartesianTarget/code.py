# r: compas_fab>=2.0.1
# r: tesseract-robotics-nanobind==0.35.0.6
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
from compas_fab.backends.tesseract.native_pose import WorkingFramePose
from compas_fab.backends.tesseract.native_targets import build_cartesian_target
from compas_fab.backends.tesseract.native_targets import cartesian_target_from_native
from compas_fab.backends.tesseract.native_targets import move_type_from_name
from compas_fab.ghpython import ensure_value_list
from compas_fab.ghpython.input_semantics import optional_connected_input

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
            connected_move_type = optional_connected_input(ghenv.Component, "move_type", move_type)  # noqa: F821
            connected_profile = optional_connected_input(ghenv.Component, "profile", profile)  # noqa: F821
            native_move_type = move_type_from_name("FREESPACE" if connected_move_type is None else connected_move_type)
            native_profile = "DEFAULT" if connected_profile is None else connected_profile
            if isinstance(pose, WorkingFramePose):
                return cartesian_target_from_native(
                    pose,
                    native_move_type,
                    native_profile,
                )
            return build_cartesian_target(
                pose,
                native_move_type,
                native_profile,
            )
        except TesseractBackendError as backend_error:
            error(ghenv.Component, str(backend_error))  # noqa: F821
            return None
