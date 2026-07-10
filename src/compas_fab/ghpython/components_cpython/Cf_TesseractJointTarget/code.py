# r: compas_fab>=2.0.1
# r: tesseract-robotics-nanobind>=0.35.0.6,<0.36
"""Build an exact native JointTarget in native joint units.

Explicit joint-name order is retained. Omitted names remain absent for the
Tesseract Motion Program component to resolve from its native robot group.

COMPAS FAB v2.0.1
"""

import Grasshopper
import Rhino
import System
from compas_ghpython import error

from compas_fab.backends.tesseract.errors import TesseractBackendError
from compas_fab.backends.tesseract.native_targets import build_joint_target
from compas_fab.backends.tesseract.native_targets import move_type_from_name
from compas_fab.ghpython import ensure_value_list

_MOVE_TYPES = ["FREESPACE", "LINEAR", "CIRCULAR"]


class TesseractJointTargetComponent(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(
        self,
        positions,
        joint_names,
        move_type: str,
        profile: str,
    ):
        ensure_value_list(
            ghenv.Component,  # noqa: F821
            "move_type",
            _MOVE_TYPES,
            default="FREESPACE",
        )
        if positions is None:
            return None

        try:
            return build_joint_target(
                positions,
                joint_names or None,
                move_type_from_name(move_type or "FREESPACE"),
                profile or "DEFAULT",
            )
        except TesseractBackendError as backend_error:
            error(ghenv.Component, str(backend_error))  # noqa: F821
            return None
