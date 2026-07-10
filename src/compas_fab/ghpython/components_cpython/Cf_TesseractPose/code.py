# r: compas_fab>=2.0.1
# r: tesseract-robotics-nanobind>=0.35.0.6,<0.36
"""Convert a Rhino Plane or COMPAS Frame to an exact native metre Pose.

The explicit scale is the only geometry-unit conversion in the native planning
graph. The pose remains relative to the working frame later selected by the
Tesseract Motion Program component.

COMPAS FAB v2.0.1
"""

import Grasshopper
import Rhino
import System
from compas.geometry import Frame
from compas_ghpython import error
from compas_rhino.conversions import plane_to_compas_frame

from compas_fab.backends.tesseract.errors import TesseractBackendError
from compas_fab.backends.tesseract.native_pose import pose_from_user_frame


class TesseractPoseComponent(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, frame, metres_per_user_unit: float):
        if frame is None:
            return None

        try:
            compas_frame = frame if isinstance(frame, Frame) else plane_to_compas_frame(frame)
            return pose_from_user_frame(
                compas_frame,
                metres_per_user_unit,
            )
        except TesseractBackendError as backend_error:
            error(ghenv.Component, str(backend_error))  # noqa: F821
            return None
