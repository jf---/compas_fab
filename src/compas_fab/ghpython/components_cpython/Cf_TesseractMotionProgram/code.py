# r: compas_fab>=2.0.1
# r: tesseract-robotics-nanobind>=0.35.0.6,<0.36
"""Build exact native MotionProgram and CompositeInstruction values.

The native robot resolves group joint order and an omitted TCP. Ordered target
objects and their authored move types are retained without convenience-method
overrides.

COMPAS FAB v2.0.1
"""

import Grasshopper
import Rhino
import System
from compas_ghpython import error

from compas_fab.backends.tesseract.errors import TesseractBackendError
from compas_fab.backends.tesseract.native_program_builder import build_motion_program


class TesseractMotionProgramComponent(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(
        self,
        native_robot,
        targets,
        group_name: str,
        tcp_frame: str,
        working_frame: str,
        profile: str,
    ):
        if native_robot is None or targets is None:
            return (None, None, None, None)

        try:
            built = build_motion_program(
                native_robot,
                targets,
                group_name,
                tcp_frame or None,
                working_frame or "base_link",
                profile or "DEFAULT",
            )
        except TesseractBackendError as backend_error:
            error(ghenv.Component, str(backend_error))  # noqa: F821
            return (None, None, None, None)

        return (
            built.motion_program,
            built.composite_instruction,
            list(built.joint_names),
            built.tcp_frame,
        )
