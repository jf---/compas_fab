# r: compas_fab>=2.0.1
# r: tesseract-robotics-nanobind==0.35.0.7
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
from compas_fab.ghpython.input_semantics import optional_connected_input


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
            connected_tcp = optional_connected_input(ghenv.Component, "tcp_frame", tcp_frame)  # noqa: F821
            connected_working_frame = optional_connected_input(ghenv.Component, "working_frame", working_frame)  # noqa: F821
            connected_profile = optional_connected_input(ghenv.Component, "profile", profile)  # noqa: F821
            built = build_motion_program(
                native_robot,
                targets,
                group_name,
                connected_tcp,
                "base_link" if connected_working_frame is None else connected_working_frame,
                "DEFAULT" if connected_profile is None else connected_profile,
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
