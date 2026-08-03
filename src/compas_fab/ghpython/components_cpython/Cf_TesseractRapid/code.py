# r: compas_fab>=2.0.1
# r: tesseract-robotics-nanobind==0.35.0.7
"""Emit ABB RAPID from one exact native CompositeInstruction.

Tesseract remains the instruction dispatcher, unit converter, and formatter.
This component is pure: it returns a content-addressed artifact and source but
never writes a file, unwraps a planning result, or projects a trajectory.

COMPAS FAB v2.0.1
"""

import Grasshopper
import Rhino
import System
from compas_ghpython import error
from tesseract_robotics.emitters.rapid import RapidEmitterError

from compas_fab.backends.tesseract.errors import TesseractBackendError
from compas_fab.backends.tesseract.rapid_emitter import TesseractRapidEmitter
from compas_fab.backends.tesseract.rapid_profiles import merge_rapid_profile_maps


class TesseractRapidComponent(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(
        self,
        program,
        profile_maps,
        module_name: str,
        procedure_name: str,
    ):
        if program is None:
            return (None, None, None)

        try:
            profiles = merge_rapid_profile_maps(profile_maps or [])
            rapid_program = TesseractRapidEmitter.emit(
                program,
                profiles,
                module_name or "main_program",
                procedure_name or "main",
            )
        except (TesseractBackendError, RapidEmitterError) as emission_error:
            error(ghenv.Component, str(emission_error))  # noqa: F821
            return (None, None, None)

        return (
            rapid_program,
            str(rapid_program.source),
            str(rapid_program.identity.digest),
        )
