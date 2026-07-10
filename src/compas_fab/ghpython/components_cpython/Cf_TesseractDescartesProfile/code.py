# r: compas_fab>=2.0.1
# r: tesseract-robotics-nanobind>=0.35.0.6,<0.36
"""Build an exact native ProfileDictionary for Descartes pipelines.

Every released native factory argument is exposed. Unconnected controls remain
None, so Tesseract 0.35.0.6 remains the sole authority for defaults.

COMPAS FAB v2.0.1
"""

import Grasshopper
import Rhino
import System
from compas_ghpython import error

from compas_fab.backends.tesseract.descartes_profiles import build_descartes_profiles
from compas_fab.backends.tesseract.errors import TesseractBackendError


class TesseractDescartesProfileComponent(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(
        self,
        profile_names,
        enable_collision: bool,
        enable_edge_collision: bool,
        num_threads: int,
        sample_axis,
        sample_resolution: float,
        sample_min: float,
        sample_max: float,
        ik_solver: str,
        use_redundant_joint_solutions: bool,
        move_profile,
    ):
        try:
            return build_descartes_profiles(
                profile_names or None,
                enable_collision,
                enable_edge_collision,
                num_threads,
                sample_axis or None,
                sample_resolution,
                sample_min,
                sample_max,
                ik_solver or None,
                use_redundant_joint_solutions,
                move_profile,
            )
        except TesseractBackendError as backend_error:
            error(ghenv.Component, str(backend_error))  # noqa: F821
            return None
