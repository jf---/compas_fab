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
from compas_fab.ghpython.input_semantics import optional_connected_input


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
                optional_connected_input(ghenv.Component, "profile_names", profile_names),  # noqa: F821
                optional_connected_input(ghenv.Component, "enable_collision", enable_collision),  # noqa: F821
                optional_connected_input(ghenv.Component, "enable_edge_collision", enable_edge_collision),  # noqa: F821
                optional_connected_input(ghenv.Component, "num_threads", num_threads),  # noqa: F821
                optional_connected_input(ghenv.Component, "sample_axis", sample_axis),  # noqa: F821
                optional_connected_input(ghenv.Component, "sample_resolution", sample_resolution),  # noqa: F821
                optional_connected_input(ghenv.Component, "sample_min", sample_min),  # noqa: F821
                optional_connected_input(ghenv.Component, "sample_max", sample_max),  # noqa: F821
                optional_connected_input(ghenv.Component, "ik_solver", ik_solver),  # noqa: F821
                optional_connected_input(  # noqa: F821
                    ghenv.Component,
                    "use_redundant_joint_solutions",
                    use_redundant_joint_solutions,
                ),
                optional_connected_input(ghenv.Component, "move_profile", move_profile),  # noqa: F821
            )
        except TesseractBackendError as backend_error:
            error(ghenv.Component, str(backend_error))  # noqa: F821
            return None
