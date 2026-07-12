# r: compas_fab>=2.0.1
# r: tesseract-robotics-nanobind==0.35.0.6
"""Build one exact native Tesseract ContactRequest.

Unconnected controls retain Tesseract's native defaults.

COMPAS FAB v2.0.1
"""

import Grasshopper
import Rhino
import System
from compas_ghpython import error

from compas_fab.backends.tesseract.collision import build_contact_request
from compas_fab.backends.tesseract.errors import TesseractBackendError
from compas_fab.ghpython import ensure_value_list
from compas_fab.ghpython.input_semantics import optional_connected_input

_CONTACT_TEST_TYPES = ["FIRST", "CLOSEST", "ALL", "LIMITED"]


class TesseractContactRequestComponent(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(
        self,
        test_type: str,
        calculate_distance: bool,
        calculate_penetration: bool,
        contact_limit: int,
    ):
        ensure_value_list(
            ghenv.Component,  # noqa: F821
            "test_type",
            _CONTACT_TEST_TYPES,
            default="ALL",
        )
        try:
            return build_contact_request(
                optional_connected_input(ghenv.Component, "test_type", test_type),  # noqa: F821
                optional_connected_input(ghenv.Component, "calculate_distance", calculate_distance),  # noqa: F821
                optional_connected_input(ghenv.Component, "calculate_penetration", calculate_penetration),  # noqa: F821
                optional_connected_input(ghenv.Component, "contact_limit", contact_limit),  # noqa: F821
            )
        except TesseractBackendError as backend_error:
            error(ghenv.Component, str(backend_error))  # noqa: F821
            return None
