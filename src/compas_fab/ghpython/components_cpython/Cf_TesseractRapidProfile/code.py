# r: compas_fab>=2.0.1
# r: tesseract-robotics-nanobind==0.35.0.6
"""Bind exact Tesseract profile names to native ABB RAPID variables.

The output remains a native RapidProfile map. Each RAPID variable kind uses
the corresponding Tesseract type-tagged name; duplicate Tesseract names fail
instead of acquiring Grasshopper connection-order precedence.

COMPAS FAB v2.0.1
"""

import Grasshopper
import Rhino
import System
from compas_ghpython import error
from tesseract_robotics.emitters.rapid import RapidProfile
from tesseract_robotics.emitters.rapid.rapid_writer import SpeedName
from tesseract_robotics.emitters.rapid.rapid_writer import ToolName
from tesseract_robotics.emitters.rapid.rapid_writer import WobjName
from tesseract_robotics.emitters.rapid.rapid_writer import ZoneName

from compas_fab.backends.tesseract.errors import InvalidRapidProfileMapError
from compas_fab.backends.tesseract.errors import TesseractBackendError
from compas_fab.backends.tesseract.rapid_profiles import merge_rapid_profile_maps


def _rapid_variable_name(value, kind):
    if not isinstance(value, str) or not value.strip():
        raise InvalidRapidProfileMapError("RAPID {} variable name must be a non-empty string.".format(kind))
    return value


class TesseractRapidProfileComponent(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(
        self,
        profile_names,
        speed: str,
        zone: str,
        tool: str,
        workobject: str,
    ):
        if not profile_names:
            return None

        try:
            speed_name = speed or "v200"
            zone_name = zone or "z10"
            tool_name = tool or "tool0"
            workobject_name = workobject or "wobj0"
            profile = RapidProfile(
                speed=SpeedName(_rapid_variable_name(speed_name, "speed")),
                zone=ZoneName(_rapid_variable_name(zone_name, "zone")),
                tool=ToolName(_rapid_variable_name(tool_name, "tool")),
                wobj=WobjName(_rapid_variable_name(workobject_name, "workobject")),
            )
            profiles = merge_rapid_profile_maps([{name: profile} for name in profile_names])
        except TesseractBackendError as backend_error:
            error(ghenv.Component, str(backend_error))  # noqa: F821
            return None

        return profiles
