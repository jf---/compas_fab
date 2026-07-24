# r: compas_fab>=2.0.1
# r: abb-robot-client
"""Stop RAPID execution on an ABB controller, once per button press.

A mutation component: it calls ``stop`` on the rising edge of the ``stop`` button
(via :func:`rising_edge`), so a held button or a plain recompute never re-issues
the stop. The stop is immediate -- no arming ceremony and no readback.

COMPAS FAB v2.0.1
"""

import Grasshopper
import Rhino
import System
from abb_robot_client.rws import ABBException
from compas_ghpython import error
from scriptcontext import sticky

from compas_fab.backends.abb.errors import AbbControllerError
from compas_fab.ghpython.button_edge import rising_edge

# Per-component sticky slot the stop button's last state is tracked under.
_STOP_SLOT = "stop"


class AbbStopComponent(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, controller, stop: bool):
        if controller is None:
            return None
        if not rising_edge(ghenv.Component, stop, sticky, _STOP_SLOT):  # noqa: F821
            return None

        try:
            controller.stop()
            return "stopped"
        except (ABBException, AbbControllerError) as controller_error:
            error(ghenv.Component, str(controller_error))  # noqa: F821
            return None
