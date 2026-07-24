# r: compas_fab>=2.0.1
# r: abb-robot-client
"""Reset the RAPID program pointer to main on an ABB controller, once per press.

A mutation component: it calls ``resetpp`` only on the rising edge of the ``arm``
button (via :func:`rising_edge`), so a held button or a plain recompute never
re-resets the program pointer.

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

# Per-component sticky slot the arm button's last state is tracked under.
_ARM_SLOT = "reset"


class AbbResetComponent(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, controller, arm: bool):
        if controller is None:
            return None
        if not rising_edge(ghenv.Component, arm, sticky, _ARM_SLOT):  # noqa: F821
            return None

        try:
            controller.resetpp()
            return "pp reset to main"
        except (ABBException, AbbControllerError) as controller_error:
            error(ghenv.Component, str(controller_error))  # noqa: F821
            return None
