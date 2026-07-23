# r: compas_fab>=2.0.1
# r: abb-robot-client
"""Read an ABB controller's state, operating mode, and RAPID execution state.

A pure observe component: it drives three reads through the owner's serialized
worker thread and never mutates the controller. The owner connects lazily, so
the session is established on the first read here and a connection failure
surfaces as a named AbbControllerError at that point.

COMPAS FAB v2.0.1
"""

import Grasshopper
import Rhino
import System
from compas_ghpython import error

from compas_fab.backends.abb.errors import AbbControllerError


class AbbControllerStateComponent(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, controller):
        if controller is None:
            return (None, None, None)

        try:
            controller_state = controller.submit_read(lambda session: session.get_controller_state())
            operation_mode = controller.submit_read(lambda session: session.get_operation_mode())
            execution_state = controller.submit_read(lambda session: session.get_execution_state())
            return (controller_state, operation_mode, execution_state)
        except AbbControllerError as controller_error:
            error(ghenv.Component, str(controller_error))  # noqa: F821
            return (None, None, None)
