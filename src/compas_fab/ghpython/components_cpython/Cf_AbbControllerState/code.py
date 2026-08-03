# r: compas_fab>=2.0.1
# r: abb-robot-client
"""Read an ABB controller's state, operating mode, and RAPID execution state.

A pure observe component: it calls three synchronous RWS reads directly and never
mutates the controller. An unconnected session yields empty outputs with no
error; a controller communication failure surfaces via ``error(...)``.

COMPAS FAB v2.0.1
"""

import Grasshopper
import Rhino
import System
from abb_robot_client.rws import ABBException
from compas_ghpython import error

from compas_fab.backends.abb.errors import AbbControllerError


class AbbControllerStateComponent(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, controller):
        if controller is None:
            return (None, None, None)

        try:
            return (
                controller.get_controller_state(),
                controller.get_operation_mode(),
                controller.get_execution_state(),
            )
        except (ABBException, AbbControllerError) as controller_error:
            error(ghenv.Component, str(controller_error))  # noqa: F821
            return (None, None, None)
