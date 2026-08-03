# r: compas_fab>=2.0.1
# r: abb-robot-client
"""Start RAPID execution on an ABB controller, once per button press.

A mutation component: it calls ``start`` only on the rising edge of the ``arm``
button (via :func:`rising_edge`), so a held button or a plain recompute never
restarts execution. The ``cycle`` defaults to "asis"; ``tasks`` is passed only
when wired, otherwise the RWS default task set is used.

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
from compas_fab.ghpython.input_semantics import optional_connected_input

# Default RAPID run cycle when the cycle input is left unconnected or blank.
_DEFAULT_CYCLE = "asis"
# Per-component sticky slot the arm button's last state is tracked under.
_ARM_SLOT = "start"


class AbbStartComponent(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, controller, cycle: str, tasks, arm: bool):
        if controller is None:
            return None
        if not rising_edge(ghenv.Component, arm, sticky, _ARM_SLOT):  # noqa: F821
            return None

        connected_cycle = optional_connected_input(ghenv.Component, "cycle", cycle)  # noqa: F821
        resolved_cycle = _DEFAULT_CYCLE if not connected_cycle or not connected_cycle.strip() else connected_cycle.strip()
        connected_tasks = optional_connected_input(ghenv.Component, "tasks", tasks)  # noqa: F821

        try:
            # Omit tasks when unwired so RWS applies its own default task set.
            if connected_tasks:
                controller.start(resolved_cycle, connected_tasks)
            else:
                controller.start(resolved_cycle)
            return "started {}".format(resolved_cycle)
        except (ABBException, AbbControllerError) as controller_error:
            error(ghenv.Component, str(controller_error))  # noqa: F821
            return None
