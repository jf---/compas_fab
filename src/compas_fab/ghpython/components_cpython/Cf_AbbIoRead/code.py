# r: compas_fab>=2.0.1
# r: abb-robot-client
"""Read a digital or analog I/O signal from an ABB controller.

A pure observe component: the read runs through the owner's serialized worker
thread and never sets a signal. The ``analog`` toggle routes the read to
get_analog_io (a float) versus get_digital_io (an int); it defaults to digital.

COMPAS FAB v2.0.1
"""

import Grasshopper
import Rhino
import System
from compas_ghpython import error

from compas_fab.backends.abb.errors import AbbControllerError
from compas_fab.ghpython import ensure_boolean_toggle
from compas_fab.ghpython.input_semantics import optional_connected_input


class AbbIoReadComponent(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, controller, signal: str, analog: bool):
        ensure_boolean_toggle(
            ghenv.Component,  # noqa: F821
            "analog",
            default=False,
        )
        if controller is None or not signal or not signal.strip():
            return None

        resolved_signal = signal.strip()
        # Unconnected/False -> digital read; only a wired True routes to analog.
        read_analog = bool(optional_connected_input(ghenv.Component, "analog", analog))  # noqa: F821

        try:
            if read_analog:
                return controller.submit_read(lambda session: session.get_analog_io(resolved_signal))
            return controller.submit_read(lambda session: session.get_digital_io(resolved_signal))
        except AbbControllerError as controller_error:
            error(ghenv.Component, str(controller_error))  # noqa: F821
            return None
