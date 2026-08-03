# r: compas_fab>=2.0.1
# r: abb-robot-client
"""Write a digital or analog I/O signal to an ABB controller, once per button press.

A mutation component: it sets a signal only on the rising edge of the ``arm``
button (via :func:`rising_edge`), so a held button or a plain recompute never
re-commands the controller. After writing it reads the signal back and compares
it to the written setpoint -- a mismatch is surfaced as an error rather than a
silent belief the write took. The ``analog`` toggle routes the write to
``set_analog_io``/``get_analog_io`` versus ``set_digital_io``/``get_digital_io``.

COMPAS FAB v2.0.1
"""

import Grasshopper
import Rhino
import System
from abb_robot_client.rws import ABBException
from compas_ghpython import error
from scriptcontext import sticky

from compas_fab.backends.abb.errors import AbbControllerError
from compas_fab.ghpython import ensure_boolean_toggle
from compas_fab.ghpython.button_edge import rising_edge
from compas_fab.ghpython.input_semantics import optional_connected_input

# Per-component sticky slot the arm button's last state is tracked under.
_ARM_SLOT = "io_write"


class AbbIoWriteComponent(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, controller, signal: str, value: float, analog: bool, arm: bool):
        ensure_boolean_toggle(
            ghenv.Component,  # noqa: F821
            "analog",
            default=False,
        )
        if controller is None:
            return (None, None)
        # The button edge is the only re-fire guard: consult it only once a
        # controller is present, so connecting one with the button already held
        # still fires on the next solve rather than being silently swallowed.
        if not rising_edge(ghenv.Component, arm, sticky, _ARM_SLOT):  # noqa: F821
            return (None, None)

        resolved_signal = (signal or "").strip()
        write_analog = bool(optional_connected_input(ghenv.Component, "analog", analog))  # noqa: F821

        try:
            if write_analog:
                written = float(value)
                controller.set_analog_io(resolved_signal, written)
                readback = controller.get_analog_io(resolved_signal)
            else:
                # Digital signals are integer-valued; a float input is coerced.
                written = int(value)
                controller.set_digital_io(resolved_signal, written)
                readback = controller.get_digital_io(resolved_signal)
            # The readback is the controller echoing the setpoint we just wrote,
            # not a physical measurement, so an exact compare is the right check.
            if readback != written:
                error(ghenv.Component, "Readback mismatch: wrote {} read {}".format(written, readback))  # noqa: F821
                return (None, None)
            return ("wrote {}={}".format(resolved_signal, written), readback)
        except (ABBException, AbbControllerError) as controller_error:
            error(ghenv.Component, str(controller_error))  # noqa: F821
            return (None, None)
