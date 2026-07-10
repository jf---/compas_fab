# r: compas_fab>=2.0.1
# r: tesseract-robotics-nanobind>=0.35.0.6,<0.36
"""Build an exact native StateTarget with optional native dynamics.

Unconnected velocities, accelerations, and time remain absent. The component
never creates zeros to make a partial state appear complete.

COMPAS FAB v2.0.1
"""

import Grasshopper
import Rhino
import System
from compas_ghpython import error

from compas_fab.backends.tesseract.errors import TesseractBackendError
from compas_fab.backends.tesseract.native_quantities import NativeJointAccelerations
from compas_fab.backends.tesseract.native_quantities import NativeJointNames
from compas_fab.backends.tesseract.native_quantities import NativeJointPositions
from compas_fab.backends.tesseract.native_quantities import NativeJointVelocities
from compas_fab.backends.tesseract.native_quantities import NativeTime
from compas_fab.backends.tesseract.native_targets import move_type_from_name
from compas_fab.backends.tesseract.native_targets import state_target_from_native
from compas_fab.ghpython import ensure_value_list
from compas_fab.ghpython.input_semantics import optional_connected_input

_MOVE_TYPES = ["FREESPACE", "LINEAR", "CIRCULAR"]


class TesseractStateTargetComponent(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(
        self,
        positions,
        joint_names,
        velocities,
        accelerations,
        time: float,
        move_type: str,
        profile: str,
    ):
        ensure_value_list(
            ghenv.Component,  # noqa: F821
            "move_type",
            _MOVE_TYPES,
            default="FREESPACE",
        )
        if positions is None:
            return None

        try:
            connected_names = optional_connected_input(ghenv.Component, "joint_names", joint_names)  # noqa: F821
            connected_velocities = optional_connected_input(ghenv.Component, "velocities", velocities)  # noqa: F821
            connected_accelerations = optional_connected_input(ghenv.Component, "accelerations", accelerations)  # noqa: F821
            connected_time = optional_connected_input(ghenv.Component, "time", time)  # noqa: F821
            connected_move_type = optional_connected_input(ghenv.Component, "move_type", move_type)  # noqa: F821
            connected_profile = optional_connected_input(ghenv.Component, "profile", profile)  # noqa: F821
            native_positions = NativeJointPositions.build(positions)
            size = len(native_positions.values)
            native_names = None if connected_names is None else NativeJointNames.build(connected_names, size)
            native_velocities = None if connected_velocities is None else NativeJointVelocities.build(connected_velocities)
            native_accelerations = None if connected_accelerations is None else NativeJointAccelerations.build(connected_accelerations)
            native_time = None if connected_time is None else NativeTime.build(connected_time)
            return state_target_from_native(
                native_positions,
                native_names,
                native_velocities,
                native_accelerations,
                native_time,
                move_type_from_name("FREESPACE" if connected_move_type is None else connected_move_type),
                "DEFAULT" if connected_profile is None else connected_profile,
            )
        except TesseractBackendError as backend_error:
            error(ghenv.Component, str(backend_error))  # noqa: F821
            return None
