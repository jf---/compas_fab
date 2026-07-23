# r: compas_fab>=2.0.1
# r: abb-robot-client
"""Connect to an ABB controller and expose a reusable read/observe owner.

Resolves the addressing triple (endpoint, RobotWare version, credential handle)
into a content-addressed ControllerId plus a lazy session factory, then acquires
a sticky-cached ControllerOwner keyed to this component. Acquisition is lazy: no
connection is opened here, so a Grasshopper recompute stays cheap. The session
connects, and a connection error surfaces, at the first read on a downstream
component -- never on this component's solve.

The credential handle is a NAME (or "default"), never a secret. Passwords are
resolved at connect time from the environment inside the session factory and
never reach the canvas, the document, or the summary output.

COMPAS FAB v2.0.1
"""

import Grasshopper
import Rhino
import System
from compas_ghpython import create_id
from compas_ghpython import error
from scriptcontext import sticky

from compas_fab.backends.abb.arming import ControllerId
from compas_fab.backends.abb.credentials import build_controller_session_factory
from compas_fab.backends.abb.errors import AbbControllerError
from compas_fab.backends.abb.registry import acquire_owner
from compas_fab.ghpython import ensure_value_list
from compas_fab.ghpython.input_semantics import optional_connected_input

# RobotStudio virtual controller on the local host: the zero-wiring dev default
# applied when the endpoint input is left unconnected.
_DEFAULT_ENDPOINT = "http://127.0.0.1:80"
# RobotWare versions offered on the auto-created dropdown; RW6 is the default.
_RW_VERSIONS = ["RW6", "RW7"]
_DEFAULT_RW_VERSION = "RW6"
# Stable stand-in handle name for the zero-config (unconnected/blank) path: the
# ControllerId identity stays deterministic while the session factory still
# receives None and resolves the RobotStudio VC defaults.
_DEFAULT_CREDENTIAL_HANDLE = "default"
# Per-component sticky slot the acquired owner is cached under across solves.
_OWNER_SLOT = "abb_controller"


class AbbControllerComponent(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, endpoint: str, rw_version: str, credential_handle: str):
        ensure_value_list(
            ghenv.Component,  # noqa: F821
            "rw_version",
            _RW_VERSIONS,
            default=_DEFAULT_RW_VERSION,
        )

        try:
            connected_endpoint = optional_connected_input(ghenv.Component, "endpoint", endpoint)  # noqa: F821
            connected_rw_version = optional_connected_input(ghenv.Component, "rw_version", rw_version)  # noqa: F821
            connected_handle = optional_connected_input(ghenv.Component, "credential_handle", credential_handle)  # noqa: F821

            resolved_endpoint = _DEFAULT_ENDPOINT if connected_endpoint is None else connected_endpoint
            resolved_rw_version = _DEFAULT_RW_VERSION if connected_rw_version is None else connected_rw_version
            # A blank/unconnected handle is the zero-config path: keep it None for
            # the factory (RobotStudio VC defaults) but tag the identity with the
            # stable "default" name so the ControllerId stays deterministic.
            stripped_handle = connected_handle.strip() if connected_handle else ""
            handle = stripped_handle or None

            controller_id = ControllerId.build(
                resolved_endpoint,
                resolved_rw_version,
                handle or _DEFAULT_CREDENTIAL_HANDLE,
            )
            factory = build_controller_session_factory(resolved_endpoint, resolved_rw_version, handle)
            owner = acquire_owner(
                sticky,
                create_id(ghenv.Component, _OWNER_SLOT),  # noqa: F821
                controller_id,
                factory,
            )
            # Handle NAME only; the resolved password is never surfaced here.
            summary = "endpoint={}  rw_version={}  credential_handle={}".format(
                controller_id.endpoint,
                controller_id.rw_version,
                controller_id.credential_handle,
            )
            return (owner, summary)
        except AbbControllerError as controller_error:
            error(ghenv.Component, str(controller_error))  # noqa: F821
            return (None, None)
