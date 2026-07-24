# r: compas_fab>=2.0.1
# r: abb-robot-client
"""Connect to an ABB controller and expose a reusable, sticky-cached RWS session.

Resolves the addressing triple (endpoint, RobotWare version, credential handle)
and builds a synchronous ``abb_robot_client`` ``RWS`` through a sticky cache keyed
to this component, so a Grasshopper recompute reuses the same session instead of
reconnecting. Changing endpoint, version, or handle closes the old session and
builds a fresh one. The session object is passed straight to the READ and
mutation components, which call its methods directly.

The credential handle is a NAME (or "default"), never a secret. Passwords are
resolved at connect time from the environment and never reach the canvas, the
document, or the summary output.

COMPAS FAB v2.0.1
"""

import Grasshopper
import Rhino
import System
from abb_robot_client.rws import ABBException
from compas_ghpython import create_id
from compas_ghpython import error
from scriptcontext import sticky

from compas_fab.backends.abb.connection import cached_session
from compas_fab.backends.abb.errors import AbbControllerError
from compas_fab.ghpython import ensure_value_list
from compas_fab.ghpython.input_semantics import optional_connected_input

# RobotStudio virtual controller on the local host: the zero-wiring dev default
# applied when the endpoint input is left unconnected.
_DEFAULT_ENDPOINT = "http://127.0.0.1:80"
# RobotWare versions offered on the auto-created dropdown; RW6 is the default.
_RW_VERSIONS = ["RW6", "RW7"]
_DEFAULT_RW_VERSION = "RW6"
# Human-readable stand-in for the zero-config (unconnected/blank) handle: the
# session factory still receives None and resolves the RobotStudio VC defaults.
_DEFAULT_CREDENTIAL_HANDLE = "default"
# Per-component sticky slot the cached session is stored under across solves.
_SESSION_SLOT = "abb_session"


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
            # the factory (RobotStudio VC defaults) but name it "default" in the
            # summary so the safe handle name is always shown, never a secret.
            stripped_handle = connected_handle.strip() if connected_handle else ""
            handle = stripped_handle or None

            rws = cached_session(
                sticky,
                create_id(ghenv.Component, _SESSION_SLOT),  # noqa: F821
                resolved_endpoint,
                resolved_rw_version,
                handle,
            )
            # Handle NAME only; the resolved password is never surfaced here.
            summary = "endpoint={}  rw_version={}  credential_handle={}".format(
                resolved_endpoint,
                resolved_rw_version,
                handle or _DEFAULT_CREDENTIAL_HANDLE,
            )
            return (rws, summary)
        except (ABBException, AbbControllerError) as controller_error:
            error(ghenv.Component, str(controller_error))  # noqa: F821
            return (None, None)
