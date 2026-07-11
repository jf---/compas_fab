# r: compas_fab>=2.0.1
# r: tesseract-robotics-nanobind==0.35.0.6
"""Create a local Tesseract planner while retaining native access.

The client is cached by artifact identity and reused across canvas solutions.
Every native plan runs against an isolated environment clone. The native_robot
output is another isolated clone for direct nanobind workflows.

COMPAS FAB v2.0.1
"""

import Grasshopper
import Rhino
import System
from compas_ghpython import create_id
from compas_ghpython import error
from scriptcontext import sticky as st

from compas_fab.backends.tesseract.client import TesseractClient
from compas_fab.backends.tesseract.errors import InvalidTesseractSelectionError
from compas_fab.backends.tesseract.errors import TesseractBackendError
from compas_fab.backends.tesseract.planner import TesseractPlanner


class TesseractPlannerComponent(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(
        self,
        artifact,
        robot_cell,
        robot_cell_state,
        warmup_pipelines,
        warmup_all: bool,
        reload: bool,
    ):
        if artifact is None or robot_cell is None:
            return (None, None)

        try:
            pipelines = []
            for pipeline in warmup_pipelines or []:
                if not isinstance(pipeline, str) or not pipeline.strip():
                    raise InvalidTesseractSelectionError("warmup_pipelines must contain non-empty pipeline names.")
                pipelines.append(pipeline.strip())
            if warmup_all is not None and not isinstance(warmup_all, bool):
                raise InvalidTesseractSelectionError("warmup_all must be bool.")
            if warmup_all and pipelines:
                raise InvalidTesseractSelectionError("Select warmup_all or warmup_pipelines, not both.")
        except TesseractBackendError as backend_error:
            error(ghenv.Component, str(backend_error))  # noqa: F821
            return (None, None)

        warmup = True if warmup_all else pipelines if pipelines else False
        warmup_identity = (bool(warmup_all), tuple(pipelines))
        key = create_id(ghenv.Component, "tesseract_planner")  # noqa: F821
        cached = st.get(key)  # (client, planner, artifact digest, warmup identity)

        # The artifact digest is the complete environment cache key. A changed
        # robot resource or native plugin selection therefore rebuilds the client.
        rebuild = bool(reload) or cached is None or cached[2] != artifact.identity.digest or cached[3] != warmup_identity or not cached[0].is_connected
        if rebuild:
            # Disconnect the previous native lifetimes before replacing the sticky
            # entry; planner requests never cross artifact identities.
            if cached is not None:
                cached[0].disconnect()
            client = TesseractClient(artifact, warmup=warmup)
            try:
                client.connect()
            except TesseractBackendError as backend_error:
                st.pop(key, None)
                error(ghenv.Component, str(backend_error))  # noqa: F821
                return (None, None)
            planner = TesseractPlanner(client)
            cached = (client, planner, artifact.identity.digest, warmup_identity)
            st[key] = cached

        client, planner = cached[0], cached[1]
        try:
            # RobotCell is the conventional COMPAS projection. Reapplying it on
            # every solve keeps current tools/state visible without rebuilding the
            # immutable native environment artifact.
            planner.set_robot_cell(robot_cell, robot_cell_state)
        except TesseractBackendError as backend_error:
            client.disconnect()
            st.pop(key, None)
            error(ghenv.Component, str(backend_error))  # noqa: F821
            return (None, None)

        # Expose a clone, not the planner's master environment, so direct native
        # experiments cannot mutate the cached planning baseline.
        native_robot = client.clone_robot()
        return (planner, native_robot)
