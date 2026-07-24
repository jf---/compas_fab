# r: compas_fab>=2.0.1
# r: tesseract-robotics-nanobind==0.35.0.7
"""Execute one exact native Tesseract request with observable cache safety.

The program is content-addressed on every solve. Profiles remain exact required
native objects; the opaque 0.35.0.6 dictionary cannot reveal same-object
mutation, so callers using that advanced path must trigger compute again.

COMPAS FAB v2.0.1
"""

import Grasshopper
import Rhino
import System
from compas_ghpython import create_id
from compas_ghpython import error
from scriptcontext import sticky as st

from compas_fab.backends.tesseract.errors import TesseractBackendError
from compas_fab.backends.tesseract.native_plan import NativePlanCall
from compas_fab.backends.tesseract.native_plan import required_native_plan_bool


class TesseractNativePlanComponent(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(
        self,
        planner,
        program,
        pipeline: str,
        profiles,
        auto_seed: bool,
        compute: bool,
    ):
        key = create_id(
            ghenv.Component,  # noqa: F821
            "tesseract_native_plan",
        )
        if planner is None or program is None or profiles is None:
            st.pop(key, None)
            return None

        try:
            should_compute = required_native_plan_bool(compute, "compute")
            call = NativePlanCall.build(
                planner,
                program,
                pipeline,
                profiles,
                auto_seed,
            )
            signature = call.signature
            cached = st.get(key)
            if not should_compute:
                if cached is not None and cached[0] == signature:
                    return cached[1]
                st.pop(key, None)
                return None

            st.pop(key, None)
            result = call.execute()
            st[key] = (signature, result)
            return result
        except TesseractBackendError as backend_error:
            st.pop(key, None)
            error(ghenv.Component, str(backend_error))  # noqa: F821
            return None
