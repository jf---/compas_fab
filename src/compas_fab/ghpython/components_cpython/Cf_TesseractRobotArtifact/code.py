# r: compas_fab>=2.0.1
# r: tesseract-robotics-nanobind>=0.35.0.6,<0.36
"""Build an exact, content-addressed Tesseract robot artifact.

The component keeps URDF/SRDF text and every file in each referenced package.
All generated KDL and collision plugins are visible inputs; no native selection
is silently substituted. The artifact output remains available for direct use
with the Tesseract nanobind API.

COMPAS FAB v2.0.1
"""

from pathlib import Path

import Grasshopper
import Rhino
import System
from compas_ghpython import error

from compas_fab.backends.tesseract.artifact import CollisionMeshPolicy
from compas_fab.backends.tesseract.artifact import ContinuousContactManager
from compas_fab.backends.tesseract.artifact import DiscreteContactManager
from compas_fab.backends.tesseract.artifact import KdlInverseKinematics
from compas_fab.backends.tesseract.artifact_loader import ResourceRoot
from compas_fab.backends.tesseract.artifact_loader import RobotArtifactLoader
from compas_fab.backends.tesseract.compas_artifact import CompasRobotArtifactCompiler
from compas_fab.backends.tesseract.errors import InvalidTesseractSelectionError
from compas_fab.backends.tesseract.errors import TesseractBackendError
from compas_fab.ghpython import ensure_value_list

_COLLISION_MESH_POLICIES = {
    "preserve": CollisionMeshPolicy.PRESERVE,
    "convex_hull": CollisionMeshPolicy.CONVEX_HULL,
}
_INVERSE_KINEMATICS = {
    "kdl_lma": KdlInverseKinematics.LMA,
    "kdl_newton_raphson": KdlInverseKinematics.NEWTON_RAPHSON,
}
_DISCRETE_CONTACT_MANAGERS = {
    "bullet_bvh": DiscreteContactManager.BULLET_BVH,
    "bullet_simple": DiscreteContactManager.BULLET_SIMPLE,
    "fcl_bvh": DiscreteContactManager.FCL_BVH,
}
_CONTINUOUS_CONTACT_MANAGERS = {
    "bullet_cast_bvh": ContinuousContactManager.BULLET_CAST_BVH,
    "bullet_cast_simple": ContinuousContactManager.BULLET_CAST_SIMPLE,
}


def _selection(options, value, default, label):
    name = (value or default).strip().lower()
    selection = options.get(name)
    if selection is None:
        raise InvalidTesseractSelectionError("Unknown {} {!r}; expected one of {}.".format(label, name, ", ".join(sorted(options))))
    return selection


class TesseractRobotArtifactComponent(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(
        self,
        robot_cell,
        urdf_filename: str,
        srdf_filename: str,
        resource_roots,
        planning_groups,
        collision_mesh_policy: str,
        inverse_kinematics: str,
        discrete_contact_manager: str,
        continuous_contact_manager: str,
    ):
        # Each native choice gets its own visible canvas control. Changing one
        # input causes a different content-addressed artifact identity.
        ensure_value_list(ghenv.Component, "collision_mesh_policy", _COLLISION_MESH_POLICIES, default="convex_hull")  # noqa: F821
        ensure_value_list(ghenv.Component, "inverse_kinematics", _INVERSE_KINEMATICS, default="kdl_lma")  # noqa: F821
        ensure_value_list(ghenv.Component, "discrete_contact_manager", _DISCRETE_CONTACT_MANAGERS, default="bullet_bvh")  # noqa: F821
        ensure_value_list(ghenv.Component, "continuous_contact_manager", _CONTINUOUS_CONTACT_MANAGERS, default="bullet_cast_bvh")  # noqa: F821

        if robot_cell is None or not urdf_filename or not srdf_filename:
            return (None, None)

        try:
            roots = [ResourceRoot.build(Path(str(root))) for root in (resource_roots or []) if root]
            groups = [str(group) for group in (planning_groups or []) if group]
            if not groups:
                groups = [robot_cell.main_group_name]
            loader = RobotArtifactLoader.build(
                Path(urdf_filename),
                Path(srdf_filename),
                roots,
            )
            compiler = CompasRobotArtifactCompiler.build(
                loader=loader,
                collision_mesh_policy=_selection(
                    _COLLISION_MESH_POLICIES,
                    collision_mesh_policy,
                    "convex_hull",
                    "collision mesh policy",
                ),
                groups=groups,
                inverse_kinematics=_selection(
                    _INVERSE_KINEMATICS,
                    inverse_kinematics,
                    "kdl_lma",
                    "inverse kinematics",
                ),
                discrete_contact_manager=_selection(
                    _DISCRETE_CONTACT_MANAGERS,
                    discrete_contact_manager,
                    "bullet_bvh",
                    "discrete contact manager",
                ),
                continuous_contact_manager=_selection(
                    _CONTINUOUS_CONTACT_MANAGERS,
                    continuous_contact_manager,
                    "bullet_cast_bvh",
                    "continuous contact manager",
                ),
            )
            artifact = compiler.compile(robot_cell)
        except TesseractBackendError as backend_error:
            error(ghenv.Component, str(backend_error))  # noqa: F821
            return (None, None)

        return (artifact, artifact.identity.digest)
