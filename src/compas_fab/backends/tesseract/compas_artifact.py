"""Typed compilation of a COMPAS robot cell into native plugin resources."""

from __future__ import annotations

from typing import Sequence

from attrs import define

from compas_fab.robots import RobotCell

from .artifact import CollisionMeshPolicy
from .artifact import ContinuousContactManager
from .artifact import DiscreteContactManager
from .artifact import KdlInverseKinematics
from .artifact import KdlKinematics
from .artifact import RobotArtifact
from .artifact_loader import RobotArtifactLoader
from .errors import InvalidKinematicsConfigError


@define(frozen=True, slots=True)
class CompasRobotArtifactCompiler:
    """Compile explicit KDL and contact plugins for selected COMPAS groups."""

    loader: RobotArtifactLoader
    collision_mesh_policy: CollisionMeshPolicy
    groups: tuple[str, ...]
    inverse_kinematics: KdlInverseKinematics
    discrete_contact_manager: DiscreteContactManager
    continuous_contact_manager: ContinuousContactManager

    @classmethod
    def build(
        cls,
        loader: RobotArtifactLoader,
        groups: Sequence[str],
        inverse_kinematics: KdlInverseKinematics,
        discrete_contact_manager: DiscreteContactManager,
        continuous_contact_manager: ContinuousContactManager,
        collision_mesh_policy: CollisionMeshPolicy = CollisionMeshPolicy.CONVEX_HULL,
    ) -> CompasRobotArtifactCompiler:
        """Validate one explicit COMPAS-to-Tesseract plugin selection.

        Args:
            loader: Captured exact URDF, SRDF, and package resolution.
            groups: Exact semantic groups that require KDL plugins.
            inverse_kinematics: Native KDL inverse solver for every selected group.
            discrete_contact_manager: Native discrete collision plugin.
            continuous_contact_manager: Native continuous collision plugin.
            collision_mesh_policy: Native collision-mesh treatment. Defaults to
                convex hulls; pass `CollisionMeshPolicy.PRESERVE` for exact
                triangle meshes.

        Returns:
            Immutable compiler.

        Raises:
            InvalidKinematicsConfigError: Groups are empty, blank, or duplicated.
        """
        selected_groups = tuple(groups)
        if not selected_groups:
            raise InvalidKinematicsConfigError("At least one COMPAS planning group must select KDL kinematics.")
        if any(not group for group in selected_groups):
            raise InvalidKinematicsConfigError("COMPAS planning-group names must not be empty.")
        if len(selected_groups) != len(set(selected_groups)):
            raise InvalidKinematicsConfigError("COMPAS planning-group selections must be unique.")
        return cls(
            loader,
            collision_mesh_policy,
            selected_groups,
            inverse_kinematics,
            discrete_contact_manager,
            continuous_contact_manager,
        )

    def compile(self, robot_cell: RobotCell) -> RobotArtifact:
        """Apply explicit plugins without replacing exact source configuration.

        Args:
            robot_cell: COMPAS projection used only to resolve selected group chains.

        Returns:
            Content-addressed Tesseract artifact with explicit native plugins.

        Raises:
            InvalidKinematicsConfigError: A group is unknown or has no joints.
            KinematicsPluginConflictError: Source SRDF already selects kinematics.
            ContactManagerPluginConflictError: Source SRDF already selects managers.
        """
        unknown = sorted(set(self.groups) - set(robot_cell.group_names))
        if unknown:
            raise InvalidKinematicsConfigError("COMPAS robot cell has no selected planning groups: {}.".format(", ".join(unknown)))
        jointless = [group for group in self.groups if not robot_cell.get_configurable_joint_names(group)]
        if jointless:
            raise InvalidKinematicsConfigError("KDL planning groups contain no configurable joints: {}.".format(", ".join(jointless)))

        configurations = [
            KdlKinematics.build(
                group,
                robot_cell.get_base_link_name(group),
                robot_cell.get_end_effector_link_name(group),
                self.inverse_kinematics,
            )
            for group in self.groups
        ]
        artifact = self.loader.load(self.collision_mesh_policy)
        artifact = artifact.with_kdl_kinematics(configurations)
        return artifact.with_contact_managers(
            self.discrete_contact_manager,
            self.continuous_contact_manager,
        )
