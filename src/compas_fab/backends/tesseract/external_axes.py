"""Arm-vs-external joint classification for coordinated kinematic groups.

A coordinated (coupled) planning group spans a robot arm plus one or more external
axes -- a linear track that carries the whole robot, or a rotary positioner that
carries the workpiece. Downstream consumers need to know, for such a group, which
ordered joints form the arm and which are external, and for each external axis its
kinematic role (track vs. positioner), the unit its coordinated value is emitted in
(millimetre vs. degree), and its index in the group's ordered joint vector.

The RAPID emitter uses this layout to split the native joint vector at the arm/
external boundary and convert each external value to the right unit; the ROP/REP
Cartesian planner uses it to drive the external degrees of freedom. The layout is
derived once from the artifact's [`RobotCell`][compas_fab.robots.RobotCell].
"""

from __future__ import annotations

from enum import Enum

from attrs import define
from compas_robots.model import JointType  # type: ignore[import-untyped]

from compas_fab.robots import RobotCell

from .errors import UnknownKinematicTopologyError


class ExternalAxisRole(Enum):
    """Kinematic role an external axis plays in a coordinated group."""

    #: Prismatic external axis -- a linear rail or gantry carrying the robot.
    TRACK = "track"
    #: Revolute or continuous external axis -- a rotary table/positioner.
    POSITIONER = "positioner"


class ExternalAxisUnit(Enum):
    """Physical unit a coordinated external-axis value is emitted in."""

    #: Linear travel of a track axis.
    MILLIMETRE = "mm"
    #: Rotation of a positioner axis.
    DEGREE = "deg"


# The role -> unit contract: a track measures linear travel (mm); a positioner
# measures rotation (deg). Declared once so the pairing has a single source of
# truth and can never disagree between an axis's role and its emitted unit.
_ROLE_UNIT: dict[ExternalAxisRole, ExternalAxisUnit] = {
    ExternalAxisRole.TRACK: ExternalAxisUnit.MILLIMETRE,
    ExternalAxisRole.POSITIONER: ExternalAxisUnit.DEGREE,
}

# COMPAS joint types that classify as an external axis, and the role each implies:
# prismatic -> linear track; revolute/continuous -> rotary positioner. Any other
# configurable type (planar, floating) is an unknown topology and is rejected.
_JOINT_TYPE_ROLE: dict[JointType, ExternalAxisRole] = {
    JointType.PRISMATIC: ExternalAxisRole.TRACK,
    JointType.REVOLUTE: ExternalAxisRole.POSITIONER,
    JointType.CONTINUOUS: ExternalAxisRole.POSITIONER,
}


@define(frozen=True, slots=True)
class ExternalAxis:
    """One external (non-arm) axis of a coordinated kinematic group.

    An external axis is a configurable joint of a coupled planning group that is not
    part of the manipulator arm -- a linear track that carries the robot, or a rotary
    positioner that carries the workpiece. It carries its kinematic role, the unit its
    coordinated value is emitted in, and its position in the coupled group's ordered
    configurable-joint vector (so an emitter can split arm from external DOF).
    """

    name: str
    role: ExternalAxisRole
    unit: ExternalAxisUnit
    index: int

    @classmethod
    def build(cls, name: str, role: ExternalAxisRole, index: int) -> ExternalAxis:
        """Validate one external-axis classification.

        The emitted unit is derived from the role -- a track is millimetres, a
        positioner is degrees -- so an axis can never carry a role/unit mismatch.

        Args:
            name: Exact configurable-joint name of the external axis.
            role: Whether the axis is a linear track or a rotary positioner.
            index: Position in the coupled group's ordered configurable-joint list.

        Returns:
            Validated immutable external-axis classification.

        Raises:
            UnknownKinematicTopologyError: The name is empty or the index is negative.
        """
        if not name:
            raise UnknownKinematicTopologyError("External axis name is empty.")
        if index < 0:
            raise UnknownKinematicTopologyError("External axis '{}' has a negative index {}.".format(name, index))
        return cls(name, role, _ROLE_UNIT[role], index)


@define(frozen=True, slots=True)
class CoupledGroupLayout:
    """Arm-vs-external split of a coordinated (coupled) kinematic group.

    A coupled group spans a robot arm plus one or more external axes (a track that
    carries the robot, a positioner that carries the workpiece). This layout names the
    arm joints in order and classifies each external axis (role, unit, and its index in
    the coupled group's ordered configurable-joint vector), so a coordinated planner can
    drive the external DOF and an emitter can split the joint vector at the arm/external
    boundary.
    """

    arm_joint_names: tuple[str, ...]
    external_axes: tuple[ExternalAxis, ...]

    @classmethod
    def build(cls, robot_cell: RobotCell, coupled_group: str, manipulator_group: str) -> CoupledGroupLayout:
        """Classify the arm and external axes of a coupled planning group.

        The manipulator group's configurable joints must appear inside the coupled
        group's ordered configurable joints, in the same relative order. Every coupled
        joint that is not an arm joint is classified as an external axis by its type.

        Args:
            robot_cell: The cell whose semantics and model define both groups.
            coupled_group: Name of the coordinated group spanning arm plus external
                axes (e.g. the SRDF ``full_manipulator`` chain).
            manipulator_group: Name of the arm-only sub-group nested in the coupled
                group (e.g. the SRDF ``manipulator`` chain).

        Returns:
            The ordered arm joints and the classified external axes.

        Raises:
            UnknownKinematicTopologyError: The coupled group does not contain the
                manipulator joints, contains them out of order, or has an external
                joint whose type is neither prismatic, revolute, nor continuous.
        """
        coupled_joints = robot_cell.get_configurable_joints(coupled_group)
        arm_joint_names = tuple(robot_cell.get_configurable_joint_names(manipulator_group))

        coupled_position = {joint.name: index for index, joint in enumerate(coupled_joints)}

        missing = [name for name in arm_joint_names if name not in coupled_position]
        if missing:
            raise UnknownKinematicTopologyError("Coupled group '{}' does not contain manipulator joints {} from group '{}'.".format(coupled_group, missing, manipulator_group))

        arm_positions = [coupled_position[name] for name in arm_joint_names]
        if arm_positions != sorted(arm_positions):
            raise UnknownKinematicTopologyError(
                "Manipulator joints {} do not appear in coupled group '{}' in the same relative order.".format(list(arm_joint_names), coupled_group)
            )

        arm_names = set(arm_joint_names)
        external_axes = []
        for index, joint in enumerate(coupled_joints):
            if joint.name in arm_names:
                continue
            role = _JOINT_TYPE_ROLE.get(joint.type)
            if role is None:
                raise UnknownKinematicTopologyError("External joint '{}' in coupled group '{}' has unclassifiable type '{}'.".format(joint.name, coupled_group, joint.type))
            external_axes.append(ExternalAxis.build(joint.name, role, index))

        return cls(arm_joint_names, tuple(external_axes))
