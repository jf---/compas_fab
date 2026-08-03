"""Ordered COMPAS-to-native Tesseract scene command construction and application.

The builders translate a COMPAS ``RobotCell`` plus ``RobotCellState`` into the
native command sequence Tesseract applies to a per-plan environment clone:

1. one ``AddLinkCommand`` per registered tool and rigid body present in the
   state, mounted to the robot base link;
2. one ``MoveLinkCommand`` per attachment, reparenting the link onto its
   end-effector link, robot link, or owning tool link at the attachment frame;
3. a single ``ModifyAllowedCollisionsCommand`` adding every touch-link and
   touch-body pair.

All COMPAS frames are in metres (the compas_fab native unit); ``scale`` maps a
state frame translation to metres and is therefore ``1.0`` for an in-metres
cell. Meshes arrive pre-scaled through ``RigidBody.visual_meshes_in_meters`` and
the tool link accessors.
"""

from __future__ import annotations

from typing import Optional

from compas.geometry import Frame  # type: ignore[import-untyped]
from compas.geometry import Transformation
from compas_robots import ToolModel  # type: ignore[import-untyped]
from tesseract_robotics.planning import Pose
from tesseract_robotics.planning import Robot
from tesseract_robotics.tesseract_common import AllowedCollisionMatrix
from tesseract_robotics.tesseract_common import Isometry3d
from tesseract_robotics.tesseract_environment import AddLinkCommand
from tesseract_robotics.tesseract_environment import Command
from tesseract_robotics.tesseract_environment import Environment
from tesseract_robotics.tesseract_environment import ModifyAllowedCollisionsCommand
from tesseract_robotics.tesseract_environment import ModifyAllowedCollisionsType
from tesseract_robotics.tesseract_environment import MoveLinkCommand
from tesseract_robotics.tesseract_geometry import Geometry
from tesseract_robotics.tesseract_scene_graph import Collision
from tesseract_robotics.tesseract_scene_graph import Joint
from tesseract_robotics.tesseract_scene_graph import JointType
from tesseract_robotics.tesseract_scene_graph import Link
from tesseract_robotics.tesseract_scene_graph import Visual

from compas_fab.robots import RigidBody
from compas_fab.robots import RigidBodyState
from compas_fab.robots import RobotCell
from compas_fab.robots import RobotCellState

from .errors import NativeSceneCommandRejectedError
from .errors import UnknownAttachmentParentError
from .errors import UnregisteredSceneModelError
from .frames import MetersPerUserUnit
from .frames import WorldFrameMeters
from .frames import isometry_from_robot_frame
from .frames import robot_frame_from_world
from .frames import world_meters_frame
from .native_geometry import native_convex_collision
from .native_geometry import native_mesh
from .native_pose import pose_from_user_frame

MOUNT_JOINT_SUFFIX = "_mount_joint"
ATTACH_JOINT_SUFFIX = "_attach_joint"
TOUCH_LINK_REASON = "compas_fab touch link"
TOUCH_BODY_REASON = "compas_fab touch body"


def rigid_body_link(name: str, body: RigidBody) -> Link:
    """Build a native link from a rigid body's metre visual and collision meshes.

    Args:
        name: Native link name; the state references this as the body id.
        body: COMPAS rigid body whose meshes are returned already in metres.

    Returns:
        A native link with per-mesh visuals and convex-hull collisions.

    Raises:
        DegenerateSceneMeshError: A visual or collision mesh is empty.
    """
    link = Link(name)
    for mesh in body.visual_meshes_in_meters:
        _add_visual(link, native_mesh(mesh))
    for mesh in body.collision_meshes_in_meters:
        _add_collision(link, native_convex_collision(mesh))
    return link


def tool_link(name: str, tool: ToolModel) -> Link:
    """Collapse a tool's link meshes into a single native link in the base frame.

    Args:
        name: Native link name; the state references this as the tool id.
        tool: COMPAS tool model whose link meshes are expressed in metres in the
            tool base frame.

    Returns:
        A native link with the tool's visual and convex-hull collision geometry.

    Raises:
        DegenerateSceneMeshError: A visual or collision mesh is empty.
    """
    link = Link(name)
    for tool_link_model in tool.iter_links():
        for mesh in tool.get_link_visual_meshes(tool_link_model):
            _add_visual(link, native_mesh(mesh))
        for mesh in tool.get_link_collision_meshes(tool_link_model):
            _add_collision(link, native_convex_collision(mesh))
    return link


def fixed_joint(joint_name: str, parent_link_name: str, child_link_name: str, origin: Isometry3d) -> Joint:
    """Build a native FIXED joint whose origin is the parent-relative transform.

    Args:
        joint_name: Unique native joint name.
        parent_link_name: Name of the parent link.
        child_link_name: Name of the child link.
        origin: Parent-to-child transform in metres (``Pose`` is accepted).

    Returns:
        A native FIXED joint carrying ``parent_to_joint_origin_transform``.
    """
    joint = Joint(joint_name)
    joint.parent_link_name = parent_link_name
    joint.child_link_name = child_link_name
    joint.type = JointType.FIXED
    joint.parent_to_joint_origin_transform = origin
    return joint


def scene_commands(
    cell: RobotCell,
    state: RobotCellState,
    robot_base_frame: WorldFrameMeters,
    scale: MetersPerUserUnit,
) -> list[Command]:
    """Build the ordered native commands realizing a cell state's scene.

    The returned order is every ``AddLinkCommand``, then every
    ``MoveLinkCommand``, then one ``ModifyAllowedCollisionsCommand`` (when any
    touch pair exists). Hidden tools and rigid bodies are omitted.

    Args:
        cell: The COMPAS robot cell owning the tool and rigid-body models.
        state: The cell state naming which models are present and how attached.
        robot_base_frame: World placement of the robot base, in metres, used to
            place unattached bodies relative to the native root link.
        scale: Metres per user unit for the state's frame translations (``1.0``
            for an in-metres cell).

    Returns:
        The ordered native command list to apply to an environment clone.

    Raises:
        UnregisteredSceneModelError: A tool or body state names a model absent
            from the cell.
        UnknownAttachmentParentError: An attachment names a link, group, or tool
            that cannot be resolved.
    """
    base_link_name = cell.get_base_link_name()
    robot_link_names = set(cell.get_link_names())

    add_commands: list[Command] = []
    move_commands: list[Command] = []
    touch_pairs: list[tuple[str, str, str]] = []

    for tool_id, tool_state in state.tool_states.items():
        if tool_id not in cell.tool_models:
            raise UnregisteredSceneModelError("Tool state {!r} has no model in cell.tool_models.".format(tool_id))
        if tool_state.is_hidden:
            continue
        tool = cell.tool_models[tool_id]
        add_commands.append(
            AddLinkCommand(
                tool_link(tool_id, tool),
                fixed_joint(tool_id + MOUNT_JOINT_SUFFIX, base_link_name, tool_id, _mount_origin(tool_state.frame, robot_base_frame, scale)),
            )
        )
        if tool_state.attached_to_group is not None:
            parent_link_name = _group_end_effector_link(cell, tool_state.attached_to_group, robot_link_names)
            move_commands.append(MoveLinkCommand(fixed_joint(tool_id + ATTACH_JOINT_SUFFIX, parent_link_name, tool_id, _relative_origin(tool_state.attachment_frame, scale))))
        for touch_link in tool_state.touch_links:
            touch_pairs.append((tool_id, touch_link, TOUCH_LINK_REASON))

    for body_id, body_state in state.rigid_body_states.items():
        if body_id not in cell.rigid_body_models:
            raise UnregisteredSceneModelError("Rigid body state {!r} has no model in cell.rigid_body_models.".format(body_id))
        if body_state.is_hidden:
            continue
        body = cell.rigid_body_models[body_id]
        add_commands.append(
            AddLinkCommand(
                rigid_body_link(body_id, body),
                fixed_joint(body_id + MOUNT_JOINT_SUFFIX, base_link_name, body_id, _mount_origin(body_state.frame, robot_base_frame, scale)),
            )
        )
        move_command = _rigid_body_move_command(cell, body_id, body_state, scale, robot_link_names)
        if move_command is not None:
            move_commands.append(move_command)
        for touch_link in body_state.touch_links:
            touch_pairs.append((body_id, touch_link, TOUCH_LINK_REASON))
        for touch_body in body_state.touch_bodies:
            touch_pairs.append((body_id, touch_body, TOUCH_BODY_REASON))

    commands: list[Command] = [*add_commands, *move_commands]
    if touch_pairs:
        commands.append(_allowed_collisions_command(touch_pairs))
    return commands


def apply_scene(robot: Robot, commands: list[Command]) -> None:
    """Apply each native scene command to the robot's environment in order.

    Args:
        robot: The per-plan native robot whose environment receives the commands.
        commands: The ordered command list from ``scene_commands``.

    Raises:
        NativeSceneCommandRejectedError: The environment rejected a command or a
            command type outside the supported scene set was supplied.
    """
    environment = robot.env
    for command in commands:
        if not _apply_command(environment, command):
            raise NativeSceneCommandRejectedError("Native environment rejected scene command {}.".format(type(command).__name__))


def _apply_command(environment: Environment, command: Command) -> bool:
    """Dispatch one scene command to its typed ``applyCommand`` overload."""
    if isinstance(command, AddLinkCommand):
        return environment.applyCommand(command)
    if isinstance(command, MoveLinkCommand):
        return environment.applyCommand(command)
    if isinstance(command, ModifyAllowedCollisionsCommand):
        return environment.applyCommand(command)
    raise NativeSceneCommandRejectedError("Unsupported native scene command {}.".format(type(command).__name__))


def _rigid_body_move_command(
    cell: RobotCell,
    body_id: str,
    body_state: RigidBodyState,
    scale: MetersPerUserUnit,
    robot_link_names: set[str],
) -> Optional[MoveLinkCommand]:
    """Build the reparenting command for an attached rigid body, if attached."""
    if body_state.attached_to_link is not None:
        parent_link_name = body_state.attached_to_link
        if parent_link_name not in robot_link_names:
            raise UnknownAttachmentParentError("Rigid body {!r} attaches to unknown robot link {!r}.".format(body_id, parent_link_name))
        return MoveLinkCommand(fixed_joint(body_id + ATTACH_JOINT_SUFFIX, parent_link_name, body_id, _relative_origin(body_state.attachment_frame, scale)))
    if body_state.attached_to_tool is not None:
        tool_id = body_state.attached_to_tool
        if tool_id not in cell.tool_models:
            raise UnknownAttachmentParentError("Rigid body {!r} attaches to unregistered tool {!r}.".format(body_id, tool_id))
        origin = _tool_relative_origin(cell.tool_models[tool_id], body_state.attachment_frame, scale)
        return MoveLinkCommand(fixed_joint(body_id + ATTACH_JOINT_SUFFIX, tool_id, body_id, origin))
    return None


def _group_end_effector_link(cell: RobotCell, group: str, robot_link_names: set[str]) -> str:
    """Resolve a planning group's end-effector link name, failing loudly."""
    try:
        link_name = cell.get_end_effector_link_name(group)
    except (KeyError, ValueError) as error:
        raise UnknownAttachmentParentError("Tool attaches to group {!r} with no resolvable end-effector link: {}.".format(group, error)) from error
    if link_name not in robot_link_names:
        raise UnknownAttachmentParentError("Tool attaches to group {!r} whose end-effector link {!r} is not a robot link.".format(group, link_name))
    return link_name


def _mount_origin(world_frame: Optional[Frame], robot_base_frame: WorldFrameMeters, scale: MetersPerUserUnit) -> Isometry3d:
    """Return the initial mount origin relative to the robot base link.

    An attached object has ``world_frame`` ``None`` and mounts at identity; a
    ``MoveLinkCommand`` repositions it. An unattached object mounts at its world
    placement expressed relative to the robot base.
    """
    if world_frame is None:
        return Isometry3d.Identity()
    return _world_origin(world_frame, robot_base_frame, scale)


def _world_origin(world_frame: Frame, robot_base_frame: WorldFrameMeters, scale: MetersPerUserUnit) -> Pose:
    """Convert a world placement frame to a robot-base-relative native pose."""
    robot_frame = robot_frame_from_world(world_meters_frame(world_frame.scaled(scale)), robot_base_frame)
    return isometry_from_robot_frame(robot_frame)


def _relative_origin(attachment_frame: Optional[Frame], scale: MetersPerUserUnit) -> Pose:
    """Convert a parent-relative attachment frame to a native metre pose."""
    return pose_from_user_frame(attachment_frame if attachment_frame is not None else Frame.worldXY(), scale)


def _tool_relative_origin(tool: ToolModel, attachment_frame: Optional[Frame], scale: MetersPerUserUnit) -> Pose:
    """Compose a tool-tip-relative grasp into a tool-base-relative native pose.

    A rigid body attached to a tool carries an ``attachment_frame`` relative to
    the tool coordinate frame (TCF), while the native tool link sits at the tool
    base. The joint origin is therefore ``frame_in_tool0_frame * attachment``.
    """
    tool_base_to_tip = Transformation.from_frame(tool.frame)
    tip_to_body = Transformation.from_frame(attachment_frame if attachment_frame is not None else Frame.worldXY())
    return pose_from_user_frame(Frame.from_transformation(tool_base_to_tip * tip_to_body), scale)


def _allowed_collisions_command(touch_pairs: list[tuple[str, str, str]]) -> ModifyAllowedCollisionsCommand:
    """Build one ADD command carrying every touch-link and touch-body pair."""
    matrix = AllowedCollisionMatrix()
    for first, second, reason in touch_pairs:
        matrix.addAllowedCollision(first, second, reason)
    return ModifyAllowedCollisionsCommand(matrix, ModifyAllowedCollisionsType.ADD)


def _add_visual(link: Link, geometry: Geometry) -> None:
    """Attach an identity-origin visual with the given native geometry."""
    visual = Visual()
    visual.origin = Isometry3d.Identity()
    visual.geometry = geometry
    link.addVisual(visual)


def _add_collision(link: Link, geometry: Geometry) -> None:
    """Attach an identity-origin collision with the given native geometry."""
    collision = Collision()
    collision.origin = Isometry3d.Identity()
    collision.geometry = geometry
    link.addCollision(collision)
