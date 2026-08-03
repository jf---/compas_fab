"""Contract tests for the COMPAS-to-native Tesseract scene bridge."""

import numpy as np
import pytest
from compas.datastructures import Mesh
from compas.geometry import Box
from compas.geometry import Frame
from tesseract_robotics.tesseract_environment import AddLinkCommand
from tesseract_robotics.tesseract_environment import ModifyAllowedCollisionsCommand
from tesseract_robotics.tesseract_environment import MoveLinkCommand
from tesseract_robotics.tesseract_scene_graph import Link

from compas_fab.backends.tesseract.errors import NativeSceneCommandRejectedError
from compas_fab.backends.tesseract.errors import UnknownAttachmentParentError
from compas_fab.backends.tesseract.errors import UnregisteredSceneModelError
from compas_fab.backends.tesseract.frames import isometry_from_robot_frame
from compas_fab.backends.tesseract.frames import meters_per_user_unit
from compas_fab.backends.tesseract.frames import robot_frame_from_world
from compas_fab.backends.tesseract.frames import world_meters_frame
from compas_fab.backends.tesseract.native_scene import apply_scene
from compas_fab.backends.tesseract.native_scene import fixed_joint
from compas_fab.backends.tesseract.native_scene import rigid_body_link
from compas_fab.backends.tesseract.native_scene import scene_commands
from compas_fab.backends.tesseract.native_scene import tool_link
from compas_fab.robots import RigidBody
from compas_fab.robots import RobotCell
from compas_fab.robots import RobotCellState

from .conftest import BODY_GRASP_Z
from .conftest import BODY_ID
from .conftest import TOOL_ID
from .conftest import TOOL_TCF_Z

METRES_UNIT = meters_per_user_unit(1.0)


def _base_meters(state):
    return world_meters_frame(state.robot_base_frame)


def _joint_translation(command):
    return np.asarray(command.getJoint().parent_to_joint_origin_transform.matrix)[:3, 3]


def _body_on_link_cell_state(model_semantics, link_name, attachment_frame):
    """A cell + state with one box body attached to a named robot link."""
    model, semantics = model_semantics
    body = RigidBody.from_mesh(Mesh.from_shape(Box(0.2, 0.2, 0.2)), name="widget")
    cell = RobotCell(model, semantics, rigid_body_models={"widget": body})
    state = RobotCellState.from_robot_cell(cell)
    state.set_rigid_body_attached_to_link("widget", link_name, attachment_frame=attachment_frame)
    return cell, state


@pytest.fixture
def model_semantics(one_joint_cell_with_tool):
    return one_joint_cell_with_tool.robot_model, one_joint_cell_with_tool.robot_semantics


# ---------------------------------------------------------------------------
# Pure builders
# ---------------------------------------------------------------------------


def test_rigid_body_link_carries_visual_and_collision():
    body = RigidBody.from_mesh(Mesh.from_shape(Box(0.2, 0.2, 0.2)), name="widget")
    link = rigid_body_link("widget", body)
    assert link.getName() == "widget"
    assert len(link.visual) == 1
    assert len(link.collision) == 1


def test_tool_link_carries_geometry(one_joint_cell_with_tool):
    tool = one_joint_cell_with_tool.tool_models[TOOL_ID]
    link = tool_link(TOOL_ID, tool)
    assert link.getName() == TOOL_ID
    assert len(link.visual) >= 1
    assert len(link.collision) >= 1


def test_scene_commands_orders_add_then_move_then_acm(one_joint_cell_with_tool, one_joint_state_with_tool):
    commands = scene_commands(one_joint_cell_with_tool, one_joint_state_with_tool, _base_meters(one_joint_state_with_tool), METRES_UNIT)
    kinds = [type(command).__name__ for command in commands]
    # Two AddLink (tool + body), two MoveLink (attach tool + body), one ACM.
    assert kinds == [
        AddLinkCommand.__name__,
        AddLinkCommand.__name__,
        MoveLinkCommand.__name__,
        MoveLinkCommand.__name__,
        ModifyAllowedCollisionsCommand.__name__,
    ]


def test_scene_commands_tool_attaches_to_group_end_effector(one_joint_cell_with_tool, one_joint_state_with_tool):
    commands = scene_commands(one_joint_cell_with_tool, one_joint_state_with_tool, _base_meters(one_joint_state_with_tool), METRES_UNIT)
    tool_move = next(c for c in commands if isinstance(c, MoveLinkCommand) and c.getJoint().child_link_name == TOOL_ID)
    assert tool_move.getJoint().parent_link_name == "tip"


def test_scene_commands_body_to_tool_origin_composes_tcf(one_joint_cell_with_tool, one_joint_state_with_tool):
    # A body grasped by a tool has its attachment_frame relative to the TCF, so
    # the native joint origin (relative to the tool base link) is TCF * grasp.
    commands = scene_commands(one_joint_cell_with_tool, one_joint_state_with_tool, _base_meters(one_joint_state_with_tool), METRES_UNIT)
    body_move = next(c for c in commands if isinstance(c, MoveLinkCommand) and c.getJoint().child_link_name == BODY_ID)
    assert body_move.getJoint().parent_link_name == TOOL_ID
    np.testing.assert_allclose(_joint_translation(body_move), [0.0, 0.0, TOOL_TCF_Z + BODY_GRASP_Z], atol=1e-9)


def test_scene_commands_body_to_link_origin_is_attachment_frame(model_semantics):
    cell, state = _body_on_link_cell_state(model_semantics, "tip", Frame([0.3, 0.0, 0.0], [1, 0, 0], [0, 1, 0]))
    commands = scene_commands(cell, state, _base_meters(state), METRES_UNIT)
    body_move = next(c for c in commands if isinstance(c, MoveLinkCommand))
    assert body_move.getJoint().parent_link_name == "tip"
    np.testing.assert_allclose(_joint_translation(body_move), [0.3, 0.0, 0.0], atol=1e-9)


def test_scene_commands_scale_scales_attachment_translation(model_semantics):
    cell, state = _body_on_link_cell_state(model_semantics, "tip", Frame([1.0, 0.0, 0.0], [1, 0, 0], [0, 1, 0]))
    commands = scene_commands(cell, state, _base_meters(state), meters_per_user_unit(0.001))
    body_move = next(c for c in commands if isinstance(c, MoveLinkCommand))
    np.testing.assert_allclose(_joint_translation(body_move), [0.001, 0.0, 0.0], atol=1e-12)


def test_scene_commands_unregistered_tool_raises(one_joint_cell_with_tool, one_joint_state_with_tool):
    del one_joint_cell_with_tool.tool_models[TOOL_ID]
    with pytest.raises(UnregisteredSceneModelError):
        scene_commands(one_joint_cell_with_tool, one_joint_state_with_tool, _base_meters(one_joint_state_with_tool), METRES_UNIT)


def test_scene_commands_unknown_link_parent_raises(model_semantics):
    cell, state = _body_on_link_cell_state(model_semantics, "nonexistent_link", Frame.worldXY())
    with pytest.raises(UnknownAttachmentParentError):
        scene_commands(cell, state, _base_meters(state), METRES_UNIT)


def test_scene_commands_unknown_tool_parent_raises(model_semantics):
    model, semantics = model_semantics
    body = RigidBody.from_mesh(Mesh.from_shape(Box(0.2, 0.2, 0.2)), name="widget")
    cell = RobotCell(model, semantics, rigid_body_models={"widget": body})
    state = RobotCellState.from_robot_cell(cell)
    state.set_rigid_body_attached_to_tool("widget", "ghost_tool", attachment_frame=Frame.worldXY())
    with pytest.raises(UnknownAttachmentParentError):
        scene_commands(cell, state, _base_meters(state), METRES_UNIT)


def test_scene_commands_hidden_body_is_omitted(model_semantics):
    cell, state = _body_on_link_cell_state(model_semantics, "tip", Frame.worldXY())
    state.rigid_body_states["widget"].is_hidden = True
    commands = scene_commands(cell, state, _base_meters(state), METRES_UNIT)
    assert commands == []


# ---------------------------------------------------------------------------
# Application against a real cloned environment
# ---------------------------------------------------------------------------


def test_apply_scene_populates_cloned_environment(tesseract_robot, one_joint_cell_with_tool, one_joint_state_with_tool):
    commands = scene_commands(one_joint_cell_with_tool, one_joint_state_with_tool, _base_meters(one_joint_state_with_tool), METRES_UNIT)
    apply_scene(tesseract_robot, commands)

    link_names = set(tesseract_robot.env.getLinkNames())
    assert {TOOL_ID, BODY_ID} <= link_names

    acm = tesseract_robot.env.getAllowedCollisionMatrix()
    assert acm.isCollisionAllowed(TOOL_ID, "tip")
    assert acm.isCollisionAllowed(BODY_ID, TOOL_ID)


def test_apply_scene_places_attached_links_by_forward_kinematics(tesseract_robot, one_joint_cell_with_tool, one_joint_state_with_tool):
    commands = scene_commands(one_joint_cell_with_tool, one_joint_state_with_tool, _base_meters(one_joint_state_with_tool), METRES_UNIT)
    apply_scene(tesseract_robot, commands)

    # tip link sits at z=1 (joint origin), tool base coincides with it, the TCF
    # adds 0.1 and the grasp adds 0.02, so the block rests at z = 1.12.
    tool_z = np.asarray(tesseract_robot.env.getLinkTransform(TOOL_ID).matrix)[2, 3]
    body_z = np.asarray(tesseract_robot.env.getLinkTransform(BODY_ID).matrix)[2, 3]
    assert tool_z == pytest.approx(1.0, abs=1e-9)
    assert body_z == pytest.approx(1.0 + TOOL_TCF_Z + BODY_GRASP_Z, abs=1e-9)


def test_apply_scene_rejected_command_raises(tesseract_robot):
    # Re-adding an existing link with replace disallowed is rejected by the env.
    duplicate = AddLinkCommand(
        Link("tip"),
        fixed_joint("tip_dup_joint", "base", "tip", isometry_from_robot_frame(robot_frame_from_world(world_meters_frame(Frame.worldXY()), world_meters_frame(Frame.worldXY())))),
    )
    with pytest.raises(NativeSceneCommandRejectedError):
        apply_scene(tesseract_robot, [duplicate])
