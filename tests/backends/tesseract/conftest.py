import pytest
from compas.datastructures import Mesh
from compas.geometry import Box
from compas.geometry import Frame
from compas_robots import Configuration
from compas_robots import RobotModel
from compas_robots import ToolModel
from compas_robots.model import Joint

from compas_fab.backends.tesseract.artifact import CollisionMeshPolicy
from compas_fab.backends.tesseract.artifact import ContinuousContactManager
from compas_fab.backends.tesseract.artifact import DiscreteContactManager
from compas_fab.backends.tesseract.artifact import KdlInverseKinematics
from compas_fab.backends.tesseract.artifact import KdlKinematics
from compas_fab.backends.tesseract.artifact import RobotArtifact
from compas_fab.backends.tesseract.client import TesseractClient
from compas_fab.robots import RigidBody
from compas_fab.robots import RobotCell
from compas_fab.robots import RobotCellState
from compas_fab.robots import RobotSemantics


URDF = """<?xml version="1.0"?>
<robot name="one_joint">
  <link name="base"/>
  <link name="tip"/>
  <joint name="joint1" type="revolute">
    <parent link="base"/>
    <child link="tip"/>
    <origin xyz="0 0 1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/>
  </joint>
</robot>
"""

SRDF = """<?xml version="1.0"?>
<robot name="one_joint">
  <group name="manipulator">
    <chain base_link="base" tip_link="tip"/>
  </group>
</robot>
"""


@pytest.fixture
def kdl_artifact():
    artifact = RobotArtifact.from_compas_urdf(
        URDF,
        SRDF,
        {},
        CollisionMeshPolicy.PRESERVE,
    )
    artifact = artifact.with_kdl_kinematics(
        [
            KdlKinematics.build(
                "manipulator",
                "base",
                "tip",
                KdlInverseKinematics.LMA,
            )
        ]
    )
    return artifact


@pytest.fixture
def tesseract_artifact(kdl_artifact):
    return kdl_artifact.with_contact_managers(
        DiscreteContactManager.BULLET_BVH,
        ContinuousContactManager.BULLET_CAST_BVH,
    )


@pytest.fixture
def tesseract_robot(tesseract_artifact, tmp_path):
    client = TesseractClient(tesseract_artifact, cache_root=tmp_path)
    client.connect()
    try:
        yield client.clone_robot()
    finally:
        client.disconnect()


@pytest.fixture
def one_joint_cell():
    model = RobotModel.from_urdf_string(URDF)
    semantics = RobotSemantics.from_srdf_string(SRDF, model)
    return RobotCell(model, semantics)


@pytest.fixture
def one_joint_state():
    configuration = Configuration([0.0], [Joint.REVOLUTE], ["joint1"])
    return RobotCellState(robot_configuration=configuration)


# Scene fixture geometry (metres). Named so scene tests can assert placement.
TOOL_ID = "gripper"
BODY_ID = "block"
TOOL_TCF_Z = 0.1  # Tool coordinate frame offset above the tool base link.
BODY_GRASP_Z = 0.02  # Rigid body grasp offset above the tool coordinate frame.
BOX_SIZE = 0.2


@pytest.fixture
def one_joint_cell_with_tool():
    """A one-joint cell carrying a box tool and a box rigid body."""
    model = RobotModel.from_urdf_string(URDF)
    semantics = RobotSemantics.from_srdf_string(SRDF, model)
    tool = ToolModel(
        Mesh.from_shape(Box(BOX_SIZE, BOX_SIZE, BOX_SIZE)),
        Frame([0.0, 0.0, TOOL_TCF_Z], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]),
        name=TOOL_ID,
    )
    body = RigidBody.from_mesh(Mesh.from_shape(Box(BOX_SIZE, BOX_SIZE, BOX_SIZE)), name=BODY_ID)
    return RobotCell(model, semantics, tool_models={TOOL_ID: tool}, rigid_body_models={BODY_ID: body})


@pytest.fixture
def one_joint_state_with_tool(one_joint_cell_with_tool):
    """State attaching the tool to the group tip and the body to the tool tip."""
    state = RobotCellState.from_robot_cell(one_joint_cell_with_tool)
    state.set_tool_attached_to_group(TOOL_ID, "manipulator", touch_links=["tip"])
    state.set_rigid_body_attached_to_tool(
        BODY_ID,
        TOOL_ID,
        attachment_frame=Frame([0.0, 0.0, BODY_GRASP_Z], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]),
    )
    state.rigid_body_states[BODY_ID].touch_bodies = [TOOL_ID]
    return state
