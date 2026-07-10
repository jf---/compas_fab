import pytest
from compas_robots import Configuration
from compas_robots import RobotModel
from compas_robots.model import Joint

from compas_fab.backends.tesseract.artifact import CollisionMeshPolicy
from compas_fab.backends.tesseract.artifact import ContinuousContactManager
from compas_fab.backends.tesseract.artifact import DiscreteContactManager
from compas_fab.backends.tesseract.artifact import KdlInverseKinematics
from compas_fab.backends.tesseract.artifact import KdlKinematics
from compas_fab.backends.tesseract.artifact import RobotArtifact
from compas_fab.backends.tesseract.client import TesseractClient
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
