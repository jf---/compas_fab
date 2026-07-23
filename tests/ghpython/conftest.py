import pytest

from compas_fab.backends.tesseract.artifact import CollisionMeshPolicy
from compas_fab.backends.tesseract.artifact import ContinuousContactManager
from compas_fab.backends.tesseract.artifact import DiscreteContactManager
from compas_fab.backends.tesseract.artifact import KdlInverseKinematics
from compas_fab.backends.tesseract.artifact import KdlKinematics
from compas_fab.backends.tesseract.artifact import RobotArtifact
from compas_fab.backends.tesseract.client import TesseractClient
from compas_fab.backends.tesseract.planner import TesseractPlanner

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
def tesseract_client(tmp_path):
    artifact = RobotArtifact.from_compas_urdf(URDF, SRDF, {}, CollisionMeshPolicy.PRESERVE)
    artifact = artifact.with_kdl_kinematics([KdlKinematics.build("manipulator", "base", "tip", KdlInverseKinematics.LMA)])
    artifact = artifact.with_contact_managers(
        DiscreteContactManager.BULLET_BVH,
        ContinuousContactManager.BULLET_CAST_BVH,
    )
    client = TesseractClient(artifact, cache_root=tmp_path)
    client.connect()
    try:
        yield client
    finally:
        client.disconnect()


@pytest.fixture
def native_robot(tesseract_client):
    return tesseract_client.clone_robot()


@pytest.fixture
def tesseract_planner(tesseract_client):
    return TesseractPlanner(tesseract_client)
