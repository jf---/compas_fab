from pathlib import Path

import pytest

from compas_fab.backends.tesseract.artifact import CollisionMeshPolicy
from compas_fab.backends.tesseract.artifact import RobotArtifact
from compas_fab.backends.tesseract.environment import TesseractEnvironment
from compas_fab.backends.tesseract.errors import TesseractEnvironmentInitializationError
from compas_fab.backends.tesseract.errors import UnknownRobotResourceError
from compas_fab.backends.tesseract.errors import RobotResourceContainmentError
from compas_fab.backends.tesseract.errors import UnsafeRobotResourceUrlError
from compas_fab.backends.tesseract.materialization import MaterializedArtifact


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


def _artifact(resources=None):
    return RobotArtifact.from_compas_urdf(
        URDF,
        SRDF,
        resources or {},
        CollisionMeshPolicy.PRESERVE,
    )


def test_environment_initializes_exact_artifact(tmp_path):
    artifact = _artifact()

    environment = TesseractEnvironment.build(artifact, tmp_path)

    assert environment.artifact is artifact
    assert environment.robot.get_joint_names("manipulator") == ["joint1"]


def test_environment_clones_do_not_share_joint_state(tmp_path):
    environment = TesseractEnvironment.build(_artifact(), tmp_path)
    left = environment.clone_robot()
    right = environment.clone_robot()

    left.set_joints({"joint1": 0.5})

    assert left.get_state(["joint1"])["joint1"] == 0.5
    assert right.get_state(["joint1"])["joint1"] == 0.0


def test_resources_are_materialized_under_build_identity(tmp_path):
    url = "package://one_joint/clouds/workpiece.pcd"
    artifact = _artifact({url: b"point-cloud"})

    environment = TesseractEnvironment.build(artifact, tmp_path)
    located = environment.locator.locateResource(url)
    located_path = Path(located.getFilePath())

    assert located_path.read_bytes() == b"point-cloud"
    assert artifact.identity.digest in located_path.parts
    assert located_path.relative_to(environment.materialized.root).parts == (
        "package",
        "one_joint",
        "clouds",
        "workpiece.pcd",
    )


def test_materialization_revalidates_retained_resource_url(tmp_path):
    artifact = _artifact({"package://one_joint/clouds/workpiece.pcd": b"point-cloud"})
    object.__setattr__(
        artifact.resources[0],
        "url",
        "package://one_joint//escaped.pcd",
    )

    with pytest.raises(UnsafeRobotResourceUrlError, match="package resource URL"):
        MaterializedArtifact.build(artifact, tmp_path)


def test_materialization_target_cannot_follow_symlink_outside_artifact_root(
    tmp_path,
):
    artifact = _artifact({"package://one_joint/clouds/workpiece.pcd": b"point-cloud"})
    cache_root = tmp_path / "cache"
    artifact_root = cache_root / artifact.identity.digest
    artifact_root.mkdir(parents=True)
    outside = tmp_path / "outside"
    outside.mkdir()
    (artifact_root / "package").symlink_to(
        outside,
        target_is_directory=True,
    )

    with pytest.raises(RobotResourceContainmentError, match="artifact root"):
        MaterializedArtifact.build(artifact, cache_root)

    assert not (outside / "one_joint" / "clouds" / "workpiece.pcd").exists()


def test_unknown_resource_fails_loudly(tmp_path):
    environment = TesseractEnvironment.build(_artifact(), tmp_path)

    with pytest.raises(UnknownRobotResourceError, match="missing.stl"):
        environment.locator.locateResource("package://one_joint/missing.stl")


def test_invalid_environment_reports_artifact_identity(tmp_path):
    artifact = RobotArtifact.build("<not-a-robot/>", SRDF, {})

    with pytest.raises(TesseractEnvironmentInitializationError, match=artifact.identity.digest):
        TesseractEnvironment.build(artifact, tmp_path)
