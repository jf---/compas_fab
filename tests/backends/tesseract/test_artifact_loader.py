from pathlib import Path

import pytest

from compas_fab.backends.tesseract.artifact import CollisionMeshPolicy
from compas_fab.backends.tesseract.artifact_loader import ResourceRoot
from compas_fab.backends.tesseract.artifact_loader import RobotArtifactLoader
from compas_fab.backends.tesseract.errors import AmbiguousRobotPackageError
from compas_fab.backends.tesseract.errors import InvalidRobotResourceError
from compas_fab.backends.tesseract.errors import MissingRobotPackageError
from compas_fab.backends.tesseract.errors import MissingRobotResourceRootError
from compas_fab.backends.tesseract.errors import RobotResourceContainmentError


URDF = """<?xml version="1.0"?>
<robot name="loader">
  <link name="base">
    <visual>
      <geometry>
        <mesh filename="package://robot_support/meshes/base.dae"/>
      </geometry>
    </visual>
  </link>
</robot>
"""

SRDF = """<?xml version="1.0"?>
<robot name="loader"/>
"""


def _write_descriptions(root: Path) -> tuple[Path, Path]:
    urdf_path = root / "robot.urdf"
    srdf_path = root / "robot.srdf"
    urdf_path.write_bytes(URDF.replace("\n", "\r\n").encode("utf-8"))
    srdf_path.write_bytes(SRDF.encode("utf-8"))
    return urdf_path, srdf_path


def test_resource_root_factory_rejects_missing_directory(tmp_path):
    with pytest.raises(MissingRobotResourceRootError, match="missing"):
        ResourceRoot.build(tmp_path / "missing")


def test_loader_retains_exact_source_text_and_complete_referenced_package(tmp_path):
    urdf_path, srdf_path = _write_descriptions(tmp_path)
    package = tmp_path / "resources" / "robot_support"
    (package / "meshes").mkdir(parents=True)
    (package / "meshes" / "base.dae").write_bytes(b"exact mesh")
    (package / "textures").mkdir()
    (package / "textures" / "base.png").write_bytes(b"exact texture")

    loader = RobotArtifactLoader.build(
        urdf_path,
        srdf_path,
        [ResourceRoot.build(tmp_path / "resources")],
    )
    artifact = loader.load(CollisionMeshPolicy.PRESERVE)

    assert "\r\n" in loader.urdf
    assert loader.srdf == SRDF
    assert artifact.resource("package://robot_support/meshes/base.dae").content == b"exact mesh"
    assert artifact.resource("package://robot_support/textures/base.png").content == b"exact texture"


def test_loader_does_not_hash_unreferenced_packages(tmp_path):
    urdf_path, srdf_path = _write_descriptions(tmp_path)
    resource_root = tmp_path / "resources"
    package = resource_root / "robot_support" / "meshes"
    package.mkdir(parents=True)
    (package / "base.dae").write_bytes(b"exact mesh")
    unrelated = resource_root / "unrelated"
    unrelated.mkdir()
    (unrelated / "changing.bin").write_bytes(b"first")

    loader = RobotArtifactLoader.build(
        urdf_path,
        srdf_path,
        [ResourceRoot.build(resource_root)],
    )
    first = loader.load(CollisionMeshPolicy.PRESERVE)
    (unrelated / "changing.bin").write_bytes(b"second")
    second = loader.load(CollisionMeshPolicy.PRESERVE)

    assert first.identity == second.identity
    assert all(not resource.url.startswith("package://unrelated/") for resource in first.resources)


def test_loader_fails_when_referenced_package_is_missing(tmp_path):
    urdf_path, srdf_path = _write_descriptions(tmp_path)
    resource_root = tmp_path / "resources"
    resource_root.mkdir()
    loader = RobotArtifactLoader.build(
        urdf_path,
        srdf_path,
        [ResourceRoot.build(resource_root)],
    )

    with pytest.raises(MissingRobotPackageError, match="robot_support"):
        loader.load(CollisionMeshPolicy.PRESERVE)


def test_loader_fails_when_package_resolution_is_ambiguous(tmp_path):
    urdf_path, srdf_path = _write_descriptions(tmp_path)
    roots = []
    for name in ("first", "second"):
        root = tmp_path / name
        package = root / "robot_support" / "meshes"
        package.mkdir(parents=True)
        (package / "base.dae").write_bytes(name.encode("utf-8"))
        roots.append(ResourceRoot.build(root))
    loader = RobotArtifactLoader.build(urdf_path, srdf_path, roots)

    with pytest.raises(AmbiguousRobotPackageError, match="robot_support"):
        loader.load(CollisionMeshPolicy.PRESERVE)


@pytest.mark.parametrize(
    "authority",
    ["..", ".", r"..\secret", r"C:\secret"],
)
def test_loader_rejects_unsafe_package_authority(tmp_path, authority):
    urdf_path, srdf_path = _write_descriptions(tmp_path)
    urdf_path.write_text(
        URDF.replace("robot_support", authority),
        encoding="utf-8",
    )
    resource_root = tmp_path / "resources"
    resource_root.mkdir()
    escaped_package = tmp_path / "meshes"
    escaped_package.mkdir()
    (escaped_package / "base.dae").write_bytes(b"escaped mesh")
    loader = RobotArtifactLoader.build(
        urdf_path,
        srdf_path,
        [ResourceRoot.build(resource_root)],
    )

    with pytest.raises(InvalidRobotResourceError, match="package resource URL"):
        loader.load(CollisionMeshPolicy.PRESERVE)


def test_loader_rejects_symlinked_package_outside_resource_root(tmp_path):
    urdf_path, srdf_path = _write_descriptions(tmp_path)
    resource_root = tmp_path / "resources"
    resource_root.mkdir()
    external_package = tmp_path / "external" / "robot_support"
    (external_package / "meshes").mkdir(parents=True)
    (external_package / "meshes" / "base.dae").write_bytes(b"escaped mesh")
    (resource_root / "robot_support").symlink_to(
        external_package,
        target_is_directory=True,
    )
    loader = RobotArtifactLoader.build(
        urdf_path,
        srdf_path,
        [ResourceRoot.build(resource_root)],
    )

    with pytest.raises(InvalidRobotResourceError, match="direct child"):
        loader.load(CollisionMeshPolicy.PRESERVE)


def test_loader_rejects_package_file_symlink_outside_package(tmp_path):
    urdf_path, srdf_path = _write_descriptions(tmp_path)
    resource_root = tmp_path / "resources"
    package = resource_root / "robot_support"
    (package / "meshes").mkdir(parents=True)
    outside = tmp_path / "outside.dae"
    outside.write_bytes(b"escaped mesh")
    (package / "meshes" / "base.dae").symlink_to(outside)
    loader = RobotArtifactLoader.build(
        urdf_path,
        srdf_path,
        [ResourceRoot.build(resource_root)],
    )

    with pytest.raises(RobotResourceContainmentError, match="package directory"):
        loader.load(CollisionMeshPolicy.PRESERVE)


def test_loader_supports_description_without_external_resources(tmp_path):
    urdf_path = tmp_path / "robot.urdf"
    srdf_path = tmp_path / "robot.srdf"
    urdf_path.write_text('<robot name="primitive"><link name="base"/></robot>', encoding="utf-8")
    srdf_path.write_text('<robot name="primitive"/>', encoding="utf-8")

    artifact = RobotArtifactLoader.build(urdf_path, srdf_path, []).load(CollisionMeshPolicy.PRESERVE)

    assert artifact.resources == ()
    assert 'tesseract:make_convex="false"' in artifact.urdf


def test_loader_defaults_collision_meshes_to_convex_hulls(tmp_path):
    urdf_path = tmp_path / "robot.urdf"
    srdf_path = tmp_path / "robot.srdf"
    urdf_path.write_text('<robot name="primitive"><link name="base"/></robot>', encoding="utf-8")
    srdf_path.write_text('<robot name="primitive"/>', encoding="utf-8")

    artifact = RobotArtifactLoader.build(urdf_path, srdf_path, []).load()

    assert 'tesseract:make_convex="true"' in artifact.urdf
