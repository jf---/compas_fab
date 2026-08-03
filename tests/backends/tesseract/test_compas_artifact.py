from pathlib import Path

import pytest

from compas_fab.backends.tesseract.artifact import CollisionMeshPolicy
from compas_fab.backends.tesseract.artifact import ContinuousContactManager
from compas_fab.backends.tesseract.artifact import DiscreteContactManager
from compas_fab.backends.tesseract.artifact import KdlInverseKinematics
from compas_fab.backends.tesseract.artifact_loader import RobotArtifactLoader
from compas_fab.backends.tesseract.compas_artifact import CompasRobotArtifactCompiler
from compas_fab.backends.tesseract.errors import InvalidKinematicsConfigError


def _loader(tmp_path: Path) -> RobotArtifactLoader:
    urdf_path = tmp_path / "robot.urdf"
    srdf_path = tmp_path / "robot.srdf"
    urdf_path.write_text(
        """<robot name="one_joint">
  <link name="base"/>
  <link name="tip"/>
  <joint name="joint1" type="revolute">
    <parent link="base"/>
    <child link="tip"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/>
  </joint>
</robot>""",
        encoding="utf-8",
    )
    srdf_path.write_text(
        """<robot name="one_joint">
  <group name="manipulator"><chain base_link="base" tip_link="tip"/></group>
</robot>""",
        encoding="utf-8",
    )
    return RobotArtifactLoader.build(urdf_path, srdf_path, [])


def test_compiler_generates_only_explicit_native_plugins(tmp_path, one_joint_cell):
    compiler = CompasRobotArtifactCompiler.build(
        loader=_loader(tmp_path),
        collision_mesh_policy=CollisionMeshPolicy.PRESERVE,
        groups=["manipulator"],
        inverse_kinematics=KdlInverseKinematics.NEWTON_RAPHSON,
        discrete_contact_manager=DiscreteContactManager.FCL_BVH,
        continuous_contact_manager=ContinuousContactManager.BULLET_CAST_SIMPLE,
    )

    artifact = compiler.compile(one_joint_cell)

    kinematics = artifact.resource("package://compas_fab/tesseract/kinematics.yaml").content
    contact = artifact.resource("package://compas_fab/tesseract/contact_managers.yaml").content
    assert b"default: KDLInvKinChainNR" in kinematics
    assert b"default: FCLDiscreteBVHManager" in contact
    assert b"default: BulletCastSimpleManager" in contact


def test_compiler_defaults_collision_meshes_to_convex_hulls(tmp_path, one_joint_cell):
    compiler = CompasRobotArtifactCompiler.build(
        loader=_loader(tmp_path),
        groups=["manipulator"],
        inverse_kinematics=KdlInverseKinematics.LMA,
        discrete_contact_manager=DiscreteContactManager.BULLET_BVH,
        continuous_contact_manager=ContinuousContactManager.BULLET_CAST_BVH,
    )

    artifact = compiler.compile(one_joint_cell)

    assert compiler.collision_mesh_policy is CollisionMeshPolicy.CONVEX_HULL
    assert 'tesseract:make_convex="true"' in artifact.urdf


def test_compiler_rejects_empty_group_selection(tmp_path):
    with pytest.raises(InvalidKinematicsConfigError, match="At least one"):
        CompasRobotArtifactCompiler.build(
            loader=_loader(tmp_path),
            collision_mesh_policy=CollisionMeshPolicy.PRESERVE,
            groups=[],
            inverse_kinematics=KdlInverseKinematics.LMA,
            discrete_contact_manager=DiscreteContactManager.BULLET_BVH,
            continuous_contact_manager=ContinuousContactManager.BULLET_CAST_BVH,
        )


def test_compiler_rejects_unknown_or_jointless_group(tmp_path, one_joint_cell):
    compiler = CompasRobotArtifactCompiler.build(
        loader=_loader(tmp_path),
        collision_mesh_policy=CollisionMeshPolicy.PRESERVE,
        groups=["missing"],
        inverse_kinematics=KdlInverseKinematics.LMA,
        discrete_contact_manager=DiscreteContactManager.BULLET_BVH,
        continuous_contact_manager=ContinuousContactManager.BULLET_CAST_BVH,
    )

    with pytest.raises(InvalidKinematicsConfigError, match="missing"):
        compiler.compile(one_joint_cell)


def test_compiler_copies_group_input(tmp_path):
    groups = ["manipulator"]
    compiler = CompasRobotArtifactCompiler.build(
        loader=_loader(tmp_path),
        collision_mesh_policy=CollisionMeshPolicy.PRESERVE,
        groups=groups,
        inverse_kinematics=KdlInverseKinematics.LMA,
        discrete_contact_manager=DiscreteContactManager.BULLET_BVH,
        continuous_contact_manager=ContinuousContactManager.BULLET_CAST_BVH,
    )
    groups.append("changed")

    assert compiler.groups == ("manipulator",)
