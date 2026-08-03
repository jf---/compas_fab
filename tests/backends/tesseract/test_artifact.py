from pathlib import Path

import pytest
from compas_robots import RobotModel

from compas_fab.backends.tesseract.artifact import CollisionMeshPolicy
from compas_fab.backends.tesseract.artifact import ContinuousContactManager
from compas_fab.backends.tesseract.artifact import DiscreteContactManager
from compas_fab.backends.tesseract.artifact import KdlInverseKinematics
from compas_fab.backends.tesseract.artifact import KdlKinematics
from compas_fab.backends.tesseract.artifact import RobotArtifact
from compas_fab.backends.tesseract.artifact import RobotResource
from compas_fab.backends.tesseract.errors import CollisionMeshPolicyConflictError
from compas_fab.backends.tesseract.errors import ContactManagerPluginConflictError
from compas_fab.backends.tesseract.errors import KinematicsPluginConflictError
from compas_fab.backends.tesseract.errors import InvalidRobotResourceError
from compas_fab.backends.tesseract.errors import RobotArtifactIdentityMismatchError
from compas_fab.backends.tesseract.errors import UnsafeRobotResourceUrlError
from compas_fab.backends.tesseract.identity import BuildIdentity
from compas_fab.robots import RobotSemantics


FIXTURES = Path(__file__).parents[2] / "robots" / "fixtures"


def _descriptions():
    return (
        (FIXTURES / "panda.urdf").read_text(),
        (FIXTURES / "panda_semantics.srdf").read_text(),
    )


def test_artifact_retains_exact_descriptions_and_round_trips_compas_contracts():
    urdf, srdf = _descriptions()

    artifact = RobotArtifact.build(urdf, srdf, {})
    model = RobotModel.from_urdf_string(artifact.urdf)
    semantics = RobotSemantics.from_srdf_string(artifact.srdf, model)

    assert artifact.urdf is urdf
    assert artifact.srdf is srdf
    assert model.name == "panda"
    assert "panda_arm" in semantics.group_names


def test_artifact_canonicalizes_and_copies_resource_mapping():
    urdf, srdf = _descriptions()
    resources = {"package://panda/b.stl": b"b", "package://panda/a.stl": b"a"}

    artifact = RobotArtifact.build(urdf, srdf, resources)
    resources["package://panda/a.stl"] = b"changed"

    assert tuple(resource.url for resource in artifact.resources) == (
        "package://panda/a.stl",
        "package://panda/b.stl",
    )
    assert artifact.resource("package://panda/a.stl").content == b"a"


def test_artifact_identity_covers_exact_inputs():
    urdf, srdf = _descriptions()
    resources = {"package://panda/a.stl": b"a"}

    artifact = RobotArtifact.build(urdf, srdf, resources)

    assert artifact.identity == BuildIdentity.build(urdf, srdf, resources)


def test_resource_raw_constructor_cannot_bypass_invariants():
    with pytest.raises(InvalidRobotResourceError):
        RobotResource("", b"content")


@pytest.mark.parametrize(
    "url",
    [
        "package://panda//etc/passwd",
        "package://panda/../secret.stl",
        r"package://panda/..\secret.stl",
        r"package://panda/C:\secret.stl",
        "package://panda/nested/C:/secret.stl",
        "package://C:/secret.stl",
    ],
)
def test_artifact_rejects_unsafe_package_resource_url(url):
    urdf, srdf = _descriptions()

    with pytest.raises(UnsafeRobotResourceUrlError, match="package resource URL"):
        RobotArtifact.build(urdf, srdf, {url: b"escaped"})


def test_artifact_raw_constructor_rejects_arbitrary_resources():
    urdf, srdf = _descriptions()
    identity = BuildIdentity.build(urdf, srdf, {})

    with pytest.raises(InvalidRobotResourceError):
        RobotArtifact(urdf, srdf, (object(),), identity)


def test_artifact_raw_constructor_rejects_forged_identity():
    urdf, srdf = _descriptions()
    valid = BuildIdentity.build(urdf, srdf, {})
    forged = BuildIdentity(
        "0" * 64,
        valid.schema_version,
        valid.compas_fab_version,
        valid.tesseract_version,
    )

    with pytest.raises(RobotArtifactIdentityMismatchError):
        RobotArtifact(urdf, srdf, (), forged)


def test_unknown_resource_fails_loudly():
    urdf, srdf = _descriptions()
    artifact = RobotArtifact.build(urdf, srdf, {})

    with pytest.raises(InvalidRobotResourceError, match="missing.stl"):
        artifact.resource("package://panda/missing.stl")


@pytest.mark.parametrize(
    ("policy", "xml_value"),
    [
        (CollisionMeshPolicy.PRESERVE, "false"),
        (CollisionMeshPolicy.CONVEX_HULL, "true"),
    ],
)
def test_compas_urdf_compilation_requires_explicit_collision_mesh_policy(policy, xml_value):
    urdf, srdf = _descriptions()

    artifact = RobotArtifact.from_compas_urdf(urdf, srdf, {}, policy)

    assert 'tesseract:make_convex="{}"'.format(xml_value) in artifact.urdf
    assert "tesseract:make_convex" not in urdf
    assert artifact.urdf != urdf


def test_compas_urdf_compilation_rejects_conflicting_existing_policy():
    urdf, srdf = _descriptions()
    tesseract_urdf = urdf.replace(
        '<robot name="panda"',
        '<robot name="panda" xmlns:tesseract="https://github.com/tesseract-robotics/tesseract" tesseract:make_convex="true"',
    )

    with pytest.raises(CollisionMeshPolicyConflictError):
        RobotArtifact.from_compas_urdf(
            tesseract_urdf,
            srdf,
            {},
            CollisionMeshPolicy.PRESERVE,
        )


def test_artifact_adds_explicit_kdl_plugin_resource():
    urdf, srdf = _descriptions()
    artifact = RobotArtifact.from_compas_urdf(
        urdf,
        srdf,
        {},
        CollisionMeshPolicy.PRESERVE,
    )
    kinematics = KdlKinematics.build(
        group="panda_arm",
        base_link="panda_link0",
        tip_link="panda_link8",
        inverse=KdlInverseKinematics.LMA,
    )

    configured = artifact.with_kdl_kinematics([kinematics])
    plugin_resource = configured.resource("package://compas_fab/tesseract/kinematics.yaml")

    assert 'kinematics_plugin_config filename="package://compas_fab/tesseract/kinematics.yaml"' in configured.srdf
    assert b"panda_arm:" in plugin_resource.content
    assert b"base_link: panda_link0" in plugin_resource.content
    assert b"tip_link: panda_link8" in plugin_resource.content
    assert b"default: KDLInvKinChainLMA" in plugin_resource.content


def test_artifact_refuses_to_replace_exact_kinematics_plugin():
    urdf, srdf = _descriptions()
    srdf_with_plugin = srdf.replace(
        "</robot>",
        '<kinematics_plugin_config filename="package://exact/plugin.yaml"/></robot>',
    )
    artifact = RobotArtifact.from_compas_urdf(
        urdf,
        srdf_with_plugin,
        {"package://exact/plugin.yaml": b"exact"},
        CollisionMeshPolicy.PRESERVE,
    )

    with pytest.raises(KinematicsPluginConflictError):
        artifact.with_kdl_kinematics(
            [
                KdlKinematics.build(
                    "panda_arm",
                    "panda_link0",
                    "panda_link8",
                    KdlInverseKinematics.LMA,
                )
            ]
        )


def test_artifact_adds_explicit_contact_manager_plugin_resource():
    urdf, srdf = _descriptions()
    artifact = RobotArtifact.from_compas_urdf(
        urdf,
        srdf,
        {},
        CollisionMeshPolicy.PRESERVE,
    )

    configured = artifact.with_contact_managers(
        DiscreteContactManager.BULLET_BVH,
        ContinuousContactManager.BULLET_CAST_BVH,
    )
    plugin_resource = configured.resource("package://compas_fab/tesseract/contact_managers.yaml")

    assert 'contact_managers_plugin_config filename="package://compas_fab/tesseract/contact_managers.yaml"' in configured.srdf
    assert b"default: BulletDiscreteBVHManager" in plugin_resource.content
    assert b"class: BulletDiscreteBVHManagerFactory" in plugin_resource.content
    assert b"default: BulletCastBVHManager" in plugin_resource.content


def test_artifact_refuses_to_replace_exact_contact_manager_plugin():
    urdf, srdf = _descriptions()
    srdf_with_plugin = srdf.replace(
        "</robot>",
        '<contact_managers_plugin_config filename="package://exact/contact.yaml"/></robot>',
    )
    artifact = RobotArtifact.from_compas_urdf(
        urdf,
        srdf_with_plugin,
        {"package://exact/contact.yaml": b"exact"},
        CollisionMeshPolicy.PRESERVE,
    )

    with pytest.raises(ContactManagerPluginConflictError):
        artifact.with_contact_managers(
            DiscreteContactManager.BULLET_BVH,
            ContinuousContactManager.BULLET_CAST_BVH,
        )
