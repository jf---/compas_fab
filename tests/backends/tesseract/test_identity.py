import pytest

from compas_fab.backends.tesseract.errors import EmptyRobotDescriptionError
from compas_fab.backends.tesseract.identity import BuildIdentity


URDF = '<robot name="test"/>'
SRDF = '<robot name="test"/>'


def test_identity_is_order_independent_for_resources():
    left = BuildIdentity.build(URDF, SRDF, {"package://test/b": b"2", "package://test/a": b"1"})
    right = BuildIdentity.build(URDF, SRDF, {"package://test/a": b"1", "package://test/b": b"2"})

    assert left == right


@pytest.mark.parametrize(
    ("urdf", "srdf", "resources"),
    [
        ('<robot name="changed"/>', SRDF, {}),
        (URDF, '<robot name="changed"/>', {}),
        (URDF, SRDF, {"package://test/a": b"changed"}),
    ],
)
def test_identity_changes_when_any_artifact_input_changes(urdf, srdf, resources):
    original = BuildIdentity.build(URDF, SRDF, {})
    changed = BuildIdentity.build(urdf, srdf, resources)

    assert original.digest != changed.digest


def test_identity_records_component_versions():
    identity = BuildIdentity.build(URDF, SRDF, {})

    assert identity.schema_version == "1"
    assert identity.compas_fab_version
    assert identity.tesseract_version.startswith("0.35.")


@pytest.mark.parametrize(("urdf", "srdf"), [("", SRDF), (URDF, ""), ("  ", SRDF)])
def test_empty_robot_description_fails_loudly(urdf, srdf):
    with pytest.raises(EmptyRobotDescriptionError):
        BuildIdentity.build(urdf, srdf, {})
