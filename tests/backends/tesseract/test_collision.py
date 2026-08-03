from pathlib import Path

from compas_robots import Configuration
from compas_robots import RobotModel
from compas_robots.model import Joint
import pytest
from compas.tolerance import TOL
from tesseract_robotics.tesseract_collision import ContactRequest
from tesseract_robotics.tesseract_collision import ContactTestType_ALL
from tesseract_robotics.tesseract_collision import ContactTestType_CLOSEST
from tesseract_robotics.tesseract_collision import ContactTestType_FIRST
from tesseract_robotics.tesseract_collision import ContactTestType_LIMITED

from compas_fab.backends.tesseract.artifact import CollisionMeshPolicy
from compas_fab.backends.tesseract.artifact import ContinuousContactManager
from compas_fab.backends.tesseract.artifact import DiscreteContactManager
from compas_fab.backends.tesseract.artifact import KdlInverseKinematics
from compas_fab.backends.tesseract.artifact import KdlKinematics
from compas_fab.backends.tesseract.artifact import RobotArtifact
from compas_fab.backends.tesseract.client import TesseractClient
from compas_fab.backends.tesseract.collision import build_contact_request
from compas_fab.backends.tesseract.collision import is_collision_distance
from compas_fab.backends.tesseract.errors import TesseractCollisionError
from compas_fab.backends.tesseract.errors import TesseractContactQueryError
from compas_fab.backends.tesseract.planner import TesseractPlanner
from compas_fab.robots import RobotCell
from compas_fab.robots import RobotCellState
from compas_fab.robots import RobotSemantics


URDF = """<robot name="slide">
  <link name="base">
    <collision><geometry><sphere radius="0.4"/></geometry></collision>
  </link>
  <link name="tip">
    <collision><geometry><sphere radius="0.4"/></geometry></collision>
  </link>
  <joint name="slide" type="prismatic">
    <parent link="base"/><child link="tip"/><axis xyz="0 0 1"/>
    <limit lower="0" upper="3" effort="10" velocity="1"/>
  </joint>
</robot>"""

SRDF = """<robot name="slide">
  <group name="manipulator"><chain base_link="base" tip_link="tip"/></group>
</robot>"""


def _planner(cache_root: Path) -> tuple[TesseractClient, TesseractPlanner]:
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
    artifact = artifact.with_contact_managers(
        DiscreteContactManager.BULLET_BVH,
        ContinuousContactManager.BULLET_CAST_BVH,
    )
    model = RobotModel.from_urdf_string(URDF)
    cell = RobotCell(model, RobotSemantics.from_srdf_string(SRDF, model))
    client = TesseractClient(artifact, cache_root=cache_root)
    client.connect()
    planner = TesseractPlanner(client)
    planner.set_robot_cell(cell)
    return client, planner


def _state(position: float) -> RobotCellState:
    return RobotCellState(robot_configuration=Configuration([position], [Joint.PRISMATIC], ["slide"]))


def test_collision_native_result_is_complete_and_conventional_check_raises(tmp_path):
    client, planner = _planner(tmp_path)
    request = ContactRequest(ContactTestType_ALL)
    try:
        result = planner.check_collision_native(_state(0.0), request)

        assert result.request is request
        assert len(result.native_map) >= 1
        assert len(result.native_results) >= 1
        with pytest.raises(TesseractCollisionError) as caught:
            planner.check_collision(_state(0.0))
    finally:
        client.disconnect()

    assert caught.value.native_result.native_results
    assert len(caught.value.native_result.native_map) >= 1
    assert ("base", "tip") in caught.value.link_pairs
    assert min(caught.value.distances) < 0.0


def test_collision_free_state_returns_no_native_contacts(tmp_path):
    client, planner = _planner(tmp_path)
    try:
        result = planner.check_collision_native(
            _state(2.0),
            ContactRequest(ContactTestType_ALL),
        )
        checked = planner.check_collision(_state(2.0))
    finally:
        client.disconnect()

    assert len(result.native_results) == 0
    assert checked is None


def test_positive_native_separation_is_not_reclassified_by_compas_tolerance():
    positive_separation = TOL.absolute / 2.0

    assert not is_collision_distance(positive_separation)
    assert is_collision_distance(0.0)
    assert is_collision_distance(-positive_separation)


@pytest.mark.parametrize(
    ("name", "native_type"),
    [
        ("FIRST", ContactTestType_FIRST),
        ("CLOSEST", ContactTestType_CLOSEST),
        ("ALL", ContactTestType_ALL),
        ("LIMITED", ContactTestType_LIMITED),
    ],
)
def test_build_contact_request_exposes_every_released_test_type(name, native_type):
    request = build_contact_request(name, None, None, None)

    assert isinstance(request, ContactRequest)
    assert request.type == native_type


def test_build_contact_request_preserves_native_defaults_for_absent_inputs():
    request = build_contact_request(None, None, None, None)

    native_default = ContactRequest()
    assert request.type == native_default.type
    assert request.calculate_distance == native_default.calculate_distance
    assert request.calculate_penetration == native_default.calculate_penetration
    assert request.contact_limit == native_default.contact_limit


def test_build_contact_request_preserves_explicit_false_and_zero():
    request = build_contact_request("LIMITED", False, False, 0)

    assert request.type == ContactTestType_LIMITED
    assert request.calculate_distance is False
    assert request.calculate_penetration is False
    assert request.contact_limit == 0


@pytest.mark.parametrize("value", [0, 1, "false", object()])
@pytest.mark.parametrize("field", ["calculate_distance", "calculate_penetration"])
def test_build_contact_request_requires_exact_booleans(field, value):
    values = {
        "calculate_distance": None,
        "calculate_penetration": None,
    }
    values[field] = value

    with pytest.raises(TesseractContactQueryError, match=field):
        build_contact_request("ALL", values["calculate_distance"], values["calculate_penetration"], None)


@pytest.mark.parametrize("value", [-1, 1.0, True, "1"])
def test_build_contact_request_requires_non_negative_exact_integer_limit(value):
    with pytest.raises(TesseractContactQueryError, match="contact_limit"):
        build_contact_request("LIMITED", None, None, value)


@pytest.mark.parametrize("value", ["UNKNOWN", 1, ContactTestType_ALL])
def test_build_contact_request_rejects_unknown_or_non_string_test_type(value):
    with pytest.raises(TesseractContactQueryError, match="test_type"):
        build_contact_request(value, None, None, None)
