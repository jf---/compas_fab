import pytest
from compas_robots import RobotModel
from tesseract_robotics.planning import TaskComposer

from compas_fab.backends.tesseract import client as client_module
from compas_fab.backends.tesseract.artifact import CollisionMeshPolicy
from compas_fab.backends.tesseract.artifact import ContinuousContactManager
from compas_fab.backends.tesseract.artifact import DiscreteContactManager
from compas_fab.backends.tesseract.artifact import KdlInverseKinematics
from compas_fab.backends.tesseract.artifact import KdlKinematics
from compas_fab.backends.tesseract.artifact import RobotArtifact
from compas_fab.backends.tesseract.client import TesseractClient
from compas_fab.backends.tesseract.errors import InvalidTesseractRuntimeConfigurationError
from compas_fab.backends.tesseract.errors import RobotArtifactMismatchError
from compas_fab.backends.tesseract.errors import TesseractCellStateMismatchError
from compas_fab.backends.tesseract.errors import TesseractRuntimeInitializationError
from compas_fab.backends.tesseract.planner import TesseractPlanner
from compas_fab.robots import RobotCell
from compas_fab.robots import RobotSemantics

from .conftest import SRDF
from .conftest import URDF


def test_client_lifecycle_owns_initialized_environment(tesseract_artifact, tmp_path):
    client = TesseractClient(tesseract_artifact, cache_root=tmp_path)

    assert not client.is_connected
    client.connect()

    assert client.is_connected
    assert client.environment.artifact is tesseract_artifact

    client.disconnect()
    assert not client.is_connected


def test_client_context_manager_connects_and_disconnects(tesseract_artifact, tmp_path):
    client = TesseractClient(tesseract_artifact, cache_root=tmp_path)

    with client as connected:
        assert connected is client
        assert client.is_connected

    assert not client.is_connected


def test_client_retains_exact_caller_supplied_task_composer(tesseract_artifact, tmp_path):
    composer = TaskComposer.from_config(warmup=False)

    with TesseractClient(tesseract_artifact, cache_root=tmp_path, composer=composer) as client:
        assert client.runtime.composer is composer


def test_client_rejects_non_native_task_composer(tesseract_artifact):
    with pytest.raises(InvalidTesseractRuntimeConfigurationError, match="TaskComposer"):
        TesseractClient(tesseract_artifact, composer=object())


def test_set_robot_cell_stores_an_input_copy(tesseract_artifact, one_joint_cell, tmp_path):
    with TesseractClient(tesseract_artifact, cache_root=tmp_path) as client:
        planner = TesseractPlanner(client)

        planner.set_robot_cell(one_joint_cell)

        assert client.robot_cell is not one_joint_cell
        assert client.robot_cell.structural_signature() == one_joint_cell.structural_signature()
        assert client.robot_cell.group_names == one_joint_cell.group_names


def test_native_scene_revision_changes_only_with_exact_projection_content(
    tesseract_artifact,
    one_joint_cell,
    one_joint_state,
    tmp_path,
):
    with TesseractClient(tesseract_artifact, cache_root=tmp_path) as client:
        planner = TesseractPlanner(client)
        assert planner.native_scene_revision == 0

        planner.set_robot_cell(one_joint_cell, one_joint_state)
        assert planner.native_scene_revision == 1

        planner.set_robot_cell(one_joint_cell.copy(), one_joint_state.copy())
        assert planner.native_scene_revision == 1

        changed = one_joint_state.copy()
        changed.robot_configuration["joint1"] = 0.5
        planner.set_robot_cell(one_joint_cell, changed)
        assert planner.native_scene_revision == 2

        moved = changed.copy()
        moved.robot_base_frame.point.x = 1.0
        planner.set_robot_cell(one_joint_cell, moved)
        assert planner.native_scene_revision == 3


@pytest.mark.parametrize(
    ("original", "replacement", "diagnostic"),
    [
        ('type="revolute"', 'type="prismatic"', "type"),
        ('axis xyz="0 0 1"', 'axis xyz="1 0 0"', "axis"),
        ('origin xyz="0 0 1"', 'origin xyz="0.1 0 1"', "origin"),
        ('upper="3.14"', 'upper="2.5"', "limits"),
    ],
)
def test_set_robot_cell_rejects_structural_joint_mismatch(
    tesseract_artifact,
    tmp_path,
    original,
    replacement,
    diagnostic,
):
    mismatched_urdf = URDF.replace(original, replacement)
    model = RobotModel.from_urdf_string(mismatched_urdf)
    cell = RobotCell(model, RobotSemantics.from_srdf_string(SRDF, model))

    with TesseractClient(tesseract_artifact, cache_root=tmp_path) as client:
        planner = TesseractPlanner(client)

        with pytest.raises(RobotArtifactMismatchError, match=diagnostic):
            planner.set_robot_cell(cell)


def test_client_clone_applies_complete_stored_robot_state(
    tesseract_artifact,
    one_joint_cell,
    one_joint_state,
    tmp_path,
):
    one_joint_state.robot_configuration["joint1"] = 0.75

    with TesseractClient(tesseract_artifact, cache_root=tmp_path) as client:
        planner = TesseractPlanner(client)
        planner.set_robot_cell(one_joint_cell, one_joint_state)

        native_robot = client.clone_robot()

    assert native_robot.env.getCurrentJointValuesByNames(["joint1"]).tolist() == pytest.approx([0.75])


def test_set_robot_cell_rejects_state_omitting_registered_tools(
    tesseract_artifact,
    one_joint_cell,
    one_joint_state,
    tmp_path,
):
    one_joint_cell.tool_models["welder"] = object()

    with TesseractClient(tesseract_artifact, cache_root=tmp_path) as client:
        planner = TesseractPlanner(client)

        with pytest.raises(TesseractCellStateMismatchError, match="welder"):
            planner.set_robot_cell(one_joint_cell, one_joint_state)


def test_set_robot_cell_rejects_compas_tip_different_from_exact_srdf_tip(tmp_path):
    urdf = """<robot name="branched_tip">
  <link name="base"/><link name="arm"/><link name="tip_a"/><link name="tip_b"/>
  <joint name="joint1" type="revolute">
    <parent link="base"/><child link="arm"/><axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
  <joint name="fixed_a" type="fixed"><parent link="arm"/><child link="tip_a"/></joint>
  <joint name="fixed_b" type="fixed"><parent link="arm"/><child link="tip_b"/></joint>
</robot>"""
    artifact_srdf = """<robot name="branched_tip">
  <group name="manipulator"><chain base_link="base" tip_link="tip_a"/></group>
</robot>"""
    compas_srdf = artifact_srdf.replace('tip_link="tip_a"', 'tip_link="tip_b"')
    artifact = RobotArtifact.from_compas_urdf(
        urdf,
        artifact_srdf,
        {},
        CollisionMeshPolicy.PRESERVE,
    ).with_kdl_kinematics([KdlKinematics.build("manipulator", "base", "tip_a", KdlInverseKinematics.LMA)])
    artifact = artifact.with_contact_managers(
        DiscreteContactManager.BULLET_BVH,
        ContinuousContactManager.BULLET_CAST_BVH,
    )
    model = RobotModel.from_urdf_string(urdf)
    cell = RobotCell(model, RobotSemantics.from_srdf_string(compas_srdf, model))

    with TesseractClient(artifact, cache_root=tmp_path) as client:
        planner = TesseractPlanner(client)

        with pytest.raises(RobotArtifactMismatchError, match="tip link"):
            planner.set_robot_cell(cell)


def test_client_allows_kinematics_artifact_without_contact_managers(kdl_artifact, tmp_path):
    client = TesseractClient(kdl_artifact, cache_root=tmp_path)

    try:
        client.connect()
        assert client.is_connected
    finally:
        client.disconnect()

    assert not client.is_connected


def test_client_wraps_native_task_composer_initialization(monkeypatch, tesseract_artifact, tmp_path):
    class FailingTaskComposer:
        @staticmethod
        def from_config(*, warmup):
            raise RuntimeError("native composer failure")

    monkeypatch.setattr(client_module, "TaskComposer", FailingTaskComposer)
    client = TesseractClient(tesseract_artifact, cache_root=tmp_path)

    with pytest.raises(TesseractRuntimeInitializationError, match="native composer failure"):
        client.connect()

    assert not client.is_connected


def test_set_robot_cell_validates_jointless_semantic_group_from_exact_srdf(tmp_path):
    srdf = SRDF.replace(
        "</robot>",
        '  <group name="endeffector"><link name="tip"/></group>\n</robot>',
    )
    model = RobotModel.from_urdf_string(URDF)
    cell = RobotCell(model, RobotSemantics.from_srdf_string(srdf, model))
    artifact = RobotArtifact.from_compas_urdf(
        URDF,
        srdf,
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

    with TesseractClient(artifact, cache_root=tmp_path) as client:
        planner = TesseractPlanner(client)
        planner.set_robot_cell(cell)

    assert client.robot_cell.group_names == ["manipulator", "endeffector"]
