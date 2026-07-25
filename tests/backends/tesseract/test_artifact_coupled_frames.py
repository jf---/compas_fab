"""RobotArtifact coupled-group detection and topology-aware working/TCP frames."""

from pathlib import Path

import tesseract_robotics

from compas_fab.backends.tesseract.artifact import CoupledKinematics
from compas_fab.backends.tesseract.artifact import CoupledTopology
from compas_fab.backends.tesseract.artifact import OpwParameters
from compas_fab.backends.tesseract.artifact import RobotArtifact

_SUPPORT_URDF = Path(tesseract_robotics.__file__).parent / "data" / "tesseract" / "support" / "urdf"
_REP = "abb_irb2400_external_positioner"
_ROP = "abb_irb2400_on_positioner"
_IRB2400_OPW = dict(
    a1=0.100,
    a2=-0.135,
    b=0.00,
    c1=0.615,
    c2=0.705,
    c3=0.755,
    c4=0.086,
    offsets=[0.0, 0.0, -1.57079632679, 0.0, 0.0, 0.0],
    sign_corrections=[1, 1, 1, 1, 1, 1],
)


def _rep_artifact() -> RobotArtifact:
    urdf = (_SUPPORT_URDF / f"{_REP}.urdf").read_text()
    srdf = (_SUPPORT_URDF / f"{_REP}.srdf").read_text()
    config = CoupledKinematics.build(
        group="full_manipulator",
        topology=CoupledTopology.ROBOT_WITH_EXTERNAL_POSITIONER,
        positioner_base_link="positioner_base_link",
        positioner_tip_link="positioner_tool0",
        manipulator_base_link="base_link",
        manipulator_tip_link="tool0",
        manipulator=OpwParameters.build(**_IRB2400_OPW),
        manipulator_reach=2.55,
        positioner_sample_resolution=[("positioner_joint_1", 0.1), ("positioner_joint_2", 0.1)],
    )
    return RobotArtifact.build(urdf, srdf, {}).with_coupled_kinematics(config)


def _rop_artifact() -> RobotArtifact:
    urdf = (_SUPPORT_URDF / f"{_ROP}.urdf").read_text()
    srdf = (_SUPPORT_URDF / f"{_ROP}.srdf").read_text()
    config = CoupledKinematics.build(
        group="full_manipulator",
        topology=CoupledTopology.ROBOT_ON_POSITIONER,
        positioner_base_link="positioner_base_link",
        positioner_tip_link="base_link",
        manipulator_base_link="base_link",
        manipulator_tip_link="tool0",
        manipulator=OpwParameters.build(**_IRB2400_OPW),
        manipulator_reach=2.55,
        positioner_sample_resolution=[("positioner_joint_1", 0.1)],
    )
    return RobotArtifact.build(urdf, srdf, {}).with_coupled_kinematics(config)


def test_is_coupled_group_true_for_rep_full_manipulator():
    assert _rep_artifact().is_coupled_group("full_manipulator") is True


def test_is_coupled_group_false_for_uncoupled_group():
    # 'manipulator' has no inv-kin plugin entry -> default solver is None -> not coupled.
    assert _rep_artifact().is_coupled_group("manipulator") is False


def test_coupled_group_frames_rep_uses_positioner_tip_and_tool0():
    assert _rep_artifact().coupled_group_frames("full_manipulator") == ("positioner_tool0", "tool0")


def test_coupled_group_frames_rop_uses_positioner_base_and_tool0():
    assert _rop_artifact().coupled_group_frames("full_manipulator") == ("positioner_base_link", "tool0")


def test_coupled_group_frames_none_for_uncoupled_group():
    assert _rep_artifact().coupled_group_frames("manipulator") is None
