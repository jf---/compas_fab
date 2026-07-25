"""Coordinated REP kinematics: our emitted plugin YAML loads an 8-DOF REPInvKin solver.

Mirrors the ROP contract in test_coupled_kinematics.py for the robot-with-external-
positioner topology on abb_irb2400_external_positioner: two prismatic positioner
joints plus six arm joints, as one coupled system, in positioner-forward order.
"""

from pathlib import Path

import tesseract_robotics
from tesseract_robotics import tesseract_kinematics
from tesseract_robotics.tesseract_common import GeneralResourceLocator
from tesseract_robotics.tesseract_environment import Environment

from compas_fab.backends.tesseract.artifact import KINEMATICS_PLUGIN_URL
from compas_fab.backends.tesseract.artifact import CoupledKinematics
from compas_fab.backends.tesseract.artifact import CoupledTopology
from compas_fab.backends.tesseract.artifact import OpwParameters
from compas_fab.backends.tesseract.artifact import RobotArtifact

_SUPPORT_URDF = Path(tesseract_robotics.__file__).parent / "data" / "tesseract" / "support" / "urdf"
_REFERENCE = "abb_irb2400_external_positioner"
_FULL_GROUP = "full_manipulator"
# Positioner-forward (KDL) order: the two positioner joints first, then the six arm joints.
_EXPECTED_JOINTS = ["positioner_joint_1", "positioner_joint_2", "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

# IRB2400 OPW constants, from tesseract's online_planning_example_plugins.yaml.
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


def _reference_rep_config() -> CoupledKinematics:
    return CoupledKinematics.build(
        group=_FULL_GROUP,
        topology=CoupledTopology.ROBOT_WITH_EXTERNAL_POSITIONER,
        positioner_base_link="positioner_base_link",
        positioner_tip_link="positioner_tool0",
        manipulator_base_link="base_link",
        manipulator_tip_link="tool0",
        manipulator=OpwParameters.build(**_IRB2400_OPW),
        manipulator_reach=2.55,
        positioner_sample_resolution=[("positioner_joint_1", 0.1), ("positioner_joint_2", 0.1)],
    )


def test_emitted_rep_yaml_loads_coordinated_eight_dof_solver():
    urdf = (_SUPPORT_URDF / f"{_REFERENCE}.urdf").read_text()
    srdf = (_SUPPORT_URDF / f"{_REFERENCE}.srdf").read_text()

    locator = GeneralResourceLocator()
    env = Environment()
    assert env.initFromUrdfSrdf(urdf, srdf, locator), "reference REP cell failed to initialize"

    artifact = RobotArtifact.build(urdf, srdf, {}).with_coupled_kinematics(_reference_rep_config())
    plugin_yaml = artifact.resource(KINEMATICS_PLUGIN_URL).content.decode("utf-8")

    factory = tesseract_kinematics.KinematicsPluginFactory(plugin_yaml, locator)
    scene_graph = env.getSceneGraph()
    scene_state = env.getState()
    solver = factory.createInvKin(_FULL_GROUP, "REPInvKin", scene_graph, scene_state)

    # A single coordinated solver spanning the two positioner joints + the six arm joints.
    # createInvKin honours the positioner_sample_resolution order -> positioner joints first.
    assert solver.numJoints() == 8
    assert list(solver.getJointNames()) == _EXPECTED_JOINTS
