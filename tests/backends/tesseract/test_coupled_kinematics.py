"""Coordinated external-axis (ROP/REP) kinematics emission.

The contract test is the load-bearing one: the plugin YAML our artifact emits is
fed to Tesseract on the shipped `abb_irb2400_on_positioner` cell and must yield a
coordinated 7-DOF `ROPInvKin` solver (the positioner joint + the six arm joints).
Anything less than a real coupled solve would leave external axes as decoupled
values, which is exactly what world-class support must not do.
"""

import gc
from pathlib import Path

import pytest
import tesseract_robotics
import yaml
from tesseract_robotics import tesseract_kinematics
from tesseract_robotics.tesseract_common import GeneralResourceLocator
from tesseract_robotics.tesseract_environment import Environment

from compas_fab.backends.tesseract.artifact import KINEMATICS_PLUGIN_URL
from compas_fab.backends.tesseract.artifact import CoupledKinematics
from compas_fab.backends.tesseract.artifact import CoupledTopology
from compas_fab.backends.tesseract.artifact import OpwParameters
from compas_fab.backends.tesseract.artifact import RobotArtifact
from compas_fab.backends.tesseract.errors import InvalidKinematicsConfigError
from compas_fab.backends.tesseract.errors import KinematicsPluginConflictError

# The shipped reference cell: an IRB2400 riding a prismatic positioner (a track
# carrying the robot -> ROP). Its combined group spans positioner + arm.
_TESSERACT_DATA = Path(tesseract_robotics.__file__).parent / "data" / "tesseract"
_SUPPORT_URDF = _TESSERACT_DATA / "support" / "urdf"
_REFERENCE = "abb_irb2400_on_positioner"
_FULL_GROUP = "full_manipulator"
_EXPECTED_JOINTS = ["positioner_joint_1", "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

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


def _reference_rop_config():
    return CoupledKinematics.build(
        group=_FULL_GROUP,
        topology=CoupledTopology.ROBOT_ON_POSITIONER,
        positioner_base_link="positioner_base_link",
        positioner_tip_link="base_link",
        manipulator_base_link="base_link",
        manipulator_tip_link="tool0",
        manipulator=OpwParameters.build(**_IRB2400_OPW),
        manipulator_reach=2.55,
        positioner_sample_resolution=[("positioner_joint_1", 0.1)],
    )


# --- factory invariants (no native load) ------------------------------------


def test_opw_parameters_reject_wrong_offset_count():
    bad = dict(_IRB2400_OPW, offsets=[0.0, 0.0, 0.0])
    with pytest.raises(InvalidKinematicsConfigError):
        OpwParameters.build(**bad)


def test_opw_parameters_reject_non_unit_sign_correction():
    bad = dict(_IRB2400_OPW, sign_corrections=[1, 1, 2, 1, 1, 1])
    with pytest.raises(InvalidKinematicsConfigError):
        OpwParameters.build(**bad)


def test_coupled_kinematics_reject_nonpositive_reach():
    with pytest.raises(InvalidKinematicsConfigError):
        CoupledKinematics.build(
            group=_FULL_GROUP,
            topology=CoupledTopology.ROBOT_ON_POSITIONER,
            positioner_base_link="positioner_base_link",
            positioner_tip_link="base_link",
            manipulator_base_link="base_link",
            manipulator_tip_link="tool0",
            manipulator=OpwParameters.build(**_IRB2400_OPW),
            manipulator_reach=0.0,
            positioner_sample_resolution=[("positioner_joint_1", 0.1)],
        )


def test_coupled_kinematics_reject_empty_sample_resolution():
    with pytest.raises(InvalidKinematicsConfigError):
        CoupledKinematics.build(
            group=_FULL_GROUP,
            topology=CoupledTopology.ROBOT_ON_POSITIONER,
            positioner_base_link="positioner_base_link",
            positioner_tip_link="base_link",
            manipulator_base_link="base_link",
            manipulator_tip_link="tool0",
            manipulator=OpwParameters.build(**_IRB2400_OPW),
            manipulator_reach=2.55,
            positioner_sample_resolution=[],
        )


def test_with_coupled_kinematics_rejects_unknown_group():
    srdf = (_SUPPORT_URDF / f"{_REFERENCE}.srdf").read_text()
    urdf = (_SUPPORT_URDF / f"{_REFERENCE}.urdf").read_text()
    artifact = RobotArtifact.build(urdf, srdf, {})
    config = CoupledKinematics.build(
        group="does_not_exist",
        topology=CoupledTopology.ROBOT_ON_POSITIONER,
        positioner_base_link="positioner_base_link",
        positioner_tip_link="base_link",
        manipulator_base_link="base_link",
        manipulator_tip_link="tool0",
        manipulator=OpwParameters.build(**_IRB2400_OPW),
        manipulator_reach=2.55,
        positioner_sample_resolution=[("positioner_joint_1", 0.1)],
    )
    with pytest.raises(InvalidKinematicsConfigError):
        artifact.with_coupled_kinematics(config)


def test_with_coupled_kinematics_conflicts_with_existing_plugin():
    urdf = (_SUPPORT_URDF / f"{_REFERENCE}.urdf").read_text()
    srdf = (_SUPPORT_URDF / f"{_REFERENCE}.srdf").read_text()
    artifact = RobotArtifact.build(urdf, srdf, {}).with_coupled_kinematics(_reference_rop_config())
    with pytest.raises(KinematicsPluginConflictError):
        artifact.with_coupled_kinematics(_reference_rop_config())


# --- emission structure ------------------------------------------------------


def test_coupled_yaml_names_rop_factory_and_both_search_libraries():
    urdf = (_SUPPORT_URDF / f"{_REFERENCE}.urdf").read_text()
    srdf = (_SUPPORT_URDF / f"{_REFERENCE}.srdf").read_text()
    artifact = RobotArtifact.build(urdf, srdf, {}).with_coupled_kinematics(_reference_rop_config())

    document = yaml.safe_load(artifact.resource(KINEMATICS_PLUGIN_URL).content.decode("utf-8"))
    plugins = document["kinematic_plugins"]
    assert plugins["search_libraries"] == [
        "tesseract_kinematics_kdl_factories",
        "tesseract_kinematics_opw_factories",
    ]
    group = plugins["inv_kin_plugins"][_FULL_GROUP]
    assert group["default"] == "ROPInvKin"
    assert group["plugins"]["ROPInvKin"]["class"] == "ROPInvKinFactory"
    assert group["plugins"]["ROPInvKin"]["config"]["manipulator"]["class"] == "OPWInvKinFactory"


# --- the contract: our emitted YAML loads a coordinated solver ---------------


def test_emitted_yaml_loads_coordinated_seven_dof_solver():
    urdf = (_SUPPORT_URDF / f"{_REFERENCE}.urdf").read_text()
    srdf = (_SUPPORT_URDF / f"{_REFERENCE}.srdf").read_text()

    locator = GeneralResourceLocator()
    env = Environment()
    assert env.initFromUrdfSrdf(urdf, srdf, locator), "reference cell failed to initialize"

    artifact = RobotArtifact.build(urdf, srdf, {}).with_coupled_kinematics(_reference_rop_config())
    plugin_yaml = artifact.resource(KINEMATICS_PLUGIN_URL).content.decode("utf-8")

    factory = tesseract_kinematics.KinematicsPluginFactory(plugin_yaml, locator)
    scene_graph = env.getSceneGraph()
    scene_state = env.getState()
    solver = factory.createInvKin(_FULL_GROUP, "ROPInvKin", scene_graph, scene_state)
    try:
        num_joints = solver.numJoints()
        joint_names = list(solver.getJointNames())
    finally:
        # The native solver holds raw pointers into the scene graph/state; if the
        # scene is freed before the solver, teardown double-frees and segfaults.
        # Release in reverse dependency order (solver -> scene -> env) so pytest's
        # GC of these locals cannot crash the interpreter mid-suite. This is a
        # tesseract_nanobind ownership gap (createInvKin should keep the scene
        # alive); the ordered release is the workaround until the binding is fixed.
        del solver, scene_state, scene_graph, factory, env, locator
        gc.collect()

    # A single coordinated solver spanning the positioner joint + the six arm joints.
    assert num_joints == 7
    assert joint_names == _EXPECTED_JOINTS
