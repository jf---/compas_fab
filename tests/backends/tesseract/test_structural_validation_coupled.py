"""A full three-group REP cell loads through set_robot_cell (coupled branch)."""

import xml.etree.ElementTree as ElementTree
from pathlib import Path

import tesseract_robotics
from compas_robots import Configuration
from compas_robots import RobotModel
from compas_robots.model import Joint

from compas_fab.backends.tesseract.artifact import CollisionMeshPolicy
from compas_fab.backends.tesseract.artifact import CoupledKinematics
from compas_fab.backends.tesseract.artifact import CoupledTopology
from compas_fab.backends.tesseract.artifact import OpwParameters
from compas_fab.backends.tesseract.artifact_loader import ResourceRoot
from compas_fab.backends.tesseract.artifact_loader import RobotArtifactLoader
from compas_fab.backends.tesseract.client import TesseractClient
from compas_fab.backends.tesseract.planner import TesseractPlanner
from compas_fab.robots import RobotCell
from compas_fab.robots import RobotCellState
from compas_fab.robots import RobotSemantics

_DATA = Path(tesseract_robotics.__file__).parent / "data" / "tesseract"
_URDF = _DATA / "support" / "urdf" / "abb_irb2400_external_positioner.urdf"
_SRDF = _DATA / "support" / "urdf" / "abb_irb2400_external_positioner.srdf"
_RESOURCE_ROOT = _DATA.parent
_FULL_GROUP = "full_manipulator"
_COUPLED_JOINTS = ["positioner_joint_1", "positioner_joint_2", "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
_COUPLED_TYPES = [Joint.PRISMATIC, Joint.PRISMATIC, Joint.REVOLUTE, Joint.REVOLUTE, Joint.REVOLUTE, Joint.REVOLUTE, Joint.REVOLUTE, Joint.REVOLUTE]
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


def _joint_list_srdf() -> str:
    """The shipped SRDF with full_manipulator rewritten to a positioner-forward <joint> list.

    Sidesteps COMPAS's cross-fork chain walker and aligns the scene-graph joint order
    with the KDL solver so calcInvKin and calcFwdKin agree. All three groups are kept.
    """
    root = ElementTree.fromstring(_SRDF.read_text())
    for group in root.findall("group"):
        if group.get("name") == _FULL_GROUP:
            for chain in list(group.findall("chain")):
                group.remove(chain)
            for joint_name in _COUPLED_JOINTS:
                ElementTree.SubElement(group, "joint", {"name": joint_name})
    return ElementTree.tostring(root, encoding="unicode")


def _rep_config() -> CoupledKinematics:
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


def _rep_artifact(tmp_path):
    srdf_path = tmp_path / "rep_jointgroup.srdf"
    srdf_path.write_text(_joint_list_srdf())
    loader = RobotArtifactLoader.build(_URDF, srdf_path, [ResourceRoot.build(_RESOURCE_ROOT)])
    return loader.load(CollisionMeshPolicy.CONVEX_HULL).with_coupled_kinematics(_rep_config())


def _rep_cell() -> RobotCell:
    model = RobotModel.from_urdf_string(_URDF.read_text())
    semantics = RobotSemantics.from_srdf_string(_joint_list_srdf(), model)
    return RobotCell(model, semantics)


def test_full_three_group_rep_cell_loads_through_set_robot_cell(tmp_path):
    cell = _rep_cell()
    assert set(cell.group_names) == {"manipulator", "positioner", "full_manipulator"}
    state = RobotCellState.from_robot_cell(cell)
    state.robot_configuration = Configuration([0.0] * 8, _COUPLED_TYPES, _COUPLED_JOINTS)

    with TesseractClient(_rep_artifact(tmp_path), cache_root=tmp_path) as client:
        planner = TesseractPlanner(client)
        planner.set_robot_cell(cell, state)  # must not raise: coupled branch validates full_manipulator
