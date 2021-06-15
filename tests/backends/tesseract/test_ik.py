import numpy as np
import numpy.testing as nptest
from tesseract import tesseract_kinematics_kdl, tesseract_common


from compas_fab.backends import tesseract

def test_TesseractInverseKinematics():
    tik = tesseract.TesseractInverseKinematics()

def test_TesseractForwardKinematics():
    tfk = tesseract.TesseractForwardKinematics()

def run_inv_kin_test(inv_kin, fwd_kin):
    pose = np.eye(4)
    pose[2, 3] = 1.306

    seed = np.array([-0.785398, 0.785398, -0.785398, 0.785398, -0.785398, 0.785398, -0.785398])
    solutions = inv_kin.calcInvKin(tesseract_common.Isometry3d(pose), seed)
    assert len(solutions) > 0

    result = fwd_kin.calcFwdKin(solutions[0])

    nptest.assert_almost_equal(pose, result.matrix(), decimal=3)


def test_kdl_kin_chain_lma_inverse_kinematic(get_scene_graph):
    inv_kin = tesseract_kinematics_kdl.KDLInvKinChainLMA()
    fwd_kin = tesseract_kinematics_kdl.KDLFwdKinChain()
    scene_graph = get_scene_graph
    assert inv_kin.init(scene_graph, "base_link", "tool0", "manip")
    assert fwd_kin.init(scene_graph, "base_link", "tool0", "manip")

    run_inv_kin_test(inv_kin, fwd_kin)


def test_kdl_kin_chain_nr_inverse_kinematic(get_scene_graph):
    inv_kin = tesseract_kinematics_kdl.KDLInvKinChainLMA()
    fwd_kin = tesseract_kinematics_kdl.KDLFwdKinChain()
    scene_graph = get_scene_graph
    assert inv_kin.init(scene_graph, "base_link", "tool0", "manip")
    assert fwd_kin.init(scene_graph, "base_link", "tool0", "manip")

    run_inv_kin_test(inv_kin, fwd_kin)


def test_jacobian(get_scene_graph):
    kin = tesseract_kinematics_kdl.KDLFwdKinChain()
    scene_graph = get_scene_graph
    assert kin.init(scene_graph, "base_link", "tool0", "manip")

    jvals = np.array([-0.785398, 0.785398, -0.785398, 0.785398, -0.785398, 0.785398, -0.785398])

    link_name = "tool0"
    jacobian = kin.calcJacobian(jvals, link_name)

    assert jacobian.shape == (6, 7)
