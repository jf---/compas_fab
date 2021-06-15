from compas.robots import RobotModel
from tesseract import tesseract_kinematics, tesseract_environment
from tesseract.tesseract_common import Isometry3d, ManipulatorInfo
from tesseract.tesseract_environment import Environment

from compas_fab.backends.tesseract import TesseractInverseKinematics
from compas_fab.backends.tesseract.conversions import robot_from_tesseract_env


# def test_compas_robot_from_env_panda(get_panda_env) -> RobotModel:
#     env, manip, urdf_pth, srdf_pth = get_panda_env
#     robot = robot_from_tesseract_env(env)
#     assert isinstance(robot, RobotModel)
#     # TODO
#     # robot.get_end_effector_link()

def test_compas_robot_from_env_iiwa(get_iiwa_env) -> RobotModel:
    env, manip, urdf_pth, srdf_pth = get_iiwa_env
    robot = robot_from_tesseract_env(env)
    assert isinstance(robot, RobotModel)
    # TODO
    # robot.get_end_effector_link()


def test_TesseractInverseKinematics(get_iiwa_env):
    env, manip, urdf_pth, srdf_pth = get_iiwa_env
    # robot = robot_from_tesseract_env(env)
    # tik.robot = robot
    # tik.robot.tesseract_env: Environment

    manip: tesseract_environment.ManipulatorManager = env.getManipulatorManager()

    fk_solvers = [manip.getAvailableFwdKinematicsSolvers()[i] for i in range(manip.getAvailableFwdKinematicsSolvers().size())]
    ik_solvers = [manip.getAvailableInvKinematicsSolvers()[i] for i in range(manip.getAvailableInvKinematicsSolvers().size())]

    mi = ManipulatorInfo()
    mi.manipulator = "manipulator"
    mi.manipulator_ik_solver = "OPWInvKin"
    #
    # fwd_kin: tesseract_kinematics.ForwardKinematics = manip.getFwdKinematicSolver(env.getName())
    # inv_kin: tesseract_kinematics.InverseKinematics = manip.getInvKinematicSolver(ik_solvers[0])
    # joint_names = fwd_kin.getJointNames()
    # cur_state: tesseract_environment.EnvState = env.getCurrentState()
    #
    # print(f"tip of the link: {inv_kin.getTipLinkName()}")
    # print(f"IK solver: {inv_kin.getSolverName()}")
    #
    # pose = Isometry3d()
    # seed = cur_state.joints
    #
    # ik_sol = inv_kin.calcInvKin(pose, seed)
