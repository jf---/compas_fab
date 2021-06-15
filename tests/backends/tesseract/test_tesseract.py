from compas.robots import RobotModel

from compas_fab.backends.tesseract.conversions import robot_from_tesseract_env


def test_compas_robot_from_env(get_environment) -> RobotModel:
    env, manip, urdf_pth, srdf_pth = get_environment
    robot = robot_from_tesseract_env(env)
    assert isinstance(robot, RobotModel)

    # TODO
    # robot.get_end_effector_link()
