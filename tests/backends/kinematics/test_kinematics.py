from compas.geometry import allclose

from compas_fab.backends.kinematics.solvers import UR5eKinematics
from compas_fab.backends.kinematics.solvers import Staubli_TX260LKinematics
from compas_fab.backends.kinematics.solvers import ABB_IRB4600_40_255Kinematics


def test_kinematic_functions():
    kin = UR5eKinematics()  # one UR is enough, they are all the same
    q = [0.2, 0.5, 1.4, 1.3, 2.6, 2.3]
    frame = kin.forward(q)
    sol = kin.inverse(frame)
    assert(allclose(sol[0], q))

    kin = Staubli_TX260LKinematics()
    q = [0.2, 0.5, 1.4, 1.3, 2.6, 2.3]
    frame = kin.forward(q)
    sol = kin.inverse(frame)
    assert(allclose(sol[0], q))

    kin = ABB_IRB4600_40_255Kinematics()
    q = [0.2, 0.5, 1.4, 1.3, 2.6, 2.3]
    frame = kin.forward(q)
    sol = kin.inverse(frame)
    assert(allclose(sol[0], q))
