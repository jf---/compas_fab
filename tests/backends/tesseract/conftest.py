import os
from pathlib import Path
from typing import Callable

import pytest
import tesseract
from tesseract import tesseract_scene_graph, tesseract_urdf
from tesseract.tesseract_common import ManipulatorInfo
from tesseract.tesseract_environment import Environment

from compas_fab.backends.tesseract.utils import _locate_resource


@pytest.fixture(scope="session")
def tesseract_support_path() -> Path:
    _tesseract_support_pth = Path(tesseract.__file__).joinpath(
        "..", "..", "..", "..", "opt", "tesseract_robotics", "share", "tesseract_support").resolve()

    assert _tesseract_support_pth.exists(), f"tesseract_support expected at {_tesseract_support_pth}, but not found"
    os.environ["TESSERACT_SUPPORT_DIR"] = str(_tesseract_support_pth)
    return _tesseract_support_pth


@pytest.fixture()
def hhh(get_environment):
    env, manip, urdf_pth, srdf_pth = get_environment
    fwd_kin = env.getManipulatorManager().getFwdKinematicSolver(manip.manipulator)
    inv_kin = env.getManipulatorManager().getInvKinematicSolver(manip.manipulator)
    joint_names = fwd_kin.getJointNames()
    cur_state = env.getCurrentState()
    print(env)


@pytest.fixture(scope="session")
def iiwa_urdf(tesseract_support_path) -> Path:
    pth = tesseract_support_path.joinpath("urdf", "lbr_iiwa_14_r820.urdf").resolve()
    assert pth.is_file()
    return pth


@pytest.fixture(scope="session")
def iiwa_srdf(tesseract_support_path) -> Path:
    pth = tesseract_support_path.joinpath("urdf", "lbr_iiwa_14_r820.srdf").resolve()
    assert pth.is_file()
    return pth


@pytest.fixture(scope="session")
def get_environment() -> Callable:
    def _get_environment(urdf_pth: Path, srdf_pth: Path):
        from compas_fab.backends.tesseract import load_env
        assert urdf_pth.is_file()
        assert srdf_pth.is_file()
        # too easy to mix up one for the other, so check
        assert "srdf" in srdf_pth.suffix.lower()
        assert "urdf" in urdf_pth.suffix.lower()

        env = load_env(urdf_pth, srdf_pth)
        manip_info = ManipulatorInfo()
        manip_info.manipulator = "manipulator"
        manip_info.manipulator_ik_solver = "OPWInvKin"
        return env, manip_info, urdf_pth, srdf_pth

    return _get_environment


@pytest.fixture(scope="session")
def get_iiwa_env(iiwa_urdf, iiwa_srdf, get_environment) -> Environment:
    return get_environment(iiwa_urdf, iiwa_srdf)


@pytest.fixture(scope="session")
def get_panda_env(panda_urdf, panda_srdf, get_environment) -> Environment:
    return get_environment(iiwa_urdf, iiwa_srdf)

@pytest.fixture(scope="session")
def get_scene_graph(iiwa_urdf):
    locator_fn = tesseract_scene_graph.SimpleResourceLocatorFn(_locate_resource)
    locator = tesseract_scene_graph.SimpleResourceLocator(locator_fn)
    return tesseract_urdf.parseURDFFile(str(iiwa_urdf), locator)

