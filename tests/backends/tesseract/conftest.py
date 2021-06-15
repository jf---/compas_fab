import os
import re
import sys
import traceback
from pathlib import Path
from typing import Tuple

import pytest
import tesseract
from tesseract.tesseract_common import ManipulatorInfo, FilesystemPath
from tesseract.tesseract_environment import Environment
from tesseract.tesseract_scene_graph import SimpleResourceLocatorFn, SimpleResourceLocator

_tesseract_support_pth = Path(tesseract.__file__).joinpath(
    "..", "..", "..", "..", "opt", "tesseract_robotics", "share", "tesseract_support").resolve()

assert _tesseract_support_pth.exists(), f"tesseract_support expected at {_tesseract_support_pth}, but not found"

os.environ["TESSERACT_SUPPORT_DIR"] = str(_tesseract_support_pth)


def _locate_resource(url):
    try:
        url_match = re.match(r"^package:\/\/tesseract_support\/(.*)$", url)
        if (url_match is None):
            return ""
        if not "TESSERACT_SUPPORT_DIR" in os.environ:
            return ""
        tesseract_support = os.environ["TESSERACT_SUPPORT_DIR"]
        return os.path.join(tesseract_support, os.path.normpath(url_match.group(1)))
    except:
        traceback.print_exc()


@pytest.fixture
def get_environment() -> Tuple[Environment, ManipulatorInfo, Path, Path]:
    locate_resource_fn = SimpleResourceLocatorFn(_locate_resource)
    locator = SimpleResourceLocator(locate_resource_fn)
    env = Environment()
    tesseract_support = os.environ["TESSERACT_SUPPORT_DIR"]
    urdf_path = FilesystemPath(os.path.join(tesseract_support, "urdf", "lbr_iiwa_14_r820.urdf"))
    srdf_path = FilesystemPath(os.path.join(tesseract_support, "urdf", "lbr_iiwa_14_r820.srdf"))

    if "win" in sys.platform:
        # replace / -> \
        print("windows")
        urdf_pth = Path(urdf_path.string().replace("/", "\\")).resolve()
        srdf_pth = Path(srdf_path.string().replace("/", "\\")).resolve()
    else:
        urdf_pth = Path(urdf_path.string()).resolve()
        srdf_pth = Path(srdf_path.string()).resolve()

    assert urdf_pth.is_file(), f"{urdf_path} invalid path"
    assert srdf_pth.is_file(), f"{srdf_path} invalid path"

    assert env.init(urdf_path, srdf_path, locator)
    manip_info = ManipulatorInfo()
    manip_info.manipulator = "manipulator"
    manip_info.manipulator_ik_solver = "OPWInvKin"

    return env, manip_info, urdf_pth, srdf_pth


@pytest.fixture()
def hhh(get_environment):
    env, manip, urdf_pth, srdf_pth = get_environment
    fwd_kin = env.getManipulatorManager().getFwdKinematicSolver(manip.manipulator)
    inv_kin = env.getManipulatorManager().getInvKinematicSolver(manip.manipulator)
    joint_names = fwd_kin.getJointNames()
    cur_state = env.getCurrentState()
    print(env)
