import numpy as np
import pytest
from compas.geometry import Frame
from compas.tolerance import TOL
from tesseract_robotics.planning import Pose

from compas_fab.backends.tesseract.errors import InvalidTesseractPoseError
from compas_fab.backends.tesseract.native_pose import pose_from_user_frame


def test_pose_converts_only_position_from_millimetres_to_metres():
    pose = pose_from_user_frame(
        Frame([1000, 2000, 3000], [0, 1, 0], [-1, 0, 0]),
        0.001,
    )

    assert isinstance(pose, Pose)
    np.testing.assert_allclose(pose.translation, [1.0, 2.0, 3.0])
    np.testing.assert_allclose(
        pose.rotation_matrix,
        [[0, -1, 0], [1, 0, 0], [0, 0, 1]],
        atol=TOL.absolute,
        rtol=TOL.relative,
    )


@pytest.mark.parametrize(
    "scale",
    [None, True, 0.0, -1.0, float("inf"), float("nan")],
)
def test_pose_requires_explicit_finite_positive_scale(scale):
    with pytest.raises(InvalidTesseractPoseError):
        pose_from_user_frame(Frame.worldXY(), scale)


def test_pose_rejects_non_frame():
    with pytest.raises(InvalidTesseractPoseError):
        pose_from_user_frame(object(), 1.0)


def test_pose_rejects_non_finite_coordinates():
    frame = Frame.worldXY()
    frame.point.x = float("nan")

    with pytest.raises(InvalidTesseractPoseError):
        pose_from_user_frame(frame, 1.0)
