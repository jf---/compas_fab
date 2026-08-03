from compas.geometry import Frame  # type: ignore[import-untyped]
from compas.geometry import axis_angle_from_quaternion
from compas.tolerance import Tolerance  # type: ignore[import-untyped]


# Shared projections must agree within 1 mm and 0.01 rad. This is deliberately
# looser than Tesseract's native tests because PyBullet IK is iterative.
CONTRACT_TOL = Tolerance(
    unit="M",
    absolute=1e-3,
    relative=1e-6,
    angular=1e-2,
)


def assert_frames_close(actual: Frame, expected: Frame) -> None:
    position_error = actual.point.distance_to_point(expected.point)
    delta = expected.to_local_coordinates(actual)
    _, orientation_error = axis_angle_from_quaternion(delta.quaternion)
    assert CONTRACT_TOL.is_zero(position_error)
    assert CONTRACT_TOL.is_angle_zero(orientation_error)
