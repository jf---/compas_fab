import numpy as np
import pytest
from compas_robots.model import Joint
from tesseract_robotics.planning.composer import PlanningResult
from tesseract_robotics.planning.composer import TrajectoryPoint
from tesseract_robotics.tesseract_command_language import CompositeInstruction
from tesseract_robotics.tesseract_command_language import ProfileDictionary
from tesseract_robotics.tesseract_command_language import SetDigitalInstruction

from compas_fab.backends.tesseract.conversions import joint_trajectory_from_result
from compas_fab.backends.tesseract.errors import InconsistentTesseractJointOrderError
from compas_fab.backends.tesseract.errors import MissingTesseractJointTypeError
from compas_fab.backends.tesseract.errors import MissingTesseractTrajectoryFieldError
from compas_fab.backends.tesseract.errors import NonMonotonicTesseractTrajectoryError
from compas_fab.backends.tesseract.native import TesseractPlanningRequest
from compas_fab.backends.tesseract.native import TesseractPlanningResult


JOINT_TYPES = {"revolute": Joint.REVOLUTE, "prismatic": Joint.PRISMATIC}


def _result(points):
    program = CompositeInstruction("DEFAULT")
    program.push_back(SetDigitalInstruction("do_test", 0, True))
    request = TesseractPlanningRequest.build(
        program,
        "FreespacePipeline",
        ProfileDictionary(),
        False,
    )
    return TesseractPlanningResult.build(
        request,
        PlanningResult(successful=True, trajectory=points, raw_results=program),
    )


def _point(names=None, positions=None, velocities=None, accelerations=None, time=0.0):
    return TrajectoryPoint(
        joint_names=names or ["revolute", "prismatic"],
        positions=np.array(positions or [0.0, 0.1]),
        velocities=np.array(velocities or [0.2, 0.3]),
        accelerations=np.array(accelerations or [0.4, 0.5]),
        time=time,
    )


def test_projection_preserves_native_points_without_mutation_or_resampling():
    points = [_point(time=0.0), _point(positions=[0.6, 0.7], time=1.25)]
    positions_before = [point.positions.copy() for point in points]

    trajectory = joint_trajectory_from_result(_result(points), JOINT_TYPES)

    assert trajectory.joint_names == ["revolute", "prismatic"]
    assert len(trajectory.points) == len(points)
    assert list(trajectory.points[1].joint_values) == [0.6, 0.7]
    assert list(trajectory.points[1].joint_types) == [Joint.REVOLUTE, Joint.PRISMATIC]
    assert list(trajectory.points[1].velocities) == [0.2, 0.3]
    assert list(trajectory.points[1].accelerations) == [0.4, 0.5]
    assert trajectory.points[1].time_from_start.seconds == 1.25
    assert trajectory.attributes["tesseract_time_seconds"] == [0.0, 1.25]
    assert all(np.array_equal(point.positions, before) for point, before in zip(points, positions_before))


def test_inconsistent_joint_order_fails_loudly():
    points = [_point(), _point(names=["prismatic", "revolute"], positions=[0.1, 0.2], time=1.0)]

    with pytest.raises(InconsistentTesseractJointOrderError):
        joint_trajectory_from_result(_result(points), JOINT_TYPES)


def test_missing_joint_type_fails_loudly():
    with pytest.raises(MissingTesseractJointTypeError, match="prismatic"):
        joint_trajectory_from_result(_result([_point()]), {"revolute": Joint.REVOLUTE})


@pytest.mark.parametrize("field", ["velocities", "accelerations", "time"])
def test_missing_native_dynamics_fails_loudly(field):
    point = _point()
    setattr(point, field, None)

    with pytest.raises(MissingTesseractTrajectoryFieldError, match=field):
        joint_trajectory_from_result(_result([point]), JOINT_TYPES)


def test_decreasing_native_time_fails_loudly():
    points = [_point(time=1.0), _point(time=0.5)]

    with pytest.raises(NonMonotonicTesseractTrajectoryError):
        joint_trajectory_from_result(_result(points), JOINT_TYPES)
