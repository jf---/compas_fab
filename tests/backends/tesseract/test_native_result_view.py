from attrs import evolve
import numpy as np
import pytest
from tesseract_robotics.planning import TrajectoryPoint
from tesseract_robotics.planning.composer import PlanningResult
from tesseract_robotics.tesseract_command_language import CompositeInstruction
from tesseract_robotics.tesseract_command_language import ProfileDictionary
from tesseract_robotics.tesseract_command_language import SetDigitalInstruction

from compas_fab.backends.tesseract.errors import MalformedTesseractNativeResultError
from compas_fab.backends.tesseract.native import TesseractPlanningRequest
from compas_fab.backends.tesseract.native import TesseractPlanningResult
from compas_fab.backends.tesseract.native_result_view import TesseractNativeResultView


def _result(points):
    program = CompositeInstruction("DEFAULT")
    program.push_back(SetDigitalInstruction("do_test", 0, True))
    request = TesseractPlanningRequest.build(
        program,
        "DescartesFPipeline",
        ProfileDictionary(),
        False,
    )
    native_result = PlanningResult(
        successful=True,
        message="ok",
        trajectory=points,
        raw_results=program,
    )
    return TesseractPlanningResult.build(request, native_result)


@pytest.fixture
def native_result_without_dynamics():
    return _result(
        [TrajectoryPoint(["joint1"], np.array([0.0]))],
    )


@pytest.fixture
def native_result_with_dynamics():
    return _result(
        [
            TrajectoryPoint(
                ["joint1", "joint2"],
                np.array([0.0, 1.0]),
                np.array([0.0, 0.0]),
                np.array([0.0, 0.0]),
                0.0,
            ),
            TrajectoryPoint(
                ["joint1", "joint2"],
                np.array([0.5, 1.5]),
                np.array([0.1, 0.2]),
                np.array([0.01, 0.02]),
                1.0,
            ),
        ],
    )


def test_result_view_retains_every_exact_native_object(
    native_result_without_dynamics,
):
    view = TesseractNativeResultView.build(
        native_result_without_dynamics,
    )

    assert view.result is native_result_without_dynamics
    assert view.request is native_result_without_dynamics.request
    assert view.native_result is native_result_without_dynamics.native_result
    assert view.raw_program is native_result_without_dynamics.raw_program
    assert len(view.trajectory_points) == 1
    assert view.trajectory_points[0] is native_result_without_dynamics.native_result.trajectory[0]
    assert view.message == "ok"
    assert view.joint_names == ("joint1",)
    assert view.positions == ((0.0,),)
    assert view.velocities is None
    assert view.accelerations is None
    assert view.times is None


def test_result_view_exposes_ordered_native_arrays(
    native_result_with_dynamics,
):
    view = TesseractNativeResultView.build(native_result_with_dynamics)

    assert view.joint_names == ("joint1", "joint2")
    assert view.positions == ((0.0, 1.0), (0.5, 1.5))
    assert view.velocities == ((0.0, 0.0), (0.1, 0.2))
    assert view.accelerations == ((0.0, 0.0), (0.01, 0.02))
    assert view.times == (0.0, 1.0)


def test_result_view_accepts_empty_exact_trajectory():
    view = TesseractNativeResultView.build(_result([]))

    assert view.trajectory_points == ()
    assert view.joint_names == ()
    assert view.positions == ()
    assert view.velocities is None
    assert view.accelerations is None
    assert view.times is None


def test_result_view_rejects_wrong_boundary_type():
    with pytest.raises(MalformedTesseractNativeResultError):
        TesseractNativeResultView.build(object())


def test_result_view_rejects_mixed_optional_field_presence():
    result = _result(
        [
            TrajectoryPoint(
                ["joint1"],
                np.array([0.0]),
                velocities=None,
            ),
            TrajectoryPoint(
                ["joint1"],
                np.array([1.0]),
                velocities=np.array([0.1]),
            ),
        ],
    )

    with pytest.raises(
        MalformedTesseractNativeResultError,
        match="velocities",
    ):
        TesseractNativeResultView.build(result)


@pytest.mark.parametrize(
    "points",
    [
        [
            TrajectoryPoint(["joint1"], np.array([0.0])),
            TrajectoryPoint(["other"], np.array([1.0])),
        ],
        [TrajectoryPoint(["joint1"], np.array([0.0, 1.0]))],
        [TrajectoryPoint(["joint1"], np.array([float("nan")]))],
        [TrajectoryPoint(["joint1", "joint1"], np.array([0.0, 1.0]))],
        [TrajectoryPoint([""], np.array([0.0]))],
        [TrajectoryPoint(["joint1"], np.array([0.0]), time=-1.0)],
        [
            TrajectoryPoint(["joint1"], np.array([0.0]), time=1.0),
            TrajectoryPoint(["joint1"], np.array([1.0]), time=0.5),
        ],
        [
            TrajectoryPoint(["joint1"], np.array([0.0]), time=None),
            TrajectoryPoint(["joint1"], np.array([1.0]), time=1.0),
        ],
        [
            TrajectoryPoint(
                ["joint1"],
                np.array([0.0]),
                velocities=np.array([0.0, 1.0]),
            ),
        ],
        [
            TrajectoryPoint(
                ["joint1"],
                np.array([0.0]),
                accelerations=np.array([float("inf")]),
            ),
        ],
    ],
)
def test_result_view_rejects_malformed_native_trajectory(points):
    with pytest.raises(MalformedTesseractNativeResultError):
        TesseractNativeResultView.build(_result(points))


def test_result_view_rejects_non_native_trajectory_point():
    result = _result([object()])

    with pytest.raises(MalformedTesseractNativeResultError):
        TesseractNativeResultView.build(result)


def test_result_view_raw_constructor_cannot_bypass_invariants(
    native_result_without_dynamics,
):
    view = TesseractNativeResultView.build(
        native_result_without_dynamics,
    )

    with pytest.raises(MalformedTesseractNativeResultError):
        evolve(view, message="not the native message")
