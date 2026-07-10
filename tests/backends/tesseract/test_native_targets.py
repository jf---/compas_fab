import numpy as np
import pytest
from tesseract_robotics.planning import CartesianTarget
from tesseract_robotics.planning import JointTarget
from tesseract_robotics.planning import MoveType
from tesseract_robotics.planning import Pose
from tesseract_robotics.planning import StateTarget

from compas_fab.backends.tesseract.errors import InvalidTesseractTargetError
from compas_fab.backends.tesseract.native_targets import build_cartesian_target
from compas_fab.backends.tesseract.native_targets import build_joint_target
from compas_fab.backends.tesseract.native_targets import build_state_target
from compas_fab.backends.tesseract.native_targets import move_type_from_name


@pytest.mark.parametrize("move_type", list(MoveType))
def test_cartesian_target_retains_exact_pose_and_every_move_type(move_type):
    pose = Pose.from_xyz(0.1, 0.2, 0.3)

    target = build_cartesian_target(pose, move_type, "CARTESIAN")

    assert isinstance(target, CartesianTarget)
    assert target.pose is pose
    assert target.move_type is move_type
    assert target.profile == "CARTESIAN"


def test_joint_target_preserves_explicit_name_order():
    target = build_joint_target(
        [1.0, 2.0],
        ["joint_b", "joint_a"],
        MoveType.FREESPACE,
        "DEFAULT",
    )

    assert isinstance(target, JointTarget)
    np.testing.assert_array_equal(target.positions, [1.0, 2.0])
    assert target.names == ["joint_b", "joint_a"]


def test_joint_target_keeps_omitted_names_absent():
    target = build_joint_target(
        [1.0, 2.0],
        None,
        MoveType.FREESPACE,
        "DEFAULT",
    )

    assert target.names is None


def test_state_target_keeps_unconnected_dynamics_absent():
    target = build_state_target(
        [1.0],
        None,
        None,
        None,
        None,
        MoveType.LINEAR,
        "DEFAULT",
    )

    assert isinstance(target, StateTarget)
    assert target.names is None
    assert target.velocities is None
    assert target.accelerations is None
    assert target.time is None


def test_state_target_retains_complete_native_state():
    target = build_state_target(
        [1.0, 2.0],
        ["joint_1", "joint_2"],
        [0.1, 0.2],
        [0.01, 0.02],
        1.5,
        MoveType.CIRCULAR,
        "TIMED",
    )

    np.testing.assert_array_equal(target.positions, [1.0, 2.0])
    np.testing.assert_array_equal(target.velocities, [0.1, 0.2])
    np.testing.assert_array_equal(target.accelerations, [0.01, 0.02])
    assert target.names == ["joint_1", "joint_2"]
    assert target.time == 1.5
    assert target.move_type is MoveType.CIRCULAR
    assert target.profile == "TIMED"


@pytest.mark.parametrize("move_type", list(MoveType))
def test_move_type_name_resolves_exact_native_enum(move_type):
    assert move_type_from_name(move_type.name.lower()) is move_type


@pytest.mark.parametrize("value", [None, object(), "", "spline"])
def test_unknown_move_type_name_fails(value):
    with pytest.raises(InvalidTesseractTargetError):
        move_type_from_name(value)


@pytest.mark.parametrize(
    "pose,move_type,profile",
    [
        (object(), MoveType.FREESPACE, "DEFAULT"),
        (Pose(), object(), "DEFAULT"),
        (Pose(), MoveType.FREESPACE, ""),
        (Pose(), MoveType.FREESPACE, "   "),
    ],
)
def test_cartesian_target_rejects_invalid_native_inputs(
    pose,
    move_type,
    profile,
):
    with pytest.raises(InvalidTesseractTargetError):
        build_cartesian_target(pose, move_type, profile)


@pytest.mark.parametrize(
    "positions,names",
    [
        ([], None),
        (0.0, None),
        ([[0.0]], None),
        ([0.0, float("nan")], None),
        ([0.0, float("inf")], None),
        ([True], None),
        ([0.0], ["joint", "extra"]),
        ([0.0, 1.0], ["joint", "joint"]),
        ([0.0], [""]),
        ([0.0], "joint"),
    ],
)
def test_joint_target_rejects_invalid_positions_or_names(positions, names):
    with pytest.raises(InvalidTesseractTargetError):
        build_joint_target(
            positions,
            names,
            MoveType.FREESPACE,
            "DEFAULT",
        )


@pytest.mark.parametrize(
    "velocities,accelerations,time",
    [
        ([0.0, 1.0], None, None),
        (None, [0.0, 1.0], None),
        ([float("nan")], None, None),
        (None, [float("inf")], None),
        (None, None, -1.0),
        (None, None, float("nan")),
        (None, None, True),
    ],
)
def test_state_target_rejects_malformed_optional_dynamics(
    velocities,
    accelerations,
    time,
):
    with pytest.raises(InvalidTesseractTargetError):
        build_state_target(
            [0.0],
            ["joint"],
            velocities,
            accelerations,
            time,
            MoveType.FREESPACE,
            "DEFAULT",
        )
