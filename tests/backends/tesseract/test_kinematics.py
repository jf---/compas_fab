import pytest
import numpy as np

import compas
from compas.geometry import Frame, Point, Vector, allclose
from compas_robots import Configuration

from pathlib import Path

if not compas.IPY:
    from compas_fab.backends.tesseract.client import TesseractClient
    from compas_fab.backends.tesseract.conversions import (
        frame_to_isometry,
        isometry_to_frame,
        configuration_to_joint_waypoint,
    )


def test_frame_to_isometry_conversion():
    """Test frame to isometry matrix conversion."""
    if compas.IPY:
        pytest.skip("Not supported in IronPython")

    frame = Frame(Point(1, 2, 3), Vector(1, 0, 0), Vector(0, 1, 0))
    matrix = frame_to_isometry(frame)

    # Check matrix shape
    assert matrix.shape == (4, 4)

    # Check translation
    assert allclose(matrix[:3, 3], [1, 2, 3])

    # Check rotation (identity for default axes)
    assert allclose(matrix[:3, 0], [1, 0, 0])  # xaxis
    assert allclose(matrix[:3, 1], [0, 1, 0])  # yaxis
    assert allclose(matrix[:3, 2], [0, 0, 1])  # zaxis

    # Check homogeneous row
    assert allclose(matrix[3, :], [0, 0, 0, 1])


def test_isometry_to_frame_conversion():
    """Test isometry matrix to frame conversion."""
    if compas.IPY:
        pytest.skip("Not supported in IronPython")

    # Create identity transform with translation
    matrix = np.eye(4)
    matrix[:3, 3] = [5, 6, 7]

    frame = isometry_to_frame(matrix)

    assert allclose(list(frame.point), [5, 6, 7])
    assert allclose(list(frame.xaxis), [1, 0, 0])
    assert allclose(list(frame.yaxis), [0, 1, 0])


def test_frame_roundtrip_conversion():
    """Test frame -> isometry -> frame roundtrip."""
    if compas.IPY:
        pytest.skip("Not supported in IronPython")

    original = Frame(
        Point(0.5, -0.3, 0.8),
        Vector(0.707, 0.707, 0),
        Vector(-0.707, 0.707, 0)
    )

    matrix = frame_to_isometry(original)
    result = isometry_to_frame(matrix)

    assert allclose(list(original.point), list(result.point), tol=1e-6)
    assert allclose(list(original.xaxis), list(result.xaxis), tol=1e-6)
    assert allclose(list(original.yaxis), list(result.yaxis), tol=1e-6)


def test_configuration_to_joint_waypoint():
    """Test configuration to joint waypoint conversion."""
    if compas.IPY:
        pytest.skip("Not supported in IronPython")

    try:
        from tesseract_robotics.tesseract_command_language import JointWaypoint
    except ImportError:
        pytest.skip("tesseract_robotics not installed")

    joint_names = ["j1", "j2", "j3", "j4", "j5", "j6"]
    config = Configuration(
        joint_values=[0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
        joint_types=[0] * 6,
        joint_names=joint_names
    )

    waypoint = configuration_to_joint_waypoint(config, joint_names)

    positions = list(waypoint.getPosition())
    assert allclose(positions, [0.1, 0.2, 0.3, 0.4, 0.5, 0.6])


def test_configuration_reordering():
    """Test that configuration values are reordered correctly."""
    if compas.IPY:
        pytest.skip("Not supported in IronPython")

    try:
        from tesseract_robotics.tesseract_command_language import JointWaypoint
    except ImportError:
        pytest.skip("tesseract_robotics not installed")

    # Config with different joint order
    config = Configuration(
        joint_values=[0.6, 0.5, 0.4, 0.3, 0.2, 0.1],
        joint_types=[0] * 6,
        joint_names=["j6", "j5", "j4", "j3", "j2", "j1"]
    )

    # Expected order
    expected_names = ["j1", "j2", "j3", "j4", "j5", "j6"]

    waypoint = configuration_to_joint_waypoint(config, expected_names)

    positions = list(waypoint.getPosition())
    # Values should be reordered to match expected_names
    assert allclose(positions, [0.1, 0.2, 0.3, 0.4, 0.5, 0.6])


@pytest.fixture
def client():
    """Create TesseractClient context for tests."""
    if compas.IPY:
        pytest.skip("Not supported in IronPython")
    return TesseractClient()


# Use simplified URDF/SRDF fixtures without mesh dependencies
_fixtures_dir = Path(__file__).parent.parent.parent.parent / "src" / "compas_fab" / "backends" / "tesseract" / "tests" / "fixtures"
urdf_filename = str(_fixtures_dir / "ur5.urdf")
srdf_filename = str(_fixtures_dir / "ur5.srdf")


def test_fk_ik_consistency(client):
    """Test FK and IK produce consistent results."""
    with client:
        robot = client.load_robot(urdf_filename, srdf_filename)

        # Various test configurations
        test_configs = [
            [0.0, -1.57, 1.57, -1.57, -1.57, 0.0],
            [0.5, -1.0, 1.0, -1.0, -1.57, 0.5],
            [-0.5, -1.2, 1.4, -0.8, -1.57, -0.5],
        ]

        for joint_values in test_configs:
            config = Configuration.from_revolute_values(joint_values)

            # FK
            frame = client.forward_kinematics(robot, config)

            # IK - requires kinematics plugin
            try:
                result_config = client.inverse_kinematics(robot, frame)
            except RuntimeError as e:
                if "KinematicsPluginFactory" in str(e):
                    pytest.skip("IK requires kinematics plugin configuration")
                raise

            if result_config is not None:
                # FK of IK result
                result_frame = client.forward_kinematics(robot, result_config)

                # Positions should match
                assert allclose(list(frame.point), list(result_frame.point), tol=0.01)


def test_fk_deterministic(client):
    """Test FK produces same result for same input."""
    with client:
        robot = client.load_robot(urdf_filename, srdf_filename)

        config = Configuration.from_revolute_values([0.1, -1.2, 1.3, -1.4, -1.5, 0.6])

        frame1 = client.forward_kinematics(robot, config)
        frame2 = client.forward_kinematics(robot, config)

        assert allclose(list(frame1.point), list(frame2.point), tol=1e-10)
        assert allclose(list(frame1.xaxis), list(frame2.xaxis), tol=1e-10)
        assert allclose(list(frame1.yaxis), list(frame2.yaxis), tol=1e-10)
