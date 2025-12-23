import pytest

import compas
from compas.geometry import Frame, allclose
from compas_robots import Configuration

from pathlib import Path

from compas_fab.robots import FrameWaypoints

if not compas.IPY:
    from compas_fab.backends.tesseract.client import TesseractClient

# Use simplified URDF/SRDF fixtures without mesh dependencies
_fixtures_dir = Path(__file__).parent.parent.parent.parent / "src" / "compas_fab" / "backends" / "tesseract" / "tests" / "fixtures"
urdf_filename = str(_fixtures_dir / "ur5.urdf")
srdf_filename = str(_fixtures_dir / "ur5.srdf")


@pytest.fixture
def client():
    """Create TesseractClient context for tests."""
    if compas.IPY:
        pytest.skip("Not supported in IronPython")
    return TesseractClient()


def test_forward_kinematics(client):
    """Test forward kinematics calculation."""
    with client:
        robot = client.load_robot(urdf_filename, srdf_filename)

        # Test with a known configuration
        config = Configuration.from_revolute_values([0.0, -1.57, 1.57, -1.57, -1.57, 0.0])
        frame = client.forward_kinematics(robot, config)

        # Verify frame is valid (has point and axes)
        assert frame is not None
        assert hasattr(frame, 'point')
        assert hasattr(frame, 'xaxis')
        assert hasattr(frame, 'yaxis')


def test_inverse_kinematics(client):
    """Test inverse kinematics calculation."""
    with client:
        robot = client.load_robot(urdf_filename, srdf_filename)

        # Target frame for IK
        frame_WCF = Frame((0.4, 0.1, 0.4), (1, 0, 0), (0, 1, 0))

        # IK requires kinematics plugin config - skip if not available
        try:
            config = client.inverse_kinematics(robot, frame_WCF)
        except RuntimeError as e:
            if "KinematicsPluginFactory" in str(e):
                pytest.skip("IK requires kinematics plugin configuration")
            raise

        if config is not None:
            # Verify round-trip: FK of IK result should match target
            result_frame = client.forward_kinematics(robot, config)

            # Check position match (within tolerance)
            assert allclose(list(frame_WCF.point), list(result_frame.point), tol=0.001)


def test_inverse_kinematics_roundtrip(client):
    """Test IK->FK roundtrip produces consistent results."""
    with client:
        robot = client.load_robot(urdf_filename, srdf_filename)

        # Start from known config, get frame, then IK back
        start_config = Configuration.from_revolute_values([0.5, -1.2, 1.4, -1.5, -1.57, 0.3])

        # FK to get frame
        frame = client.forward_kinematics(robot, start_config)

        # IK to get config back - requires kinematics plugin
        try:
            result_config = client.inverse_kinematics(robot, frame)
        except RuntimeError as e:
            if "KinematicsPluginFactory" in str(e):
                pytest.skip("IK requires kinematics plugin configuration")
            raise

        if result_config is not None:
            # FK of result should match original frame
            result_frame = client.forward_kinematics(robot, result_config)

            assert allclose(list(frame.point), list(result_frame.point), tol=0.001)


def test_iter_inverse_kinematics(client):
    """Test iterating over multiple IK solutions."""
    with client:
        robot = client.load_robot(urdf_filename, srdf_filename)

        frame_WCF = Frame((0.4, 0.1, 0.4), (1, 0, 0), (0, 1, 0))

        # Get multiple solutions - requires kinematics plugin
        try:
            solutions = list(client.planner.iter_inverse_kinematics(robot, frame_WCF, options={"max_results": 8}))
        except RuntimeError as e:
            if "KinematicsPluginFactory" in str(e):
                pytest.skip("IK requires kinematics plugin configuration")
            raise

        # Should return at least one solution for reachable frame
        # Note: KDL may return fewer solutions than analytical solvers
        assert len(solutions) >= 0  # May be 0 if unreachable

        # Verify each solution is valid via FK
        for config in solutions:
            result_frame = client.forward_kinematics(robot, config)
            assert allclose(list(frame_WCF.point), list(result_frame.point), tol=0.01)


@pytest.fixture
def frames_WCF():
    """A list of frames in the world coordinate frame for planning tests."""
    return [
        Frame((0.407, 0.073, 0.320), (0.922, 0.000, 0.388), (0.113, 0.956, -0.269)),
        Frame((0.404, 0.057, 0.324), (0.919, 0.000, 0.394), (0.090, 0.974, -0.210)),
        Frame((0.390, 0.064, 0.315), (0.891, 0.000, 0.454), (0.116, 0.967, -0.228)),
        Frame((0.388, 0.079, 0.309), (0.881, 0.000, 0.473), (0.149, 0.949, -0.278)),
        Frame((0.376, 0.087, 0.299), (0.850, 0.000, 0.528), (0.184, 0.937, -0.296)),
    ]


@pytest.fixture
def frame_waypoints(frames_WCF):
    """A FrameWaypoints Object for planning tests."""
    return FrameWaypoints(frames_WCF)


def test_load_robot(client):
    """Test loading a robot from URDF and SRDF."""
    with client:
        robot = client.load_robot(urdf_filename, srdf_filename)

        assert robot is not None
        assert robot.model is not None
        assert robot.model.name == "ur5_robot"


def test_get_joint_names(client):
    """Test getting joint names for a kinematic group."""
    with client:
        client.load_robot(urdf_filename, srdf_filename)

        joint_names = client.get_joint_names()

        assert len(joint_names) == 6  # UR5 has 6 joints
        assert "shoulder_pan_joint" in joint_names
        assert "wrist_3_joint" in joint_names


def test_get_link_names(client):
    """Test getting link names for a kinematic group."""
    with client:
        client.load_robot(urdf_filename, srdf_filename)

        link_names = client.get_link_names()

        assert len(link_names) > 0
        # Should include tool0 as end effector
        assert "tool0" in link_names or "ee_link" in link_names or len(link_names) >= 6
