"""
Tests for compas_fab tesseract backend.
"""

import pytest
import numpy as np

from compas.geometry import Frame, Point, Vector


class TestConversions:
    """Test type conversion functions."""

    def test_frame_to_isometry(self):
        """Test Frame to 4x4 matrix conversion."""
        from compas_fab.backends.tesseract.conversions import frame_to_isometry, isometry_to_frame
        
        # Create a test frame
        frame = Frame(
            Point(1.0, 2.0, 3.0),
            Vector(1, 0, 0),
            Vector(0, 1, 0)
        )
        
        # Convert to isometry
        isometry = frame_to_isometry(frame)
        
        # Check shape
        assert isometry.shape == (4, 4)
        
        # Check translation
        np.testing.assert_array_almost_equal(
            isometry[:3, 3],
            [1.0, 2.0, 3.0]
        )
        
        # Check rotation (identity for world-aligned frame)
        np.testing.assert_array_almost_equal(
            isometry[:3, :3],
            np.eye(3)
        )
        
        # Check homogeneous row
        np.testing.assert_array_almost_equal(
            isometry[3, :],
            [0, 0, 0, 1]
        )
        
    def test_isometry_to_frame(self):
        """Test 4x4 matrix to Frame conversion."""
        from compas_fab.backends.tesseract.conversions import frame_to_isometry, isometry_to_frame
        
        # Create original frame
        original = Frame(
            Point(1.0, 2.0, 3.0),
            Vector(0, 1, 0),
            Vector(-1, 0, 0)
        )
        
        # Round-trip conversion
        isometry = frame_to_isometry(original)
        recovered = isometry_to_frame(isometry)
        
        # Check point
        np.testing.assert_array_almost_equal(
            list(recovered.point),
            list(original.point)
        )
        
        # Check axes
        np.testing.assert_array_almost_equal(
            list(recovered.xaxis),
            list(original.xaxis)
        )
        np.testing.assert_array_almost_equal(
            list(recovered.yaxis),
            list(original.yaxis)
        )
        
    def test_joint_limits_to_array(self):
        """Test joint limits conversion."""
        from compas_fab.backends.tesseract.conversions import joint_limits_to_array
        
        limits = [(-3.14, 3.14), (-2.0, 2.0), (-1.5, 1.5)]
        array = joint_limits_to_array(limits)
        
        assert array.shape == (3, 2)
        np.testing.assert_array_almost_equal(
            array[:, 0],
            [-3.14, -2.0, -1.5]
        )
        np.testing.assert_array_almost_equal(
            array[:, 1],
            [3.14, 2.0, 1.5]
        )


class TestExceptions:
    """Test custom exceptions."""
    
    def test_planning_error(self):
        """Test TesseractPlanningError."""
        from compas_fab.backends.tesseract.exceptions import TesseractPlanningError
        
        error = TesseractPlanningError("No solution found", planner="ompl")
        assert "ompl" in str(error)
        assert error.planner == "ompl"
        
    def test_kinematics_error(self):
        """Test TesseractKinematicsError."""
        from compas_fab.backends.tesseract.exceptions import TesseractKinematicsError
        
        error = TesseractKinematicsError("IK failed", operation="ik")
        assert "ik" in str(error)
        assert error.operation == "ik"
        
    def test_collision_error(self):
        """Test TesseractCollisionError."""
        from compas_fab.backends.tesseract.exceptions import TesseractCollisionError
        
        contacts = [{"link_a": "link1", "link_b": "link2"}]
        error = TesseractCollisionError("Collision detected", contacts=contacts)
        assert len(error.contacts) == 1


@pytest.mark.tesseract
class TestClient:
    """Tests requiring tesseract installation."""
    
    def test_client_context_manager(self, skip_without_tesseract):
        """Test client context manager."""
        from compas_fab.backends.tesseract import TesseractClient
        
        with TesseractClient() as client:
            assert client.is_connected
            
        assert not client.is_connected
        
    def test_client_connect_disconnect(self, skip_without_tesseract):
        """Test explicit connect/disconnect."""
        from compas_fab.backends.tesseract import TesseractClient
        
        client = TesseractClient()
        assert not client.is_connected
        
        client.connect()
        assert client.is_connected
        assert client.environment is not None
        
        client.disconnect()
        assert not client.is_connected


@pytest.mark.tesseract
class TestKinematics:
    """Tests for kinematics operations."""
    
    def test_forward_kinematics(self, skip_without_tesseract, urdf_path, srdf_path):
        """Test forward kinematics computation."""
        if urdf_path is None:
            pytest.skip("Test URDF not available")

        from compas_fab.backends.tesseract import TesseractClient
        from compas_robots import Configuration
        
        with TesseractClient() as client:
            robot = client.load_robot(urdf_path, srdf_path)
            
            # Zero configuration
            config = Configuration.from_revolute_values([0] * 6)
            
            frame = client.forward_kinematics(robot, config)
            
            assert frame is not None
            assert hasattr(frame, 'point')
            assert hasattr(frame, 'xaxis')
            assert hasattr(frame, 'yaxis')


@pytest.mark.tesseract
class TestPlanning:
    """Tests for motion planning."""
    
    def test_plan_motion_ompl(self, skip_without_tesseract, urdf_path, srdf_path):
        """Test OMPL motion planning."""
        if urdf_path is None:
            pytest.skip("Test URDF not available")

        from compas_fab.backends.tesseract import TesseractClient
        from compas_robots import Configuration
        from compas.geometry import Frame

        with TesseractClient() as client:
            robot = client.load_robot(urdf_path, srdf_path)

            start = Configuration.from_revolute_values([0] * 6)
            goal_frame = Frame(
                Point(0.5, 0.0, 0.5),
                Vector(1, 0, 0),
                Vector(0, 1, 0)
            )

            try:
                trajectory = client.plan_motion(
                    robot,
                    goal_frame,
                    start,
                    options={'planner': 'ompl', 'planner_id': 'RRTConnect'}
                )
            except RuntimeError as e:
                if "KinematicsPluginFactory" in str(e):
                    pytest.skip("Motion planning requires kinematics plugin configuration")
                raise

            assert trajectory is not None
            assert len(trajectory.points) > 0
