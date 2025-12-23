"""
Tesseract backend client for compas_fab.

This module provides the main client interface for using Tesseract
as a motion planning backend with compas_fab.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Optional, List, Any
from pathlib import Path

if TYPE_CHECKING:
    from compas_robots import RobotModel
    from compas_fab.robots import Robot


__all__ = ['TesseractClient']


class TesseractClient:
    """Backend client for Tesseract motion planning framework.

    This client provides access to Tesseract's motion planning capabilities
    including OMPL, TrajOpt, and Simple planners, as well as kinematics
    and collision checking.

    Uses the high-level tesseract_robotics.planning API internally.

    Parameters
    ----------
    connection_type : str, optional
        Not used (for API compatibility). Default is 'direct'.

    Attributes
    ----------
    environment : tesseract_environment.Environment
        The Tesseract environment containing the robot and scene.

    Examples
    --------
    >>> from compas_fab.backends.tesseract import TesseractClient
    >>> with TesseractClient() as client:
    ...     robot = client.load_robot(urdf_path, srdf_path)
    ...     frame = robot.forward_kinematics(configuration)
    """

    def __init__(self, connection_type: str = 'direct'):
        self._tesseract_robot = None  # High-level planning.Robot
        self._robot = None  # compas_fab Robot
        self._planner = None
        self._is_connected = False
        
    def __enter__(self):
        self.connect()
        return self
        
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()
        return False
        
    @property
    def is_connected(self) -> bool:
        """bool : True if the client is connected."""
        return self._is_connected
        
    @property
    def environment(self):
        """tesseract_environment.Environment : The Tesseract environment."""
        if self._tesseract_robot is not None:
            return self._tesseract_robot.env
        return None

    @property
    def tesseract_robot(self):
        """tesseract_robotics.planning.Robot : The high-level robot interface."""
        return self._tesseract_robot

    @property
    def planner(self) -> 'TesseractPlanner':
        """TesseractPlanner : The planner interface."""
        if self._planner is None:
            from .planner import TesseractPlanner
            self._planner = TesseractPlanner(self)
        return self._planner

    def connect(self):
        """Initialize the Tesseract client.

        This is called automatically when using the context manager.
        """
        if self._is_connected:
            return

        # Import tesseract to verify it's installed
        try:
            from tesseract_robotics.planning import Robot as TesseractRobotClass  # noqa: F401
        except ImportError as e:
            raise ImportError(
                "tesseract_robotics package not found. "
                "Install it from https://github.com/tesseract-robotics/tesseract_nanobind"
            ) from e

        self._is_connected = True

    def disconnect(self):
        """Clean up Tesseract resources.

        This is called automatically when exiting the context manager.
        Cleanup order is important to avoid segfaults with nanobind.
        """
        if not self._is_connected:
            return

        # Explicit cleanup in correct order (nanobind requirement)
        self._planner = None
        self._robot = None
        self._tesseract_robot = None
        self._is_connected = False
        
    def load_robot(
        self,
        urdf_file: str,
        srdf_file: Optional[str] = None,
        resource_locator: Optional[Any] = None,
    ) -> 'Robot':
        """Load a robot from URDF and SRDF files.

        Parameters
        ----------
        urdf_file : str
            Path to the URDF file.
        srdf_file : str, optional
            Path to the SRDF file. If not provided, kinematic groups
            must be defined manually.
        resource_locator : object, optional
            Resource locator for mesh files. If not provided, uses
            GeneralResourceLocator.

        Returns
        -------
        :class:`compas_fab.robots.Robot`
            The loaded robot with this client attached.
        """
        from compas_robots import RobotModel
        from compas_fab.robots import Robot
        from tesseract_robotics.planning import Robot as TesseractRobotClass

        if not self._is_connected:
            self.connect()

        # Resolve paths
        urdf_path = Path(urdf_file).resolve()
        srdf_path = Path(srdf_file).resolve() if srdf_file else None

        # Load using high-level Robot API (handles path-based resource resolution)
        if srdf_path:
            self._tesseract_robot = TesseractRobotClass.from_files(
                urdf_path, srdf_path, resource_locator
            )
        else:
            # For URDF-only, use lower-level API
            from tesseract_robotics.tesseract_common import (
                FilesystemPath,
                GeneralResourceLocator,
            )
            from tesseract_robotics.tesseract_environment import Environment

            locator = resource_locator or GeneralResourceLocator()
            env = Environment()
            if not env.init(FilesystemPath(str(urdf_path)), locator):
                raise RuntimeError(f"Failed to initialize environment from {urdf_path}")
            # Wrap in high-level Robot
            self._tesseract_robot = TesseractRobotClass(env, locator)

        # Create compas Robot from URDF
        robot_model = RobotModel.from_urdf_file(str(urdf_path))

        # Load semantics if SRDF was provided
        semantics = None
        if srdf_file is not None:
            from compas_fab.robots import RobotSemantics
            semantics = RobotSemantics.from_srdf_file(str(srdf_path), robot_model)

        # Create Robot with this client
        self._robot = Robot(robot_model, semantics=semantics, client=self)

        return self._robot

    def _get_default_group(self) -> str:
        """Get the first available kinematic group name."""
        groups = list(self.environment.getGroupNames())
        if not groups:
            raise ValueError("No kinematic groups defined in environment")
        return groups[0]

    def get_kinematic_group(
        self,
        group: Optional[str] = None
    ) -> 'KinematicGroup':
        """Get a kinematic group from the environment.

        Parameters
        ----------
        group : str, optional
            Name of the kinematic group. If not provided, uses
            the first available group.

        Returns
        -------
        KinematicGroup
            The kinematic group for IK/FK operations.
        """
        if group is None:
            group = self._get_default_group()

        return self.environment.getKinematicGroup(group)

    def get_joint_group(self, group: Optional[str] = None) -> 'JointGroup':
        """Get a joint group from the environment.

        Parameters
        ----------
        group : str, optional
            Name of the group. If not provided, uses the first available group.

        Returns
        -------
        JointGroup
            The joint group for FK operations.
        """
        if group is None:
            group = self._get_default_group()

        return self.environment.getJointGroup(group)

    def get_joint_names(self, group: Optional[str] = None) -> List[str]:
        """Get joint names for a kinematic group.

        Parameters
        ----------
        group : str, optional
            Name of the kinematic group.

        Returns
        -------
        list of str
            Joint names in the group.
        """
        if group is None:
            group = self._get_default_group()
        return self._tesseract_robot.get_joint_names(group)

    def get_link_names(self, group: Optional[str] = None) -> List[str]:
        """Get link names for a kinematic group.

        Parameters
        ----------
        group : str, optional
            Name of the kinematic group.

        Returns
        -------
        list of str
            Link names in the group.
        """
        if group is None:
            # Return all link names
            return self._tesseract_robot.get_link_names()
        # For specific group, use joint group
        joint_group = self.get_joint_group(group)
        return list(joint_group.getLinkNames())
        
    # =========================================================================
    # ClientInterface forwarding methods
    # =========================================================================
    
    def forward_kinematics(self, *args, **kwargs):
        """Calculate forward kinematics. See :meth:`TesseractPlanner.forward_kinematics`."""
        return self.planner.forward_kinematics(*args, **kwargs)
        
    def inverse_kinematics(self, *args, **kwargs):
        """Calculate inverse kinematics. See :meth:`TesseractPlanner.inverse_kinematics`."""
        return self.planner.inverse_kinematics(*args, **kwargs)
        
    def plan_motion(self, *args, **kwargs):
        """Plan motion. See :meth:`TesseractPlanner.plan_motion`."""
        return self.planner.plan_motion(*args, **kwargs)
        
    def plan_cartesian_motion(self, *args, **kwargs):
        """Plan cartesian motion. See :meth:`TesseractPlanner.plan_cartesian_motion`."""
        return self.planner.plan_cartesian_motion(*args, **kwargs)
        
    def add_collision_mesh(self, *args, **kwargs):
        """Add collision mesh. See :meth:`TesseractPlanner.add_collision_mesh`."""
        return self.planner.add_collision_mesh(*args, **kwargs)
        
    def remove_collision_mesh(self, *args, **kwargs):
        """Remove collision mesh. See :meth:`TesseractPlanner.remove_collision_mesh`."""
        return self.planner.remove_collision_mesh(*args, **kwargs)
        
    def add_attached_collision_mesh(self, *args, **kwargs):
        """Add attached collision mesh. See :meth:`TesseractPlanner.add_attached_collision_mesh`."""
        return self.planner.add_attached_collision_mesh(*args, **kwargs)
        
    def remove_attached_collision_mesh(self, *args, **kwargs):
        """Remove attached collision mesh. See :meth:`TesseractPlanner.remove_attached_collision_mesh`."""
        return self.planner.remove_attached_collision_mesh(*args, **kwargs)
