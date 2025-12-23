"""
Tesseract planner implementation for compas_fab.

This module implements the PlannerInterface for Tesseract, providing
access to OMPL, TrajOpt, and Simple motion planners.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Optional, List, Dict, Any, Generator
import numpy as np

if TYPE_CHECKING:
    from compas.geometry import Frame
    from compas_fab.robots import Robot, Configuration, JointTrajectory


__all__ = ['TesseractPlanner']


class TesseractPlanner:
    """Tesseract motion planner implementing compas_fab PlannerInterface.
    
    This planner provides access to:
    - OMPL motion planners (RRTConnect, RRT*, PRM, etc.)
    - TrajOpt trajectory optimization
    - Simple interpolation planner
    - KDL-based forward/inverse kinematics
    
    Parameters
    ----------
    client : :class:`TesseractClient`
        The Tesseract client instance.
    """
    
    def __init__(self, client: 'TesseractClient'):
        self.client = client
        
    @property
    def environment(self):
        """The Tesseract environment."""
        return self.client.environment
        
    # =========================================================================
    # Forward Kinematics
    # =========================================================================
    
    def forward_kinematics(
        self,
        robot: 'Robot',
        configuration: 'Configuration',
        group: Optional[str] = None,
        options: Optional[Dict] = None
    ) -> 'Frame':
        """Calculate the robot's forward kinematics.
        
        Parameters
        ----------
        robot : :class:`compas_fab.robots.Robot`
            The robot instance.
        configuration : :class:`compas_fab.robots.Configuration`
            The joint configuration.
        group : str, optional
            The planning group. Defaults to robot's main group.
        options : dict, optional
            Additional options:
            - 'link_name': str - The link to calculate FK for.
            
        Returns
        -------
        :class:`compas.geometry.Frame`
            The frame of the end-effector or specified link.
        """
        from compas.geometry import Frame
        from .conversions import isometry_to_frame

        options = options or {}
        group = group or robot.main_group_name

        # Get joint group (works without kinematics plugin config)
        joint_group = self.environment.getJointGroup(group)
        joint_names = list(joint_group.getJointNames())

        # Build joint values array
        if configuration.joint_names:
            joint_dict = configuration.joint_dict
            values = np.array([joint_dict.get(name, 0.0) for name in joint_names])
        else:
            values = np.array(configuration.joint_values)

        # Get state solver and compute FK
        state_solver = self.environment.getStateSolver()
        scene_state = state_solver.getState(joint_names, values)

        # Get link transform
        link_name = options.get('link_name')
        if link_name is None:
            # Get tip link from robot semantics or group info
            if robot.semantics and group in robot.semantics.group_names:
                group_info = robot.semantics.get_end_effector_link_name(group)
                link_name = group_info if group_info else "tool0"
            else:
                # Default to tool0 for UR robots
                link_name = "tool0"

        transform = scene_state.link_transforms[link_name]

        return isometry_to_frame(transform)
        
    # =========================================================================
    # Inverse Kinematics
    # =========================================================================
    
    def inverse_kinematics(
        self,
        robot: 'Robot',
        frame_WCF: 'Frame',
        start_configuration: Optional['Configuration'] = None,
        group: Optional[str] = None,
        options: Optional[Dict] = None
    ) -> 'Configuration':
        """Calculate the robot's inverse kinematics.

        Parameters
        ----------
        robot : :class:`compas_fab.robots.Robot`
            The robot instance.
        frame_WCF : :class:`compas.geometry.Frame`
            The target frame in world coordinates.
        start_configuration : :class:`compas_fab.robots.Configuration`, optional
            The starting configuration for the IK solver.
        group : str, optional
            The planning group.
        options : dict, optional
            Additional options:
            - 'max_results': int - Maximum IK solutions to return.
            - 'return_all': bool - Return all solutions as generator.

        Returns
        -------
        :class:`compas_fab.robots.Configuration`
            A valid configuration, or None if no solution found.
        """
        from compas_robots import Configuration
        from tesseract_robotics.tesseract_common import Isometry3d
        from tesseract_robotics.tesseract_kinematics import KinGroupIKInput
        from .conversions import frame_to_isometry

        options = options or {}
        group = group or robot.main_group_name

        # Get joint names
        tesseract_robot = self.client.tesseract_robot
        joint_names = tesseract_robot.get_joint_names(group)

        # Convert frame to isometry
        target_matrix = frame_to_isometry(frame_WCF)
        target_iso = Isometry3d(target_matrix)

        # Get seed values
        if start_configuration is not None:
            if start_configuration.joint_names:
                joint_dict = start_configuration.joint_dict
                seed = np.array([joint_dict.get(name, 0.0) for name in joint_names], dtype=np.float64)
            else:
                seed = np.array(start_configuration.joint_values, dtype=np.float64)
        else:
            seed = np.zeros(len(joint_names), dtype=np.float64)

        # Get kinematic group info
        kin_group = self.environment.getKinematicGroup(group)
        base_link = kin_group.getBaseLinkName()
        tip_link = list(kin_group.getActiveLinkNames())[-1]

        # Create IK input and solve
        ik_input = KinGroupIKInput(target_iso, base_link, tip_link)
        solutions = kin_group.calcInvKin(ik_input, seed)

        if not solutions or len(solutions) == 0:
            return None

        # Return first solution as Configuration
        solution = solutions[0]

        return Configuration(
            joint_values=list(solution),
            joint_types=[0] * len(solution),  # Assume revolute
            joint_names=joint_names
        )
        
    def iter_inverse_kinematics(
        self,
        robot: 'Robot',
        frame_WCF: 'Frame',
        start_configuration: Optional['Configuration'] = None,
        group: Optional[str] = None,
        options: Optional[Dict] = None
    ) -> Generator['Configuration', None, None]:
        """Iterate over all IK solutions.

        Parameters
        ----------
        robot : :class:`compas_fab.robots.Robot`
            The robot instance.
        frame_WCF : :class:`compas.geometry.Frame`
            The target frame.
        start_configuration : :class:`compas_fab.robots.Configuration`, optional
            The starting configuration.
        group : str, optional
            The planning group.
        options : dict, optional
            Additional options:
            - 'max_results': int - Maximum solutions to yield.

        Yields
        ------
        :class:`compas_fab.robots.Configuration`
            Valid configurations.
        """
        from compas_robots import Configuration
        from tesseract_robotics.tesseract_common import Isometry3d
        from tesseract_robotics.tesseract_kinematics import KinGroupIKInput
        from .conversions import frame_to_isometry

        options = options or {}
        group = group or robot.main_group_name
        max_results = options.get('max_results', 8)

        # Get joint names
        tesseract_robot = self.client.tesseract_robot
        joint_names = tesseract_robot.get_joint_names(group)

        # Convert frame to isometry
        target_matrix = frame_to_isometry(frame_WCF)
        target_iso = Isometry3d(target_matrix)

        # Get seed values
        if start_configuration is not None:
            if start_configuration.joint_names:
                joint_dict = start_configuration.joint_dict
                seed = np.array([joint_dict.get(name, 0.0) for name in joint_names], dtype=np.float64)
            else:
                seed = np.array(start_configuration.joint_values, dtype=np.float64)
        else:
            seed = np.zeros(len(joint_names), dtype=np.float64)

        # Get kinematic group info
        kin_group = self.environment.getKinematicGroup(group)
        base_link = kin_group.getBaseLinkName()
        tip_link = list(kin_group.getActiveLinkNames())[-1]

        # Create IK input and solve
        ik_input = KinGroupIKInput(target_iso, base_link, tip_link)
        solutions = kin_group.calcInvKin(ik_input, seed)

        for i, solution in enumerate(solutions):
            if i >= max_results:
                break

            yield Configuration(
                joint_values=list(solution),
                joint_types=[0] * len(solution),
                joint_names=joint_names
            )
            
    # =========================================================================
    # Motion Planning
    # =========================================================================
    
    def plan_motion(
        self,
        robot: 'Robot',
        goal_constraints: List,
        start_configuration: Optional['Configuration'] = None,
        group: Optional[str] = None,
        options: Optional[Dict] = None
    ) -> 'JointTrajectory':
        """Plan a motion from start to goal.
        
        Parameters
        ----------
        robot : :class:`compas_fab.robots.Robot`
            The robot instance.
        goal_constraints : list
            Goal constraints (frame-based or joint-based).
        start_configuration : :class:`compas_fab.robots.Configuration`, optional
            Starting configuration.
        group : str, optional
            Planning group.
        options : dict, optional
            Planning options:
            - 'planner': str - 'ompl', 'trajopt', or 'simple'. Default 'ompl'.
            - 'planner_id': str - Specific planner (e.g., 'RRTConnect').
            - 'planning_time': float - Max planning time in seconds.
            - 'num_planning_attempts': int - Number of attempts.
            
        Returns
        -------
        :class:`compas_fab.robots.JointTrajectory`
            The planned trajectory.
        """
        options = options or {}
        planner_type = options.get('planner', 'ompl')
        group = group or robot.main_group_name
        
        if planner_type == 'ompl':
            return self._plan_motion_ompl(
                robot, goal_constraints, start_configuration, group, options
            )
        elif planner_type == 'trajopt':
            return self._plan_motion_trajopt(
                robot, goal_constraints, start_configuration, group, options
            )
        elif planner_type == 'simple':
            return self._plan_motion_simple(
                robot, goal_constraints, start_configuration, group, options
            )
        else:
            raise ValueError(f"Unknown planner type: {planner_type}")
            
    def _plan_motion_ompl(
        self,
        robot: 'Robot',
        goal_constraints: List,
        start_configuration: Optional['Configuration'],
        group: str,
        options: Dict
    ) -> 'JointTrajectory':
        """Plan motion using OMPL planner."""
        from tesseract_robotics.tesseract_motion_planners_ompl import (
            OMPLMotionPlanner,
            OMPLRealVectorPlanProfile,
            ProfileDictionary_addOMPLProfile,
        )
        from tesseract_robotics.tesseract_motion_planners import PlannerRequest
        from tesseract_robotics.tesseract_command_language import (
            CompositeInstruction,
            MoveInstruction,
            MoveInstructionType_FREESPACE,
            JointWaypointPoly_wrap_JointWaypoint,
            MoveInstructionPoly_wrap_MoveInstruction,
            ProfileDictionary,
        )
        from tesseract_robotics.tesseract_common import ManipulatorInfo
        from .conversions import (
            configuration_to_joint_waypoint,
            composite_instruction_to_trajectory,
        )

        OMPL_DEFAULT_NAMESPACE = "OMPLMotionPlannerTask"
        joint_names = self.client.get_joint_names(group)

        # Create profile dictionary with default OMPL profile
        profile_dict = ProfileDictionary()
        ompl_profile = OMPLRealVectorPlanProfile()
        ProfileDictionary_addOMPLProfile(profile_dict, OMPL_DEFAULT_NAMESPACE, "DEFAULT", ompl_profile)

        # Create manipulator info for the planning group
        manip_info = ManipulatorInfo()
        manip_info.manipulator = group
        manip_info.tcp_frame = "tool0"
        manip_info.working_frame = "base_link"

        # Create composite instruction (program)
        program = CompositeInstruction("DEFAULT")
        program.setManipulatorInfo(manip_info)

        # Set start state (first instruction in program)
        if start_configuration is not None:
            start_wp = configuration_to_joint_waypoint(start_configuration, joint_names)
            start_wp_poly = JointWaypointPoly_wrap_JointWaypoint(start_wp)
            start_instr = MoveInstruction(start_wp_poly, MoveInstructionType_FREESPACE, "DEFAULT")
            program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(start_instr))

        # Convert goal constraints to waypoints
        goal_wp = self._constraints_to_waypoint(goal_constraints, joint_names, robot, group)
        goal_wp_poly = JointWaypointPoly_wrap_JointWaypoint(goal_wp)
        goal_instr = MoveInstruction(goal_wp_poly, MoveInstructionType_FREESPACE, "DEFAULT")
        program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(goal_instr))

        # Create planner request
        request = PlannerRequest()
        request.instructions = program
        request.env = self.environment
        request.profiles = profile_dict

        # Create and run planner
        planner = OMPLMotionPlanner(OMPL_DEFAULT_NAMESPACE)
        response = planner.solve(request)

        if not response.successful:
            raise RuntimeError(f"OMPL planning failed: {response.message}")

        # Convert result to JointTrajectory
        return composite_instruction_to_trajectory(
            response.results,
            joint_names,
            [0] * len(joint_names)
        )
        
    def _plan_motion_trajopt(
        self,
        robot: 'Robot',
        goal_constraints: List,
        start_configuration: Optional['Configuration'],
        group: str,
        options: Dict
    ) -> 'JointTrajectory':
        """Plan motion using TrajOpt planner."""
        from tesseract_robotics.tesseract_motion_planners_trajopt import (
            TrajOptMotionPlanner,
            TrajOptDefaultPlanProfile,
            TrajOptDefaultCompositeProfile,
            ProfileDictionary_addTrajOptPlanProfile,
            ProfileDictionary_addTrajOptCompositeProfile,
        )
        from tesseract_robotics.tesseract_motion_planners import PlannerRequest
        from tesseract_robotics.tesseract_command_language import (
            CompositeInstruction,
            MoveInstruction,
            MoveInstructionType_FREESPACE,
            JointWaypointPoly_wrap_JointWaypoint,
            MoveInstructionPoly_wrap_MoveInstruction,
            ProfileDictionary,
        )
        from tesseract_robotics.tesseract_common import ManipulatorInfo
        from .conversions import (
            configuration_to_joint_waypoint,
            composite_instruction_to_trajectory,
        )

        TRAJOPT_DEFAULT_NAMESPACE = "TrajOptMotionPlannerTask"
        joint_names = self.client.get_joint_names(group)

        # Create profile dictionary
        profile_dict = ProfileDictionary()

        # Configure TrajOpt profiles
        plan_profile = TrajOptDefaultPlanProfile()
        composite_profile = TrajOptDefaultCompositeProfile()

        ProfileDictionary_addTrajOptPlanProfile(profile_dict, TRAJOPT_DEFAULT_NAMESPACE, "DEFAULT", plan_profile)
        ProfileDictionary_addTrajOptCompositeProfile(profile_dict, TRAJOPT_DEFAULT_NAMESPACE, "DEFAULT", composite_profile)

        # Create manipulator info for the planning group
        manip_info = ManipulatorInfo()
        manip_info.manipulator = group
        manip_info.tcp_frame = "tool0"
        manip_info.working_frame = "base_link"

        # Create composite instruction
        program = CompositeInstruction("DEFAULT")
        program.setManipulatorInfo(manip_info)

        # Set start state
        if start_configuration is not None:
            start_wp = configuration_to_joint_waypoint(start_configuration, joint_names)
            start_wp_poly = JointWaypointPoly_wrap_JointWaypoint(start_wp)
            start_instr = MoveInstruction(start_wp_poly, MoveInstructionType_FREESPACE, "DEFAULT")
            program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(start_instr))

        # Convert goal constraints to waypoints
        goal_wp = self._constraints_to_waypoint(goal_constraints, joint_names, robot, group)
        goal_wp_poly = JointWaypointPoly_wrap_JointWaypoint(goal_wp)
        goal_instr = MoveInstruction(goal_wp_poly, MoveInstructionType_FREESPACE, "DEFAULT")
        program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(goal_instr))

        # Create planner request
        request = PlannerRequest()
        request.instructions = program
        request.env = self.environment
        request.profiles = profile_dict

        # Create and run planner
        planner = TrajOptMotionPlanner(TRAJOPT_DEFAULT_NAMESPACE)
        response = planner.solve(request)

        if not response.successful:
            raise RuntimeError(f"TrajOpt planning failed: {response.message}")

        return composite_instruction_to_trajectory(
            response.results,
            joint_names,
            [0] * len(joint_names)
        )
        
    def _plan_motion_simple(
        self,
        robot: 'Robot',
        goal_constraints: List,
        start_configuration: Optional['Configuration'],
        group: str,
        options: Dict
    ) -> 'JointTrajectory':
        """Plan motion using simple interpolation planner."""
        from tesseract_robotics.tesseract_motion_planners_simple import SimpleMotionPlanner
        from tesseract_robotics.tesseract_motion_planners import PlannerRequest
        from tesseract_robotics.tesseract_command_language import (
            CompositeInstruction,
            MoveInstruction,
            MoveInstructionType_FREESPACE,
            JointWaypointPoly_wrap_JointWaypoint,
            MoveInstructionPoly_wrap_MoveInstruction,
            ProfileDictionary,
        )
        from tesseract_robotics.tesseract_common import ManipulatorInfo
        from .conversions import (
            configuration_to_joint_waypoint,
            composite_instruction_to_trajectory,
        )

        joint_names = self.client.get_joint_names(group)

        # Create manipulator info for the planning group
        manip_info = ManipulatorInfo()
        manip_info.manipulator = group
        manip_info.tcp_frame = "tool0"
        manip_info.working_frame = "base_link"

        # Create composite instruction
        program = CompositeInstruction("DEFAULT")
        program.setManipulatorInfo(manip_info)

        # Set start state
        if start_configuration is not None:
            start_wp = configuration_to_joint_waypoint(start_configuration, joint_names)
            start_wp_poly = JointWaypointPoly_wrap_JointWaypoint(start_wp)
            start_instr = MoveInstruction(start_wp_poly, MoveInstructionType_FREESPACE, "DEFAULT")
            program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(start_instr))

        # Convert goal constraints to waypoints
        goal_wp = self._constraints_to_waypoint(goal_constraints, joint_names, robot, group)
        goal_wp_poly = JointWaypointPoly_wrap_JointWaypoint(goal_wp)
        goal_instr = MoveInstruction(goal_wp_poly, MoveInstructionType_FREESPACE, "DEFAULT")
        program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(goal_instr))

        # Create planner request
        profile_dict = ProfileDictionary()
        request = PlannerRequest()
        request.instructions = program
        request.env = self.environment
        request.profiles = profile_dict

        planner = SimpleMotionPlanner()
        response = planner.solve(request)

        if not response.successful:
            raise RuntimeError(f"Simple planning failed: {response.message}")

        return composite_instruction_to_trajectory(
            response.results,
            joint_names,
            [0] * len(joint_names)
        )
        
    def _constraints_to_waypoint(
        self,
        constraints: List,
        joint_names: List[str],
        robot: 'Robot',
        group: str
    ):
        """Convert compas_fab constraints to tesseract waypoint.
        
        This handles both frame-based and joint-based constraints.
        """
        from tesseract_robotics.tesseract_command_language import JointWaypoint
        from compas.geometry import Frame
        
        # Check constraint types
        if not constraints:
            raise ValueError("No constraints provided")
            
        # Simple case: if constraints contain a Frame directly
        if isinstance(constraints, Frame):
            # Use IK to get joint configuration
            config = self.inverse_kinematics(robot, constraints, group=group)
            if config is None:
                raise RuntimeError("Could not find IK solution for goal frame")
            return JointWaypoint(joint_names, np.array(config.joint_values))
            
        # If it's a list of constraints from compas_fab
        # Extract the goal frame or joint values
        for constraint in constraints:
            if hasattr(constraint, 'frame'):
                config = self.inverse_kinematics(robot, constraint.frame, group=group)
                if config is None:
                    raise RuntimeError("Could not find IK solution for goal frame")
                return JointWaypoint(joint_names, np.array(config.joint_values))
            elif hasattr(constraint, 'joint_values'):
                return JointWaypoint(joint_names, np.array(constraint.joint_values))
                
        raise ValueError("Could not extract goal from constraints")
        
    # =========================================================================
    # Cartesian Motion Planning
    # =========================================================================
    
    def plan_cartesian_motion(
        self,
        robot: 'Robot',
        frames_WCF: List['Frame'],
        start_configuration: Optional['Configuration'] = None,
        group: Optional[str] = None,
        options: Optional[Dict] = None
    ) -> 'JointTrajectory':
        """Plan a cartesian path through a sequence of frames.

        Parameters
        ----------
        robot : :class:`compas_fab.robots.Robot`
            The robot instance.
        frames_WCF : list of :class:`compas.geometry.Frame`
            Target frames in world coordinates.
        start_configuration : :class:`compas_fab.robots.Configuration`, optional
            Starting configuration.
        group : str, optional
            Planning group.
        options : dict, optional
            Planning options:
            - 'max_step': float - Maximum step size between waypoints.
            - 'avoid_collisions': bool - Check for collisions.

        Returns
        -------
        :class:`compas_fab.robots.JointTrajectory`
            The planned cartesian trajectory.
        """
        from tesseract_robotics.tesseract_motion_planners_simple import SimpleMotionPlanner
        from tesseract_robotics.tesseract_motion_planners import PlannerRequest
        from tesseract_robotics.tesseract_command_language import (
            CompositeInstruction,
            MoveInstruction,
            MoveInstructionType_FREESPACE,
            MoveInstructionType_LINEAR,
            CartesianWaypoint,
            CartesianWaypointPoly_wrap_CartesianWaypoint,
            JointWaypointPoly_wrap_JointWaypoint,
            MoveInstructionPoly_wrap_MoveInstruction,
            ProfileDictionary,
        )
        from tesseract_robotics.tesseract_common import ManipulatorInfo
        from .conversions import (
            frame_to_isometry,
            configuration_to_joint_waypoint,
            composite_instruction_to_trajectory,
        )

        options = options or {}
        group = group or robot.main_group_name
        joint_names = self.client.get_joint_names(group)

        # Create manipulator info for the planning group
        manip_info = ManipulatorInfo()
        manip_info.manipulator = group
        manip_info.tcp_frame = "tool0"
        manip_info.working_frame = "base_link"

        # Create composite instruction
        program = CompositeInstruction("DEFAULT")
        program.setManipulatorInfo(manip_info)

        # Set start state
        if start_configuration is not None:
            start_wp = configuration_to_joint_waypoint(start_configuration, joint_names)
            start_wp_poly = JointWaypointPoly_wrap_JointWaypoint(start_wp)
            start_instr = MoveInstruction(start_wp_poly, MoveInstructionType_FREESPACE, "DEFAULT")
            program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(start_instr))

        # Add cartesian waypoints
        for frame in frames_WCF:
            cart_wp = CartesianWaypoint(frame_to_isometry(frame))
            cart_wp_poly = CartesianWaypointPoly_wrap_CartesianWaypoint(cart_wp)
            cart_instr = MoveInstruction(cart_wp_poly, MoveInstructionType_LINEAR, "DEFAULT")
            program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(cart_instr))

        # Create request
        profile_dict = ProfileDictionary()
        request = PlannerRequest()
        request.instructions = program
        request.env = self.environment
        request.profiles = profile_dict

        # Solve
        planner = SimpleMotionPlanner()
        response = planner.solve(request)

        if not response.successful:
            raise RuntimeError(f"Cartesian planning failed: {response.message}")

        return composite_instruction_to_trajectory(
            response.results,
            joint_names,
            [0] * len(joint_names)
        )
        
    # =========================================================================
    # Collision Mesh Management
    # =========================================================================
    
    def add_collision_mesh(
        self,
        collision_mesh: 'CollisionMesh',
        options: Optional[Dict] = None
    ):
        """Add a collision mesh to the environment.
        
        Parameters
        ----------
        collision_mesh : :class:`compas_fab.robots.CollisionMesh`
            The collision mesh to add.
        options : dict, optional
            Additional options.
        """
        from tesseract_robotics.tesseract_environment import AddLinkCommand
        from tesseract_robotics.tesseract_scene_graph import Link, Visual, Collision
        from .conversions import mesh_to_tesseract_mesh, frame_to_isometry
        
        # Convert mesh
        tess_mesh = mesh_to_tesseract_mesh(collision_mesh.mesh)
        
        # Create link
        link = Link(collision_mesh.id)
        
        # Add collision geometry
        collision_geom = Collision()
        collision_geom.geometry = tess_mesh
        if hasattr(collision_mesh, 'frame') and collision_mesh.frame:
            collision_geom.origin = frame_to_isometry(collision_mesh.frame)
        link.collision.append(collision_geom)
        
        # Add visual geometry
        visual_geom = Visual()
        visual_geom.geometry = tess_mesh
        if hasattr(collision_mesh, 'frame') and collision_mesh.frame:
            visual_geom.origin = frame_to_isometry(collision_mesh.frame)
        link.visual.append(visual_geom)
        
        # Add to environment
        cmd = AddLinkCommand(link)
        self.environment.applyCommand(cmd)
        
    def remove_collision_mesh(
        self,
        id: str,
        options: Optional[Dict] = None
    ):
        """Remove a collision mesh from the environment.
        
        Parameters
        ----------
        id : str
            The identifier of the collision mesh.
        options : dict, optional
            Additional options.
        """
        from tesseract_robotics.tesseract_environment import RemoveLinkCommand
        
        cmd = RemoveLinkCommand(id)
        self.environment.applyCommand(cmd)
        
    def add_attached_collision_mesh(
        self,
        attached_collision_mesh: 'AttachedCollisionMesh',
        options: Optional[Dict] = None
    ):
        """Attach a collision mesh to a robot link.
        
        Parameters
        ----------
        attached_collision_mesh : :class:`compas_fab.robots.AttachedCollisionMesh`
            The attached collision mesh.
        options : dict, optional
            Additional options.
        """
        # First add the mesh as a link
        self.add_collision_mesh(attached_collision_mesh.collision_mesh, options)
        
        # Then attach it to the robot link
        from tesseract_robotics.tesseract_environment import AddAllowedCollisionCommand
        from tesseract_robotics.tesseract_scene_graph import AllowedCollisionMatrix
        
        # Allow collision between the attached mesh and the attachment link
        # This prevents self-collision detection for attached objects
        acm = self.environment.getAllowedCollisionMatrix()
        acm.addAllowedCollision(
            attached_collision_mesh.collision_mesh.id,
            attached_collision_mesh.link_name,
            "Attached object"
        )
        
    def remove_attached_collision_mesh(
        self,
        id: str,
        options: Optional[Dict] = None
    ):
        """Remove an attached collision mesh.
        
        Parameters
        ----------
        id : str
            The identifier of the attached collision mesh.
        options : dict, optional
            Additional options.
        """
        self.remove_collision_mesh(id, options)
