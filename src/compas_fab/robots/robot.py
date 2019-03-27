from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import logging

from compas.datastructures import Mesh
from compas.datastructures import mesh_transformed
from compas.geometry import Frame
from compas.geometry import Scale
from compas.geometry import Transformation
from compas.geometry import Sphere
from compas.robots import Joint
from compas.robots import RobotModel

from compas_fab.artists import BaseRobotArtist

from compas_fab.robots.configuration import Configuration
from compas_fab.robots.ros_fileserver_loader import RosFileServerLoader
from compas_fab.robots.semantics import RobotSemantics
from compas_fab.robots.constraints import JointConstraint
from compas_fab.robots.constraints import OrientationConstraint
from compas_fab.robots.constraints import PositionConstraint

LOGGER = logging.getLogger('compas_fab.robots.robot')

__all__ = [
    'Robot',
]


class Robot(object):
    """Represents a **robot** instance.

    This class binds together several building blocks, such as the robot's 
    descriptive model, its semantic information and an instance of a backend 
    client into a cohesive programmable interface. This representation builds
    upon the model described in the class :class:`compas.robots.RobotModel` of
    the **COMPAS** framework.

    Attributes
    ----------
    model : :class:`compas.robots.RobotModel`
        The robot model, usually created from an URDF structure.
    artist : :class:`compas_fab.artists.BaseRobotArtist`, optional
        Instance of the artist used to visualize the robot. Defaults to `None`.
    semantics : :class:`RobotSemantics`, optional
        The semantic model of the robot. Defaults to `None`.
    client : optional
        The backend client to use for communication, e.g. :class:`RosClient`
    """

    def __init__(self, model, artist=None, semantics=None, client=None):
        self.model = model
        self.artist = artist
        self.semantics = semantics
        self.client = client  # setter and getter ?
        self.scale(1.)

    @classmethod
    def basic(cls, name, joints=[], links=[], materials=[], **kwargs):
        """Convenience method to create the most basic instance of a robot, based only on a name.

        Parameters
        ----------
        name : str
            Name of the robot

        Returns
        -------
        :class:`Robot`
            Newly created instance of a robot.
        """
        model = RobotModel(name, joints=joints, links=links, materials=materials, **kwargs)
        return cls(model, None)

    @property
    def name(self):
        """Name of the robot, as defined by its model

        Returns
        -------
        str
            Name of the robot.
        """
        return self.model.name

    @property
    def group_names(self):
        self.ensure_semantics()
        return self.semantics.group_names

    @property
    def main_group_name(self):
        self.ensure_semantics()
        return self.semantics.main_group_name

    def get_end_effector_link_name(self, group=None):
        """Returns the name of the end effector link.

        Parameters
        ----------
        group : str
            The name of the group. Defaults to `None`.

        Returns
        -------
        str
        """
        if not self.semantics:
            return self.model.get_end_effector_link_name()
        else:
            return self.semantics.get_end_effector_link_name(group)

    def get_end_effector_link(self, group=None):
        """Returns the end effector link.

        Parameters
        ----------
        group : str
            The name of the group. Defaults to `None`.

        Returns
        -------
        :class: `compas.robots.Link`
        """
        name = self.get_end_effector_link_name(group)
        return self.model.get_link_by_name(name)

    def get_end_effector_frame(self, group=None):
        """Returns the end effector's frame.

        Parameters
        ----------
        group : str
            The name of the group. Defaults to `None`.
        
        Returns
        -------
        :class: `compas.geometry.Frame`
        """
        link = self.get_end_effector_link(group)
        return link.parent_joint.origin.copy()

    def get_base_link_name(self, group=None):
        """Returns the name of the base link.

        Parameters
        ----------
        group : str
            The name of the group. Defaults to `None`.
        
        Returns
        -------
        str
        """
        if not self.semantics:
            return self.model.get_base_link_name()
        else:
            return self.semantics.get_base_link_name(group)

    def get_base_link(self, group=None):
        """Returns the base link.

        Parameters
        ----------
        group : str
            The name of the group. Defaults to `None`.
        
        Returns
        -------
        :class: `compas.robots.Link`
        """
        name = self.get_base_link_name(group)
        return self.model.get_link_by_name(name)

    def get_base_frame(self, group=None):
        """Returns the frame of the base link, which is the robot's origin frame.

        Parameters
        ----------
        group : str
            The name of the group. Defaults to `None`.
        
        Returns
        -------
        :class: `compas.geometry.Frame`
        """
        # TODO: check this
        link = self.get_base_link(group)
        if link.parent_joint:
            base_frame = link.parent_joint.init_origin.copy()
        else:
            base_frame = Frame.worldXY()
        if not self.artist:
            base_frame.point *= self._scale_factor
        return base_frame

    def get_configurable_joints(self, group=None):
        """Returns the configurable joints.

        Parameters
        ----------
        group : str
            The name of the group. Defaults to `None`.
        
        Returns
        -------
        list of :class: `compas.robots.Joint`

        Note
        ----
        If semantics is set and no group is passed, it returns all configurable
        joints of all groups.
        """
        if self.semantics:
            if group:
                return self.semantics.get_configurable_joints(group)
            else:
                joints = []
                for group in self.group_names:
                    joints_in_group = self.semantics.get_configurable_joints(group)
                    for joint in joints_in_group:
                        if joint not in joints: # Check to not add double joints
                            joints.append(joint)
                return joints
        else:
            return self.model.get_configurable_joints()

    def get_configurable_joint_names(self, group=None):
        """Returns the configurable joint names.

        Parameters
        ----------
        group : str
            The name of the group. Defaults to `None`.
        
        Returns
        -------
        list of str

        Note
        ----
        If semantics is set and no group is passed, it returns all configurable
        joints of all groups.
        """
        configurable_joints = self.get_configurable_joints(group)
        return [j.name for j in configurable_joints]

    def get_configurable_joint_types(self, group=None):
        """Returns the configurable joint types.

        Parameters
        ----------
        group : str
            The name of the group. Defaults to `None`.
        
        Returns
        -------
        list of int

        Note
        ----
        If semantics is set and no group is passed, it returns all configurable
        joint types of all groups.
        """
        configurable_joints = self.get_configurable_joints(group)
        return [j.type for j in configurable_joints]

    # ==========================================================================
    # configurations
    # ==========================================================================

    def init_configuration(self, group=None):
        """Returns the init joint configuration.
        """
        types = [joint.type for joint in self.get_configurable_joints(group)]
        positions = [0.] * len(types)
        return Configuration(positions, types)

    def get_configuration(self, group=None):
        """Returns the current configuration.
        """
        positions = []
        types = []

        for joint in self.get_configurable_joints(group):
            positions.append(joint.position)
            types.append(joint.type)

        return Configuration(positions, types)

    def get_group_configuration(self, group, full_configuration):
        """Returns the group's configuration.

        Parameters
        ----------
        group : str
            The name of the group.
        full_configuration : :class:`compas_fab.robots.Configuration`
            The configuration for all configurable joints of the robot.
        
        Returns
        -------
        :class:`compas_fab.robots.Configuration`
        """
        values = []
        types = []
        group_joint_names = self.get_configurable_joint_names(group)
        for i, name in enumerate(self.get_configurable_joint_names()):
            if name in group_joint_names:
                types.append(full_configuration.types[i])
                values.append(full_configuration.values[i])
        return Configuration(values, types)

    def joint_positions_to_configuration(self, joint_positions, group=None):
        """Returns the robot's configuration from the passed joint_positions.
        """
        types = self.get_configurable_joint_types(group)
        if len(types) != len(joint_positions) and group != None:
            types = self.get_configurable_joint_types()
            full_configuration = Configuration(joint_positions, types)
            return self.get_group_configuration(group, full_configuration)
        else:
            return Configuration(joint_positions, types)

    def merge_group_with_full_configuration(self, group_configuration, full_configuration, group):
        """Returns the robot's full configuration by merging with the group's configuration.
        """
        all_joint_names = self.get_configurable_joint_names()
        if len(all_joint_names) != len(full_configuration.values):
            raise ValueError("Please pass a full configuration with %d values" % len(all_joint_names))
        group_joint_names = self.get_configurable_joint_names(group)
        configuration = full_configuration.copy()
        for i, name in enumerate(all_joint_names):
            if name in group_joint_names:
                gi = group_joint_names.index(name)
                configuration.values[i] = group_configuration.values[gi]
        return configuration

    def get_position_by_joint_name(self, configuration, joint_name, group=None):
        """Returns the value of the joint_name in the passed configuration.
        """
        names = self.get_configurable_joint_names(group)
        if len(names) != len(configuration.values):
            raise ValueError("Please pass a configuration with %d values or specify group" % len(names))
        return configuration.values[names.index(joint_name)]

    def get_links_with_geometry(self, group=None):
        """Returns the links with either visual or collision geometry.
        """
        # TODO: needed?
        if not group:
            group = self.main_group_name
        base_link_name = self.get_base_link_name(group)
        ee_link_name = self.get_end_effector_link_name(group)
        links_with_geometry = []
        for link in self.model.iter_link_chain(base_link_name, ee_link_name):
            if len(link.collision) or len(link.visual):
                links_with_geometry.append(link)
        return links_with_geometry
    
    def _scale_joint_values(self, values, scale_factor, group=None):
        """Scales the scaleable joint values with scale_factor.
        """
        joints = self.get_configurable_joints(group)
        if len(joints) != len(values):
            raise ValueError("Expected %d values for group %s, but received only %d." % (len(joints), group, len(values)))

        values_scaled = []
        for v, j in zip(values, joints):
            if j.is_scalable():
                v *= scale_factor
            values_scaled.append(v)
        return values_scaled
    
    def _get_scaled_joint_positions_from_start_configuration(self, start_configuration=None):
        """Checks the start configuration and returns joint_positions.
        """
        joint_names = self.get_configurable_joint_names() # full configuration
        joint_positions = [0] * len(joint_names)
        if start_configuration:
            if len(joint_names) != len(start_configuration.values):
                raise ValueError("Please pass a configuration with %d values" % len(joint_names))
            joint_positions = start_configuration.values
        # scale the prismatic joints
        joint_positions = self._scale_joint_values(joint_positions, 1./self.scale_factor)
        return joint_positions
    
    # ==========================================================================
    # transformations, coordinate frames
    # ==========================================================================

    def transformation_RCF_WCF(self, group=None):
        """Returns the transformation from the world coordinate system (WCF) to the robot's coordinate system (RCF).
        
        Parameters
        ----------
        group : str
            The name of the planning group. Defaults to `None`.
        
        Returns
        -------
        :class:`compas.geometry.Transformation`

        """
        base_frame = self.get_base_frame(group)
        return Transformation.from_frame_to_frame(Frame.worldXY(), base_frame)

    def transformation_WCF_RCF(self, group=None):
        """Returns the transformation from the robot's coordinate system (RCF) to the world coordinate system (WCF).

        Parameters
        ----------
        group : str
            The name of the planning group. Defaults to `None`.
        
        Returns
        -------
        :class:`compas.geometry.Transformation`

        """
        base_frame = self.get_base_frame(group)
        return Transformation.from_frame_to_frame(base_frame, Frame.worldXY())

    def set_RCF(self, robot_coordinate_frame, group=None):
        """Moves the origin frame of the robot to the robot_coordinate_frame.
        """
        # TODO: must be applied to the model, so that base_frame is RCF
        raise NotImplementedError

    def get_RCF(self, group=None):
        """Returns the origin frame of the robot.
        """
        return self.get_base_frame(group)

    def represent_frame_in_RCF(self, frame_WCF, group=None):
        """Returns the representation of a frame in the world coordinate frame (WCF) in the robot's coordinate frame (RCF).
        """
        frame_RCF = frame_WCF.transformed(self.transformation_WCF_RCF(group))
        return frame_RCF

    def represent_frame_in_transformed_RCF(self, frame_WCF, configuration=None, group=None):
        """Returns the frame's representation the current robot's coordinate frame (RCF).

        Parameters
        ----------
        frame_WCF : :class:`compas.geometry.Frame`
            A frame in the world coordinate frame.
        configuration : :class:`compas_fab.robots.Configuration`
            The (full) configuration from which the group's base frame is 
            calculated. Defaults to the init configuration (all zeros).
        group : str, optional 
            The planning group. Defaults to the robot's main planning group.

        Examples
        --------
        >>> from compas.geometry import Frame
        >>> from compas_fab.backends import RosClient
        >>> from compas_fab.backends import RosError
        >>> from compas_fab.robots.ur5 import Robot
        >>> client = RosClient()
        >>> client.run()
        >>> robot = Robot(client)
        >>> frame_WCF = Frame([-0.363, 0.003, -0.147], [0.388, -0.351, -0.852], [0.276, 0.926, -0.256])
        >>> frame_RCF = robot.represent_frame_in_transformed_RCF(frame_WCF)
        >>> client.close()
        >>> client.terminate()
        """

        # TODO: eventually change to not necessarily pass full config, since it
        # won't check for collisions, or?
        # TODO: have forward kin no service

        self.ensure_client()
        if not group:
            group = self.main_group_name # ensure semantics
        
        if not configuration:
            configuration = self.init_configuration()

        base_link = self.get_base_link_name(group)

        joint_names = self.get_configurable_joint_names()
        joint_positions = self._get_scaled_joint_positions_from_start_configuration(configuration)

        response = self.client.forward_kinematics(joint_positions, base_link, group, joint_names, base_link)
        base_frame_RCF = response.pose_stamped[0].pose.frame # the group's transformed base_frame in RCF
        base_frame = self.get_base_frame(group) # the group's original base_frame

        T = Transformation.from_frame(base_frame)
        base_frame = base_frame_RCF.transformed(T) # the current base frame
        T = Transformation.from_frame_to_frame(base_frame, Frame.worldXY())
        return frame_WCF.transformed(T)

    def represent_frame_in_WCF(self, frame_RCF, group=None):
        """Represents a frame from the robot's coordinate system (RCF) in the world coordinate system (WCF).

        Parameters
        ----------
        frame_RCF : :class:`compas.geometry.Frame`
            A frame in the robot's coordinate frame.
        
        Returns
        -------
        :class:`compas.geometry.Frame`
            A frame in the world coordinate frame.
        
        Examples
        --------
        >>> from compas.geometry import Frame
        >>> from compas_fab.robots.ur5 import Robot
        >>> robot = Robot()
        >>> frame_RCF = Frame([-0.363, 0.003, -0.147], [0.388, -0.351, -0.852], [0.276, 0.926, -0.256])
        >>> frame_WCF = robot.represent_frame_in_WCF(frame_RCF)
        """
        frame_WCF = frame_RCF.transformed(self.transformation_RCF_WCF(group))
        return frame_WCF

    # ==========================================================================
    # checks
    # ==========================================================================

    def ensure_client(self):
        if not self.client:
            raise Exception('This method is only callable once a client is assigned')

    def ensure_semantics(self):
        if not self.semantics:
            raise Exception('This method is only callable once a semantic model is assigned')

    # ==========================================================================
    # services
    # ==========================================================================

    def inverse_kinematics(self, frame_WCF, start_configuration=None,
                           group=None, avoid_collisions=True, 
                           constraints=None, attempts=8):
        """Calculate the robot's inverse kinematic for a given frame.

        Parameters
        ----------
        frame: :class:`compas.geometry.Frame`
            The frame to calculate the inverse for.
        start_configuration: :class:`compas_fab.robots.Configuration`, optional
            If passed, the inverse will be calculated such that the calculated
            joint positions differ the least from the start_configuration.
            Defaults to the init configuration.
        group: str, optional
            The planning group used for calculation. Defaults to the robot's
            main planning group.
        avoid_collisions: bool
            Whether or not to avoid collisions.
        constraints: list of :class:`compas_fab.robots.Constraint`
            A set of constraints that the request must obey. Defaults to None.
        attempts: int
            The maximum number of inverse kinematic attempts.
        
        Raises
        ------
        RosError
            If no configuration can be found.
        
        Returns
        -------
        :class:`compas_fab.robots.Configuration`
            The planning group's configuration.

        Examples
        --------
        >>> from compas.geometry import Frame
        >>> from compas_fab.backends import RosClient
        >>> from compas_fab.backends import RosError
        >>> from compas_fab.robots import Configuration
        >>> from compas_fab.robots.ur5 import Robot
        >>> client = RosClient()
        >>> client.run()
        >>> robot = Robot(client)
        >>> frame_WCF = Frame([0.3, 0.1, 0.5], [1, 0, 0], [0, 1, 0])
        >>> start_configuration = robot.init_configuration()
        >>> group = robot.main_group_name
        >>> configuration = robot.inverse_kinematics(frame_WCF, start_configuration, group)
        >>> client.close()
        >>> client.terminate()
        """
        self.ensure_client()
        if not group:
            group = self.main_group_name # ensure semantics
        base_link = self.get_base_link_name(group)
        joint_names = self.get_configurable_joint_names()

        joint_positions = self._get_scaled_joint_positions_from_start_configuration(start_configuration)

        # represent in RCF
        frame_RCF = self.represent_frame_in_RCF(frame_WCF, group)
        frame_RCF.point /= self.scale_factor # must be in meters

        response = self.client.inverse_kinematics(frame_RCF, base_link,
                                       group, joint_names, joint_positions,
                                       avoid_collisions, constraints, attempts)

        joint_positions = response.solution.joint_state.position
        joint_positions = self._scale_joint_values(joint_positions, self.scale_factor)
        configuration = Configuration(joint_positions, self.get_configurable_joint_types()) # full configuration # TODO group config

        return configuration

    def forward_kinematics(self, configuration, group=None):
        """Calculate the robot's forward kinematic.

        Parameters
        ----------
        configuration : :class:`compas_fab.robots.Configuration`
            The configuration to calculate the forward kinematic for.
        group : str, optional 
            The planning group used for the calculation. Defaults to the robot's
            main planning group.

        Examples
        --------
        >>> from compas_fab.backends import RosClient
        >>> from compas_fab.backends import RosError
        >>> from compas_fab.robots import Configuration
        >>> from compas_fab.robots.ur5 import Robot
        >>> client = RosClient()
        >>> client.run()
        >>> robot = Robot(client)
        >>> configuration = Configuration.from_revolute_values([-2.238, -1.153, -2.174, 0.185, 0.667, 0.000])
        >>> group = robot.main_group_name
        >>> response = robot.forward_kinematics(configuration, group)
        >>> client.close()
        >>> client.terminate()
        """
        # TODO implement with no service, only geometry transformations

        self.ensure_client()
        if not group:
            group = self.main_group_name # ensure semantics

        joint_positions = self._get_scaled_joint_positions_from_start_configuration(configuration)

        base_link = self.get_base_link_name(group)
        joint_names = self.get_configurable_joint_names()
        ee_link = self.get_end_effector_link_name(group)

        response = self.client.forward_kinematics(joint_positions, base_link, group, joint_names, ee_link)
        
        # TODO implement response types

        frame_RCF = response.pose_stamped[0].pose.frame
        frame_RCF.point *= self.scale_factor
        response.frame_RCF = frame_RCF
        response.frame_WCF = self.represent_frame_in_WCF(frame_RCF, group)

        return response

    def plan_cartesian_motion(self, frames_WCF, start_configuration=None, 
                              max_step=0.01, avoid_collisions=True, group=None,
                              path_constraints=None, 
                              attached_collision_object=None):
        """Calculates a cartesian motion path (linear in tool space).

        Parameters
        ----------
        frames_WCF: list of :class:`compas.geometry.Frame`
            The frames through which the path is defined.
        start_configuration: :class:`Configuration`, optional
            The robot's configuration at the starting position. Defaults to the
            zero configuration.
        max_step: float
            The approximate distance between the calculated points. (Defined in
            the robot's units)
        avoid_collisions: bool, optional
            Whether or not to avoid collisions. Defaults to True.
        group: str, optional
            The planning group used for calculation. Defaults to the robot's 
            main planning group.

        Examples
        --------
        >>> from compas_fab.backends import RosClient
        >>> from compas_fab.robots.ur5 import Robot
        >>> client = RosClient()
        >>> client.run()
        >>> robot = Robot(client)
        >>> frames = []
        >>> frames.append(Frame([0.3, 0.1, 0.5], [1, 0, 0], [0, 1, 0]))
        >>> frames.append(Frame([0.4, 0.3, 0.4], [0, 1, 0], [0, 0, 1]))
        >>> start_configuration = Configuration.from_revolute_values([-0.042, 4.295, -4.110, -3.327, 4.755, 0.])
        >>> group = robot.main_group_name
        >>> response = robot.plan_cartesian_motion(frames,\
                                                   start_configuration,\
                                                   max_step=0.01,\
                                                   avoid_collisions=True,\
                                                   group=group)
        >>> client.close()
        >>> client.terminate()
        """
        self.ensure_client()
        if not group:
            group = self.main_group_name # ensure semantics
        frames_RCF = []
        for frame_WCF in frames_WCF:
             # represent in RCF
            frame_RCF = self.represent_frame_in_RCF(frame_WCF, group)
            frame_RCF.point /= self.scale_factor
            frames_RCF.append(frame_RCF)
        base_link = self.get_base_link_name(group)
        joint_names = self.get_configurable_joint_names()

        joint_positions = self._get_scaled_joint_positions_from_start_configuration(start_configuration)

        ee_link = self.get_end_effector_link_name(group)
        max_step_scaled = max_step/self.scale_factor

        response = self.client.plan_cartesian_motion(frames_RCF, base_link,
                                           ee_link, group, joint_names, joint_positions,
                                           max_step_scaled, avoid_collisions, path_constraints,
                                           attached_collision_object)

        # save joint_positions into configurations
        response.configurations = self._configurations_from_joint_trajectory(response.solution.joint_trajectory, group)

        # save start state into start_configuration
        joint_positions = response.start_state.joint_state.position
        joint_positions = self._scale_joint_values(joint_positions, self.scale_factor)
        response.start_configuration = Configuration(joint_positions, self.get_configurable_joint_types())

        # save time from start into response
        time_from_start = response.solution.joint_trajectory.points[-1].time_from_start
        response.time_from_start = time_from_start.secs + time_from_start.nsecs / 1e+9

        return response
    
    # ==========================================================================
    # constraints
    # ==========================================================================

    def orientation_constraint_from_frame(self, frame_WCF, tolerances_axes, 
                                          start_configuration=None, group=None):
        """Returns an orientation constraint on the group's end-effector link.

        Parameters
        ----------
        frame_WCF: :class:`compas.geometry.Frame`
            The frame from which we create the orientation constraint.
        tolerances_axes: list of float
            Error tolerances ti for each of the frame's axes in radians. If only
            one value is passed it will be uses for all 3 axes.
        start_configuration: :class:`compas_fab.robots.Configuration`, optional
            If the planning group's origin is dislocated from it's initial 
            position, e.g. a 6-axes robot moved on a linear axis, the frame in 
            the robot coordinate system (RCF) must be calculated based on its
            start_configuration. Defaults to the zero position for all joints.
        group: str
            The planning group for which we specify the constraint. Defaults to
            the robot's main planning group.
        
        Examples
        --------
        >>> import math
        >>> from compas.geometry import Frame
        >>> from compas_fab.robots import Configuration
        >>> from compas_fab.backends import RosClient
        >>> from compas_fab.robots.ur5 import Robot
        >>> client = RosClient()
        >>> client.run()
        >>> robot = Robot(client)
        >>> frame = Frame([0.4, 0.3, 0.4], [0, 1, 0], [0, 0, 1])
        >>> tolerances_axes = math.radians(1)
        >>> group = robot.main_group_name
        >>> goal_constraints = robot.constraints_from_frame(frame, [tolerances_axes], group)
        >>> client.close()
        >>> client.terminate()

        Notes
        -----
        If you specify the tolerances_axes vector with [0.01, 0.01, 6.3], it 
        means that the frame's x-axis and y-axis are allowed to rotate about the
        z-axis by an angle of 6.3 radians, whereas the z-axis would only rotate
        by 0.01.
        """

        # Attention motion plan needs to receive the frame in the possibly 
        # (moved) group's base frame (in difference to cartesian path).
        frame_RCF = self.represent_frame_in_transformed_RCF(frame_WCF, start_configuration, group)
        frame_RCF.point /= self.scale_factor

        ee_link = self.get_end_effector_link_name(group)

        tolerances_axes = list(tolerances_axes)
        if len(tolerances_axes) == 1:
            tolerances_axes *= 3
        elif len(tolerances_axes) != 3:
            raise ValueError("Must give either one or 3 values")
        return OrientationConstraint(ee_link, frame_RCF.quaternion, tolerances_axes)
    
    def constraints_from_frame(self, frame_WCF, tolerance_position, tolerances_axes, start_configuration=None, group=None):
        """Returns a position and orientation constraint on the group's end-effector link.

        Parameters
        ----------
        frame_WCF: :class:`compas.geometry.Frame`
            The frame from which we create position and orientation constraints.
        tolerance_position: float
            The allowed tolerance to the frame's position. (Defined in the 
            robot's units)
        tolerances_axes: list of float
            Error tolerances ti for each of the frame's axes in radians. If only
            one value is passed it will be uses for all 3 axes.
        start_configuration: :class:`compas_fab.robots.Configuration`

        group: str
            The planning group for which we specify the constraint. Defaults to
            the robot's main planning group.
        
        Examples
        --------
        >>> import math
        >>> from compas.geometry import Frame
        >>> from compas_fab.robots import Configuration
        >>> from compas_fab.backends import RosClient
        >>> from compas_fab.robots.ur5 import Robot
        >>> client = RosClient()
        >>> client.run()
        >>> robot = Robot(client)
        >>> frame = Frame([0.4, 0.3, 0.4], [0, 1, 0], [0, 0, 1])
        >>> tolerance_position = 0.001
        >>> tolerances_axes = [math.radians(1)]
        >>> start_configuration = Configuration.from_revolute_values([-0.042, 4.295, -4.110, -3.327, 4.755, 0.])
        >>> group = robot.main_group_name
        >>> goal_constraints = robot.constraints_from_frame(frame, tolerance_position, tolerances_axes, start_configuration, group)
        >>> client.close()
        >>> client.terminate()

        Notes
        -----
        There are many other possibilities of how to create a position and 
        orientation constraint. Checkout :class:`compas_fab.robots.PositionConstraint`
        and :class:`compas_fab.robots.OrientationConstraint`.

        """

        # Attention motion plan needs to receive the frame in the possibly 
        # (moved) group's base frame (in difference to cartesian path).
        frame_RCF = self.represent_frame_in_transformed_RCF(frame_WCF, start_configuration, group)
        frame_RCF.point /= self.scale_factor
        tolerance_position = tolerance_position/self.scale_factor

        ee_link = self.get_end_effector_link_name(group)
        sphere = Sphere(frame_RCF.point, tolerance_position)
        pc = PositionConstraint.from_sphere(ee_link, sphere)

        oc = self.orientation_constraint_from_frame(frame_WCF, tolerances_axes, 
                                                    start_configuration, group)
        return [pc, oc]
    
    def constraints_from_configuration(self, configuration, tolerances, group=None):
        """Returns joint constraints on all joints of the configuration.

        Parameters
        ----------
        configuration: :class:`compas_fab.robots.Configuration`
            The target configuration.
        tolerances: list of float
            The tolerances on each of the joints defining the bound to be 
            achieved. If only one value is passed it will be used to create
            bounds for all joint constraints.
        group: str, optional
            The planning group for which we specify the constraint. Defaults to
            the robot's main planning group.
        
        Examples
        --------
        >>> import math
        >>> from compas_fab.robots.ur5 import Robot
        >>> from compas_fab.robots import Configuration
        >>> robot = Robot()
        >>> configuration = Configuration.from_revolute_values([-0.042, 4.295, -4.110, -3.327, 4.755, 0.])
        >>> tolerances = [math.radians(5)] * 6
        >>> group = robot.main_group_name
        >>> goal_constraints = robot.constraints_from_configuration(configuration, tolerances, group)

        Raises
        ------
        ValueError
            If the passed configuration does not correspond to the group.
        ValueError
            If the passed tolerances have a different length than the configuration.

        Notes
        -----
        Check for using the correct tolerance units for prismatic and revolute
        joints.

        """
        if not group:
            group = self.main_group_name
        joint_names = self.get_configurable_joint_names(group)
        if len(joint_names) != len(configuration.values):
            raise ValueError("The passed configuration has %d values, the group %s needs however: %d" % (len(configuration.values), group, len(joint_names)))
        if len(tolerances) == 1:
            tolerances = tolerances * len(joint_names)
        elif len(tolerances) != len(configuration.values):
            raise ValueError("The passed configuration has %d values, the tolerances however: %d" % (len(configuration.values), len(tolerances)))
        
        constraints = []
        for name, value, tolerance in zip(joint_names, configuration.values, tolerances):
            constraints.append(JointConstraint(name, value, tolerance))
        return constraints

        
    def plan_motion(self, goal_constraints, start_configuration=None,
                    group=None, path_constraints=None, planner_id='RRT', 
                    num_planning_attempts=1, allowed_planning_time=2.,
                    max_velocity_scaling_factor=1.,
                    max_acceleration_scaling_factor=1.,
                    attached_collision_object=None):
        """Calculates a motion path (linear in joint space).

        Parameters
        ----------
        goal_constraints: list of :class:`compas_fab.robots.Constraint`
            The goal to be achieved, defined in a set of constraints.        
            Constraints can be very specific, for example defining value domains
            for each joint, such that the goal configuration is included,
            or defining a volume in space, to which a specific robot link (e.g.
            the end-effector) is required to move to.
        start_configuration: :class:`compas_fab.robots.Configuration`, optional
            The robot's configuration at the starting position. Defaults to the
            all-zero configuration.
        group: str, optional
            The name of the group to plan for. Defaults to the robot's main 
            planning group.
        path_constraints: list of :class:`compas_fab.robots.Constraint`, optional
            Optional constraints that can be imposed along the solution path. 
            Note that path calculation won't work if the start_configuration 
            violates these constraints. Defaults to None. 
        planner_id: str
            The name of the algorithm used for path planning. Defaults to 'RRT'.
        num_planning_attempts: int, optional
            Normally, if one motion plan is needed, one motion plan is computed.
            However, for algorithms that use randomization in their execution 
            (like 'RRT'), it is likely that different planner executions will 
            produce different solutions. Setting this parameter to a value above
            1 will run many additional motion plans, and will report the 
            shortest solution as the final result. Defaults to 1.
        allowed_planning_time: float
            The number of seconds allowed to perform the planning. Defaults to 2.
        max_velocity_scaling_factor: float
            Defaults to 1.
        max_acceleration_scaling_factor: float
            Defaults to 1.
        attached_collision_object: ?
            Defaults to None.
        

        Examples
        --------
        >>> import math
        >>> from compas.geometry import Frame
        >>> from compas_fab.robots import Configuration
        >>> from compas_fab.backends import RosClient
        >>> from compas_fab.robots.ur5 import Robot
        >>> client = RosClient()
        >>> client.run()
        >>> robot = Robot(client)
        >>> frame = Frame([0.4, 0.3, 0.4], [0, 1, 0], [0, 0, 1])
        >>> tolerance_position = 0.001
        >>> tolerance_angle = math.radians(1)
        >>> start_configuration = Configuration.from_revolute_values([-0.042, 4.295, -4.110, -3.327, 4.755, 0.])
        >>> group = robot.main_group_name
        >>> goal_constraints = robot.constraints_from_frame(frame, tolerance_position, tolerance_orientation, start_configuration, group)
        >>> response = robot.plan_motion(goal_constraints, start_configuration, group, planner_id='RRT')
        >>> client.close()
        >>> client.terminate()

        References
        ----------
        .. [1] Defining a Motion Plan Request.
            Available at: http://docs.ros.org/kinetic/api/moveit_msgs/html/definePlanningRequest.html
        """

        # TODO: for the motion plan request a list of possible goal constraints
        # can be passed, from which the planner will try to find a path that
        # satisfies at least one of the specified goal constraints. For now only
        # one set of goal constraints is supported.

        # TODO: add workspace_parameters

        self.ensure_client()
        if not group:
            group = self.main_group_name # ensure semantics

        joint_names = self.get_configurable_joint_names()

        joint_positions = self._get_scaled_joint_positions_from_start_configuration(start_configuration)

        kwargs = {}
        kwargs['goal_constraints'] = goal_constraints
        kwargs['base_link'] = self.get_base_link_name(group)
        kwargs['ee_link'] = self.get_end_effector_link_name(group)
        kwargs['group'] = group
        kwargs['joint_names'] = joint_names
        kwargs['joint_positions'] = joint_positions
        kwargs['path_constraints'] = path_constraints
        kwargs['trajectory_constraints'] = None
        kwargs['planner_id'] = planner_id
        kwargs['num_planning_attempts'] = num_planning_attempts
        kwargs['allowed_planning_time'] = allowed_planning_time
        kwargs['max_velocity_scaling_factor'] = max_velocity_scaling_factor
        kwargs['max_acceleration_scaling_factor'] = max_acceleration_scaling_factor
        kwargs['attached_collision_object'] = attached_collision_object
        kwargs['workspace_parameters'] = None
 
        response = self.client.plan_motion(**kwargs)
        
        # save joint_positions into configurations
        response.configurations = self._configurations_from_joint_trajectory(response.trajectory.joint_trajectory, group)

        # save trajectory start into start_configuration
        joint_positions = response.trajectory_start.joint_state.position
        joint_positions = self._scale_joint_values(joint_positions, self.scale_factor)
        response.start_configuration = Configuration(joint_positions, self.get_configurable_joint_types())

         # save time from start into response
        time_from_start = response.trajectory.joint_trajectory.points[-1].time_from_start
        response.time_from_start = time_from_start.secs + time_from_start.nsecs / 1e+9

        return response
    
    def _configurations_from_joint_trajectory(self, joint_trajectory, group=None):
        configurations = []
        for point in joint_trajectory.points:
            joint_positions = point.positions
            joint_positions = self._scale_joint_values(joint_positions, self.scale_factor, group)
            configurations.append(Configuration(joint_positions, self.get_configurable_joint_types(group)))
        return configurations


    def add_collision_mesh_to_planning_scene(self, id_name, mesh, scale=False):
        """Adds a collision mesh to the robot's planning scene.

        Parameters
        ----------
            id_name (str): The identifier of the collision mesh.
            mesh (:class:`Mesh`): A triangulated COMPAS mesh.

        Examples
        --------
        """
        self.ensure_client()
        root_link_name = self.model.root.name

        if scale:
            S = Scale([1./self.scale_factor] * 3)
            mesh = mesh_transformed(mesh, S)

        self.client.collision_mesh(id_name, root_link_name, mesh, 0)

    def remove_collision_mesh_from_planning_scene(self, id_name):
        """Removes a collision mesh from the robot's planning scene.

        Parameters
        ----------
            id_name (str): The identifier of the collision mesh.

        Examples
        --------
        """
        self.ensure_client()
        root_link_name = self.model.root.name
        self.client.collision_mesh(id_name, root_link_name, None, 1)

    def append_collision_mesh_to_planning_scene(self, id_name, mesh, scale=False):
        """Appends a collision mesh to the robot's planning scene.

        Parameters
        ----------
            id_name (str): The identifier of the collision mesh.
            mesh (:class:`Mesh`): A triangulated COMPAS mesh.

        Examples
        --------
        """
        self.ensure_client()
        root_link_name = self.model.root.name
        if scale:
            S = Scale([1./self.scale_factor] * 3)
            mesh = mesh_transformed(mesh, S)

        self.client.collision_mesh(id_name, root_link_name, mesh, 2)

    def create_collision_mesh_attached_to_end_effector(self, id_name, mesh, group=None, scale=False, touch_links=None):
        """Creates a collision object that is added to the end effector's tcp.
        """
        if not group:
            group = self.main_group_name # ensure semantics
        ee_link_name = self.get_end_effector_link_name(group)

        if scale:
            S = Scale([1./self.scale_factor] * 3)
            mesh = mesh_transformed(mesh, S)

        last_link_with_geometry = self.get_links_with_geometry(group)[-1]
        if not touch_links:
            touch_links=[last_link_with_geometry.name]
        else:
            touch_links = list(touch_links)
            if last_link_with_geometry.name not in touch_links:
                touch_links.append(last_link_with_geometry.name)

        return self.client.build_attached_collision_mesh(ee_link_name, id_name, mesh, operation=0, touch_links=touch_links)

    def add_attached_collision_mesh(self, id_name, mesh, group=None, touch_links=[], scale=False):
        """Attaches a collision mesh to the robot's end-effector.

        Parameters
        ----------
            id_name (str): The identifier of the object.
            mesh (:class:`Mesh`): A triangulated COMPAS mesh.
            group (str, optional): The planning group on which's end-effector
                the object should be attached. Defaults to the robot's main
                planning group.
            touch_links(str list): The list of link names that the attached mesh
                is allowed to touch by default. The end-effector link name is
                already considered.
        """
        if not group:
            group = self.main_group_name # ensure semantics
        ee_link_name = self.get_end_effector_link_name(group)

        if scale:
            S = Scale([1./self.scale_factor] * 3)
            mesh = mesh_transformed(mesh, S)

        if ee_link_name not in touch_links:
            touch_links.append(ee_link_name)

        self.client.attached_collision_mesh(id_name, ee_link_name, mesh, 0, touch_links)

    def remove_attached_collision_mesh(self, id_name, group=None):
        """Removes an attached collision object from the robot's end-effector.

        Parameters
        ----------
            id_name (str): The identifier of the object.
            group (str, optional): The planning group on which's end-effector
                the object should be removed. Defaults to the robot's main
                planning group.
        """
        if not group:
            group = self.main_group_name # ensure semantics
        ee_link_name = self.get_end_effector_link_name(group)
        self.client.attached_collision_mesh(id_name, ee_link_name, None, 1)

    def send_frame(self):
        # (check service name with ros)
        self.ensure_client()
        raise NotImplementedError

    def send_configuration(self):
        # (check service name with ros)
        self.ensure_client()
        raise NotImplementedError

    def send_trajectory(self):
        # (check service name with ros)
        self.ensure_client()
        raise NotImplementedError

    @property
    def frames(self):
        return self.model.frames

    @property
    def axes(self):
        return self.model.axes

    def update(self, configuration, collision=True, group=None):
        names = self.get_configurable_joint_names(group)
        self.artist.update(configuration, collision, names)

    def draw_visual(self):
        return self.artist.draw_visual()

    def draw_collision(self):
        return self.artist.draw_collision()

    def draw(self):
        return self.draw_visual()

    def scale(self, factor):
        """Scale the robot.
        """
        if self.artist:
            self.artist.scale(factor)
        else:
            self._scale_factor = factor

    @property
    def scale_factor(self):
        if self.artist:
            return self.artist.scale_factor
        else:
            return self._scale_factor

    def info(self):
        print("The robot's name is '%s'." % self.name)
        if self.semantics:
            print("The planning groups are:", self.group_names)
            print("The main planning group is '%s'." % self.main_group_name)
            configurable_joints = self.get_configurable_joints(self.main_group_name)
        else:
            configurable_joints = self.get_configurable_joints()
        print("The end-effector's name is '%s'." % self.get_end_effector_link_name())
        print("The base link's name is '%s'" % self.get_base_link_name())
        print("The base_frame is:", self.get_base_frame())
        print("The robot's joints are:")
        for joint in configurable_joints:
            info = "\t* '%s' is of type '%s'" % (joint.name, list(Joint.SUPPORTED_TYPES)[joint.type])
            if joint.limit:
                info += " and has limits [%.3f, %.3f]" % (joint.limit.upper, joint.limit.lower)
            print(info)
        print("The robot's links are:")
        print([l.name for l in self.model.links])

if __name__ == "__main__":

    import doctest
    doctest.testmod()

    """

    import math
    from compas_fab.backends import RosClient
    from compas_fab.backends import RosError
    from compas_fab.robots.ur5 import Robot
    client = RosClient()
    client.run()
    robot = Robot(client)

    tolerance_position = 0.001 
    tolerance_axes = [math.radians(angle) for angle in [5, 5, 360]]

    f1 = Frame([-0.363, 0.003, -0.147], [0.388, -0.351, -0.852], [0.276, 0.926, -0.256])
    
    start_configuration = Configuration.from_revolute_values([0] * 6)

    goal_constraints = robot.constraints_from_frame(f1, tolerance_position, tolerance_axes)

    try:
        response = robot.plan_motion(goal_constraints, start_configuration=start_configuration, planner_id='RRT', num_planning_attempts=10.)
        last_configuration = response.configurations[-1]
        response = robot.forward_kinematics(last_configuration)
        f2 = response.frame_RCF
        angles = f1.xaxis.angle(f2.xaxis), f1.yaxis.angle(f2.yaxis), f1.zaxis.angle(f2.zaxis)
        print(tolerance_axes)
        print(angles)
    except RosError as error:
        print(error)


    client.close()
    client.terminate()
    """