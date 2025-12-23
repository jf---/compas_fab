"""
Type conversions between compas/compas_fab and tesseract types.

This module handles the mapping between:
- compas.geometry types <-> Eigen types (via numpy)
- compas_fab.robots types <-> tesseract command_language types
- compas meshes <-> tesseract geometry
"""

from __future__ import annotations

from typing import TYPE_CHECKING, List, Optional, Tuple
import numpy as np

from compas.geometry import Frame, Point, Vector, Quaternion, Transformation
from compas_robots import Configuration

if TYPE_CHECKING:
    from compas_fab.robots import JointTrajectory, JointTrajectoryPoint


__all__ = [
    'frame_to_isometry',
    'isometry_to_frame',
    'configuration_to_joint_waypoint',
    'configuration_to_state_waypoint',
    'composite_instruction_to_trajectory',
    'mesh_to_tesseract_mesh',
    'joint_limits_to_array',
]


def frame_to_isometry(frame: Frame) -> np.ndarray:
    """Convert a compas Frame to a 4x4 homogeneous transformation matrix.
    
    The tesseract nanobind bindings have a type caster that converts
    numpy 4x4 arrays to/from Eigen::Isometry3d.
    
    Parameters
    ----------
    frame : :class:`compas.geometry.Frame`
        The frame to convert.
        
    Returns
    -------
    numpy.ndarray
        4x4 homogeneous transformation matrix.
    """
    # Build rotation matrix from frame axes
    xaxis = np.array(frame.xaxis)
    yaxis = np.array(frame.yaxis)
    zaxis = np.array(frame.zaxis)
    point = np.array(frame.point)
    
    matrix = np.eye(4)
    matrix[:3, 0] = xaxis
    matrix[:3, 1] = yaxis
    matrix[:3, 2] = zaxis
    matrix[:3, 3] = point
    
    return matrix


def isometry_to_frame(isometry) -> Frame:
    """Convert an Isometry3d or 4x4 homogeneous matrix to a compas Frame.

    Parameters
    ----------
    isometry : Isometry3d or numpy.ndarray
        Isometry3d object or 4x4 homogeneous transformation matrix.

    Returns
    -------
    :class:`compas.geometry.Frame`
        The converted frame.
    """
    # Handle Isometry3d objects from tesseract
    if hasattr(isometry, 'matrix'):
        matrix = isometry.matrix()
    else:
        matrix = isometry

    point = Point(*matrix[:3, 3])
    xaxis = Vector(*matrix[:3, 0])
    yaxis = Vector(*matrix[:3, 1])

    return Frame(point, xaxis, yaxis)


def configuration_to_joint_waypoint(
    configuration: Configuration,
    joint_names: List[str]
) -> 'JointWaypoint':
    """Convert a compas_fab Configuration to a tesseract JointWaypoint.
    
    Parameters
    ----------
    configuration : :class:`compas_fab.robots.Configuration`
        The configuration to convert.
    joint_names : list of str
        The joint names in the order expected by tesseract.
        
    Returns
    -------
    JointWaypoint
        The tesseract joint waypoint.
        
    Note
    ----
    Import tesseract modules lazily to avoid import errors when
    tesseract is not installed.
    """
    from tesseract_robotics.tesseract_command_language import JointWaypoint
    
    # Reorder joint values to match expected joint names
    if configuration.joint_names:
        joint_dict = configuration.joint_dict
        values = [joint_dict.get(name, 0.0) for name in joint_names]
    else:
        values = list(configuration.joint_values)
    
    return JointWaypoint(joint_names, np.array(values, dtype=np.float64))


def configuration_to_state_waypoint(
    configuration: Configuration,
    joint_names: List[str]
) -> 'StateWaypoint':
    """Convert a compas_fab Configuration to a tesseract StateWaypoint.
    
    StateWaypoint is used for trajectory results and includes velocity/acceleration.
    
    Parameters
    ----------
    configuration : :class:`compas_fab.robots.Configuration`
        The configuration to convert.
    joint_names : list of str
        The joint names in the order expected by tesseract.
        
    Returns
    -------
    StateWaypoint
        The tesseract state waypoint.
    """
    from tesseract_robotics.tesseract_command_language import StateWaypoint
    
    # Reorder joint values to match expected joint names
    if configuration.joint_names:
        joint_dict = configuration.joint_dict
        values = [joint_dict.get(name, 0.0) for name in joint_names]
    else:
        values = list(configuration.joint_values)
    
    return StateWaypoint(joint_names, np.array(values, dtype=np.float64))


def composite_instruction_to_trajectory(
    composite: 'CompositeInstruction',
    joint_names: List[str],
    joint_types: Optional[List[int]] = None
) -> 'JointTrajectory':
    """Convert a tesseract CompositeInstruction result to a compas_fab JointTrajectory.

    This handles the cross-module RTTI issues by using the helper functions
    from the tesseract bindings.

    Parameters
    ----------
    composite : CompositeInstruction
        The planning result from tesseract.
    joint_names : list of str
        The joint names for the trajectory.
    joint_types : list of int, optional
        The joint types (revolute=0, prismatic=1, etc.)

    Returns
    -------
    :class:`compas_fab.robots.JointTrajectory`
        The converted trajectory.
    """
    from compas_fab.robots import JointTrajectory, JointTrajectoryPoint
    from compas_robots import Configuration

    # Import tesseract helpers for cross-module type recovery
    from tesseract_robotics.tesseract_command_language import (
        InstructionPoly_as_MoveInstructionPoly,
        flattenProgram,
    )

    # Flatten the composite instruction to a simple sequence
    flattened = flattenProgram(composite)

    trajectory_points = []

    for i, instruction in enumerate(flattened):
        if not instruction.isMoveInstruction():
            continue

        # Use helper to recover MoveInstruction across module boundary
        move_instr = InstructionPoly_as_MoveInstructionPoly(instruction)
        waypoint = move_instr.getWaypoint()

        # Extract joint values based on waypoint type
        if waypoint.isStateWaypoint():
            state_wp = waypoint.as_StateWaypointPoly()
            positions = list(state_wp.getPosition())
            velocities = list(state_wp.getVelocity()) if hasattr(state_wp, 'getVelocity') else None
            accelerations = list(state_wp.getAcceleration()) if hasattr(state_wp, 'getAcceleration') else None
        elif waypoint.isJointWaypoint():
            joint_wp = waypoint.as_JointWaypointPoly()
            positions = list(joint_wp.getPosition())
            velocities = None
            accelerations = None
        else:
            continue

        # Create trajectory point
        point = JointTrajectoryPoint(
            joint_values=positions,
            joint_types=joint_types or [0] * len(positions),
            velocities=velocities,
            accelerations=accelerations,
            joint_names=joint_names,
        )

        trajectory_points.append(point)

    # Create start configuration from first point
    if trajectory_points:
        start_config = Configuration(
            joint_values=trajectory_points[0].joint_values,
            joint_types=trajectory_points[0].joint_types,
            joint_names=joint_names,
        )
    else:
        start_config = None

    return JointTrajectory(
        trajectory_points=trajectory_points,
        joint_names=joint_names,
        start_configuration=start_config,
        fraction=1.0,
    )


def mesh_to_tesseract_mesh(
    mesh: 'compas.datastructures.Mesh',
    scale: float = 1.0
) -> 'tesseract_geometry.Mesh':
    """Convert a compas Mesh to a tesseract geometry Mesh.
    
    Parameters
    ----------
    mesh : :class:`compas.datastructures.Mesh`
        The compas mesh to convert.
    scale : float, optional
        Scale factor for the mesh. Default is 1.0.
        
    Returns
    -------
    tesseract_geometry.Mesh
        The tesseract mesh geometry.
    """
    from tesseract_robotics.tesseract_geometry import Mesh as TesseractMesh
    
    # Get vertices as numpy array
    vertices = np.array([mesh.vertex_coordinates(v) for v in mesh.vertices()], dtype=np.float64)
    vertices *= scale
    
    # Get faces as flat list of indices (triangulated)
    faces = []
    for face in mesh.faces():
        face_vertices = mesh.face_vertices(face)
        if len(face_vertices) == 3:
            faces.extend(face_vertices)
        elif len(face_vertices) == 4:
            # Triangulate quad
            faces.extend([face_vertices[0], face_vertices[1], face_vertices[2]])
            faces.extend([face_vertices[0], face_vertices[2], face_vertices[3]])
        else:
            # Fan triangulation for n-gons
            for i in range(1, len(face_vertices) - 1):
                faces.extend([face_vertices[0], face_vertices[i], face_vertices[i + 1]])
    
    faces = np.array(faces, dtype=np.int32)
    
    return TesseractMesh(vertices, faces)


def joint_limits_to_array(
    joint_limits: List[Tuple[float, float]]
) -> np.ndarray:
    """Convert joint limits to numpy array format expected by tesseract.
    
    Parameters
    ----------
    joint_limits : list of tuple
        List of (min, max) tuples for each joint.
        
    Returns
    -------
    numpy.ndarray
        Array of shape (n_joints, 2) with [min, max] columns.
    """
    return np.array(joint_limits, dtype=np.float64)


def frame_to_cartesian_waypoint(
    frame: Frame,
    seed: Optional[Configuration] = None,
    joint_names: Optional[List[str]] = None
) -> 'CartesianWaypoint':
    """Convert a compas Frame to a tesseract CartesianWaypoint.
    
    Parameters
    ----------
    frame : :class:`compas.geometry.Frame`
        The target frame.
    seed : :class:`compas_fab.robots.Configuration`, optional
        Seed configuration for IK solving.
    joint_names : list of str, optional
        Joint names for the seed configuration.
        
    Returns
    -------
    CartesianWaypoint
        The tesseract cartesian waypoint.
    """
    from tesseract_robotics.tesseract_command_language import CartesianWaypoint
    
    isometry = frame_to_isometry(frame)
    waypoint = CartesianWaypoint(isometry)
    
    if seed is not None and joint_names is not None:
        seed_values = configuration_to_joint_waypoint(seed, joint_names)
        waypoint.setSeed(seed_values)
    
    return waypoint


def constraints_to_tesseract(
    constraints: List,
    robot,
    group: str
) -> Tuple['CompositeInstruction', dict]:
    """Convert compas_fab constraints to tesseract motion planning structures.
    
    This is a placeholder for the constraint conversion logic.
    The actual implementation depends on the constraint types used.
    
    Parameters
    ----------
    constraints : list
        List of compas_fab constraint objects.
    robot : Robot
        The robot instance.
    group : str
        The planning group name.
        
    Returns
    -------
    tuple
        (CompositeInstruction, profile_dict) for tesseract planning.
    """
    # This will be implemented based on the constraint types:
    # - PositionConstraint -> CartesianWaypoint
    # - OrientationConstraint -> CartesianWaypoint with orientation
    # - JointConstraint -> JointWaypoint
    raise NotImplementedError("Constraint conversion not yet implemented")
