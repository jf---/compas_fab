from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from collections import deque
from typing import List, Dict

from compas.datastructures import Mesh
from compas.geometry import Frame

__all__ = ['pose_from_frame', 'frame_from_pose']

from compas.robots import Link, Visual, Collision, Axis, Origin, Material, RobotModel, Joint, ParentLink, ChildLink, Color
from numpy import ndarray
from tesseract import tesseract_scene_graph
from tesseract.tesseract_common import Isometry3d
from tesseract.tesseract_environment import Environment
from tesseract.tesseract_geometry import CONVEX_MESH, MESH
from tesseract_robotics.tesseract_geometry import Mesh as TaMesh


def pose_from_frame(frame):
    """Returns a Tesseract pose from a frame.

    Parameters
    ----------
    frame : :class:`compas.geometry.Frame`

    Returns
    -------
    point, quaternion : tuple
    """
    return list(frame.point), frame.quaternion.xyzw


def frame_from_pose(pose):
    """Returns a frame from a Tesseract pose.

    Parameters
    ----------
    pose : tuple

    Returns
    -------
    :class:`compas.geometry.Frame`
    """
    point, (x, y, z, w) = pose
    return Frame.from_quaternion([w, x, y, z], point=point)


def tesseract_mesh_to_compas_mesh(_mesh: TaMesh) -> Mesh:
    verts = []
    faces = []

    x = _mesh.getVertices()

    for i in range(x.size()):
        v: ndarray = x[i]
        verts.append(v.tolist())

    if _mesh.getType() == CONVEX_MESH:
        faces: deque = deque(_mesh.getFaces().tolist())

    elif _mesh.getType() == MESH:
        faces: deque = deque(_mesh.getTriangles().tolist())

    face_indx = faces.popleft()[0]

    try:
        while 1:
            _face = []
            for i in range(face_indx):
                _face.append(faces.popleft()[0])
            faces.append(_face)
    except IndexError:
        pass

    msh = Mesh.from_vertices_and_faces(verts, faces)
    return msh


def tesseract_link_to_compas_link(_link: tesseract_scene_graph.Link) -> Link:
    visual_meshes: List[Visual] = []
    collision_meshes: List[Collision] = []

    origin = _link

    for ta_visual in _link.visual:
        _visual: tesseract_scene_graph.Visual = ta_visual
        # _visual_mat = material_from_tesseract_visual()
        _visual_geom: TaMesh = ta_visual.geometry
        _visual_mesh = tesseract_mesh_to_compas_mesh(_visual_geom)
        # TODO: sort out origin
        visual = Visual(geometry=_visual_mesh, origin=None, name=_link.getName())
        visual_meshes.append(visual)

    for ta_mesh in _link.collision:
        _collision_geom: TaMesh = ta_mesh.geometry
        _collision_mesh = tesseract_mesh_to_compas_mesh(_collision_geom)
        # TODO: sort out origin
        collision = Collision(geometry=_collision_mesh, origin=None, name=_link.getName())
        collision_meshes.append(collision)

    link = Link(_link.getName(), visual=visual_meshes, collision=collision_meshes)

    return link


JOINT_MAP = {
    tesseract_scene_graph.JointType_REVOLUTE: "revolute",
    tesseract_scene_graph.JointType_PRISMATIC: "prismatic",
    tesseract_scene_graph.JointType_PLANAR: "planar",
    tesseract_scene_graph.JointType_FIXED: "fixed",
}


def axis_from_tesseract_joint(joint: tesseract_scene_graph.Joint) -> Axis:
    axis = Axis()
    axis.x = joint.axis[0]
    axis.y = joint.axis[1]
    axis.z = joint.axis[2]
    return axis


def origin_from_tesseract_joint(joint: tesseract_scene_graph.Joint) -> Origin:
    iso: Isometry3d = joint.parent_to_joint_origin_transform
    origin = Origin.from_matrix(iso.matrix().tolist())
    return origin


def material_from_tesseract_visual(viz: tesseract_scene_graph.Visual) -> Material:
    color = Color.from_data()
    material: tesseract_scene_graph.Material = viz.getMaterial()
    color: ndarray = material.getBaseColorFactor()

    material.color = color.tolist()
    mat = Material()
    return mat


def robot_from_tesseract_env(env: Environment) -> RobotModel:
    _link_names = env.getLinkNames()

    print(_link_names)

    links: Dict[str: Link] = {}

    for name in _link_names:
        ta_link: tesseract_scene_graph.Link = env.getLink(name)
        link = tesseract_link_to_compas_link(ta_link)
        # TODO: material
        links[name] = link

        ta_link.visual

    _joint_names = env.getJointNames()

    joints: List[Joint] = []
    for name in _joint_names:
        _joint: tesseract_scene_graph.Joint = env.getJoint(name)

        joint_type = JOINT_MAP[_joint.type]

        axis = axis_from_tesseract_joint(_joint)
        origin = origin_from_tesseract_joint(_joint)

        parent_link = ParentLink(links[_joint.parent_link_name])
        child_link = ChildLink(links[_joint.child_link_name])

        joint = Joint(name, joint_type, parent_link, child_link, origin=origin, axis=axis)
        joints.append(joint)

    # building the robot takes excessively long...
    # ~0.7s for loading with tesseract
    # ~24.5s for recreating the compas Robot from tesseract data...
    rm = RobotModel(env.getName(), joints, links.values())  # , materials=materials)

    return rm
