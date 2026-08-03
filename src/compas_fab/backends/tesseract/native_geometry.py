"""COMPAS mesh conversion to native Tesseract mesh and convex collision geometry."""

from __future__ import annotations

from typing import cast

import numpy as np
from compas.datastructures import Mesh  # type: ignore[import-untyped]
from numpy.typing import NDArray
from tesseract_robotics import tesseract_collision
from tesseract_robotics import tesseract_geometry

from .errors import DegenerateSceneMeshError

# A triangle is the minimum polygon Tesseract accepts per VTK face record.
MIN_FACE_VERTEX_COUNT = 3


def native_mesh(mesh: Mesh) -> tesseract_geometry.Mesh:
    """Convert a COMPAS mesh in metres to a native triangulated Tesseract mesh.

    Faces are triangulated and flattened to the VTK convention Tesseract
    expects: a flat ``int32`` array ``[n, v0, .., v(n-1), n, v0, ..]`` where
    ``n`` is the vertex count of each face.

    Args:
        mesh: COMPAS mesh whose vertex coordinates are already in metres.

    Returns:
        A native Tesseract mesh referencing the flattened triangle faces.

    Raises:
        DegenerateSceneMeshError: The mesh has no vertices, no faces, a face
            with fewer than three vertices, or a non-finite vertex coordinate.
    """
    vertices, faces = mesh.to_vertices_and_faces(triangulated=True)
    if not vertices or not faces:
        raise DegenerateSceneMeshError("Scene mesh must have at least one vertex and one face, got {} vertices and {} faces.".format(len(vertices), len(faces)))

    native_vertices: list[NDArray[np.float64]] = []
    for vertex in vertices:
        point = np.asarray(vertex, dtype=np.float64)
        if point.shape != (3,) or not np.isfinite(point).all():
            raise DegenerateSceneMeshError("Scene mesh vertex must be three finite coordinates, got {!r}.".format(vertex))
        native_vertices.append(point)

    flat_faces: list[int] = []
    for face in faces:
        if len(face) < MIN_FACE_VERTEX_COUNT:
            raise DegenerateSceneMeshError("Scene mesh face must have at least {} vertices, got {}.".format(MIN_FACE_VERTEX_COUNT, len(face)))
        flat_faces.append(len(face))
        flat_faces.extend(int(index) for index in face)

    return tesseract_geometry.Mesh(native_vertices, np.asarray(flat_faces, dtype=np.int32))


def native_convex_collision(mesh: Mesh) -> tesseract_geometry.ConvexMesh:
    """Build a native convex-hull collision mesh from a COMPAS mesh in metres.

    Args:
        mesh: COMPAS mesh whose vertex coordinates are already in metres.

    Returns:
        A native convex mesh suitable for Tesseract collision geometry.

    Raises:
        DegenerateSceneMeshError: The source mesh is empty or degenerate.
    """
    return cast(tesseract_geometry.ConvexMesh, tesseract_collision.makeConvexMesh(native_mesh(mesh)))
