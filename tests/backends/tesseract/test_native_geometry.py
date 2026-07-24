"""Contract tests for COMPAS mesh to native Tesseract geometry conversion."""

import numpy as np
import pytest
from compas.datastructures import Mesh
from compas.geometry import Box
from tesseract_robotics import tesseract_geometry

from compas_fab.backends.tesseract.errors import DegenerateSceneMeshError
from compas_fab.backends.tesseract.native_geometry import native_convex_collision
from compas_fab.backends.tesseract.native_geometry import native_mesh


@pytest.fixture
def box_mesh():
    return Mesh.from_shape(Box(0.2, 0.2, 0.2))


def test_native_mesh_preserves_vertex_count(box_mesh):
    native = native_mesh(box_mesh)
    assert native.getVertexCount() == 8


def test_native_mesh_triangulates_faces(box_mesh):
    native = native_mesh(box_mesh)
    # A box has six quad faces; triangulation yields twelve triangles.
    assert native.getFaceCount() == 12


def test_native_mesh_flat_faces_use_vtk_convention(box_mesh):
    _, faces = box_mesh.to_vertices_and_faces(triangulated=True)
    native = native_mesh(box_mesh)
    flat = np.asarray(native.getFaces()).reshape(-1)
    # VTK convention: [3, a, b, c, 3, d, e, f, ...] with a leading count per face.
    assert flat.dtype == np.int32
    assert len(flat) == len(faces) * 4
    reconstructed = []
    cursor = 0
    while cursor < len(flat):
        count = int(flat[cursor])
        assert count == 3
        reconstructed.append([int(i) for i in flat[cursor + 1 : cursor + 1 + count]])
        cursor += count + 1
    assert reconstructed == [list(face) for face in faces]


def test_native_mesh_does_not_mutate_input(box_mesh):
    before = box_mesh.number_of_faces()
    native_mesh(box_mesh)
    assert box_mesh.number_of_faces() == before


def test_native_mesh_empty_raises_degenerate():
    with pytest.raises(DegenerateSceneMeshError):
        native_mesh(Mesh())


def test_native_convex_collision_returns_convex_mesh(box_mesh):
    convex = native_convex_collision(box_mesh)
    assert isinstance(convex, tesseract_geometry.ConvexMesh)
    assert convex.getVertexCount() >= 4


def test_native_convex_collision_empty_raises_degenerate():
    with pytest.raises(DegenerateSceneMeshError):
        native_convex_collision(Mesh())
