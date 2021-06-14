from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas_fab.backends.interfaces.client import forward_docstring
from compas_fab.backends.interfaces.client import PlannerInterface
from compas_fab.backends.tesseract.backend_features.tesseract_add_attached_collision_mesh import TesseractAddAttachedCollisionMesh
from compas_fab.backends.tesseract.backend_features.tesseract_forward_kinematics import TesseractForwardKinematics
from compas_fab.backends.tesseract.backend_features.tesseract_inverse_kinematics import TesseractInverseKinematics
from compas_fab.backends.tesseract.backend_features.tesseract_add_collision_mesh import TesseractAddCollisionMesh
from compas_fab.backends.tesseract.backend_features.tesseract_append_collision_mesh import TesseractAppendCollisionMesh
from compas_fab.backends.tesseract.backend_features.tesseract_remove_attached_collision_mesh import TesseractRemoveAttachedCollisionMesh
from compas_fab.backends.tesseract.backend_features.tesseract_remove_collision_mesh import TesseractRemoveCollisionMesh


class TesseractPlanner(PlannerInterface):
    """Implement the planner backend interface for Tesseract."""
    def __init__(self, client):
        super(TesseractPlanner, self).__init__(client)

    @forward_docstring(TesseractAddAttachedCollisionMesh)
    def add_attached_collision_mesh(self, *args, **kwargs):
        return TesseractAddAttachedCollisionMesh(self.client)(*args, **kwargs)

    @forward_docstring(TesseractAddCollisionMesh)
    def add_collision_mesh(self, *args, **kwargs):
        return TesseractAddCollisionMesh(self.client)(*args, **kwargs)

    @forward_docstring(TesseractAppendCollisionMesh)
    def append_collision_mesh(self, *args, **kwargs):
        return TesseractAppendCollisionMesh(self.client)(*args, **kwargs)

    @forward_docstring(TesseractRemoveCollisionMesh)
    def remove_collision_mesh(self, *args, **kwargs):
        return TesseractRemoveCollisionMesh(self.client)(*args, **kwargs)

    @forward_docstring(TesseractRemoveAttachedCollisionMesh)
    def remove_attached_collision_mesh(self, *args, **kwargs):
        return TesseractRemoveAttachedCollisionMesh(self.client)(*args, **kwargs)

    @forward_docstring(TesseractForwardKinematics)
    def forward_kinematics(self, *args, **kwargs):
        return TesseractForwardKinematics(self.client)(*args, **kwargs)

    @forward_docstring(TesseractInverseKinematics)
    def inverse_kinematics(self, *args, **kwargs):
        return TesseractInverseKinematics(self.client)(*args, **kwargs)
