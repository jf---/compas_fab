from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas_fab.backends.exceptions import BackendError


class TesseractError(BackendError):
    """Base case for exceptions in ``compas_fab.backends.pybullet``."""
    def __init__(self, message):
        super(TesseractError, self).__init__(message)


class CollisionError(TesseractError):
    """Exception raised when two objects have been found to be in collision in Tesseract."""
    def __init__(self, name1, name2):
        message = "Tesseract: Collision between '{}' and '{}'".format(name1, name2)
        super(CollisionError, self).__init__(message)
        self.name1 = name1
        self.name2 = name2


class InverseKinematicsError(TesseractError):
    """Exception raised when no IK solution can be found in Tesseract."""
    def __init__(self):
        message = "Tesseract: No inverse kinematics solution found."
        super(InverseKinematicsError, self).__init__(message)
