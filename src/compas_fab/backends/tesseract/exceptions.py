"""
Exceptions for the compas_fab Tesseract backend.
"""

from __future__ import annotations


__all__ = [
    'TesseractError',
    'TesseractPlanningError',
    'TesseractKinematicsError',
    'TesseractCollisionError',
    'TesseractEnvironmentError',
]


class TesseractError(Exception):
    """Base exception for Tesseract backend errors."""
    pass


class TesseractPlanningError(TesseractError):
    """Raised when motion planning fails.
    
    Attributes
    ----------
    planner : str
        The planner that failed (e.g., 'ompl', 'trajopt').
    message : str
        Detailed error message from Tesseract.
    """
    
    def __init__(self, message: str, planner: str = None):
        self.planner = planner
        super().__init__(f"Planning failed ({planner}): {message}" if planner else message)


class TesseractKinematicsError(TesseractError):
    """Raised when kinematics computation fails.
    
    Attributes
    ----------
    operation : str
        The operation that failed ('fk' or 'ik').
    """
    
    def __init__(self, message: str, operation: str = None):
        self.operation = operation
        super().__init__(f"Kinematics error ({operation}): {message}" if operation else message)


class TesseractCollisionError(TesseractError):
    """Raised when collision checking detects issues.
    
    Attributes
    ----------
    contacts : list
        List of collision contacts if available.
    """
    
    def __init__(self, message: str, contacts: list = None):
        self.contacts = contacts or []
        super().__init__(message)


class TesseractEnvironmentError(TesseractError):
    """Raised when environment operations fail.
    
    This includes URDF/SRDF loading, scene graph modifications, etc.
    """
    pass
