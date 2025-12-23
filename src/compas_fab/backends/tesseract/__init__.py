"""
compas_fab_tesseract - Tesseract motion planning backend for compas_fab.

This package provides integration between the Tesseract Motion Planning Framework
and compas_fab, enabling access to OMPL, TrajOpt, and other motion planners.

Example
-------
>>> from compas_fab_tesseract import TesseractClient
>>> with TesseractClient() as client:
...     robot = client.load_robot('robot.urdf', 'robot.srdf')
...     trajectory = robot.plan_motion(goal_constraints, start_configuration)
"""

from __future__ import annotations

__version__ = '0.1.0'

from .client import TesseractClient
from .planner import TesseractPlanner
from .exceptions import (
    TesseractError,
    TesseractPlanningError,
    TesseractKinematicsError,
    TesseractCollisionError,
)

__all__ = [
    'TesseractClient',
    'TesseractPlanner',
    'TesseractError',
    'TesseractPlanningError',
    'TesseractKinematicsError',
    'TesseractCollisionError',
]


def __getattr__(name):
    """Lazy import for optional modules."""
    if name == 'conversions':
        from . import conversions
        return conversions
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
