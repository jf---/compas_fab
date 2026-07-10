"""Strict joint-order projection shared by native Tesseract operations."""

from __future__ import annotations

from typing import Mapping
from typing import Protocol

from tesseract_robotics.planning import Robot

from .errors import TesseractConfigurationMismatchError


class NamedConfiguration(Protocol):
    """Structural configuration fields needed at the native boundary."""

    joint_names: list[str]

    @property
    def joint_dict(self) -> Mapping[str, float]: ...


def ordered_joint_positions(
    configuration: NamedConfiguration,
    joint_names: list[str],
    label: str,
) -> list[float]:
    """Project named values into an exact native joint order."""
    if not configuration.joint_names:
        raise TesseractConfigurationMismatchError("{} has no joint names.".format(label))
    values_by_name = configuration.joint_dict
    missing = [name for name in joint_names if name not in values_by_name]
    if missing:
        raise TesseractConfigurationMismatchError("{} is missing joints: {}.".format(label, ", ".join(missing)))
    return [values_by_name[name] for name in joint_names]


def apply_active_joint_state(
    robot: Robot,
    configuration: NamedConfiguration,
    label: str,
) -> None:
    """Apply every active environment joint without assuming omitted zeros."""
    joint_names = list(robot.env.getActiveJointNames())
    positions = ordered_joint_positions(configuration, joint_names, label)
    robot.set_joints(dict(zip(joint_names, positions)))
