"""Exact native planning-group shape and ordered joint authority."""

from __future__ import annotations

from typing import Optional
from typing import Tuple

from attrs import define
from attrs import field
from tesseract_robotics.planning import Robot

_GROUP_SHAPE_FACTORY_TOKEN = object()


class GroupShapeError(ValueError):
    """Base failure for exact native group shapes."""


class InvalidPlanningGroupIdError(GroupShapeError):
    """Raised when a planning-group identifier is malformed."""


class InvalidJointIdError(GroupShapeError):
    """Raised when a joint identifier is malformed."""


class InvalidDegreesOfFreedomError(GroupShapeError):
    """Raised when a DOF value is not a positive exact integer."""


class EmptyGroupShapeError(GroupShapeError):
    """Raised when a group has no ordered joints."""


class DuplicateJointIdError(GroupShapeError):
    """Raised when native group order contains duplicate joints."""


class InvalidGroupShapeError(GroupShapeError):
    """Raised when stored group-shape fields disagree."""


class NativeGroupShapeQueryError(GroupShapeError):
    """Raised when the native robot cannot resolve a group."""


@define(frozen=True, slots=True)
class PlanningGroupId:
    value: str

    @classmethod
    def build(cls, value: str) -> "PlanningGroupId":
        return cls(value)

    def __attrs_post_init__(self) -> None:
        if type(self.value) is not str or not self.value or self.value != self.value.strip():
            raise InvalidPlanningGroupIdError("Planning group ID must be non-empty canonical text.")


@define(frozen=True, slots=True)
class JointId:
    value: str

    @classmethod
    def build(cls, value: str) -> "JointId":
        return cls(value)

    def __attrs_post_init__(self) -> None:
        if type(self.value) is not str or not self.value or self.value != self.value.strip():
            raise InvalidJointIdError("Joint ID must be non-empty canonical text.")


@define(frozen=True, slots=True)
class DegreesOfFreedom:
    value: int

    @classmethod
    def build(cls, value: int) -> "DegreesOfFreedom":
        return cls(value)

    def __attrs_post_init__(self) -> None:
        if type(self.value) is not int or self.value <= 0:
            raise InvalidDegreesOfFreedomError("Degrees of freedom must be a positive exact integer.")


@define(frozen=True, slots=True)
class GroupShape:
    """One native group, exact joint order, and derived DOF."""

    group_id: PlanningGroupId
    ordered_joint_ids: Tuple[JointId, ...]
    dof: DegreesOfFreedom
    _factory_token: Optional[object] = field(default=None, eq=False, repr=False)

    @classmethod
    def build(
        cls,
        group_id: PlanningGroupId,
        ordered_joint_ids: Tuple[JointId, ...],
    ) -> "GroupShape":
        if type(group_id) is not PlanningGroupId:
            raise InvalidPlanningGroupIdError("Group shape requires an exact PlanningGroupId.")
        if type(ordered_joint_ids) is not tuple:
            raise InvalidGroupShapeError("Group joint order must be an exact tuple.")
        if any(type(value) is not JointId for value in ordered_joint_ids):
            raise InvalidJointIdError("Group shape requires exact JointId values.")
        joints = ordered_joint_ids
        if not joints:
            raise EmptyGroupShapeError("Group shape requires at least one joint.")
        if len(joints) != len(set(joints)):
            raise DuplicateJointIdError("Group shape joint IDs must be unique.")
        return cls(group_id, joints, DegreesOfFreedom.build(len(joints)), _GROUP_SHAPE_FACTORY_TOKEN)

    def __attrs_post_init__(self) -> None:
        valid = (
            type(self.group_id) is PlanningGroupId
            and type(self.ordered_joint_ids) is tuple
            and bool(self.ordered_joint_ids)
            and all(type(value) is JointId for value in self.ordered_joint_ids)
            and len(set(self.ordered_joint_ids)) == len(self.ordered_joint_ids)
            and type(self.dof) is DegreesOfFreedom
            and self.dof.value == len(self.ordered_joint_ids)
            and self._factory_token is _GROUP_SHAPE_FACTORY_TOKEN
        )
        if not valid:
            raise InvalidGroupShapeError("Group shape must retain exact group, unique joint order, and derived DOF.")


class GroupShapeFactory:
    """Native boundary resolving exact group order once."""

    @staticmethod
    def from_native_robot(native_robot: Robot, group_id: PlanningGroupId) -> GroupShape:
        if not isinstance(native_robot, Robot):
            raise NativeGroupShapeQueryError("Group shape requires an exact native Robot.")
        if type(group_id) is not PlanningGroupId:
            raise InvalidPlanningGroupIdError("Group shape requires an exact PlanningGroupId.")
        group = group_id
        try:
            names = tuple(native_robot.get_joint_names(group.value))
        except (AttributeError, RuntimeError, TypeError, ValueError) as error:
            raise NativeGroupShapeQueryError("Native robot cannot resolve group {!r}: {}.".format(group, error)) from error
        try:
            joints = tuple(JointId.build(name) for name in names)
            return GroupShape.build(group, joints)
        except GroupShapeError as error:
            raise NativeGroupShapeQueryError("Native group {!r} has invalid joint order: {}.".format(group, error)) from error
