"""Exact native planning-group shape and ordered joint authority."""

from __future__ import annotations

from enum import Enum
from hashlib import sha256
from math import isfinite
from typing import Optional
from typing import Tuple
from typing import Union

from attrs import define
from attrs import field
from tesseract_robotics.planning import Robot

from compas_fab.ghpython.item_values import FixedVector
from compas_fab.ghpython.item_values import ItemShape
from compas_fab.ghpython.item_values import ShapeTag

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


class InvalidGroupFixedVectorError(GroupShapeError):
    """Raised when a quantity vector disagrees with its complete group shape."""


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

    def canonical_bytes(self) -> bytes:
        """Encode complete group and joint order without runtime objects."""
        values = (self.group_id.value,) + tuple(joint.value for joint in self.ordered_joint_ids)
        return b"".join(len(value.encode("utf-8")).to_bytes(8, "big") + value.encode("utf-8") for value in values)


class GroupVectorQuantity(Enum):
    POSITION = "position"
    NAME = "name"
    VELOCITY = "velocity"
    ACCELERATION = "acceleration"


GroupVectorValue = Union[float, str]


@define(frozen=True, slots=True)
class GroupFixedVector:
    """One atomic quantity vector tagged by complete native group shape."""

    group_shape: GroupShape
    quantity: GroupVectorQuantity
    vector: FixedVector[GroupVectorValue]

    @classmethod
    def positions(cls, group_shape: GroupShape, values: Tuple[float, ...]) -> "GroupFixedVector":
        return cls._numeric(group_shape, GroupVectorQuantity.POSITION, values)

    @classmethod
    def velocities(cls, group_shape: GroupShape, values: Tuple[float, ...]) -> "GroupFixedVector":
        return cls._numeric(group_shape, GroupVectorQuantity.VELOCITY, values)

    @classmethod
    def accelerations(cls, group_shape: GroupShape, values: Tuple[float, ...]) -> "GroupFixedVector":
        return cls._numeric(group_shape, GroupVectorQuantity.ACCELERATION, values)

    @classmethod
    def names(cls, group_shape: GroupShape, values: Tuple[str, ...]) -> "GroupFixedVector":
        if type(group_shape) is not GroupShape or type(values) is not tuple:
            raise InvalidGroupFixedVectorError("Group name vector requires an exact GroupShape and tuple.")
        expected = tuple(value.value for value in group_shape.ordered_joint_ids)
        if values != expected:
            raise InvalidGroupFixedVectorError("Group name vector must equal exact native joint order.")
        return cls(group_shape, GroupVectorQuantity.NAME, FixedVector.build(_vector_shape(group_shape, GroupVectorQuantity.NAME), values))

    @classmethod
    def _numeric(
        cls,
        group_shape: GroupShape,
        quantity: GroupVectorQuantity,
        values: Tuple[float, ...],
    ) -> "GroupFixedVector":
        if (
            type(group_shape) is not GroupShape
            or type(values) is not tuple
            or any(type(value) is not float or not isfinite(value) for value in values)
        ):
            raise InvalidGroupFixedVectorError("Group numeric vector requires exact finite float values.")
        if len(values) != group_shape.dof.value:
            raise InvalidGroupFixedVectorError("Group numeric vector length must equal exact group DOF.")
        return cls(group_shape, quantity, FixedVector.build(_vector_shape(group_shape, quantity), values))

    def __attrs_post_init__(self) -> None:
        valid = (
            type(self.group_shape) is GroupShape
            and type(self.quantity) is GroupVectorQuantity
            and type(self.vector) is FixedVector
            and self.vector.shape == _vector_shape(self.group_shape, self.quantity)
            and len(self.vector.values) == self.group_shape.dof.value
        )
        if self.quantity is GroupVectorQuantity.NAME:
            valid = valid and self.vector.values == tuple(value.value for value in self.group_shape.ordered_joint_ids)
        else:
            valid = valid and all(type(value) is float and isfinite(value) for value in self.vector.values)
        if not valid:
            raise InvalidGroupFixedVectorError("Group fixed vector must retain exact shape, quantity, and values.")

    def canonical_bytes(self) -> bytes:
        """Encode group, quantity, and complete ordered values."""
        values = b"".join(
            len(repr(value).encode("utf-8")).to_bytes(8, "big") + repr(value).encode("utf-8")
            for value in self.vector.values
        )
        return self.group_shape.canonical_bytes() + self.quantity.value.encode("ascii") + values


def _vector_shape(group_shape: GroupShape, quantity: GroupVectorQuantity) -> ItemShape:
    digest = sha256(group_shape.canonical_bytes()).hexdigest()
    tag = ShapeTag.build("compas_fab.tesseract.{}.{}".format(quantity.value, digest))
    return ItemShape.fixed_vector(tag, group_shape.dof.value)


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
