"""Shared typed inputs for joint and state target series."""

from __future__ import annotations

from hashlib import sha256
from typing import Union

from attrs import define
from tesseract_robotics.planning import MoveType

from compas_fab.ghpython.group_shape import GroupFixedVector
from compas_fab.ghpython.group_shape import GroupShape
from compas_fab.ghpython.group_shape import GroupVectorQuantity
from compas_fab.ghpython.item_values import ItemShape
from compas_fab.ghpython.item_values import Scalar
from compas_fab.ghpython.item_values import ShapeTag
from compas_fab.ghpython.port_semantics import BranchSemantics
from compas_fab.ghpython.port_semantics import PortSemantics
from compas_fab.ghpython.port_semantics import TopologyRole
from compas_fab.ghpython.tree_identity import TEXT_CODEC
from compas_fab.ghpython.tree_identity import ExactItemCodec
from compas_fab.ghpython.tree_identity import SourceTreeIdentity
from compas_fab.ghpython.tree_matching import MatchInput
from compas_fab.ghpython.tree_values import Tree

MoveSeriesInput = Union[Tree[MoveType], Scalar[MoveType]]
ProfileSeriesInput = Union[Tree[str], Scalar[str]]
MOVE_TYPE_CODEC = ExactItemCodec.build("tesseract-move-type/v1", MoveType, lambda value: value.name.encode("ascii"))
GROUP_FIXED_VECTOR_CODEC = ExactItemCodec.build(
    "tesseract-group-fixed-vector/v1",
    GroupFixedVector,
    lambda value: value.canonical_bytes(),
)
SCALAR_TREE_SEMANTICS = PortSemantics.build(TopologyRole.TREE, BranchSemantics.ELEMENTWISE, ItemShape.scalar())


class VectorTargetInputError(ValueError):
    """Base failure for group-vector target inputs."""


class InvalidVectorTargetSeriesParametersError(VectorTargetInputError):
    """Raised when move/profile inputs lack exact declared roles."""


class IncompatibleGroupVectorTreeError(VectorTargetInputError):
    """Raised before native work when vector group, quantity, or length differs."""


@define(frozen=True, slots=True)
class VectorTargetSeriesParameters:
    """Exact tree-or-explicit-scalar move/profile inputs."""

    move_types: MoveSeriesInput
    profiles: ProfileSeriesInput

    @classmethod
    def build(cls, move_types: MoveSeriesInput, profiles: ProfileSeriesInput) -> "VectorTargetSeriesParameters":
        return cls(move_types, profiles)

    def __attrs_post_init__(self) -> None:
        if type(self.move_types) is Scalar:
            if type(self.move_types.value) is not MoveType:
                raise InvalidVectorTargetSeriesParametersError("Move scalar must contain exact MoveType.")
        elif type(self.move_types) is not Tree:
            raise InvalidVectorTargetSeriesParametersError("Move input must be exact tree or explicit Scalar.")
        if type(self.profiles) is Scalar:
            if type(self.profiles.value) is not str or not self.profiles.value.strip():
                raise InvalidVectorTargetSeriesParametersError("Profile scalar must contain exact non-empty text.")
        elif type(self.profiles) is not Tree:
            raise InvalidVectorTargetSeriesParametersError("Profile input must be exact tree or explicit Scalar.")


def vector_semantics(group_shape: GroupShape, quantity: GroupVectorQuantity) -> PortSemantics:
    """Declare a complete group+quantity fixed-vector item shape."""
    if type(group_shape) is not GroupShape or type(quantity) is not GroupVectorQuantity:
        raise IncompatibleGroupVectorTreeError("Vector semantics require exact group and quantity.")
    digest = sha256(group_shape.canonical_bytes()).hexdigest()
    shape = ItemShape.fixed_vector(
        ShapeTag.build("compas_fab.tesseract.{}.{}".format(quantity.value, digest)),
        group_shape.dof.value,
    )
    return PortSemantics.build(TopologyRole.TREE, BranchSemantics.ELEMENTWISE, shape)


def validate_vector_tree(
    tree: Tree[GroupFixedVector],
    group_shape: GroupShape,
    quantity: GroupVectorQuantity,
) -> None:
    """Fail before native work on any non-null incompatible atomic vector."""
    if type(tree) is not Tree or type(group_shape) is not GroupShape or type(quantity) is not GroupVectorQuantity:
        raise IncompatibleGroupVectorTreeError("Vector tree validation requires exact typed inputs.")
    for branch in tree.branches:
        for item in branch.items:
            if item.is_null:
                continue
            vector = item.item
            if (
                type(vector) is not GroupFixedVector
                or vector.group_shape != group_shape
                or vector.quantity is not quantity
                or vector.vector.shape != vector_semantics(group_shape, quantity).item_shape
            ):
                raise IncompatibleGroupVectorTreeError("Every vector item must match exact group, joint order, DOF, and quantity.")


def vector_identity(
    tree: Tree[GroupFixedVector],
    group_shape: GroupShape,
    quantity: GroupVectorQuantity,
) -> SourceTreeIdentity:
    validate_vector_tree(tree, group_shape, quantity)
    return SourceTreeIdentity.build(tree, GROUP_FIXED_VECTOR_CODEC, vector_semantics(group_shape, quantity))


def move_identity(value: MoveSeriesInput) -> Union[SourceTreeIdentity, None]:
    return SourceTreeIdentity.build(value, MOVE_TYPE_CODEC, SCALAR_TREE_SEMANTICS) if type(value) is Tree else None


def profile_identity(value: ProfileSeriesInput) -> Union[SourceTreeIdentity, None]:
    return SourceTreeIdentity.build(value, TEXT_CODEC, SCALAR_TREE_SEMANTICS) if type(value) is Tree else None


def match_input(name: str, value: object) -> MatchInput:
    if type(value) is Tree:
        return MatchInput.tree(name, value)
    if type(value) is Scalar:
        return MatchInput.scalar(name, value)
    raise InvalidVectorTargetSeriesParametersError("Matched vector target input must be exact tree or Scalar.")
