"""Orthogonal Grasshopper port meaning and atomic value wrappers."""

from __future__ import annotations

from enum import Enum
from typing import Generic
from typing import Optional
from typing import Tuple
from typing import Type
from typing import TypeVar
from typing import cast

from attrs import define

from compas_fab.ghpython.tree_errors import FixedVectorLengthError
from compas_fab.ghpython.tree_errors import InvalidAtomicTreeItemTypeError
from compas_fab.ghpython.tree_errors import InvalidFixedVectorShapeError
from compas_fab.ghpython.tree_errors import InvalidItemShapeError
from compas_fab.ghpython.tree_errors import InvalidPortSemanticsError
from compas_fab.ghpython.tree_errors import InvalidShapeTagError
from compas_fab.ghpython.tree_errors import NullTreeAtomicError
from compas_fab.ghpython.tree_errors import NullTreeScalarError
from compas_fab.ghpython.tree_errors import TreeAtomicBranchCardinalityError
from compas_fab.ghpython.tree_errors import TreeAtomicItemCardinalityError
from compas_fab.ghpython.tree_errors import TreeContractError
from compas_fab.ghpython.tree_errors import TreeScalarBranchCardinalityError
from compas_fab.ghpython.tree_errors import TreeScalarItemCardinalityError
from compas_fab.ghpython.tree_values import Tree
from compas_fab.ghpython.tree_values import TreeItem

T = TypeVar("T")
SourceT = TypeVar("SourceT")
AtomicT = TypeVar("AtomicT")


class TopologyRole(Enum):
    ATOMIC = "atomic"
    TREE = "tree"


class BranchSemantics(Enum):
    ELEMENTWISE = "elementwise"
    ORDERED_SEQUENCE = "ordered_sequence"
    BATCH = "batch"


class ItemShapeKind(Enum):
    SCALAR = "scalar"
    DOMAIN_ATOMIC = "domain_atomic"
    FIXED_VECTOR = "fixed_vector"


class ItemValidationPolicy(Enum):
    VALIDATE_INDEPENDENT = "validate_independent"


class SequenceReductionPolicy(Enum):
    REQUIRE_ALL_VALID = "require_all_valid"


class BatchPublicationPolicy(Enum):
    PUBLISH_INDEPENDENT = "publish_independent"
    FAIL_BATCH = "fail_batch"


@define(frozen=True, slots=True)
class ShapeTag:
    """Stable domain label distinguishing non-interchangeable item shapes."""

    value: str

    @classmethod
    def build(cls, value: str) -> "ShapeTag":
        return cls(value)

    def __attrs_post_init__(self) -> None:
        if type(self.value) is not str or not self.value or self.value != self.value.strip():
            raise InvalidShapeTagError("Shape tag must be non-empty text without surrounding whitespace.")


@define(frozen=True, slots=True)
class ItemShape:
    """Scalar, domain-atomic, or fixed-vector item meaning."""

    kind: ItemShapeKind
    tag: Optional[ShapeTag]
    length: Optional[int]

    @classmethod
    def scalar(cls) -> "ItemShape":
        return cls(ItemShapeKind.SCALAR, None, None)

    @classmethod
    def domain_atomic(cls, tag: ShapeTag) -> "ItemShape":
        return cls(ItemShapeKind.DOMAIN_ATOMIC, tag, None)

    @classmethod
    def fixed_vector(cls, tag: ShapeTag, length: int) -> "ItemShape":
        return cls(ItemShapeKind.FIXED_VECTOR, tag, length)

    def __attrs_post_init__(self) -> None:
        if type(self.kind) is not ItemShapeKind:
            raise InvalidItemShapeError("Item shape kind must be an exact declared value.")
        if self.kind is ItemShapeKind.SCALAR:
            valid = self.tag is None and self.length is None
        elif self.kind is ItemShapeKind.DOMAIN_ATOMIC:
            valid = type(self.tag) is ShapeTag and self.length is None
        else:
            valid = type(self.tag) is ShapeTag and type(self.length) is int and self.length > 0
        if not valid:
            raise InvalidItemShapeError("Item shape tag and length must match its declared kind.")


@define(frozen=True, slots=True)
class PortSemantics:
    """Three independent axes describing a Grasshopper port contract."""

    topology_role: TopologyRole
    branch_semantics: BranchSemantics
    item_shape: ItemShape

    @classmethod
    def build(
        cls,
        topology_role: TopologyRole,
        branch_semantics: BranchSemantics,
        item_shape: ItemShape,
    ) -> "PortSemantics":
        return cls(topology_role, branch_semantics, item_shape)

    def __attrs_post_init__(self) -> None:
        if (
            type(self.topology_role) is not TopologyRole
            or type(self.branch_semantics) is not BranchSemantics
            or type(self.item_shape) is not ItemShape
        ):
            raise InvalidPortSemanticsError("Port semantics require exact topology, branch, and item-shape values.")


@define(frozen=True, slots=True)
class Scalar(Generic[T]):
    """Explicit broadcast-eligible scalar value."""

    value: T

    @classmethod
    def build(cls, value: T) -> "Scalar[T]":
        return cls(value)


class ScalarFromTree(Generic[T]):
    """Decode a one-slot transport tree without authorizing broadcast.

    Task 3 ``MatchPolicy`` is the sole authority that may broadcast the
    resulting explicit ``Scalar`` over another input's coordinates.
    """

    @classmethod
    def build(cls, tree: Tree[T]) -> Scalar[T]:
        item = _single_tree_item(tree, TreeScalarBranchCardinalityError, TreeScalarItemCardinalityError)
        if item.is_null:
            raise NullTreeScalarError("Scalar conversion requires one non-null item.")
        return Scalar.build(cast(T, item.item))


@define(frozen=True, slots=True)
class AtomicFromTree(Generic[T]):
    """One exact domain object extracted without scalar semantics."""

    value: T
    expected_type: Type[T]

    @staticmethod
    def build(tree: Tree[SourceT], expected_type: Type[AtomicT]) -> "AtomicFromTree[AtomicT]":
        item = _single_tree_item(tree, TreeAtomicBranchCardinalityError, TreeAtomicItemCardinalityError)
        if item.is_null:
            raise NullTreeAtomicError("Atomic conversion requires one non-null item.")
        value = item.item
        if not isinstance(expected_type, type) or not isinstance(value, expected_type):
            raise InvalidAtomicTreeItemTypeError("Atomic tree item does not match its expected domain type.")
        return AtomicFromTree(value, expected_type)

    def __attrs_post_init__(self) -> None:
        if not isinstance(self.expected_type, type) or not isinstance(self.value, self.expected_type):
            raise InvalidAtomicTreeItemTypeError("Atomic value does not match its recorded expected domain type.")


@define(frozen=True, slots=True)
class FixedVector(Generic[T]):
    """Atomic ordered values with an exact tagged length."""

    shape: ItemShape
    values: Tuple[T, ...]

    @classmethod
    def build(cls, shape: ItemShape, values: Tuple[T, ...]) -> "FixedVector[T]":
        return cls(shape=shape, values=values)

    def __attrs_post_init__(self) -> None:
        if type(self.shape) is not ItemShape or self.shape.kind is not ItemShapeKind.FIXED_VECTOR:
            raise InvalidFixedVectorShapeError("Fixed vector requires a fixed-vector item shape.")
        if type(self.values) is not tuple:
            raise FixedVectorLengthError("Fixed-vector values must be an exact tuple.")
        if len(self.values) != self.shape.length:
            raise FixedVectorLengthError("Fixed-vector values must match the declared vector length.")


def _single_tree_item(
    tree: Tree[T],
    branch_error: Type[TreeContractError],
    item_error: Type[TreeContractError],
) -> TreeItem[T]:
    if type(tree) is not Tree:
        raise branch_error("Tree conversion requires an exact Tree value.")
    if len(tree.branches) != 1:
        raise branch_error("Tree conversion requires exactly one branch.")
    branch = tree.branches[0]
    if len(branch.items) != 1:
        raise item_error("Tree conversion requires exactly one item slot.")
    return branch.items[0]
