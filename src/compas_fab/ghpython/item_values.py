"""Immutable scalar, shaped, and fixed-vector item values."""

from __future__ import annotations

from enum import Enum
from typing import Generic
from typing import Optional
from typing import Tuple
from typing import TypeVar

from attrs import define

from compas_fab.ghpython.tree_errors import FixedVectorLengthError
from compas_fab.ghpython.tree_errors import InvalidFixedVectorContainerError
from compas_fab.ghpython.tree_errors import InvalidFixedVectorShapeError
from compas_fab.ghpython.tree_errors import InvalidItemShapeError
from compas_fab.ghpython.tree_errors import InvalidShapeTagError

T = TypeVar("T")


class ItemShapeKind(Enum):
    SCALAR = "scalar"
    DOMAIN_ATOMIC = "domain_atomic"
    FIXED_VECTOR = "fixed_vector"


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
class Scalar(Generic[T]):
    """Explicit broadcast-eligible scalar value."""

    value: T

    @classmethod
    def build(cls, value: T) -> "Scalar[T]":
        return cls(value)


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
            raise InvalidFixedVectorContainerError("Fixed-vector values must be an exact tuple.")
        if len(self.values) != self.shape.length:
            raise FixedVectorLengthError("Fixed-vector values must match the declared vector length.")
