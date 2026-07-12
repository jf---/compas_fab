"""Strict scalar and domain-atomic decoders for transport trees."""

from __future__ import annotations

from typing import Generic
from typing import Type
from typing import TypeVar
from typing import cast

from attrs import define

from compas_fab.ghpython.item_values import Scalar
from compas_fab.ghpython.tree_errors import InvalidAtomicTreeItemTypeError
from compas_fab.ghpython.tree_errors import InvalidTreeDecoderInputError
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


def _single_tree_item(
    tree: Tree[T],
    branch_error: Type[TreeContractError],
    item_error: Type[TreeContractError],
) -> TreeItem[T]:
    if type(tree) is not Tree:
        raise InvalidTreeDecoderInputError("Tree decoder requires an exact Tree value.")
    if len(tree.branches) != 1:
        raise branch_error("Tree conversion requires exactly one branch.")
    branch = tree.branches[0]
    if len(branch.items) != 1:
        raise item_error("Tree conversion requires exactly one item slot.")
    return branch.items[0]
