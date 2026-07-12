"""Explicit absent-or-present Grasshopper tree input."""

from __future__ import annotations

from typing import Generic
from typing import Optional
from typing import TypeVar

from attrs import define
from attrs import field

from compas_fab.ghpython.tree_values import Tree

T = TypeVar("T")
_OPTIONAL_INPUT_FACTORY_TOKEN = object()


class InvalidOptionalTreeInputError(ValueError):
    """Raised when optional-input presence and retained topology disagree."""


class AbsentOptionalTreeInputError(InvalidOptionalTreeInputError):
    """Raised when an absent optional input is accessed as present."""


@define(frozen=True, slots=True)
class OptionalTreeInput(Generic[T]):
    """Immutable tagged union separating host absence from present topology."""

    tree: Optional[Tree[T]]
    _present: bool
    _factory_token: Optional[object] = field(default=None, eq=False, repr=False)

    @classmethod
    def absent(cls) -> "OptionalTreeInput[T]":
        return cls(None, False, _OPTIONAL_INPUT_FACTORY_TOKEN)

    @classmethod
    def present(cls, tree: Tree[T]) -> "OptionalTreeInput[T]":
        if type(tree) is not Tree:
            raise InvalidOptionalTreeInputError("Present optional input requires one exact Tree value.")
        return cls(tree, True, _OPTIONAL_INPUT_FACTORY_TOKEN)

    def __attrs_post_init__(self) -> None:
        valid = (
            type(self._present) is bool
            and self._factory_token is _OPTIONAL_INPUT_FACTORY_TOKEN
            and ((self._present and type(self.tree) is Tree) or (not self._present and self.tree is None))
        )
        if not valid:
            raise InvalidOptionalTreeInputError("Optional input must be factory-built as exact absence or one retained tree.")

    @property
    def is_absent(self) -> bool:
        return not self._present

    @property
    def is_present(self) -> bool:
        return self._present

    def require_present(self) -> Tree[T]:
        if self.tree is None:
            raise AbsentOptionalTreeInputError("Optional tree input is absent.")
        return self.tree
