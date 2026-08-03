"""Preserve Grasshopper connection semantics at Python component boundaries."""

from __future__ import annotations

from typing import Optional
from typing import Protocol
from typing import Sequence
from typing import TypeVar


class MissingGrasshopperInputError(LookupError):
    """Raised when component metadata omits a declared input."""


class GrasshopperInputParameter(Protocol):
    Name: str
    SourceCount: int
    PersistentDataCount: int


class GrasshopperInputCollection(Protocol):
    Input: Sequence[GrasshopperInputParameter]


class GrasshopperComponent(Protocol):
    Params: GrasshopperInputCollection


InputValue = TypeVar("InputValue")


def optional_connected_input(
    component: GrasshopperComponent,
    input_name: str,
    value: InputValue,
) -> Optional[InputValue]:
    """Return an optional input only when its Grasshopper parameter is wired.

    Grasshopper supplies type-specific falsey values for unconnected optional
    inputs. Connection state—not Python truthiness—is therefore the authority
    for whether an optional native argument was supplied.
    """
    for parameter in component.Params.Input:
        if parameter.Name == input_name:
            supplied = parameter.SourceCount > 0 or parameter.PersistentDataCount > 0
            return value if supplied else None
    raise MissingGrasshopperInputError("Grasshopper component has no input named {!r}.".format(input_name))
