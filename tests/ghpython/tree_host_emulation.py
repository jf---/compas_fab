"""Pending-Rhino transport hypotheses, never host evidence."""

from __future__ import annotations

from enum import Enum
from typing import Tuple

from attrs import define


class PendingRhinoEmulationError(ValueError):
    """Base failure for pending-Rhino transport hypotheses."""


class InvalidEmulatedParameterError(PendingRhinoEmulationError):
    """Raised when an emulated parameter schedule is malformed."""


class InvalidEmulatedInvocationError(PendingRhinoEmulationError):
    """Raised when an emulated invocation is malformed."""


class EmulatedInvocationCountError(PendingRhinoEmulationError):
    """Raised when emulated parameter schedules have unequal lengths."""


class EmulatedAccess(Enum):
    """Pending-Rhino access hypothesis, never Rhino evidence."""

    ITEM = "item"
    LIST = "list"
    TREE = "tree"


@define(frozen=True, slots=True)
class EmulatedParameter:
    """One pre-scheduled pending-Rhino transport parameter."""

    name: str
    access: EmulatedAccess
    invocation_values: Tuple[object, ...]

    @classmethod
    def item(cls, name: str, invocation_items: Tuple[object, ...]) -> "EmulatedParameter":
        """Build a pending-Rhino item-access transport schedule."""
        return cls(name, EmulatedAccess.ITEM, invocation_items)

    @classmethod
    def list(cls, name: str, invocation_lists: Tuple[Tuple[object, ...], ...]) -> "EmulatedParameter":
        """Build a pending-Rhino list-access transport schedule."""
        return cls(name, EmulatedAccess.LIST, invocation_lists)

    @classmethod
    def tree(cls, name: str, tree: object) -> "EmulatedParameter":
        """Build a single pending-Rhino tree-access transport schedule."""
        return cls(name, EmulatedAccess.TREE, (tree,))

    def __attrs_post_init__(self) -> None:
        if type(self.name) is not str or not self.name or self.name != self.name.strip():
            raise InvalidEmulatedParameterError("Emulated parameter name must be canonical non-empty text.")
        if type(self.access) is not EmulatedAccess or type(self.invocation_values) is not tuple or not self.invocation_values:
            raise InvalidEmulatedParameterError("Emulated parameter requires exact access and a non-empty invocation tuple.")
        if self.access is EmulatedAccess.TREE and len(self.invocation_values) != 1:
            raise InvalidEmulatedParameterError("Pending-Rhino tree access must retain one full-tree invocation value.")
        if self.access is EmulatedAccess.LIST and any(type(value) is not tuple for value in self.invocation_values):
            raise InvalidEmulatedParameterError("Pending-Rhino list access requires one exact item tuple per invocation.")


def _valid_emulated_parameter(parameter: object) -> bool:
    if type(parameter) is not EmulatedParameter:
        return False
    if type(parameter.name) is not str or not parameter.name or parameter.name != parameter.name.strip():
        return False
    if type(parameter.access) is not EmulatedAccess or type(parameter.invocation_values) is not tuple or not parameter.invocation_values:
        return False
    if parameter.access is EmulatedAccess.TREE and len(parameter.invocation_values) != 1:
        return False
    return parameter.access is not EmulatedAccess.LIST or all(type(value) is tuple for value in parameter.invocation_values)


@define(frozen=True, slots=True)
class EmulatedInvocation:
    """One pending-Rhino invocation containing named transported values."""

    arguments: Tuple[Tuple[str, object], ...]

    @classmethod
    def build(cls, arguments: Tuple[Tuple[str, object], ...]) -> "EmulatedInvocation":
        """Build one exact pending-Rhino invocation record."""
        return cls(arguments)

    def __attrs_post_init__(self) -> None:
        if type(self.arguments) is not tuple or not self.arguments:
            raise InvalidEmulatedInvocationError("Emulated invocation requires a non-empty exact argument tuple.")
        names = tuple(argument[0] for argument in self.arguments if type(argument) is tuple and len(argument) == 2)
        if len(names) != len(self.arguments) or any(type(name) is not str or not name for name in names) or len(set(names)) != len(names):
            raise InvalidEmulatedInvocationError("Emulated invocation requires unique named argument pairs.")

    def value(self, name: str) -> object:
        """Return one named pending-Rhino transported value."""
        for argument_name, value in self.arguments:
            if argument_name == name:
                return value
        raise InvalidEmulatedInvocationError("Emulated invocation does not contain the requested argument.")


@define(frozen=True, slots=True)
class PendingRhinoSolveModel:
    """Record pre-scheduled transport without emulating domain matching."""

    parameters: Tuple[EmulatedParameter, ...]

    @classmethod
    def build(cls, parameters: Tuple[EmulatedParameter, ...]) -> "PendingRhinoSolveModel":
        """Build a pending-Rhino transport-only hypothesis."""
        return cls(parameters)

    def __attrs_post_init__(self) -> None:
        if type(self.parameters) is not tuple or not self.parameters or any(not _valid_emulated_parameter(parameter) for parameter in self.parameters):
            raise InvalidEmulatedParameterError("Pending-Rhino solve model requires exact emulated parameters.")
        names = tuple(parameter.name for parameter in self.parameters)
        if len(set(names)) != len(names):
            raise InvalidEmulatedParameterError("Pending-Rhino solve model parameter names must be unique.")
        counts = tuple(len(parameter.invocation_values) for parameter in self.parameters)
        if any(count != counts[0] for count in counts):
            raise EmulatedInvocationCountError("Pre-scheduled pending-Rhino invocation counts must agree without matching.")

    def invocations(self) -> Tuple[EmulatedInvocation, ...]:
        """Return pending-Rhino transport invocations without flattening or matching."""
        count = len(self.parameters[0].invocation_values)
        return tuple(EmulatedInvocation.build(tuple((parameter.name, parameter.invocation_values[index]) for parameter in self.parameters)) for index in range(count))
