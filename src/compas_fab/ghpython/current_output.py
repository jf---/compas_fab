from __future__ import annotations

from enum import Enum
from typing import Generic
from typing import Optional
from typing import TypeVar

from compas_fab.ghpython.component_identity import ComponentInputIdentity

T = TypeVar("T")


class SupersededComponentOutputError(RuntimeError):
    pass


class ComputeDecision(Enum):
    IDLE = "idle"
    EXECUTE = "execute"
    CURRENT = "current"
    CLEARED = "cleared"


class CurrentOutputState(Generic[T]):
    def __init__(self) -> None:
        self._identity: Optional[ComponentInputIdentity] = None
        self._value: Optional[T] = None
        self._has_value = False
        self._compute_high = False

    @classmethod
    def build(cls) -> "CurrentOutputState[T]":
        return cls()

    def observe(self, identity: ComponentInputIdentity, compute: bool) -> ComputeDecision:
        if type(compute) is not bool:
            raise TypeError("compute must be bool.")
        if self._identity != identity:
            changed = self._identity is not None
            self._identity = identity
            self._value = None
            self._has_value = False
            self._compute_high = compute
            return ComputeDecision.CLEARED if changed else (ComputeDecision.EXECUTE if compute else ComputeDecision.IDLE)
        rising = compute and not self._compute_high
        self._compute_high = compute
        if rising:
            return ComputeDecision.EXECUTE
        return ComputeDecision.CURRENT if self._has_value else ComputeDecision.IDLE

    def publish(self, identity: ComponentInputIdentity, value: T) -> None:
        if identity != self._identity:
            raise SupersededComponentOutputError("Cannot publish superseded output.")
        self._value = value
        self._has_value = True

    def fail(self, identity: ComponentInputIdentity) -> None:
        if identity != self._identity:
            raise SupersededComponentOutputError("Cannot fail superseded output.")
        self._value = None
        self._has_value = False

    def current(self, identity: ComponentInputIdentity) -> Optional[T]:
        if identity == self._identity and self._has_value:
            return self._value
        return None

    def clear(self) -> None:
        self._identity = None
        self._value = None
        self._has_value = False
        self._compute_high = False
