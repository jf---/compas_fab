from __future__ import annotations

from enum import Enum
from typing import Generic
from typing import Optional
from typing import TypeVar

from compas_fab.ghpython.component_identity import ComponentInputIdentity
from compas_fab.ghpython.component_identity import InvalidComponentIdentityError

T = TypeVar("T")


class CurrentOutputTransitionError(RuntimeError):
    pass


class InvalidCurrentOutputTransitionError(CurrentOutputTransitionError):
    pass


class SupersededComponentOutputError(CurrentOutputTransitionError):
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

    @staticmethod
    def _validated_identity(identity: ComponentInputIdentity) -> ComponentInputIdentity:
        if type(identity) is not ComponentInputIdentity:
            raise InvalidCurrentOutputTransitionError("Output transitions require an exact component input identity.")
        try:
            identity.__attrs_post_init__()
        except (AttributeError, InvalidComponentIdentityError) as error:
            raise InvalidCurrentOutputTransitionError("Output transitions require a valid component input identity.") from error
        return identity

    def observe(self, identity: ComponentInputIdentity, compute: bool) -> ComputeDecision:
        identity = self._validated_identity(identity)
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
        identity = self._validated_identity(identity)
        if self._identity is None:
            raise InvalidCurrentOutputTransitionError("Cannot publish output before observing its identity.")
        if identity != self._identity:
            raise SupersededComponentOutputError("Cannot publish superseded output.")
        self._value = value
        self._has_value = True

    def fail(self, identity: ComponentInputIdentity) -> None:
        identity = self._validated_identity(identity)
        if self._identity is None:
            raise InvalidCurrentOutputTransitionError("Cannot fail output before observing its identity.")
        if identity != self._identity:
            raise SupersededComponentOutputError("Cannot fail superseded output.")
        self._value = None
        self._has_value = False

    def current(self, identity: ComponentInputIdentity) -> Optional[T]:
        identity = self._validated_identity(identity)
        if identity == self._identity and self._has_value:
            return self._value
        return None

    def clear(self) -> None:
        self._identity = None
        self._value = None
        self._has_value = False
        self._compute_high = False
