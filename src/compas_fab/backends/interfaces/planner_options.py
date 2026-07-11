from __future__ import annotations

from enum import Enum
from hashlib import sha256
from math import isfinite
from struct import pack
from typing import Dict
from typing import Mapping
from typing import Optional
from typing import Protocol
from typing import Tuple

from attrs import define

from .planner_errors import InvalidPlannerOptionsError


class OptionIdentityState(Enum):
    VERIFIED = "verified"
    UNVERIFIABLE = "unverifiable"


@define(frozen=True, slots=True)
class PlanMotionLegacyOptions:
    planner_id: Optional[str]
    num_planning_attempts: Optional[int]
    allowed_planning_time: Optional[float]

    @classmethod
    def build(
        cls,
        planner_id: Optional[str],
        attempts: Optional[int],
        allowed_time: Optional[float],
    ) -> "PlanMotionLegacyOptions":
        return cls(planner_id, attempts, allowed_time)

    def __attrs_post_init__(self) -> None:
        if self.planner_id is not None and (type(self.planner_id) is not str or not self.planner_id):
            raise InvalidPlannerOptionsError("planner_id must be non-empty str or None.")
        if self.num_planning_attempts is not None and (
            type(self.num_planning_attempts) is not int or self.num_planning_attempts <= 0
        ):
            raise InvalidPlannerOptionsError("num_planning_attempts must be positive int or None.")
        if self.allowed_planning_time is not None:
            invalid_time = (
                type(self.allowed_planning_time) is not float
                or not isfinite(self.allowed_planning_time)
                or self.allowed_planning_time <= 0.0
            )
            if invalid_time:
                raise InvalidPlannerOptionsError("allowed_planning_time must be finite positive float or None.")

    @property
    def connected(self) -> bool:
        return any(
            value is not None
            for value in (self.planner_id, self.num_planning_attempts, self.allowed_planning_time)
        )


def _part(value: bytes) -> bytes:
    return len(value).to_bytes(8, "big") + value


def _scalar(value: object) -> bytes:
    if type(value) is str:
        return b"s" + value.encode("utf-8")
    if type(value) is bool:
        return b"b1" if value else b"b0"
    if type(value) is int:
        return b"i" + str(value).encode("ascii")
    if type(value) is float and isfinite(value):
        return b"f" + pack("!d", value)
    if value is None:
        return b"n"
    raise InvalidPlannerOptionsError("Verified planner options require canonical scalar values.")


def _digest(values: Tuple[Tuple[str, object], ...]) -> str:
    payload = b""
    for key, value in values:
        if type(key) is not str or not key:
            raise InvalidPlannerOptionsError("Option names must be non-empty str.")
        payload += _part(key.encode("utf-8")) + _part(_scalar(value))
    return sha256(payload).hexdigest()


@define(frozen=True, slots=True)
class ResolvedPlannerOptions:
    values: Tuple[Tuple[str, object], ...]
    identity_state: OptionIdentityState
    identity_digest: Optional[str]

    @classmethod
    def verified(cls, values: Mapping[str, object]) -> "ResolvedPlannerOptions":
        retained = tuple(sorted(values.items()))
        return cls(retained, OptionIdentityState.VERIFIED, _digest(retained))

    @classmethod
    def unverifiable(cls, values: Mapping[str, object]) -> "ResolvedPlannerOptions":
        return cls(tuple(values.items()), OptionIdentityState.UNVERIFIABLE, None)

    def __attrs_post_init__(self) -> None:
        if type(self.identity_state) is not OptionIdentityState:
            raise InvalidPlannerOptionsError("identity_state must be OptionIdentityState.")
        if type(self.values) is not tuple:
            raise InvalidPlannerOptionsError("Planner option values must be an exact tuple.")
        for pair in self.values:
            if type(pair) is not tuple or len(pair) != 2:
                raise InvalidPlannerOptionsError("Each planner option must be an exact key-value tuple.")
            if type(pair[0]) is not str or not pair[0]:
                raise InvalidPlannerOptionsError("Planner option names must be non-empty str.")
        keys = tuple(pair[0] for pair in self.values)
        if len(set(keys)) != len(keys):
            raise InvalidPlannerOptionsError("Planner option names must be unique.")
        if self.identity_state is OptionIdentityState.VERIFIED:
            if self.identity_digest != _digest(self.values):
                raise InvalidPlannerOptionsError("Verified option digest is inconsistent.")
        elif self.identity_digest is not None:
            raise InvalidPlannerOptionsError("Unverifiable options cannot carry a digest.")

    @property
    def cacheable(self) -> bool:
        return self.identity_state is OptionIdentityState.VERIFIED

    def to_backend_options(self) -> Dict[str, object]:
        return dict(self.values)


class PlanMotionOptionsAdapter(Protocol):
    @classmethod
    def resolve(
        cls,
        legacy: PlanMotionLegacyOptions,
        native: Optional[Mapping[str, object]],
    ) -> ResolvedPlannerOptions: ...
