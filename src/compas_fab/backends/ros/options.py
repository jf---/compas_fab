from math import isfinite
from typing import Dict
from typing import Mapping
from typing import Optional
from typing import Tuple
from typing import cast

from attrs import define

from compas_fab.backends.interfaces.planner_errors import ConflictingPlannerOptionsError
from compas_fab.backends.interfaces.planner_options import PlanMotionLegacyOptions
from compas_fab.backends.interfaces.planner_options import ResolvedPlannerOptions
from compas_fab.backends.ros.exceptions import InvalidMoveItPlanMotionOptionsError


@define(frozen=True, slots=True)
class MoveItPlanMotionOptions:
    planner_id: Optional[str] = None
    num_planning_attempts: Optional[int] = None
    allowed_planning_time: Optional[float] = None
    max_velocity_scaling_factor: Optional[float] = None
    max_acceleration_scaling_factor: Optional[float] = None
    base_link: Optional[str] = None
    path_constraints: object = None
    trajectory_constraints: object = None

    NAMES = frozenset(
        (
            "planner_id",
            "num_planning_attempts",
            "allowed_planning_time",
            "max_velocity_scaling_factor",
            "max_acceleration_scaling_factor",
            "base_link",
            "path_constraints",
            "trajectory_constraints",
        )
    )

    @classmethod
    def build(cls, values: Mapping[str, object]) -> "MoveItPlanMotionOptions":
        unknown = set(values) - cls.NAMES
        if unknown:
            raise InvalidMoveItPlanMotionOptionsError("Unknown MoveIt Plan Motion option.")
        cls._validate(values)
        return cls(
            planner_id=cast(Optional[str], values.get("planner_id")),
            num_planning_attempts=cast(Optional[int], values.get("num_planning_attempts")),
            allowed_planning_time=cast(Optional[float], values.get("allowed_planning_time")),
            max_velocity_scaling_factor=cast(Optional[float], values.get("max_velocity_scaling_factor")),
            max_acceleration_scaling_factor=cast(Optional[float], values.get("max_acceleration_scaling_factor")),
            base_link=cast(Optional[str], values.get("base_link")),
            path_constraints=values.get("path_constraints"),
            trajectory_constraints=values.get("trajectory_constraints"),
        )

    @classmethod
    def resolve(
        cls,
        legacy: PlanMotionLegacyOptions,
        native: Optional[Mapping[str, object]],
    ) -> ResolvedPlannerOptions:
        if native is not None and legacy.connected:
            raise ConflictingPlannerOptionsError("Native options conflict with connected legacy ports.")
        values = (
            dict(native)
            if native is not None
            else {
                "planner_id": legacy.planner_id,
                "num_planning_attempts": legacy.num_planning_attempts,
                "allowed_planning_time": legacy.allowed_planning_time,
            }
        )
        return cls.build({key: value for key, value in values.items() if value is not None}).as_resolved()

    def __attrs_post_init__(self) -> None:
        self._validate(self.to_backend_values())

    def to_backend_values(self) -> Dict[str, object]:
        pairs: Tuple[Tuple[str, object], ...] = (
            ("planner_id", self.planner_id),
            ("num_planning_attempts", self.num_planning_attempts),
            ("allowed_planning_time", self.allowed_planning_time),
            ("max_velocity_scaling_factor", self.max_velocity_scaling_factor),
            ("max_acceleration_scaling_factor", self.max_acceleration_scaling_factor),
            ("base_link", self.base_link),
            ("path_constraints", self.path_constraints),
            ("trajectory_constraints", self.trajectory_constraints),
        )
        return {name: value for name, value in pairs if value is not None}

    def as_resolved(self) -> ResolvedPlannerOptions:
        values = self.to_backend_values()
        if self.path_constraints is not None or self.trajectory_constraints is not None:
            return ResolvedPlannerOptions.unverifiable(values)
        return ResolvedPlannerOptions.verified(values)

    @classmethod
    def _validate(cls, values: Mapping[str, object]) -> None:
        planner_id = values.get("planner_id")
        base_link = values.get("base_link")
        attempts = values.get("num_planning_attempts")
        allowed_time = values.get("allowed_planning_time")
        if planner_id is not None and (type(planner_id) is not str or not planner_id):
            raise InvalidMoveItPlanMotionOptionsError("planner_id must be non-empty str.")
        if base_link is not None and (type(base_link) is not str or not base_link):
            raise InvalidMoveItPlanMotionOptionsError("base_link must be non-empty str.")
        if attempts is not None and (type(attempts) is not int or attempts <= 0):
            raise InvalidMoveItPlanMotionOptionsError("num_planning_attempts must be positive int.")
        if allowed_time is not None and (
            type(allowed_time) is not float or not isfinite(allowed_time) or allowed_time <= 0.0
        ):
            raise InvalidMoveItPlanMotionOptionsError("allowed_planning_time must be finite positive float.")
        for name in ("max_velocity_scaling_factor", "max_acceleration_scaling_factor"):
            value = values.get(name)
            if value is not None and (
                type(value) is not float or not isfinite(value) or not 0.0 <= value <= 1.0
            ):
                raise InvalidMoveItPlanMotionOptionsError(
                    "{} must be a finite float in [0, 1].".format(name)
                )
