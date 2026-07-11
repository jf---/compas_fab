from enum import Enum


class PlannerOperation(Enum):
    INVERSE_KINEMATICS = "inverse_kinematics"
    PLAN_MOTION = "plan_motion"
    PLAN_CARTESIAN_MOTION = "plan_cartesian_motion"
    CHECK_COLLISION = "check_collision"
