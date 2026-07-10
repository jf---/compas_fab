"""Existing COMPAS free-motion interface compiled to native Tesseract."""

from __future__ import annotations

from typing import TYPE_CHECKING
from typing import Optional

from tesseract_robotics.planning import MotionProgram
from tesseract_robotics.planning import StateTarget

from compas_fab.backends.interfaces import PlanMotion
from compas_fab.robots import ConfigurationTarget
from compas_fab.robots import JointTrajectory
from compas_fab.robots import RobotCell
from compas_fab.robots import RobotCellState
from compas_fab.robots import Target

from ..cell_state_validation import require_matching_cell_state
from ..client import TesseractClient
from ..conversions import joint_trajectory_from_result
from ..errors import MissingTesseractStartStateError
from ..errors import UnsupportedTesseractCellStateError
from ..errors import UnsupportedTesseractTargetError
from ..errors import UnsupportedTesseractToleranceError
from ..joint_state import apply_active_joint_state
from ..joint_state import ordered_joint_positions
from ..native import TesseractPlanningRequest
from ..native import TesseractPlanningResult
from ..options import TesseractPlanOptions

INSTRUCTION_PROFILE = "FREESPACE"


class TesseractPlanMotion(PlanMotion):
    """Compile losslessly representable COMPAS targets to Tesseract."""

    if TYPE_CHECKING:
        client: TesseractClient

    def plan_motion(
        self,
        target: Target,
        start_state: RobotCellState,
        group: Optional[str] = None,
        options: Optional[dict[str, object]] = None,
    ) -> JointTrajectory:
        """Plan through the existing COMPAS interface and return its projection."""
        result = self.plan_motion_native(target, start_state, group, options)
        robot_cell: RobotCell = self.client.robot_cell
        group_name = group or robot_cell.main_group_name
        joint_types = {joint.name: joint.type for joint in robot_cell.get_configurable_joints(group_name)}
        trajectory = joint_trajectory_from_result(result, joint_types)
        trajectory.start_state = start_state.copy()
        self.client._robot_cell_state = start_state.copy()
        return trajectory

    def plan_motion_native(
        self,
        target: Target,
        start_state: RobotCellState,
        group: Optional[str] = None,
        options: Optional[dict[str, object]] = None,
    ) -> TesseractPlanningResult:
        """Plan a COMPAS target while retaining the complete native result."""
        if not isinstance(target, ConfigurationTarget):
            raise UnsupportedTesseractTargetError("Phase 1 free motion supports ConfigurationTarget, got {}.".format(type(target).__name__))
        if target.tolerance_above is not None or target.tolerance_below is not None:
            raise UnsupportedTesseractToleranceError("ConfigurationTarget tolerances cannot be represented by StateWaypoint.")

        client: TesseractClient = self.client
        robot_cell: RobotCell = client.robot_cell
        if robot_cell is None:
            raise MissingTesseractStartStateError("Call set_robot_cell before motion planning.")
        require_matching_cell_state(robot_cell, start_state, "motion planning")
        if start_state.tool_states or start_state.rigid_body_states:
            raise UnsupportedTesseractCellStateError("Phase 1 does not yet apply tools or rigid bodies to the environment clone.")
        if start_state.robot_configuration is None:
            raise MissingTesseractStartStateError("start_state.robot_configuration must be provided.")

        group_name = group or robot_cell.main_group_name
        plan_options = TesseractPlanOptions.build(options)
        client.require_planning_contact_managers()
        robot = client.environment.clone_robot()
        joint_names = robot.get_joint_names(group_name)
        start_positions = ordered_joint_positions(start_state.robot_configuration, joint_names, "start configuration")
        goal_positions = ordered_joint_positions(target.target_configuration, joint_names, "target configuration")
        apply_active_joint_state(
            robot,
            start_state.robot_configuration,
            "complete planning start configuration",
        )

        program = (
            MotionProgram(
                group_name,
                tcp_frame=robot_cell.get_end_effector_link_name(group_name),
                working_frame=robot_cell.get_base_link_name(group_name),
                profile=INSTRUCTION_PROFILE,
            )
            .set_joint_names(joint_names)
            .move_to(
                StateTarget(
                    start_positions,
                    names=joint_names,
                    profile=INSTRUCTION_PROFILE,
                )
            )
            .move_to(
                StateTarget(
                    goal_positions,
                    names=joint_names,
                    profile=INSTRUCTION_PROFILE,
                )
            )
            .to_composite_instruction(joint_names, robot_cell.get_end_effector_link_name(group_name))
        )
        request = TesseractPlanningRequest.build(
            program,
            plan_options.pipeline,
            plan_options.profiles,
            plan_options.auto_seed,
        )
        return client.runtime.execute(robot, request)
