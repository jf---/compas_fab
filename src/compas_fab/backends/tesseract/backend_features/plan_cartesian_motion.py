"""Coordinated Cartesian planning compiled to native Tesseract Descartes.

Maps a COMPAS `FrameWaypoints` sequence to native `WorkingFrameCartesianTarget`
waypoints and plans them through `DescartesFPipeline`. When the planning group is
a coordinated robot + external-axis group, the group's configured default solver
is the native ROP/REP coupled solver, so the Descartes ladder graph enumerates
coordinated whole-system inverse-kinematics at every waypoint and the returned
trajectory moves the external axis together with the arm. The solver is a fact of
the loaded cell, never a caller option.
"""

from __future__ import annotations

from typing import TYPE_CHECKING
from typing import Optional
from typing import cast

from compas.geometry import Frame  # type: ignore[import-untyped]
from tesseract_robotics.planning import MoveType

from compas_fab.backends.interfaces import PlanCartesianMotion
from compas_fab.robots import FrameWaypoints
from compas_fab.robots import JointTrajectory
from compas_fab.robots import RobotCellState
from compas_fab.robots import Waypoints

from ..cell_state_validation import require_matching_cell_state
from ..client import TesseractClient
from ..conversions import joint_trajectory_from_result
from ..descartes_profiles import build_descartes_profiles
from ..errors import MissingTesseractStartStateError
from ..errors import UnknownTesseractOptionError
from ..errors import UnsupportedTesseractCellStateError
from ..errors import UnsupportedTesseractTargetError
from ..errors import UnsupportedTesseractToleranceError
from ..frames import meters_per_user_unit
from ..frames import robot_frame_from_world
from ..frames import world_meters_frame
from ..joint_state import apply_active_joint_state
from ..native import TesseractPlanningRequest
from ..native import TesseractPlanningResult
from ..native_pose import WorkingFrameUserUnits
from ..native_pose import pose_from_working_frame
from ..native_program_builder import build_motion_program
from ..native_targets import cartesian_target_from_native

# Descartes ladder-graph pipeline: sample per-waypoint inverse kinematics (the
# coupled ROP/REP solver when the group is coordinated) and search the minimum
# cost joint path. Not CartesianPipeline: that refines with TrajOpt and its
# profile factory cannot carry the coupled solver selection.
CARTESIAN_PIPELINE = "DescartesFPipeline"
# Registered Descartes profile name (create_descartes_pipeline_profiles defaults
# register "DEFAULT"); every waypoint and the program reference it.
INSTRUCTION_PROFILE = "DEFAULT"
# Cartesian waypoints are straight-line tool moves.
CARTESIAN_MOVE_TYPE = MoveType.LINEAR
# Descartes needs a joint seed for its waypoints; auto-seed assigns the applied
# start configuration so coordinated-solution selection is anchored to it.
CARTESIAN_AUTO_SEED = True

_DESCARTES_OPTION_NAMES = frozenset(
    (
        "num_threads",
        "use_redundant_joint_solutions",
        "enable_collision",
        "enable_edge_collision",
    )
)
_KNOWN_OPTION_NAMES = _DESCARTES_OPTION_NAMES | frozenset(("pipeline",))


class TesseractPlanCartesianMotion(PlanCartesianMotion):
    """Plan a COMPAS Cartesian waypoint path as a coordinated native trajectory."""

    if TYPE_CHECKING:
        client: TesseractClient

    def plan_cartesian_motion(
        self,
        waypoints: Waypoints,
        start_state: RobotCellState,
        group: Optional[str] = None,
        options: Optional[dict[str, object]] = None,
    ) -> JointTrajectory:
        """Plan a Cartesian path and return its conventional COMPAS projection."""
        result = self.plan_cartesian_motion_native(waypoints, start_state, group, options)
        robot_cell = self.client._require_robot_cell()
        group_name = group or robot_cell.main_group_name
        joint_types = {joint.name: joint.type for joint in robot_cell.get_configurable_joints(group_name)}
        trajectory = joint_trajectory_from_result(result, joint_types)
        trajectory.start_state = start_state.copy()
        self.client._store_robot_cell_state(start_state)
        return trajectory

    def plan_cartesian_motion_native(
        self,
        waypoints: Waypoints,
        start_state: RobotCellState,
        group: Optional[str] = None,
        options: Optional[dict[str, object]] = None,
    ) -> TesseractPlanningResult:
        """Plan a Cartesian waypoint path while retaining the exact native result."""
        if not isinstance(waypoints, FrameWaypoints):
            raise UnsupportedTesseractTargetError("Tesseract Cartesian planning supports FrameWaypoints, got {}.".format(type(waypoints).__name__))
        if waypoints.tolerance_position is not None or waypoints.tolerance_orientation is not None:
            raise UnsupportedTesseractToleranceError("FrameWaypoints tolerances are not inputs to the native Descartes waypoints.")

        client: TesseractClient = self.client
        robot_cell = client._require_robot_cell()
        if robot_cell is None:
            raise MissingTesseractStartStateError("Call set_robot_cell before Cartesian motion planning.")
        require_matching_cell_state(robot_cell, start_state, "Cartesian motion planning")
        if start_state.tool_states or start_state.rigid_body_states:
            raise UnsupportedTesseractCellStateError("Phase 1 does not yet apply tools or rigid bodies to the environment clone.")
        if start_state.robot_configuration is None:
            raise MissingTesseractStartStateError("start_state.robot_configuration must be provided.")

        group_name = group or robot_cell.main_group_name
        pipeline, profile_kwargs = _resolve_options(options)

        normalized = waypoints.normalized_to_meters()
        if not isinstance(normalized, FrameWaypoints):
            raise UnsupportedTesseractTargetError("Normalized Tesseract Cartesian waypoints are not FrameWaypoints.")
        start_state.assert_target_mode_match(normalized.target_mode, group_name)

        client.require_planning_contact_managers()
        robot = client.environment.clone_robot()
        apply_active_joint_state(
            robot,
            start_state.robot_configuration,
            "complete Cartesian planning start configuration",
        )
        coupled_frames = client.artifact.coupled_group_frames(group_name)
        if coupled_frames is not None:
            # A coordinated ROP/REP group: COMPAS's joint-group accessors cannot report
            # the working/TCP frames of a cross-branch group (they give the wrong tip),
            # so source them from the coupled config -- the authoritative emitted bytes.
            working_frame, tcp_frame = coupled_frames
        else:
            working_frame = robot_cell.get_base_link_name(group_name)
            tcp_frame = robot_cell.get_end_effector_link_name(group_name)

        world_pcf_frames = cast(
            "list[Frame]",
            robot_cell.target_frames_to_pcf(
                start_state,
                list(normalized.target_frames),
                normalized.target_mode,
                group_name,
            ),
        )
        base_world = world_meters_frame(start_state.robot_base_frame)
        scale = meters_per_user_unit(normalized.native_scale)
        targets = [
            cartesian_target_from_native(
                pose_from_working_frame(
                    WorkingFrameUserUnits.build(
                        robot_frame_from_world(world_meters_frame(world_pcf), base_world).value,
                        scale,
                        working_frame,
                    )
                ),
                CARTESIAN_MOVE_TYPE,
                INSTRUCTION_PROFILE,
            )
            for world_pcf in world_pcf_frames
        ]

        program = build_motion_program(
            robot,
            targets,
            group_name,
            tcp_frame,
            working_frame,
            INSTRUCTION_PROFILE,
        )
        ik_solver = client.artifact.default_inv_kin_solver(group_name)
        profiles = build_descartes_profiles(
            None,
            profile_kwargs["enable_collision"],
            profile_kwargs["enable_edge_collision"],
            profile_kwargs["num_threads"],
            None,
            None,
            None,
            None,
            ik_solver,
            profile_kwargs["use_redundant_joint_solutions"],
            None,
        )
        request = TesseractPlanningRequest.build(
            program.composite_instruction,
            pipeline,
            profiles,
            CARTESIAN_AUTO_SEED,
        )
        return client.runtime.execute(robot, request)


def _resolve_options(options: Optional[dict[str, object]]) -> tuple[str, dict[str, object]]:
    """Validate the option boundary and split pipeline from Descartes knobs.

    The coupled ROP/REP solver is derived from the loaded cell, never an option,
    so `ik_solver` is absent here; the remaining Descartes knobs are validated by
    `build_descartes_profiles`.

    Args:
        options: Existing COMPAS backend option dictionary or None.

    Returns:
        The pipeline name and the Descartes profile keyword values.

    Raises:
        UnknownTesseractOptionError: An option name is unknown or pipeline is not
            a non-empty string.
    """
    values = dict(options or {})
    unknown = sorted(set(values) - _KNOWN_OPTION_NAMES)
    if unknown:
        raise UnknownTesseractOptionError("Unknown Tesseract Cartesian planning options: {}.".format(", ".join(unknown)))
    pipeline = values.get("pipeline", CARTESIAN_PIPELINE)
    if not isinstance(pipeline, str) or not pipeline.strip():
        raise UnknownTesseractOptionError("Tesseract Cartesian pipeline must be a non-empty string.")
    profile_kwargs = {name: values.get(name) for name in _DESCARTES_OPTION_NAMES}
    return pipeline, profile_kwargs
