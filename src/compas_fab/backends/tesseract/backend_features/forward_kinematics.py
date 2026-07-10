"""Native Tesseract forward kinematics and conventional COMPAS projection."""

from __future__ import annotations

from typing import TYPE_CHECKING
from typing import Optional
from typing import cast

from compas.geometry import Frame  # type: ignore[import-untyped]
from tesseract_robotics.planning import Pose

from compas_fab.backends.interfaces import ForwardKinematics
from compas_fab.robots import RobotCell
from compas_fab.robots import RobotCellState
from compas_fab.robots import TargetMode

from ..cell_state_validation import require_matching_cell_state
from ..client import TesseractClient
from ..errors import MissingTesseractStartStateError
from ..errors import TesseractKinematicsPluginError
from ..errors import UnknownTesseractOptionError
from ..frames import frame_in_user_units
from ..frames import meters_per_user_unit
from ..frames import robot_frame_from_isometry
from ..frames import world_frame_from_robot
from ..frames import world_meters_frame
from ..joint_state import apply_active_joint_state
from ..joint_state import ordered_joint_positions


class TesseractForwardKinematics(ForwardKinematics):
    """Run Tesseract KDL FK before projecting frames through COMPAS state."""

    if TYPE_CHECKING:
        client: TesseractClient

    def forward_kinematics_native(
        self,
        robot_cell_state: RobotCellState,
        link_name: str,
        group: Optional[str] = None,
    ) -> Pose:
        """Return the exact native robot-relative metre pose for one link."""
        client: TesseractClient = self.client
        robot_cell: RobotCell = client.robot_cell
        if robot_cell is None:
            raise MissingTesseractStartStateError("Call set_robot_cell before forward kinematics.")
        require_matching_cell_state(robot_cell, robot_cell_state, "forward kinematics")
        configuration = robot_cell_state.robot_configuration
        if configuration is None:
            raise MissingTesseractStartStateError("robot_cell_state.robot_configuration is required for forward kinematics.")

        group_name = group or robot_cell.main_group_name
        robot = client.environment.clone_robot()
        try:
            apply_active_joint_state(
                robot,
                configuration,
                "complete forward-kinematics configuration",
            )
            joint_names = robot.get_joint_names(group_name)
            positions = ordered_joint_positions(
                configuration,
                joint_names,
                "forward-kinematics configuration",
            )
            pose = robot.fk(group_name, positions, tip_link=link_name)
        except (KeyError, RuntimeError, ValueError) as error:
            raise TesseractKinematicsPluginError("Tesseract forward kinematics failed for group {!r}, link {!r}: {}.".format(group_name, link_name, error)) from error
        return pose

    def forward_kinematics(
        self,
        robot_cell_state: RobotCellState,
        target_mode: TargetMode,
        group: Optional[str] = None,
        native_scale: Optional[float] = None,
        options: Optional[dict[str, object]] = None,
    ) -> Frame:
        """Project exact native FK into a requested COMPAS target frame."""
        if options:
            raise UnknownTesseractOptionError("forward_kinematics accepts no Tesseract options: {}.".format(", ".join(sorted(options))))
        client: TesseractClient = self.client
        robot_cell: RobotCell = client.robot_cell
        if robot_cell is None:
            raise MissingTesseractStartStateError("Call set_robot_cell before forward kinematics.")
        group_name = group or robot_cell.main_group_name
        robot_cell_state.assert_target_mode_match(target_mode, group_name)
        link_name = robot_cell.get_end_effector_link_name(group_name)
        native_pose = self.forward_kinematics_native(
            robot_cell_state,
            link_name,
            group_name,
        )
        robot_frame = robot_frame_from_isometry(native_pose)
        world_pcf = world_frame_from_robot(
            robot_frame,
            world_meters_frame(robot_cell_state.robot_base_frame),
        )
        target_frame = cast(
            Frame,
            robot_cell.pcf_to_target_frames(
                robot_cell_state,
                world_pcf.value,
                target_mode,
                group_name,
            ),
        )
        client._store_robot_cell_state(robot_cell_state)
        return frame_in_user_units(
            world_meters_frame(target_frame),
            meters_per_user_unit(native_scale),
        )

    def forward_kinematics_to_link(
        self,
        robot_cell_state: RobotCellState,
        link_name: Optional[str] = None,
        native_scale: Optional[float] = None,
        options: Optional[dict[str, object]] = None,
    ) -> Frame:
        """Project one exact native link pose into the COMPAS world frame."""
        values = dict(options or {})
        unknown = sorted(set(values) - {"group"})
        if unknown:
            raise UnknownTesseractOptionError("Unknown forward_kinematics_to_link options: {}.".format(", ".join(unknown)))
        client: TesseractClient = self.client
        robot_cell: RobotCell = client.robot_cell
        if robot_cell is None:
            raise MissingTesseractStartStateError("Call set_robot_cell before forward kinematics.")
        group_value = values.get("group", robot_cell.main_group_name)
        if not isinstance(group_value, str) or not group_value:
            raise UnknownTesseractOptionError("forward kinematics group must be a non-empty string.")
        selected_link = link_name or robot_cell.get_end_effector_link_name(group_value)
        native_pose = self.forward_kinematics_native(
            robot_cell_state,
            selected_link,
            group_value,
        )
        world_frame = world_frame_from_robot(
            robot_frame_from_isometry(native_pose),
            world_meters_frame(robot_cell_state.robot_base_frame),
        )
        client._store_robot_cell_state(robot_cell_state)
        return frame_in_user_units(world_frame, meters_per_user_unit(native_scale))
