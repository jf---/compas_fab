"""Native Tesseract inverse kinematics and conventional COMPAS projection."""

from __future__ import annotations

from collections.abc import Iterator
from typing import TYPE_CHECKING
from typing import Optional
from typing import cast

import numpy as np
from compas.geometry import Frame  # type: ignore[import-untyped]
from compas_robots import Configuration  # type: ignore[import-untyped]
from tesseract_robotics.tesseract_kinematics import KinGroupIKInput

from compas_fab.backends.interfaces import InverseKinematics
from compas_fab.robots import FrameTarget
from compas_fab.robots import RobotCellState
from compas_fab.robots import Target

from ..cell_state_validation import require_matching_cell_state
from ..client import TesseractClient
from ..errors import MissingTesseractStartStateError
from ..errors import TesseractInverseKinematicsError
from ..errors import TesseractKinematicsPluginError
from ..errors import UnknownTesseractOptionError
from ..errors import UnsupportedTesseractTargetError
from ..errors import UnsupportedTesseractToleranceError
from ..frames import isometry_from_robot_frame
from ..frames import meters_per_user_unit
from ..frames import robot_frame_from_world
from ..frames import world_meters_frame
from ..joint_state import apply_active_joint_state
from ..joint_state import ordered_joint_positions
from ..kinematics import JointVector
from ..kinematics import TesseractInverseKinematicsResult


class TesseractInverseKinematics(InverseKinematics):
    """Run a selected native Tesseract IK plugin and retain every solution."""

    if TYPE_CHECKING:
        client: TesseractClient

    def inverse_kinematics_native(
        self,
        target: Target,
        robot_cell_state: RobotCellState,
        group: Optional[str] = None,
        ik_solver_name: str = "",
    ) -> TesseractInverseKinematicsResult:
        """Return the exact native IK input and complete solution list."""
        if not isinstance(target, FrameTarget):
            raise UnsupportedTesseractTargetError("Tesseract inverse kinematics requires FrameTarget, got {}.".format(type(target).__name__))
        if target.tolerance_position is not None or target.tolerance_orientation is not None:
            raise UnsupportedTesseractToleranceError("FrameTarget tolerances are not inputs to the selected native IK plugin.")
        meters_per_user_unit(target.native_scale)

        client: TesseractClient = self.client
        robot_cell = client._require_robot_cell()
        if robot_cell is None:
            raise MissingTesseractStartStateError("Call set_robot_cell before inverse kinematics.")
        require_matching_cell_state(robot_cell, robot_cell_state, "inverse kinematics")
        configuration = robot_cell_state.robot_configuration
        if configuration is None:
            raise MissingTesseractStartStateError("robot_cell_state.robot_configuration is required as the native IK seed.")

        group_name = group or robot_cell.main_group_name
        normalized_target = target.normalized_to_meters()
        if not isinstance(normalized_target, FrameTarget):
            raise UnsupportedTesseractTargetError("Normalized Tesseract IK target is not a FrameTarget.")
        robot_cell_state.assert_target_mode_match(normalized_target.target_mode, group_name)
        world_pcf = cast(
            Frame,
            robot_cell.target_frames_to_pcf(
                robot_cell_state,
                normalized_target.target_frame,
                normalized_target.target_mode,
                group_name,
            ),
        )
        target_pose = isometry_from_robot_frame(
            robot_frame_from_world(
                world_meters_frame(world_pcf),
                world_meters_frame(robot_cell_state.robot_base_frame),
            )
        )

        robot = client.environment.clone_robot()
        try:
            apply_active_joint_state(
                robot,
                configuration,
                "complete inverse-kinematics seed",
            )
            joint_names = robot.get_joint_names(group_name)
            seed = np.asarray(
                ordered_joint_positions(configuration, joint_names, "inverse-kinematics seed"),
                dtype=np.float64,
            )
            kinematic_group = robot.env.getKinematicGroup(group_name, ik_solver_name)
            native_input = KinGroupIKInput(
                target_pose,
                kinematic_group.getBaseLinkName(),
                robot_cell.get_end_effector_link_name(group_name),
            )
            native_solutions: list[JointVector] = kinematic_group.calcInvKin(native_input, seed)
        except (KeyError, RuntimeError, ValueError) as error:
            raise TesseractKinematicsPluginError(
                "Tesseract inverse kinematics failed for group {!r}, solver {!r}: {}.".format(group_name, ik_solver_name or "<default>", error)
            ) from error
        return TesseractInverseKinematicsResult.build(
            group_name,
            joint_names,
            target_pose,
            native_input,
            native_solutions,
        )

    def inverse_kinematics(
        self,
        target: Target,
        robot_cell_state: Optional[RobotCellState] = None,
        group: Optional[str] = None,
        options: Optional[dict[str, object]] = None,
    ) -> Configuration:
        """Return the first conventional projection of all native solutions."""
        return next(self.iter_inverse_kinematics(target, robot_cell_state, group, options))

    def iter_inverse_kinematics(
        self,
        target: Target,
        robot_cell_state: Optional[RobotCellState] = None,
        group: Optional[str] = None,
        options: Optional[dict[str, object]] = None,
    ) -> Iterator[Configuration]:
        """Yield every native solution in native order without randomized fallback."""
        if robot_cell_state is None:
            raise MissingTesseractStartStateError("robot_cell_state is required as an explicit native IK seed.")
        values = dict(options or {})
        unknown = sorted(set(values) - {"ik_solver_name", "return_full_configuration"})
        if unknown:
            raise UnknownTesseractOptionError("Unknown Tesseract inverse-kinematics options: {}.".format(", ".join(unknown)))
        solver_value = values.get("ik_solver_name", "")
        if not isinstance(solver_value, str):
            raise UnknownTesseractOptionError("ik_solver_name must be a string.")
        return_full_value = values.get("return_full_configuration", False)
        if not isinstance(return_full_value, bool):
            raise UnknownTesseractOptionError("return_full_configuration must be bool.")

        result = self.inverse_kinematics_native(
            target,
            robot_cell_state,
            group,
            solver_value,
        )
        if not result:
            target_pcf = target.target_frame if isinstance(target, FrameTarget) else None
            raise TesseractInverseKinematicsError(
                "Native Tesseract IK returned no solution.",
                target_pcf,
            )
        client: TesseractClient = self.client
        robot_cell = client._require_robot_cell()
        group_name = group or robot_cell.main_group_name
        client._store_robot_cell_state(robot_cell_state)
        for solution in result.native_solutions:
            yield self._build_configuration(
                solution.tolist(),
                list(result.joint_names),
                group_name,
                return_full_value,
                robot_cell_state.robot_configuration,
            )
