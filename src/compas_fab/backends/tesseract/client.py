"""Local Tesseract client lifecycle."""

from __future__ import annotations

from pathlib import Path
from typing import Optional

from compas.data import json_dumps  # type: ignore[import-untyped]
from tesseract_robotics.planning import Robot
from tesseract_robotics.planning import TaskComposer

from compas_fab.backends.interfaces.client import ClientInterface
from compas_fab.robots import RobotCell
from compas_fab.robots import RobotCellState

from .artifact import RobotArtifact
from .cell_state_validation import require_matching_cell_state
from .environment import TesseractEnvironment
from .errors import InvalidTesseractRuntimeConfigurationError
from .errors import MissingContactManagerPluginError
from .errors import MissingTesseractStartStateError
from .errors import TesseractClientNotConnectedError
from .errors import TesseractRuntimeInitializationError
from .errors import UnsupportedTesseractCellStateError
from .joint_state import apply_active_joint_state
from .runtime import TesseractRuntime
from .warmup import WarmupSelection
from .warmup import warmup_composer

NativeProjectionIdentity = tuple[
    str,
    Optional[str],
]


class TesseractClient(ClientInterface):
    """Own exact robot resources and a reusable local Task Composer runtime."""

    def __init__(
        self,
        artifact: RobotArtifact,
        cache_root: Optional[Path] = None,
        warmup: WarmupSelection = False,
        composer: Optional[TaskComposer] = None,
    ) -> None:
        object.__init__(self)
        self._robot_cell: Optional[RobotCell] = None
        self._robot_cell_state: Optional[RobotCellState] = None
        self._native_scene_revision = 0
        self._scene_projection_identity: Optional[NativeProjectionIdentity] = None
        self.artifact = artifact
        self.cache_root = cache_root
        if composer is not None and not isinstance(composer, TaskComposer):
            raise InvalidTesseractRuntimeConfigurationError("composer must be TaskComposer, got {}.".format(type(composer).__name__))
        self.warmup = list(warmup) if isinstance(warmup, list) else warmup
        self._provided_composer = composer
        self.warmed_pipelines: tuple[str, ...] = ()
        self._environment: Optional[TesseractEnvironment] = None
        self._runtime: Optional[TesseractRuntime] = None

    @property
    def is_connected(self) -> bool:
        """Whether environment and Task Composer lifetimes are active."""
        return self._environment is not None and self._runtime is not None

    @property
    def environment(self) -> TesseractEnvironment:
        """Return the initialized environment owner.

        Raises:
            TesseractClientNotConnectedError: `connect()` has not completed.
        """
        if self._environment is None:
            raise TesseractClientNotConnectedError("Tesseract client is not connected.")
        return self._environment

    @property
    def runtime(self) -> TesseractRuntime:
        """Return the initialized execution runtime.

        Raises:
            TesseractClientNotConnectedError: `connect()` has not completed.
        """
        if self._runtime is None:
            raise TesseractClientNotConnectedError("Tesseract client is not connected.")
        return self._runtime

    @property
    def native_scene_revision(self) -> int:
        """Monotonic revision of exact state consumed by native clones."""
        return self._native_scene_revision

    def connect(self) -> None:
        """Initialize the exact environment and reusable Task Composer."""
        if self.is_connected:
            return
        environment = TesseractEnvironment.build(self.artifact, self.cache_root)
        try:
            composer = self._provided_composer or TaskComposer.from_config(warmup=False)
            warmed_pipelines = warmup_composer(composer, self.warmup)
        except (RuntimeError, ValueError) as error:
            raise TesseractRuntimeInitializationError("Tesseract Task Composer initialization failed: {}.".format(error)) from error
        self._environment = environment
        self._runtime = TesseractRuntime.build(composer)
        self.warmed_pipelines = warmed_pipelines

    def disconnect(self) -> None:
        """Release local native runtime and environment references."""
        self._runtime = None
        self._environment = None
        self.warmed_pipelines = ()

    def clone_robot(self) -> Robot:
        """Clone the environment and apply the complete stored robot state."""
        robot = self.environment.clone_robot()
        state = self._robot_cell_state
        if state is None:
            return robot
        cell = self._robot_cell
        if cell is None:
            raise MissingTesseractStartStateError("Stored robot_cell_state has no installed RobotCell.")
        require_matching_cell_state(cell, state, "native clone")
        if state.tool_states or state.rigid_body_states:
            raise UnsupportedTesseractCellStateError("Stored tools and rigid bodies must be applied to the native environment before cloning.")
        configuration = state.robot_configuration
        if configuration is None:
            raise MissingTesseractStartStateError("Stored robot_cell_state.robot_configuration is required for a stateful native clone.")
        apply_active_joint_state(robot, configuration, "complete stored robot-cell configuration")
        return robot

    def _store_robot_cell_projection(
        self,
        robot_cell: RobotCell,
        robot_cell_state: Optional[RobotCellState],
    ) -> None:
        """Copy a validated cell projection and revise only on content change."""
        cell_copy = robot_cell.copy()
        state_copy = robot_cell_state.copy() if robot_cell_state is not None else None
        self._store_projection(cell_copy, state_copy)

    def _store_robot_cell_state(self, robot_cell_state: RobotCellState) -> None:
        """Copy a validated state and revise the native clone input exactly."""
        cell = self._robot_cell
        if cell is None:
            raise MissingTesseractStartStateError("Cannot store robot_cell_state before RobotCell.")
        self._store_projection(cell, robot_cell_state.copy())

    def _store_projection(
        self,
        robot_cell: RobotCell,
        robot_cell_state: Optional[RobotCellState],
    ) -> None:
        identity = _projection_identity(robot_cell, robot_cell_state)
        if identity != self._scene_projection_identity:
            self._native_scene_revision += 1
            self._scene_projection_identity = identity
        self._robot_cell = robot_cell
        self._robot_cell_state = robot_cell_state

    def _mark_native_scene_changed(self) -> None:
        """Advance the revision after a direct native scene command."""
        self._native_scene_revision += 1

    def require_planning_contact_managers(self) -> None:
        """Require both managers used by the conventional free-motion path."""
        _assert_contact_managers(self.environment)

    def __enter__(self) -> TesseractClient:
        self.connect()
        return self

    def __exit__(self, *args: object) -> None:
        self.disconnect()


def _assert_contact_managers(environment: TesseractEnvironment) -> None:
    native_environment = environment.robot.env
    try:
        discrete = native_environment.getDiscreteContactManager()
        continuous = native_environment.getContinuousContactManager()
    except RuntimeError as error:
        raise MissingContactManagerPluginError("Tesseract contact-manager plugin loading failed: {}".format(error)) from error
    if discrete is None or continuous is None:
        raise MissingContactManagerPluginError("Tesseract artifact must configure discrete and continuous contact managers.")


def _projection_identity(
    robot_cell: RobotCell,
    robot_cell_state: Optional[RobotCellState],
) -> NativeProjectionIdentity:
    return (
        robot_cell.structural_signature(),
        None if robot_cell_state is None else json_dumps(robot_cell_state.__data__, pretty=False),
    )
