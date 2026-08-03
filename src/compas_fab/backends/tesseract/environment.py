"""Initialized Tesseract environment ownership and clone isolation."""

from __future__ import annotations

from pathlib import Path
from typing import Optional
from typing import cast

from attrs import define
from platformdirs import user_cache_path
from tesseract_robotics.planning import Robot
from tesseract_robotics.tesseract_common import GeneralResourceLocator
from tesseract_robotics.tesseract_environment import Environment

from .artifact import RobotArtifact
from .errors import TesseractEnvironmentInitializationError
from .materialization import ArtifactResourceLocator
from .materialization import MaterializedArtifact


@define(slots=True)
class TesseractEnvironment:
    """Own one initialized environment and its immutable resource lifetime."""

    artifact: RobotArtifact
    materialized: MaterializedArtifact
    locator: ArtifactResourceLocator
    robot: Robot

    @classmethod
    def build(
        cls,
        artifact: RobotArtifact,
        cache_root: Optional[Path] = None,
    ) -> TesseractEnvironment:
        """Initialize Tesseract from exact artifact inputs.

        Args:
            artifact: Exact content-addressed robot artifact.
            cache_root: Optional materialization parent used by tests and hosts.

        Returns:
            Initialized environment owner.

        Raises:
            TesseractEnvironmentInitializationError: Native initialization fails.
        """
        root = cache_root or user_cache_path("compas_fab") / "tesseract" / "artifacts"
        materialized = MaterializedArtifact.build(artifact, root)
        locator = ArtifactResourceLocator(materialized)
        native_environment = Environment()
        try:
            initialized = native_environment.initFromUrdfSrdf(
                artifact.urdf,
                artifact.srdf,
                locator,
            )
        except (RuntimeError, ValueError) as error:
            raise TesseractEnvironmentInitializationError(artifact.identity.digest, str(error)) from error
        if not initialized:
            raise TesseractEnvironmentInitializationError(artifact.identity.digest, "native initialization returned false")

        robot = Robot(native_environment, cast(GeneralResourceLocator, locator))
        return cls(artifact, materialized, locator, robot)

    def clone_robot(self) -> Robot:
        """Return an isolated native robot environment clone."""
        cloned_environment = self.robot.env.clone()
        return Robot(cloned_environment, cast(GeneralResourceLocator, self.locator))
