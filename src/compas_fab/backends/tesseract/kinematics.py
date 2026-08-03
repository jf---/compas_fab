"""Lossless native inverse-kinematics result value."""

from __future__ import annotations

import numpy as np
from attrs import define
from numpy.typing import NDArray
from tesseract_robotics.planning import Pose
from tesseract_robotics.tesseract_kinematics import KinGroupIKInput

from .errors import MalformedTesseractKinematicsResultError

JointVector = NDArray[np.float64]


@define(frozen=True, slots=True)
class TesseractInverseKinematicsResult:
    """Exact native IK input and every array returned by its selected solver."""

    group: str
    joint_names: tuple[str, ...]
    target_pose: Pose
    native_input: KinGroupIKInput
    native_solutions: list[JointVector]

    @classmethod
    def build(
        cls,
        group: str,
        joint_names: list[str],
        target_pose: Pose,
        native_input: KinGroupIKInput,
        native_solutions: list[JointVector],
    ) -> TesseractInverseKinematicsResult:
        """Validate solution shapes while retaining exact native objects."""
        expected_shape = (len(joint_names),)
        malformed = [index for index, solution in enumerate(native_solutions) if solution.shape != expected_shape]
        if malformed:
            raise MalformedTesseractKinematicsResultError("Native IK solutions have invalid shape at indices: {}.".format(", ".join(str(index) for index in malformed)))
        return cls(
            group,
            tuple(joint_names),
            target_pose,
            native_input,
            native_solutions,
        )

    def __len__(self) -> int:
        return len(self.native_solutions)

    def __getitem__(self, index: int) -> JointVector:
        return self.native_solutions[index]

    def __bool__(self) -> bool:
        return bool(self.native_solutions)
