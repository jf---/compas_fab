"""Named failures raised by the Tesseract backend."""

from compas_fab.backends.exceptions import BackendError
from compas_fab.backends.exceptions import CollisionCheckError
from compas_fab.backends.exceptions import InverseKinematicsError


class TesseractBackendError(BackendError):
    """Base for failures raised at a Tesseract backend boundary."""

    def __init__(self, message: str) -> None:
        Exception.__init__(self, message)
        self.message = message


class EmptyRobotDescriptionError(TesseractBackendError):
    """A required URDF or SRDF description is empty."""


class InvalidRobotResourceError(TesseractBackendError):
    """A robot resource URL or payload cannot be content-addressed."""


class MissingRobotDescriptionFileError(TesseractBackendError):
    """A required URDF or SRDF source file does not exist."""


class InvalidRobotDescriptionEncodingError(TesseractBackendError):
    """A URDF or SRDF source file is not exact UTF-8 text."""


class RobotDescriptionReadError(TesseractBackendError):
    """A URDF or SRDF source cannot be read from the filesystem."""


class RobotResourceReadError(TesseractBackendError):
    """A referenced robot package resource cannot be read exactly."""


class MissingRobotResourceRootError(TesseractBackendError):
    """A configured package-resource root does not exist."""


class InvalidRobotResourceRootError(TesseractBackendError):
    """A configured package-resource root is not a directory."""


class DuplicateRobotResourceRootError(TesseractBackendError):
    """The same package-resource root was configured more than once."""


class MissingRobotPackageError(TesseractBackendError):
    """No configured resource root contains a referenced package."""


class AmbiguousRobotPackageError(TesseractBackendError):
    """More than one configured resource root contains a referenced package."""


class InvalidUrdfError(TesseractBackendError):
    """A source string is not a valid URDF robot document."""


class CollisionMeshPolicyConflictError(TesseractBackendError):
    """A URDF's Tesseract mesh policy conflicts with the requested policy."""


class InvalidKinematicsConfigError(TesseractBackendError):
    """A typed Tesseract kinematics configuration violates its contract."""


class KinematicsPluginConflictError(TesseractBackendError):
    """An exact SRDF already selects a kinematics plugin resource."""


class ContactManagerPluginConflictError(TesseractBackendError):
    """An exact SRDF already selects a contact-manager plugin resource."""


class MissingContactManagerPluginError(TesseractBackendError):
    """A planning environment has no loadable native contact manager."""


class InvalidSrdfError(TesseractBackendError):
    """A source string is not a valid SRDF robot document."""


class UnsafeRobotResourceUrlError(TesseractBackendError):
    """A robot resource URL would escape its content-addressed root."""


class ArtifactMaterializationError(TesseractBackendError):
    """Materialized resource bytes violate their artifact identity."""


class UnknownRobotResourceError(TesseractBackendError):
    """Tesseract requested a resource absent from the exact artifact."""


class TesseractEnvironmentInitializationError(TesseractBackendError):
    """Tesseract could not initialize an exact robot artifact."""

    def __init__(self, digest: str, diagnostic: str) -> None:
        super().__init__("Tesseract environment initialization failed for artifact {}: {}".format(digest, diagnostic))
        self.digest = digest
        self.diagnostic = diagnostic


class TesseractClientNotConnectedError(TesseractBackendError):
    """A backend operation requires an initialized local runtime."""


class TesseractRuntimeInitializationError(TesseractBackendError):
    """The native Task Composer runtime could not be constructed."""


class InvalidTesseractRuntimeConfigurationError(TesseractBackendError):
    """A supplied Task Composer or warmup selection has the wrong type."""


class TesseractPipelineWarmupError(TesseractBackendError):
    """One or more explicitly selected Task Composer pipelines failed to load."""


class TesseractProgramCopyError(TesseractBackendError):
    """A caller-owned native program could not be copied before execution."""


class InvalidTesseractSelectionError(TesseractBackendError):
    """A frontend selected an unknown explicit native backend option."""


class InvalidTesseractNativeScaleError(TesseractBackendError):
    """A user-unit scale cannot map coordinates to native metres."""


class InvalidTesseractFrameError(TesseractBackendError):
    """A frame-tagged Tesseract boundary received a non-COMPAS frame."""


class TesseractKinematicsPluginError(TesseractBackendError):
    """A requested native kinematics group or solver cannot be loaded."""


class MalformedTesseractKinematicsResultError(TesseractBackendError):
    """A native kinematics result violates its group shape contract."""


class TesseractInverseKinematicsError(InverseKinematicsError):
    """Native Tesseract kinematics returned no inverse solution."""

    def __init__(self, message: str, target_pcf: object = None) -> None:
        Exception.__init__(self, message)
        self.message = message
        self.target_pcf = target_pcf


class TesseractContactQueryError(TesseractBackendError):
    """The native discrete contact-manager query failed."""


class TesseractCollisionError(CollisionCheckError):
    """A native Tesseract contact query found penetrating or touching links."""

    def __init__(
        self,
        message: str,
        native_result: object,
        link_pairs: tuple[tuple[str, str], ...],
        distances: tuple[float, ...],
    ) -> None:
        Exception.__init__(self, message)
        self.message = message
        self.collision_pairs = list(link_pairs)
        self.native_result = native_result
        self.link_pairs = link_pairs
        self.distances = distances


class RobotArtifactMismatchError(TesseractBackendError):
    """A COMPAS robot cell disagrees with the exact Tesseract artifact."""


class TesseractCellStateMismatchError(TesseractBackendError):
    """A COMPAS cell state does not match the installed robot cell."""


class UnknownTesseractOptionError(TesseractBackendError):
    """An existing COMPAS call supplied an unknown Tesseract option."""


class UnsupportedTesseractToleranceError(TesseractBackendError):
    """A COMPAS tolerance cannot be represented by the selected native path."""


class UnsupportedTesseractTargetError(TesseractBackendError):
    """A COMPAS target cannot be lowered to Tesseract without information loss."""


class UnsupportedTesseractCellStateError(TesseractBackendError):
    """A COMPAS cell-state feature is not yet applied to the native environment."""


class MissingTesseractStartStateError(TesseractBackendError):
    """Motion planning lacks a complete named start configuration."""


class TesseractConfigurationMismatchError(TesseractBackendError):
    """A named configuration does not cover the requested native joint group."""


class MissingTesseractProfilesError(TesseractBackendError):
    """A custom Task Composer pipeline lacks an explicit profile dictionary."""


class InvalidTesseractProfilesError(TesseractBackendError):
    """A COMPAS option is not a native Tesseract profile dictionary."""


class EmptyTesseractProgramError(TesseractBackendError):
    """A native Tesseract motion program contains no instructions."""


class InvalidTesseractProgramError(TesseractBackendError):
    """A native planning request did not receive CompositeInstruction."""


class InvalidTesseractPipelineError(TesseractBackendError):
    """A Task Composer pipeline name is empty or invalid."""


class MissingTesseractOutputError(TesseractBackendError):
    """Successful native planning did not produce a motion program."""


class EmptyTesseractTrajectoryError(TesseractBackendError):
    """A successful native result contains no trajectory points."""


class MalformedTesseractTrajectoryError(TesseractBackendError):
    """Native trajectory fields disagree in shape or contain invalid values."""


class InconsistentTesseractJointOrderError(TesseractBackendError):
    """Native trajectory points use different joint-name orders."""


class MissingTesseractJointTypeError(TesseractBackendError):
    """COMPAS joint type information is absent for a native joint."""


class MissingTesseractTrajectoryFieldError(TesseractBackendError):
    """A native pipeline omitted dynamics required by COMPAS projection."""


class MissingTesseractTrajectoryNativeResultError(TesseractBackendError):
    """A conventional trajectory was not produced by this Tesseract backend."""


class NonMonotonicTesseractTrajectoryError(TesseractBackendError):
    """Native trajectory time is negative or decreases between points."""


class TesseractPlanningFailedError(TesseractBackendError):
    """A requested native Tesseract pipeline failed."""

    def __init__(self, pipeline: str, diagnostic: str):
        super().__init__("Tesseract pipeline {!r} failed: {}".format(pipeline, diagnostic))
        self.pipeline = pipeline
        self.diagnostic = diagnostic
