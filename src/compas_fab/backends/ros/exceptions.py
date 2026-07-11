from compas_fab.backends.exceptions import BackendError

__all__ = [
    "RosError",
    "RosValidationError",
]


class RosError(BackendError):
    """Wraps an exception that occurred on the communication with ROS."""

    def __init__(self, message: str, error_code: object) -> None:
        super(RosError, self).__init__("Error code: " + str(error_code) + "; " + message)  # type: ignore[no-untyped-call]
        self.error_code = error_code


class RosValidationError(BackendError):
    """Wraps an exception that occurred on validation of a ROS response."""

    def __init__(self, original_exception: Exception, response: object) -> None:
        super(RosValidationError, self).__init__(str(original_exception))  # type: ignore[no-untyped-call]
        self.response = response
        self.original_exception = original_exception


class InvalidMoveItPlanMotionOptionsError(BackendError):
    """MoveIt Plan Motion options violate the declared typed contract."""

    def __init__(self, message: str) -> None:
        Exception.__init__(self, message)
        self.message = message
