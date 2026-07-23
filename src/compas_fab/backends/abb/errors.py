"""Named failures raised by the ABB controller safety core."""


class AbbControllerError(Exception):
    """Base for failures raised by the ABB controller safety core."""

    def __init__(self, message: str) -> None:
        Exception.__init__(self, message)
        self.message = message


class InvalidArmingInputError(AbbControllerError):
    """A safety-core factory received a malformed arming input."""


class CommandArmExpiredError(AbbControllerError):
    """An armed command's bound controller revision is no longer current."""


class CommandPreconditionError(AbbControllerError):
    """An armed command was presented against an unintended controller scope."""


class CommandAlreadyConsumedError(AbbControllerError):
    """An invocation nonce was already fired; a recompute must not re-fire."""


class ReadbackMismatchError(AbbControllerError):
    """A post-mutation controller readback disagrees with the commanded intent."""
