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


class ControllerConnectionError(AbbControllerError):
    """A controller session could not be established by its session factory.

    Raised when the owner's lazy connect (or an explicit reconnect) invokes the
    session factory and it fails, so callers never receive a half-built session.
    """


class ControllerClosedError(AbbControllerError):
    """An operation was submitted to a controller owner that is already closed.

    Raised by the owner's submit gate once ``close`` has run, so a mutation can
    never reach a torn-down worker thread or an already-logged-out session.
    """


class RwsCommandError(AbbControllerError):
    """A Robot Web Services operation failed on the controller.

    The concrete RWS adapter raises this in place of the client's bare
    ``ABBException`` so the owner and components only ever see a named failure.
    """
