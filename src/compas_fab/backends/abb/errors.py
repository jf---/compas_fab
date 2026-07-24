"""Named failures for the ABB controller integration."""


class AbbControllerError(Exception):
    """Base for failures raised by the ABB controller integration."""

    def __init__(self, message: str) -> None:
        Exception.__init__(self, message)
        self.message = message


class ControllerConnectionError(AbbControllerError):
    """An RWS session could not be constructed for a controller endpoint."""


class CredentialResolutionError(AbbControllerError):
    """A credential handle could not be resolved to a username and password.

    Raised when a named handle points at environment credentials that are unset,
    so a connection never proceeds with missing or empty credentials.
    """


class UnknownRobotWareVersionError(AbbControllerError):
    """A RobotWare version name is neither ``RW6`` nor ``RW7``."""
