"""Credential resolution for ABB controller sessions — secrets out of the GHX.

A Grasshopper component serializes only an endpoint, a RobotWare version, and an
optional credential *handle* (a name) into its document. The password is never a
canvas input and is resolved at connect time by a ``CredentialProvider``, so it
never reaches disk. The common case — a RobotStudio virtual controller — needs
no handle at all and resolves to the VC defaults, so a freshly dropped component
connects with zero credential wiring. A physical controller names a handle whose
username/password live in environment variables, set once, outside the document.
"""

from __future__ import annotations

from os import environ
from typing import Callable
from typing import Optional
from typing import Protocol
from typing import Tuple
from typing import runtime_checkable

from abb_robot_client import RobotWareVersion

from .errors import CredentialResolutionError
from .errors import UnknownRobotWareVersionError
from .rws_session import build_rws_session_factory
from .session import ControllerSession

# RobotStudio virtual-controller defaults: the zero-configuration dev path.
_ROBOTSTUDIO_DEFAULT_USERNAME = "Default User"
_ROBOTSTUDIO_DEFAULT_PASSWORD = "robotics"

_ROBOTWARE_VERSIONS = {"RW6": RobotWareVersion.RW6, "RW7": RobotWareVersion.RW7}


@runtime_checkable
class CredentialProvider(Protocol):
    """Resolves a credential handle to a ``(username, password)`` pair."""

    def resolve(self, handle: Optional[str]) -> Tuple[str, str]:
        """Return the username and password for ``handle`` (``None`` = default)."""
        ...


class RobotStudioDefaultProvider:
    """Resolve an absent handle to the RobotStudio VC defaults, else the env.

    An absent (``None``/blank) handle is the zero-config dev path and returns the
    RobotStudio virtual-controller defaults. A named handle is resolved from
    environment variables so a physical controller needs no code change and no
    secret in the document. Nothing is stored on the instance.
    """

    def resolve(self, handle: Optional[str]) -> Tuple[str, str]:
        """Resolve ``handle`` to a ``(username, password)`` pair.

        Args:
            handle: A credential name, or ``None``/blank for the RobotStudio
                virtual-controller defaults.

        Returns:
            The resolved username and password.

        Raises:
            CredentialResolutionError: A named handle's environment variables are
                unset or empty.
        """
        if handle is None or not handle.strip():
            return _ROBOTSTUDIO_DEFAULT_USERNAME, _ROBOTSTUDIO_DEFAULT_PASSWORD
        key = handle.strip().upper()
        username = environ.get("ABB_{}_USERNAME".format(key))
        password = environ.get("ABB_{}_PASSWORD".format(key))
        if not username or not password:
            raise CredentialResolutionError("Credential handle {!r} requires ABB_{}_USERNAME and ABB_{}_PASSWORD in the environment.".format(handle, key, key))
        return username, password


def robotware_version(name: str) -> RobotWareVersion:
    """Map a ``'RW6'``/``'RW7'`` name to a ``RobotWareVersion``.

    Args:
        name: The RobotWare version name.

    Returns:
        The corresponding ``RobotWareVersion``.

    Raises:
        UnknownRobotWareVersionError: ``name`` is not ``'RW6'`` or ``'RW7'``.
    """
    try:
        return _ROBOTWARE_VERSIONS[name.strip().upper()]
    except (AttributeError, KeyError):
        raise UnknownRobotWareVersionError("RobotWare version must be 'RW6' or 'RW7', got {!r}.".format(name))


def build_controller_session_factory(
    endpoint: str,
    rw_version_name: str,
    credential_handle: Optional[str],
    provider: Optional[CredentialProvider] = None,
) -> Callable[[], ControllerSession]:
    """Assemble a session factory from component-level inputs.

    Resolves the RobotWare version and credentials, then delegates to
    :func:`build_rws_session_factory`. The default provider makes an unconnected
    handle connect to a RobotStudio VC with no wiring.

    Args:
        endpoint: Controller base URL.
        rw_version_name: ``'RW6'`` or ``'RW7'``.
        credential_handle: Credential name, or ``None`` for the RobotStudio defaults.
        provider: Credential resolver; defaults to :class:`RobotStudioDefaultProvider`.

    Returns:
        A zero-argument factory for ``ControllerOwner``/``acquire_owner``.

    Raises:
        UnknownRobotWareVersionError: The version name is invalid.
        CredentialResolutionError: A named handle's credentials are unset.
    """
    resolver = provider if provider is not None else RobotStudioDefaultProvider()
    version = robotware_version(rw_version_name)
    username, password = resolver.resolve(credential_handle)
    return build_rws_session_factory(endpoint, version, username, password)
