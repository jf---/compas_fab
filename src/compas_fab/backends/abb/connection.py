"""Build and sticky-cache a synchronous RWS session for Grasshopper components.

A component builds a sync ``abb_robot_client`` ``RWS`` from an endpoint, a
RobotWare version, and an optional credential handle, then reuses it across
solves through a sticky cache keyed by that configuration — so a recompute does
not reconnect. Changing the configuration closes the old session and builds a
fresh one. Calls are synchronous: a mutation runs directly in ``RunScript``,
matching how the rival ABB Grasshopper tooling behaves.
"""

from __future__ import annotations

from contextlib import suppress
from typing import MutableMapping
from typing import Optional
from typing import cast

from abb_robot_client.rws import RWS
from abb_robot_client.rws import ABBException

from .credentials import CredentialProvider
from .credentials import RobotStudioDefaultProvider
from .credentials import robotware_version
from .errors import ControllerConnectionError


def build_session(
    endpoint: str,
    rw_version_name: str,
    handle: Optional[str],
    provider: Optional[CredentialProvider] = None,
) -> RWS:
    """Construct a sync RWS session with an explicit version and resolved credentials.

    The version is passed explicitly so RWS never runs its blocking auto-detect
    probe; construction opens no network connection (that happens on the first
    call).

    Args:
        endpoint: Controller base URL.
        rw_version_name: ``'RW6'`` or ``'RW7'``.
        handle: Credential name, or ``None`` for the RobotStudio VC defaults.
        provider: Credential resolver; defaults to :class:`RobotStudioDefaultProvider`.

    Returns:
        A constructed (not yet connected) ``RWS``.

    Raises:
        UnknownRobotWareVersionError: The version name is invalid.
        CredentialResolutionError: A named handle's credentials are unset.
        ControllerConnectionError: RWS construction failed.
    """
    resolver = provider if provider is not None else RobotStudioDefaultProvider()
    version = robotware_version(rw_version_name)
    username, password = resolver.resolve(handle)
    try:
        return RWS(base_url=endpoint, username=username, password=password, version=version)
    except (ABBException, OSError) as exc:
        raise ControllerConnectionError("Could not construct RWS for {}: {}".format(endpoint, exc))


def _config_key(endpoint: str, rw_version_name: str, handle: Optional[str]) -> str:
    return "{}|{}|{}".format(endpoint, rw_version_name.strip().upper(), (handle or "").strip())


def cached_session(
    sticky: MutableMapping[str, object],
    slot_key: str,
    endpoint: str,
    rw_version_name: str,
    handle: Optional[str],
    provider: Optional[CredentialProvider] = None,
) -> RWS:
    """Return the cached RWS for this configuration, rebuilding only on change.

    Reuses the sticky-cached session while the endpoint/version/handle are
    unchanged (no reconnect on recompute); when they change, the old session is
    closed (best-effort) and a fresh one built.

    Args:
        sticky: The runtime-only cache (``scriptcontext.sticky`` in Grasshopper).
        slot_key: The owning component's sticky slot.
        endpoint: Controller base URL.
        rw_version_name: ``'RW6'`` or ``'RW7'``.
        handle: Credential name, or ``None`` for the RobotStudio VC defaults.
        provider: Credential resolver; defaults to :class:`RobotStudioDefaultProvider`.

    Returns:
        The cached-or-fresh ``RWS`` for this configuration.
    """
    key = _config_key(endpoint, rw_version_name, handle)
    cached = sticky.get(slot_key)
    if isinstance(cached, tuple) and len(cached) == 2:
        if cached[0] == key:
            return cast(RWS, cached[1])
        # Configuration changed: log the discarded session out best-effort (a
        # failure on an already-unreachable controller must not block the rebuild).
        with suppress(ABBException, OSError):
            cast(RWS, cached[1]).close()
    session = build_session(endpoint, rw_version_name, handle, provider)
    sticky[slot_key] = (key, session)
    return session
