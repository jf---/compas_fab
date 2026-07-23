"""Concrete adapter: an ``abb_robot_client`` RWS driven as a ``ControllerSession``.

The owner and components are typed against the ``ControllerSession`` Protocol,
never the RWS library. This module is the single place that imports
``abb_robot_client`` and maps it onto that Protocol, wrapping the client's bare
``ABBException`` into a named ``RwsCommandError`` so no untyped failure escapes
into the safety layer.
"""

from __future__ import annotations

from typing import Callable
from typing import Sequence
from typing import TypeVar
from typing import cast

from abb_robot_client import RobotWareVersion
from abb_robot_client.rws import RWS
from abb_robot_client.rws import ABBException

from .errors import ControllerConnectionError
from .errors import RwsCommandError
from .session import ControllerSession

T = TypeVar("T")

# The RWS I/O and start defaults; the Protocol leaves them to the concrete
# adapter and these mirror abb_robot_client.rws.RWS's own signature defaults.
_DEFAULT_IO_NETWORK = "Local"
_DEFAULT_IO_UNIT = "DRV_1"
_DEFAULT_START_CYCLE = "asis"
_DEFAULT_START_TASKS = ("T_ROB1",)


class RwsControllerSession:
    """Adapt a live RWS client to the :class:`ControllerSession` Protocol.

    Each method delegates to the wrapped RWS, converting the client's bare
    ``ABBException`` into a named :class:`RwsCommandError`. NOT thread-safe: the
    owner drives every call from its single worker thread.
    """

    def __init__(self, client: RWS) -> None:
        """Wrap an already-constructed RWS client.

        Args:
            client: A live ``abb_robot_client.rws.RWS`` instance.
        """
        self._client = client

    def _guard(self, operation: str, call: Callable[[], T]) -> T:
        try:
            return call()
        except ABBException as exc:
            raise RwsCommandError("RWS operation {!r} failed: {}".format(operation, exc)) from exc

    def get_controller_state(self) -> str:
        """Return the controller state, wrapping a client failure."""
        # cast: the untyped client returns Any; the RWS contract is a str.
        return cast(str, self._guard("get_controller_state", self._client.get_controller_state))

    def get_operation_mode(self) -> str:
        """Return the operating mode, wrapping a client failure."""
        return cast(str, self._guard("get_operation_mode", self._client.get_operation_mode))

    def get_execution_state(self) -> object:
        """Return the RAPID execution state (opaque here), wrapping a failure."""
        return self._guard("get_execution_state", self._client.get_execution_state)

    def get_digital_io(self, signal: str, network: str = _DEFAULT_IO_NETWORK, unit: str = _DEFAULT_IO_UNIT) -> int:
        """Read a digital signal."""
        return cast(int, self._guard("get_digital_io", lambda: self._client.get_digital_io(signal, network, unit)))

    def set_digital_io(self, signal: str, value: int, network: str = _DEFAULT_IO_NETWORK, unit: str = _DEFAULT_IO_UNIT) -> None:
        """Set a digital signal (the caller reads back to confirm)."""
        self._guard("set_digital_io", lambda: self._client.set_digital_io(signal, value, network, unit))

    def get_analog_io(self, signal: str, network: str = _DEFAULT_IO_NETWORK, unit: str = _DEFAULT_IO_UNIT) -> float:
        """Read an analog signal."""
        return cast(float, self._guard("get_analog_io", lambda: self._client.get_analog_io(signal, network, unit)))

    def set_analog_io(self, signal: str, value: float, network: str = _DEFAULT_IO_NETWORK, unit: str = _DEFAULT_IO_UNIT) -> None:
        """Set an analog signal (the caller reads back to confirm)."""
        self._guard("set_analog_io", lambda: self._client.set_analog_io(signal, value, network, unit))

    def start(self, cycle: str = _DEFAULT_START_CYCLE, tasks: Sequence[str] = _DEFAULT_START_TASKS) -> None:
        """Start RAPID execution for ``tasks`` under ``cycle``."""
        self._guard("start", lambda: self._client.start(cycle, list(tasks)))

    def stop(self) -> None:
        """Stop RAPID execution; instant and unconditional."""
        self._guard("stop", self._client.stop)

    def resetpp(self) -> None:
        """Reset the program pointer to main."""
        self._guard("resetpp", self._client.resetpp)

    def is_alive(self) -> bool:
        """Return True if the controller answers, False on any transport/RWS failure.

        Liveness converts a failure into ``False`` (the owner acts on it by
        reconnecting) rather than raising; ``requests`` transport errors subclass
        ``OSError``, so both they and ``ABBException`` are caught.
        """
        try:
            self._client.get_operation_mode()
        except (ABBException, OSError):
            return False
        return True

    def logout(self) -> None:
        """Tear the session down, closing the underlying HTTP session too.

        Delegates to ``RWS.close`` (not bare ``logout``) so the requests session
        is closed as well and the owner's single logout on close never leaks it.
        """
        self._guard("close", self._client.close)


def build_rws_session_factory(
    endpoint: str,
    rw_version: RobotWareVersion,
    username: str,
    password: str,
) -> Callable[[], ControllerSession]:
    """Build a zero-argument factory that connects one RWS session.

    The RobotWare version is passed explicitly so RWS never runs its blocking
    auto-detect probe. Construction itself is cheap (no network until the first
    call); a construction failure is surfaced as :class:`ControllerConnectionError`.

    Args:
        endpoint: Controller base URL (e.g. RobotStudio ``http://127.0.0.1:80``).
        rw_version: Explicit ``RobotWareVersion`` (RW6 or RW7).
        username: Resolved controller username (credential resolution is upstream).
        password: Resolved controller password.

    Returns:
        A factory suitable for ``ControllerOwner``/``acquire_owner``.
    """

    def factory() -> ControllerSession:
        try:
            client = RWS(base_url=endpoint, username=username, password=password, version=rw_version)
        except (ABBException, OSError) as exc:
            raise ControllerConnectionError("Could not construct RWS for {}: {}".format(endpoint, exc)) from exc
        return RwsControllerSession(client)

    return factory
