"""The structural contract the controller owner drives a live session through.

``ControllerSession`` is the exact subset of ``abb_robot_client.rws.RWS`` (and
its async twin, adapted) that the owner ever calls. Typing the owner against
this ``Protocol`` — never against the concrete client — keeps the RWS library
out of the owner's imports, so the safety layer stays testable with a fake and
free of any transport dependency. Method signatures mirror the real ``RWS``
(see ``.superpowers/sdd/abb-api-map.md`` sections 3-5); ``...`` marks parameters
whose defaults live in the concrete client, not in this contract.
"""

from __future__ import annotations

from typing import Protocol
from typing import Sequence
from typing import runtime_checkable


@runtime_checkable
class ControllerSession(Protocol):
    """A connected controller session the owner reads from and mutates.

    A ``ControllerSession`` is single-connection and NOT thread-safe: the owner
    is responsible for driving every call from its one worker thread. The
    ``Protocol`` declares only what the owner uses, so any object with these
    methods — the real ``RWS`` adapter or a test fake — satisfies it.
    """

    # --- state reads -----------------------------------------------------

    def get_controller_state(self) -> str:
        """Return the controller state, e.g. ``'motoron'`` or ``'guardstop'``."""
        ...

    def get_operation_mode(self) -> str:
        """Return the operating mode, e.g. ``'AUTO'``, ``'MANR'``, ``'MANF'``."""
        ...

    def get_execution_state(self) -> object:
        """Return the RAPID execution state (opaque to the owner)."""
        ...

    # --- I/O -------------------------------------------------------------

    def get_digital_io(self, signal: str, network: str = ..., unit: str = ...) -> int:
        """Read a digital signal's current value."""
        ...

    def set_digital_io(self, signal: str, value: int, network: str = ..., unit: str = ...) -> None:
        """Set a digital signal; read it back to confirm before belief."""
        ...

    def get_analog_io(self, signal: str, network: str = ..., unit: str = ...) -> float:
        """Read an analog signal's current value."""
        ...

    def set_analog_io(self, signal: str, value: float, network: str = ..., unit: str = ...) -> None:
        """Set an analog signal; read it back to confirm before belief."""
        ...

    # --- RAPID execution -------------------------------------------------

    def start(self, cycle: str = ..., tasks: Sequence[str] = ...) -> None:
        """Start RAPID execution for the given tasks under the given cycle."""
        ...

    def stop(self) -> None:
        """Stop RAPID execution. Stop is instant and unconditional."""
        ...

    def resetpp(self) -> None:
        """Reset the program pointer to main."""
        ...

    # --- lifecycle / health ---------------------------------------------

    def is_alive(self) -> bool:
        """Cheap liveness probe (a concrete adapter reads controller state)."""
        ...

    def logout(self) -> None:
        """Tear the session down; the owner calls this exactly once on close."""
        ...
