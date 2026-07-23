"""The serialized owner that mediates every command to one ABB controller.

Grasshopper may call a component from its UI thread while asynchronous work is
in flight, so two threads can reach the fire-once ledger at once. The ledger's
``consume`` is a check-then-act (``if nonce in set: raise; set.add(nonce)``); it
is race-free ONLY if serialized. This owner is that serialization: every session
and ledger operation runs inside a callable submitted to a single-worker
``ThreadPoolExecutor``. One worker means ``consume`` has one linearization
point, so even simultaneous submits of the same armed command fire the robot at
most once. A racy fire-once guard could double-fire a physical robot; the single
worker is the whole point of this class.
"""

from __future__ import annotations

import threading
from concurrent.futures import Future
from concurrent.futures import ThreadPoolExecutor
from typing import Callable
from typing import Optional
from typing import Tuple
from typing import TypeVar

from .arming import ArmedCommand
from .arming import ControllerId
from .arming import FireOnceLedger
from .arming import FireReceipt
from .errors import AbbControllerError
from .errors import ControllerClosedError
from .errors import ControllerConnectionError
from .session import ControllerSession

# One worker is a safety requirement, not a performance tuning knob: it is what
# serializes FireOnceLedger.consume so its check-then-act cannot race and
# double-fire the robot. Do not raise this.
_SERIALIZED_WORKER_COUNT = 1

# Names the owner's worker thread so a stack trace during a robot command is
# attributable to this serialization layer rather than an anonymous pool thread.
_WORKER_THREAD_NAME_PREFIX = "abb-controller-owner"

T = TypeVar("T")


class ControllerOwner:
    """Owns one controller session and serializes all access to it.

    The owner is constructed cheaply and connects lazily: the session factory is
    invoked on the first read/mutation (or an explicit :meth:`connect`). It holds
    a :class:`FireOnceLedger` scoped to ``controller_id`` and runs every session
    and ledger touch on a single worker thread, which is what makes the ledger's
    fire-once guard race-free.

    The owner is stateful and NOT frozen. It is safe to call its ``submit_*``,
    :meth:`reconnect`, and :meth:`close` methods from any thread; they marshal
    onto the worker (or, for :meth:`close`, drain it) under a lifecycle lock.
    """

    def __init__(self, controller_id: ControllerId, session_factory: Callable[[], ControllerSession]) -> None:
        """Create an owner for one controller, not yet connected.

        Args:
            controller_id: Identity of the controller this owner mediates; also
                scopes the fire-once ledger.
            session_factory: Zero-argument builder of a fresh
                :class:`ControllerSession`. Called on first use, on
                :meth:`connect`, and on :meth:`reconnect`. Never called
                concurrently — the worker thread is the only caller.
        """
        self._controller_id = controller_id
        self._session_factory = session_factory
        self._ledger = FireOnceLedger.build(controller_id)
        # max_workers=1 is the serialization primitive (see module docstring).
        self._executor = ThreadPoolExecutor(max_workers=_SERIALIZED_WORKER_COUNT, thread_name_prefix=_WORKER_THREAD_NAME_PREFIX)
        self._session: Optional[ControllerSession] = None
        # Guards the submit gate and the close transition against each other so
        # a submit can never reach an already-shut-down executor.
        self._lifecycle_lock = threading.Lock()
        self._closed = False

    @property
    def controller_id(self) -> ControllerId:
        """The identity of the controller this owner mediates."""
        return self._controller_id

    # -- worker-thread helpers (only ever run inside a submitted task) --------

    def _connected_session(self) -> ControllerSession:
        """Return the live session, building it on first use. Worker-thread only.

        Runs exclusively on the single worker thread, so the ``None`` check and
        assignment need no lock. A factory failure is wrapped as a named
        :class:`ControllerConnectionError` so a half-built session never escapes.
        """
        if self._session is None:
            try:
                self._session = self._session_factory()
            except AbbControllerError:
                raise
            except Exception as exc:
                raise ControllerConnectionError("Failed to connect controller {}: {}".format(self._controller_id.digest, exc)) from exc
        return self._session

    # -- submit gate ----------------------------------------------------------

    def _submit(self, task: Callable[[], T]) -> "Future[T]":
        """Enqueue ``task`` on the worker thread, refusing once closed.

        The lock makes "check closed, then submit" atomic against
        :meth:`close`'s "set closed, then shut down", so a task is never handed
        to an executor that is already shutting down.
        """
        with self._lifecycle_lock:
            if self._closed:
                raise ControllerClosedError("Controller owner for {} is closed; no further commands accepted.".format(self._controller_id.digest))
            return self._executor.submit(task)

    # -- public API -----------------------------------------------------------

    def connect(self) -> None:
        """Eagerly establish the session, surfacing a connection failure now.

        Raises:
            ControllerConnectionError: The session factory failed.
            ControllerClosedError: The owner is already closed.
        """
        self._submit(self._connected_session).result()

    def submit_read(self, operation: Callable[[ControllerSession], T]) -> T:
        """Run a read against the live session on the worker thread.

        Reads never touch the ledger, so they are repeatable and free of
        fire-once accounting.

        Args:
            operation: Callable given the live session; its result is returned.

        Returns:
            The value ``operation`` returns.

        Raises:
            ControllerConnectionError: The session factory failed.
            ControllerClosedError: The owner is already closed.
        """

        def task() -> T:
            return operation(self._connected_session())

        return self._submit(task).result()

    def submit_mutation(self, armed_command: ArmedCommand, operation: Callable[[ControllerSession], T]) -> Tuple[FireReceipt, T]:
        """Fire a mutation exactly once, then run it against the live session.

        On the worker thread, and only there: ensure the session is live, then
        ``ledger.consume`` the armed command (the fire-once linearization point),
        then run ``operation``. Because consume runs on the single worker, its
        check-then-act cannot race a concurrent submit of the same command.

        Ensuring the session precedes consume deliberately: a transient
        connection failure then raises before the nonce is spent, so the user's
        armed command survives to be retried. Once consume succeeds the nonce is
        spent even if ``operation`` later fails — a fired command is never
        silently retried against a physical robot.

        Args:
            armed_command: The single-use command to fire.
            operation: Callable given the live session, run only after a
                successful consume.

        Returns:
            A ``(receipt, result)`` pair: the fire receipt proving the command
            fired, and whatever ``operation`` returned.

        Raises:
            CommandAlreadyConsumedError: The nonce was already fired (a replay).
            CommandPreconditionError: The command targets a different controller.
            ControllerConnectionError: The session factory failed.
            ControllerClosedError: The owner is already closed.
        """

        def task() -> Tuple[FireReceipt, T]:
            session = self._connected_session()
            receipt = self._ledger.consume(armed_command)
            result = operation(session)
            return receipt, result

        return self._submit(task).result()

    def reconnect(self) -> None:
        """Log out the current session and build a fresh one, keeping the ledger.

        Used for a health-failure recovery or an explicit reconnect. The whole
        swap runs on the worker thread, so no read or mutation can observe a
        half-swapped session. The fire-once ledger is deliberately NOT reset: a
        nonce already fired must stay fired across a reconnect, or a Grasshopper
        recompute could re-fire a command that a reconnect had "forgotten".

        Raises:
            ControllerConnectionError: The fresh session factory failed.
            ControllerClosedError: The owner is already closed.
        """

        def task() -> None:
            if self._session is not None:
                self._session.logout()
                self._session = None
            # Rebuild eagerly so a connection failure surfaces from reconnect
            # itself rather than being deferred to the next command.
            self._connected_session()

        self._submit(task).result()

    def close(self) -> None:
        """Drain the worker, log out once, and refuse all further submits.

        Idempotent. Sets the closed flag under the lifecycle lock (so in-flight
        submits are already enqueued and later ones are refused), shuts the
        executor down waiting for the in-flight task, then reads the session
        after the worker has joined — this sees any session a just-drained task
        created, so no session is ever leaked — and logs it out exactly once.
        """
        with self._lifecycle_lock:
            if self._closed:
                return
            self._closed = True
        # The worker is joined after this returns, establishing a happens-before
        # edge: reading self._session below sees the final value set by any task.
        self._executor.shutdown(wait=True)
        session = self._session
        if session is not None:
            session.logout()
            self._session = None
