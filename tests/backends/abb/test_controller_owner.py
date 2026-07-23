"""Behavioural tests for the serialized ABB controller owner and registry.

The load-bearing property under test is that the single-worker serialization
turns the ``FireOnceLedger``'s check-then-act ``consume`` into a race-free
linearization point: even when many threads fire the *same* armed command at
once, exactly one wins and the physical mutation runs exactly once.
"""

import threading

import pytest

from compas_fab.backends.abb.arming import ArmedCommand
from compas_fab.backends.abb.arming import ControllerId
from compas_fab.backends.abb.arming import FireReceipt
from compas_fab.backends.abb.arming import IntentDigest
from compas_fab.backends.abb.arming import InvocationNonce
from compas_fab.backends.abb.arming import ObservationRevision
from compas_fab.backends.abb.arming import verify_readback
from compas_fab.backends.abb.controller_owner import ControllerOwner
from compas_fab.backends.abb.errors import CommandAlreadyConsumedError
from compas_fab.backends.abb.errors import CommandArmExpiredError
from compas_fab.backends.abb.errors import ControllerClosedError
from compas_fab.backends.abb.errors import ControllerConnectionError
from compas_fab.backends.abb.errors import ReadbackMismatchError
from compas_fab.backends.abb.registry import acquire_owner


# ------------------------------------------------------------------- fixtures


def _controller(endpoint: str = "http://192.168.125.1:80") -> ControllerId:
    return ControllerId.build(endpoint=endpoint, rw_version="7.9", credential_handle="operator-handle")


def _intent() -> IntentDigest:
    return IntentDigest.build("set_digital_io", b"DO1=1")


def _armed(controller: ControllerId, nonce: str = "edge-1", revision: int = 0) -> ArmedCommand:
    return ArmedCommand.arm(_intent(), controller, ObservationRevision.build(revision), InvocationNonce.build(nonce))


class FakeSession:
    """Records every call plus the thread it ran on; canned controller reads.

    ``apply_writes=False`` models a controller that acknowledges a set but does
    not reflect it on read-back, so ``verify_readback`` can observe a mismatch.
    """

    def __init__(self, controller_state: str = "motoron", digital=None, apply_writes: bool = True, alive: bool = True) -> None:
        self.logout_count = 0
        self.mutation_count = 0
        self.calls = []  # list of (method_name, thread_ident)
        self._controller_state = controller_state
        self._digital = dict(digital or {})
        self._apply_writes = apply_writes
        self._alive = alive

    def _record(self, name: str) -> None:
        self.calls.append((name, threading.get_ident()))

    def method_idents(self, name: str):
        return [ident for (called, ident) in self.calls if called == name]

    def get_controller_state(self) -> str:
        self._record("get_controller_state")
        return self._controller_state

    def get_operation_mode(self) -> str:
        self._record("get_operation_mode")
        return "AUTO"

    def get_execution_state(self) -> object:
        self._record("get_execution_state")
        return "stopped"

    def get_digital_io(self, signal, network="Local", unit="DRV_1") -> int:
        self._record("get_digital_io")
        return int(self._digital.get(signal, 0))

    def set_digital_io(self, signal, value, network="Local", unit="DRV_1") -> None:
        self.mutation_count += 1
        self._record("set_digital_io")
        if self._apply_writes:
            self._digital[signal] = int(bool(value))

    def get_analog_io(self, signal, network="Local", unit="DRV_1") -> float:
        self._record("get_analog_io")
        return 0.0

    def set_analog_io(self, signal, value, network="Local", unit="DRV_1") -> None:
        self.mutation_count += 1
        self._record("set_analog_io")

    def start(self, cycle="asis", tasks=("T_ROB1",)) -> None:
        self.mutation_count += 1
        self._record("start")

    def stop(self) -> None:
        self.mutation_count += 1
        self._record("stop")

    def resetpp(self) -> None:
        self.mutation_count += 1
        self._record("resetpp")

    def is_alive(self) -> bool:
        self._record("is_alive")
        return self._alive

    def logout(self) -> None:
        self.logout_count += 1
        self._record("logout")


def _handing_factory(*sessions: FakeSession):
    """A factory that hands out the given sessions in order, one per call."""
    iterator = iter(sessions)

    def factory() -> FakeSession:
        return next(iterator)

    return factory


# --------------------------------------------- HEADLINE: fire-once under load


def test_fire_once_under_concurrency_runs_mutation_exactly_once() -> None:
    # THE test that matters: ONE armed command, fired from MANY threads at once.
    # The single-worker executor serializes ledger.consume so the check-then-act
    # is race-free -> exactly one FireReceipt, all others rejected, robot mutated once.
    controller = _controller()
    fake = FakeSession()
    owner = ControllerOwner(controller, lambda: fake)
    armed = _armed(controller, nonce="button-edge-42")

    thread_count = 32
    barrier = threading.Barrier(thread_count)
    lock = threading.Lock()
    receipts = []
    rejections = []

    def worker() -> None:
        barrier.wait()  # release all threads into submit_mutation simultaneously
        try:
            receipt, _ = owner.submit_mutation(armed, lambda session: session.set_digital_io("DO1", 1))
        except CommandAlreadyConsumedError as exc:
            with lock:
                rejections.append(exc)
        else:
            with lock:
                receipts.append(receipt)

    threads = [threading.Thread(target=worker) for _ in range(thread_count)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()

    assert len(receipts) == 1
    assert isinstance(receipts[0], FireReceipt)
    assert len(rejections) == thread_count - 1
    assert fake.mutation_count == 1  # the physical set fired exactly once
    owner.close()


# ------------------------------------------------------------- submit_read


def test_submit_read_returns_value_and_runs_off_the_calling_thread() -> None:
    controller = _controller()
    fake = FakeSession(controller_state="guardstop")
    owner = ControllerOwner(controller, lambda: fake)

    state = owner.submit_read(lambda session: session.get_controller_state())

    assert state == "guardstop"
    read_idents = fake.method_idents("get_controller_state")
    assert read_idents
    assert all(ident != threading.get_ident() for ident in read_idents)
    owner.close()


def test_submit_read_never_touches_the_ledger_so_it_can_repeat() -> None:
    controller = _controller()
    fake = FakeSession()
    owner = ControllerOwner(controller, lambda: fake)

    for _ in range(3):
        assert owner.submit_read(lambda session: session.get_operation_mode()) == "AUTO"
    owner.close()


# ------------------------------------------------------------ submit_mutation


def test_submit_mutation_returns_receipt_and_operation_result() -> None:
    controller = _controller()
    fake = FakeSession()
    owner = ControllerOwner(controller, lambda: fake)

    receipt, result = owner.submit_mutation(armed_command=_armed(controller), operation=lambda session: (session.start(), "started")[1])

    assert isinstance(receipt, FireReceipt)
    assert receipt.controller_id == controller
    assert result == "started"
    assert fake.mutation_count == 1
    owner.close()


def test_already_consumed_command_does_not_run_the_operation() -> None:
    controller = _controller()
    fake = FakeSession()
    owner = ControllerOwner(controller, lambda: fake)
    armed = _armed(controller)

    owner.submit_mutation(armed, lambda session: session.set_digital_io("DO1", 1))
    assert fake.mutation_count == 1

    with pytest.raises(CommandAlreadyConsumedError):
        owner.submit_mutation(armed, lambda session: session.set_digital_io("DO1", 1))
    assert fake.mutation_count == 1  # the replay's operation never ran
    owner.close()


def test_expired_arm_is_caught_before_submit_and_never_runs_the_operation() -> None:
    # require_current is the caller's pre-submit guard; a stale arm must raise
    # before submit_mutation is ever reached, so the operation cannot run.
    controller = _controller()
    fake = FakeSession()
    owner = ControllerOwner(controller, lambda: fake)
    armed = _armed(controller, revision=4)

    with pytest.raises(CommandArmExpiredError):
        armed.require_current(ObservationRevision.build(5))
    assert fake.mutation_count == 0
    owner.close()


# ------------------------------------------------------------- readback


def test_mutation_readback_passes_when_controller_reflects_the_command() -> None:
    controller = _controller()
    fake = FakeSession(digital={"DO1": 0}, apply_writes=True)
    owner = ControllerOwner(controller, lambda: fake)

    def op(session) -> int:
        session.set_digital_io("DO1", 1)
        observed = session.get_digital_io("DO1")
        verify_readback(1, observed)
        return observed

    receipt, result = owner.submit_mutation(_armed(controller), op)
    assert isinstance(receipt, FireReceipt)
    assert result == 1
    owner.close()


def test_mutation_readback_raises_when_controller_does_not_reflect_the_command() -> None:
    controller = _controller()
    fake = FakeSession(digital={"DO1": 0}, apply_writes=False)
    owner = ControllerOwner(controller, lambda: fake)

    def op(session) -> None:
        session.set_digital_io("DO1", 1)
        verify_readback(1, session.get_digital_io("DO1"))  # fake still reads 0

    with pytest.raises(ReadbackMismatchError):
        owner.submit_mutation(_armed(controller), op)
    owner.close()


# ----------------------------------------------------------------- close


def test_close_shuts_down_logs_out_once_and_blocks_further_submits() -> None:
    controller = _controller()
    fake = FakeSession()
    owner = ControllerOwner(controller, lambda: fake)
    owner.connect()

    owner.close()
    assert fake.logout_count == 1

    owner.close()  # idempotent
    assert fake.logout_count == 1

    with pytest.raises(ControllerClosedError):
        owner.submit_read(lambda session: session.get_controller_state())
    with pytest.raises(ControllerClosedError):
        owner.submit_mutation(_armed(controller), lambda session: session.stop())


def test_close_without_a_connection_does_not_log_out() -> None:
    controller = _controller()
    fake = FakeSession()
    owner = ControllerOwner(controller, lambda: fake)

    owner.close()
    assert fake.logout_count == 0


# --------------------------------------------------------------- reconnect


def test_reconnect_logs_out_old_builds_fresh_and_preserves_the_ledger() -> None:
    controller = _controller()
    first, second = FakeSession(), FakeSession()
    owner = ControllerOwner(controller, _handing_factory(first, second))
    armed = _armed(controller, nonce="edge-1")

    owner.submit_mutation(armed, lambda session: session.set_digital_io("DO1", 1))
    assert first.mutation_count == 1

    owner.reconnect()
    assert first.logout_count == 1  # old session logged out

    owner.submit_read(lambda session: session.get_controller_state())
    assert second.method_idents("get_controller_state")  # fresh session now in use

    # A nonce already fired stays fired across the reconnect: the ledger is not reset.
    with pytest.raises(CommandAlreadyConsumedError):
        owner.submit_mutation(armed, lambda session: session.set_digital_io("DO1", 1))
    assert second.mutation_count == 0  # replay never touched the fresh session

    owner.close()
    assert second.logout_count == 1


# ---------------------------------------------------------- connect failures


def test_failed_lazy_connect_raises_controller_connection_error() -> None:
    controller = _controller()

    def exploding_factory():
        raise RuntimeError("controller unreachable")

    owner = ControllerOwner(controller, exploding_factory)
    with pytest.raises(ControllerConnectionError):
        owner.submit_read(lambda session: session.get_controller_state())
    owner.close()


# ---------------------------------------------------------------- registry


def test_acquire_owner_reuses_the_same_owner_for_the_same_id() -> None:
    sticky = {}
    controller = _controller()
    first = acquire_owner(sticky, "abb-slot", controller, lambda: FakeSession())
    second = acquire_owner(sticky, "abb-slot", controller, lambda: FakeSession())

    assert first is second
    assert sticky["abb-slot"] is first
    first.close()


def test_acquire_owner_replaces_and_closes_the_old_on_id_change() -> None:
    sticky = {}
    old_session = FakeSession()
    old = acquire_owner(sticky, "abb-slot", _controller("http://a:80"), lambda: old_session)
    old.connect()  # give the old owner a live session to log out

    new = acquire_owner(sticky, "abb-slot", _controller("http://b:80"), lambda: FakeSession())

    assert new is not old
    assert old_session.logout_count == 1  # replacing closed the old session
    assert sticky["abb-slot"] is new
    new.close()


def test_acquire_owner_creates_when_the_slot_is_absent() -> None:
    sticky = {}
    owner = acquire_owner(sticky, "abb-slot", _controller(), lambda: FakeSession())

    assert isinstance(owner, ControllerOwner)
    assert sticky["abb-slot"] is owner
    owner.close()
