from attrs import evolve
import pytest

from compas_fab.ghpython.component_identity import CanonicalField, ComponentInputIdentity, ComponentInstanceId, InvalidComponentIdentityError
from compas_fab.ghpython.current_output import ComputeDecision, CurrentOutputState, SupersededComponentOutputError


def identity(value: str) -> ComponentInputIdentity:
    return ComponentInputIdentity.build(
        ComponentInstanceId.build("node"),
        "test/v1",
        (CanonicalField.text("value", value),),
    )


def test_identity_is_ordered_length_prefixed_and_raw_safe() -> None:
    first = (CanonicalField.text("x", "a|b"), CanonicalField.bytes("y", b"c"))
    second = (CanonicalField.text("x", "a"), CanonicalField.bytes("y", b"b|c"))
    left = ComponentInputIdentity.build(ComponentInstanceId.build("node"), "s/v1", first)
    right = ComponentInputIdentity.build(ComponentInstanceId.build("node"), "s/v1", second)
    assert left.digest != right.digest
    with pytest.raises(InvalidComponentIdentityError):
        evolve(left, digest="0" * 64)


def test_rising_edge_change_failure_and_late_publication() -> None:
    state = CurrentOutputState[str].build()
    first = identity("first")
    second = identity("second")
    assert state.observe(first, False) is ComputeDecision.IDLE
    assert state.observe(first, True) is ComputeDecision.EXECUTE
    state.publish(first, "current")
    assert state.observe(first, True) is ComputeDecision.CURRENT
    assert state.observe(second, True) is ComputeDecision.CLEARED
    with pytest.raises(SupersededComponentOutputError):
        state.publish(first, "late")
    assert state.current(second) is None
    state.fail(second)


def test_clear_restores_initial_edge_state() -> None:
    state = CurrentOutputState[str].build()
    first = identity("first")
    assert state.observe(first, True) is ComputeDecision.EXECUTE
    state.publish(first, "current")

    state.clear()

    assert state.current(first) is None
    assert state.observe(first, False) is ComputeDecision.IDLE
    assert state.observe(first, True) is ComputeDecision.EXECUTE
