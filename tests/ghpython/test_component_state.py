from attrs import evolve
import pytest

from compas_fab.ghpython.component_identity import CanonicalField, ComponentInputIdentity, ComponentInstanceId, InvalidComponentIdentityError
from compas_fab.ghpython.current_output import ComputeDecision, CurrentOutputState, InvalidCurrentOutputTransitionError, SupersededComponentOutputError


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
    reversed_fields = ComponentInputIdentity.build(ComponentInstanceId.build("node"), "s/v1", tuple(reversed(first)))
    assert left.digest == "1ff68a7816dd2a33ecc742567cadc64e6264611150b30dd2fcbb36886863c3a5"
    assert left.digest != right.digest
    assert left.digest != reversed_fields.digest
    with pytest.raises(InvalidComponentIdentityError):
        evolve(left, digest="0" * 64)


def test_raw_identity_requires_exact_immutable_canonical_types() -> None:
    class DerivedComponentInstanceId(ComponentInstanceId):
        pass

    class DerivedCanonicalField(CanonicalField):
        pass

    valid = ComponentInputIdentity.build(
        ComponentInstanceId.build("node"),
        "s/v1",
        (CanonicalField.text("x", "value"),),
    )
    with pytest.raises(InvalidComponentIdentityError):
        ComponentInputIdentity(valid.component, valid.schema, list(valid.fields), valid.digest)
    with pytest.raises(InvalidComponentIdentityError):
        ComponentInputIdentity(DerivedComponentInstanceId("node"), valid.schema, valid.fields, valid.digest)
    with pytest.raises(InvalidComponentIdentityError):
        ComponentInputIdentity(valid.component, valid.schema, (DerivedCanonicalField("x", b"value"),), valid.digest)


def test_identity_factories_reject_invalid_runtime_types_with_named_error() -> None:
    component = ComponentInstanceId.build("node")
    field = CanonicalField.text("x", "value")
    with pytest.raises(InvalidComponentIdentityError):
        CanonicalField.bytes("payload", 4)
    with pytest.raises(InvalidComponentIdentityError):
        ComponentInputIdentity.build("node", "s/v1", (field,))
    with pytest.raises(InvalidComponentIdentityError):
        ComponentInputIdentity.build(component, b"s/v1", (field,))
    with pytest.raises(InvalidComponentIdentityError):
        ComponentInputIdentity.build(component, "s/v1", object())
    with pytest.raises(InvalidComponentIdentityError):
        ComponentInputIdentity.build(component, "s/v1", (object(),))
    assert ComponentInputIdentity.build(component, "s/v1", [field]).fields == (field,)


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
    state.publish(second, "second current")
    assert state.current(second) == "second current"
    state.fail(second)
    assert state.current(second) is None
    with pytest.raises(SupersededComponentOutputError):
        state.fail(first)


def test_clear_restores_initial_edge_state() -> None:
    state = CurrentOutputState[str].build()
    first = identity("first")
    assert state.observe(first, True) is ComputeDecision.EXECUTE
    state.publish(first, "current")

    state.clear()

    assert state.current(first) is None
    assert state.observe(first, False) is ComputeDecision.IDLE
    assert state.observe(first, True) is ComputeDecision.EXECUTE


def test_publish_before_observation_fails_named() -> None:
    state = CurrentOutputState[str].build()

    with pytest.raises(InvalidCurrentOutputTransitionError):
        state.publish(identity("first"), "unobserved")


@pytest.mark.parametrize("operation", ("observe", "publish", "fail", "current"))
@pytest.mark.parametrize("invalid_identity", (None, object()))
def test_state_operations_reject_identity_free_values_with_named_error(operation, invalid_identity) -> None:
    state = CurrentOutputState[str].build()

    with pytest.raises(InvalidCurrentOutputTransitionError):
        if operation == "observe":
            state.observe(invalid_identity, False)
        elif operation == "publish":
            state.publish(invalid_identity, "value")
        elif operation == "fail":
            state.fail(invalid_identity)
        else:
            state.current(invalid_identity)


@pytest.mark.parametrize("operation", ("observe", "publish", "fail", "current"))
def test_state_operations_reject_non_exact_identity_subclasses(operation) -> None:
    class DerivedComponentInputIdentity(ComponentInputIdentity):
        pass

    valid = identity("first")
    malformed = DerivedComponentInputIdentity(valid.component, valid.schema, valid.fields, valid.digest)
    state = CurrentOutputState[str].build()

    with pytest.raises(InvalidCurrentOutputTransitionError):
        if operation == "observe":
            state.observe(malformed, False)
        elif operation == "publish":
            state.publish(malformed, "value")
        elif operation == "fail":
            state.fail(malformed)
        else:
            state.current(malformed)


@pytest.mark.parametrize("operation", ("observe", "publish", "fail", "current"))
def test_state_operations_reject_uninitialized_exact_identities(operation) -> None:
    malformed = object.__new__(ComponentInputIdentity)
    state = CurrentOutputState[str].build()

    with pytest.raises(InvalidCurrentOutputTransitionError):
        if operation == "observe":
            state.observe(malformed, False)
        elif operation == "publish":
            state.publish(malformed, "value")
        elif operation == "fail":
            state.fail(malformed)
        else:
            state.current(malformed)
