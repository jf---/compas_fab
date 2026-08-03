import pytest

from compas_fab.backends.tesseract.errors import InvalidTesseractTargetError
from compas_fab.backends.tesseract.native_quantities import NativeJointAccelerations
from compas_fab.backends.tesseract.native_quantities import NativeJointNames
from compas_fab.backends.tesseract.native_quantities import NativeJointPositions
from compas_fab.backends.tesseract.native_quantities import NativeJointVelocities
from compas_fab.backends.tesseract.native_quantities import NativeTime


def test_native_joint_quantities_are_distinct_validated_types():
    positions = NativeJointPositions.build([1.0, 2.0])
    velocities = NativeJointVelocities.build([0.1, 0.2])
    accelerations = NativeJointAccelerations.build([0.01, 0.02])
    names = NativeJointNames.build(["joint_1", "joint_2"], 2)
    time = NativeTime.build(1.5)

    assert type(positions) is NativeJointPositions
    assert type(velocities) is NativeJointVelocities
    assert type(accelerations) is NativeJointAccelerations
    assert type(names) is NativeJointNames
    assert type(time) is NativeTime
    assert positions.values == (1.0, 2.0)
    assert time.value == 1.5


@pytest.mark.parametrize(
    ("factory", "value"),
    [
        (NativeJointPositions.build, []),
        (NativeJointPositions.build, [True]),
        (NativeJointVelocities.build, [float("nan")]),
        (NativeJointAccelerations.build, [[0.0]]),
        (NativeTime.build, -1.0),
    ],
)
def test_native_quantities_fail_loudly_on_invalid_values(factory, value):
    with pytest.raises(InvalidTesseractTargetError):
        factory(value)


def test_native_quantity_raw_constructor_cannot_bypass_invariants():
    with pytest.raises(InvalidTesseractTargetError):
        NativeJointPositions((float("nan"),))
