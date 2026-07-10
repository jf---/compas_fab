from concurrent.futures import ThreadPoolExecutor
from threading import Lock
from time import sleep

import numpy as np
import pytest
from tesseract_robotics.emitters.rapid import EmptyProgramError
from tesseract_robotics.emitters.rapid import MissingProfileError
from tesseract_robotics.emitters.rapid import RapidProfile
from tesseract_robotics.emitters.rapid import UnsupportedInstructionError
from tesseract_robotics.emitters.rapid import emit_rapid
from tesseract_robotics.planning import Pose
from tesseract_robotics.tesseract_command_language import CartesianWaypoint
from tesseract_robotics.tesseract_command_language import CartesianWaypointPoly_wrap_CartesianWaypoint
from tesseract_robotics.tesseract_command_language import CompositeInstruction
from tesseract_robotics.tesseract_command_language import JointWaypoint
from tesseract_robotics.tesseract_command_language import JointWaypointPoly_wrap_JointWaypoint
from tesseract_robotics.tesseract_command_language import MoveInstruction
from tesseract_robotics.tesseract_command_language import MoveInstructionPoly_wrap_MoveInstruction
from tesseract_robotics.tesseract_command_language import MoveInstructionType_CIRCULAR
from tesseract_robotics.tesseract_command_language import MoveInstructionType_FREESPACE
from tesseract_robotics.tesseract_command_language import MoveInstructionType_LINEAR
from tesseract_robotics.tesseract_command_language import ProfileDictionary
from tesseract_robotics.tesseract_command_language import SetAnalogInstruction
from tesseract_robotics.tesseract_command_language import SetDigitalInstruction
from tesseract_robotics.tesseract_command_language import SetToolInstruction
from tesseract_robotics.tesseract_command_language import TimerInstruction
from tesseract_robotics.tesseract_command_language import TimerInstructionType
from tesseract_robotics.tesseract_command_language import WaitInstruction

from compas_fab.backends.tesseract.errors import InvalidRapidProgramError
from compas_fab.backends.tesseract.errors import InvalidRapidProgramNameError
from compas_fab.backends.tesseract.native import TesseractPlanningRequest
from compas_fab.backends.tesseract.rapid_emitter import TesseractRapidEmitter
from compas_fab.robots import JointTrajectory

PROFILE = RapidProfile()
PROFILES = {"P": PROFILE}
THREAD_COUNT = 8
OVERLAP_WINDOW_SECONDS = 0.02


def _cartesian_move(move_type=MoveInstructionType_LINEAR, profile="P"):
    pose = Pose.from_xyz_quat([0.5, -0.2, 0.62], [0.0, 0.0, 0.0, 1.0])
    waypoint = CartesianWaypointPoly_wrap_CartesianWaypoint(CartesianWaypoint(pose))
    return MoveInstructionPoly_wrap_MoveInstruction(MoveInstruction(waypoint, move_type, profile))


def _joint_move():
    waypoint = JointWaypoint()
    waypoint.setNames(["j1", "j2", "j3", "j4", "j5", "j6"])
    waypoint.setPosition(np.asarray([0.0, 0.5, 1.0, -0.5, -1.0, 0.25], dtype=np.float64))
    poly = JointWaypointPoly_wrap_JointWaypoint(waypoint)
    return MoveInstructionPoly_wrap_MoveInstruction(MoveInstruction(poly, MoveInstructionType_LINEAR, "P"))


def _program(*instructions):
    composite = CompositeInstruction("rapid")
    for instruction in instructions:
        composite.push_back(instruction)
    return composite


def _linear_program():
    return _program(_cartesian_move())


def _freespace_program():
    return _program(_cartesian_move(MoveInstructionType_FREESPACE))


def _joint_program():
    return _program(_joint_move())


def _nested_program():
    return _program(_program(_cartesian_move()), _cartesian_move())


def _io_program():
    return _program(
        WaitInstruction(1.5),
        TimerInstruction(TimerInstructionType.DIGITAL_OUTPUT_HIGH, 2.0, 7),
        SetDigitalInstruction("do_torch", 0, True),
        SetAnalogInstruction("ao_voltage", 0, 24.5),
        SetToolInstruction(7),
    )


@pytest.mark.parametrize(
    ("program_factory", "profiles"),
    [
        (_linear_program, PROFILES),
        (_freespace_program, PROFILES),
        (_joint_program, PROFILES),
        (_nested_program, PROFILES),
        (_io_program, {}),
    ],
)
def test_adapter_output_is_byte_identical_to_native(program_factory, profiles):
    native_program = program_factory()
    expected = emit_rapid(native_program, profiles, module_name="Cell", proc_name="Run")

    actual = TesseractRapidEmitter.emit(native_program, profiles, "Cell", "Run")

    assert actual.source == expected
    assert actual.program is native_program
    assert actual.module_name == "Cell"
    assert actual.procedure_name == "Run"


def test_adapter_rejects_planning_request_trajectory_and_arbitrary_object():
    native_program = _io_program()
    request = TesseractPlanningRequest.build(
        native_program,
        "Pipeline",
        ProfileDictionary(),
        False,
    )

    for invalid in (request, JointTrajectory(), object()):
        with pytest.raises(InvalidRapidProgramError):
            TesseractRapidEmitter.emit(invalid, {})


@pytest.mark.parametrize(
    ("module_name", "procedure_name"),
    [("", "main"), ("M", "   "), (object(), "main")],
)
def test_invalid_rapid_names_fail_before_native_call(module_name, procedure_name):
    with pytest.raises(InvalidRapidProgramNameError):
        TesseractRapidEmitter.emit(_linear_program(), PROFILES, module_name, procedure_name)


def test_native_missing_profile_error_propagates_unchanged():
    with pytest.raises(MissingProfileError) as caught:
        TesseractRapidEmitter.emit(_linear_program(), {})

    assert type(caught.value) is MissingProfileError


def test_native_unsupported_instruction_error_propagates_unchanged():
    circular = _program(_cartesian_move(MoveInstructionType_CIRCULAR))

    with pytest.raises(UnsupportedInstructionError) as caught:
        TesseractRapidEmitter.emit(circular, PROFILES)

    assert type(caught.value) is UnsupportedInstructionError


def test_native_empty_program_error_propagates_unchanged():
    with pytest.raises(EmptyProgramError) as caught:
        TesseractRapidEmitter.emit(CompositeInstruction("empty"), {})

    assert type(caught.value) is EmptyProgramError


def test_adapter_serializes_native_global_writer(monkeypatch):
    state_lock = Lock()
    active_calls = 0
    maximum_active_calls = 0

    def instrumented_emit(program, profiles, *, module_name, proc_name):
        nonlocal active_calls, maximum_active_calls
        with state_lock:
            active_calls += 1
            maximum_active_calls = max(maximum_active_calls, active_calls)
        sleep(OVERLAP_WINDOW_SECONDS)
        with state_lock:
            active_calls -= 1
        return "MODULE {}\n  PROC {}()\n  ENDPROC\nENDMODULE\n".format(module_name, proc_name)

    monkeypatch.setattr("compas_fab.backends.tesseract.rapid_emitter.emit_rapid", instrumented_emit)

    with ThreadPoolExecutor(max_workers=THREAD_COUNT) as executor:
        programs = list(executor.map(lambda _: TesseractRapidEmitter.emit(_io_program(), {}), range(THREAD_COUNT)))

    assert len(programs) == THREAD_COUNT
    assert maximum_active_calls == 1
