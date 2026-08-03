"""End-to-end RunScript contract tests for the Tesseract Grasshopper components.

These tests exercise the thin-adapter surface that the static component tests
cannot reach: they exec each component's ``code.py`` under a faithful fake
Grasshopper host and call ``RunScript`` directly against the REAL Tesseract
backend. Four adapter guarantees are pinned for every component:

1. Argument translation. The native object a component returns is asserted to
   equal what the wrapped backend factory produces for the same inputs (a
   direct backend call), so a wrong argument order, a misplaced default, or a
   dropped translation is caught structurally rather than by string match.
2. ``optional_connected_input`` connection-state semantics. Each optional input
   is toggled connected/unconnected via the host's ``SourceCount`` and the
   connected value is asserted to be used, versus the documented default when
   unconnected -- never Python truthiness of the supplied value.
3. Backend error surfacing. An invalid input drives the wrapped factory to
   raise ``TesseractBackendError``; the component must invoke ``error(...)`` and
   return its exact declared None-shape (a bare ``None`` for a single output,
   an N-tuple of ``None`` for N outputs).
4. Output arity. The success and None-shape returns match the arity declared in
   ``metadata.json`` ``outputParameters``.

Fixtures ``tesseract_robot``, ``tesseract_artifact``, ``one_joint_cell``, and
``one_joint_state`` come from the sibling ``conftest.py`` and load the real
one-joint URDF/SRDF native robot. Heavy execution paths (native planning,
result projection) are covered at the reachable wiring boundary; their deep
execution coverage limits are documented in the module-level comments below.
"""

from pathlib import Path
from types import ModuleType
from types import SimpleNamespace
import json
import sys

import numpy as np
import pytest
from compas.geometry import Frame
from tesseract_robotics.planning import CartesianTarget
from tesseract_robotics.planning import JointTarget
from tesseract_robotics.planning import MoveType
from tesseract_robotics.planning import Pose
from tesseract_robotics.tesseract_command_language import ProfileDictionary

from compas_fab.backends.tesseract.native_pose import WorkingFrameUserUnits
from compas_fab.backends.tesseract.native_pose import pose_from_working_frame
from compas_fab.backends.tesseract.native_program_builder import build_motion_program
from compas_fab.backends.tesseract.native_quantities import NativeJointAccelerations
from compas_fab.backends.tesseract.native_quantities import NativeJointNames
from compas_fab.backends.tesseract.native_quantities import NativeJointPositions
from compas_fab.backends.tesseract.native_quantities import NativeJointVelocities
from compas_fab.backends.tesseract.native_quantities import NativeTime
from compas_fab.backends.tesseract.native_targets import build_cartesian_target
from compas_fab.backends.tesseract.native_targets import cartesian_target_from_native
from compas_fab.backends.tesseract.native_targets import joint_target_from_native
from compas_fab.backends.tesseract.native_targets import move_type_from_name
from compas_fab.backends.tesseract.native_targets import state_target_from_native

COMPONENTS = Path(__file__).parents[3] / "src" / "compas_fab" / "ghpython" / "components_cpython"

_CLASS = {
    "Cf_TesseractPose": "TesseractPoseComponent",
    "Cf_TesseractCartesianTarget": "TesseractCartesianTargetComponent",
    "Cf_TesseractJointTarget": "TesseractJointTargetComponent",
    "Cf_TesseractStateTarget": "TesseractStateTargetComponent",
    "Cf_TesseractMotionProgram": "TesseractMotionProgramComponent",
    "Cf_TesseractDescartesProfile": "TesseractDescartesProfileComponent",
    "Cf_TesseractRapidProfile": "TesseractRapidProfileComponent",
    "Cf_TesseractRapid": "TesseractRapidComponent",
    "Cf_TesseractNativePlan": "TesseractNativePlanComponent",
    "Cf_TesseractNativeResult": "TesseractNativeResultComponent",
    "Cf_TesseractRobotArtifact": "TesseractRobotArtifactComponent",
    "Cf_TesseractPlanner": "TesseractPlannerComponent",
}


class _FakeParameter:
    def __init__(self, name, connected):
        self.Name = name
        self.SourceCount = 1 if connected else 0
        self.PersistentDataCount = 0


class _FakeComponent:
    def __init__(self, connections):
        self.Params = SimpleNamespace(Input=[_FakeParameter(name, connected) for name, connected in connections.items()])


def _metadata(directory):
    return json.loads((COMPONENTS / directory / "metadata.json").read_text(encoding="utf-8"))


def _output_names(directory):
    return [parameter["name"] for parameter in _metadata(directory)["ghpython"]["outputParameters"]]


def _connections(directory, **states):
    """Model every declared input as wired unless a test unwires it.

    Grasshopper's ``Params.Input`` always lists every declared input; only the
    inputs a component reads through ``optional_connected_input`` consult the
    connection state, so wiring the rest is inert and mirrors the live host.
    """
    names = [parameter["name"] for parameter in _metadata(directory)["ghpython"]["inputParameters"]]
    return {name: states.get(name, True) for name in names}


def _load(monkeypatch, directory, connections):
    """Exec a component under a faithful fake host and capture ``error(...)``.

    Grasshopper, Rhino, System, scriptcontext, and ghpythonlib are faked because
    none exist outside Rhino; ``compas_ghpython`` is faked so ``error`` is
    recorded and ``create_id`` is deterministic; ``ensure_value_list`` is
    neutralised because it mutates a live canvas value list the fake host does
    not provide. Returns the component instance plus the captured error list,
    the sticky cache dict, and the host (whose identity seeds ``create_id``).
    """
    errors = []
    sticky = {}
    grasshopper = ModuleType("Grasshopper")
    grasshopper.Kernel = SimpleNamespace(GH_ScriptInstance=object)
    compas_ghpython = ModuleType("compas_ghpython")
    compas_ghpython.error = lambda component, message: errors.append(message)
    compas_ghpython.create_id = lambda component, name: "{}::{}".format(id(component), name)
    scriptcontext = ModuleType("scriptcontext")
    scriptcontext.sticky = sticky
    ghpythonlib = ModuleType("ghpythonlib")
    treehelpers = ModuleType("ghpythonlib.treehelpers")
    treehelpers.list_to_tree = lambda data: ("tree", data)
    ghpythonlib.treehelpers = treehelpers
    # The Pose component imports compas_rhino.conversions, which eagerly imports
    # Rhino.Geometry; the tests always feed a real COMPAS Frame so the plane
    # converter is never called, only its import must resolve.
    conversions = ModuleType("compas_rhino.conversions")
    conversions.plane_to_compas_frame = lambda plane: plane
    monkeypatch.setitem(sys.modules, "compas_rhino.conversions", conversions)
    monkeypatch.setitem(sys.modules, "Grasshopper", grasshopper)
    monkeypatch.setitem(sys.modules, "Rhino", ModuleType("Rhino"))
    monkeypatch.setitem(sys.modules, "System", ModuleType("System"))
    monkeypatch.setitem(sys.modules, "compas_ghpython", compas_ghpython)
    monkeypatch.setitem(sys.modules, "scriptcontext", scriptcontext)
    monkeypatch.setitem(sys.modules, "ghpythonlib", ghpythonlib)
    monkeypatch.setitem(sys.modules, "ghpythonlib.treehelpers", treehelpers)
    monkeypatch.setattr("compas_fab.ghpython.ensure_value_list", lambda *args, **kwargs: None, raising=False)
    source = COMPONENTS / directory / "code.py"
    namespace = {"__name__": "test_" + directory, "__file__": str(source)}
    exec(compile(source.read_text(encoding="utf-8"), str(source), "exec"), namespace)
    host = _FakeComponent(connections)
    namespace["ghenv"] = SimpleNamespace(Component=host)
    return namespace[_CLASS[directory]](), errors, sticky, host


# --- native-object equivalence helpers -------------------------------------


def _matrix(pose):
    return list(np.asarray(pose.matrix, dtype=np.float64).flatten())


def _assert_poses_equal(actual, expected):
    assert _matrix(actual) == pytest.approx(_matrix(expected))


def _assert_joint_targets_equal(actual, expected):
    assert list(actual.positions) == pytest.approx(list(expected.positions))
    assert actual.names == expected.names
    assert actual.move_type == expected.move_type
    assert actual.profile == expected.profile


def _optional_vector(target, attribute):
    value = getattr(target, attribute)
    return None if value is None else list(value)


def _assert_state_targets_equal(actual, expected):
    assert list(actual.positions) == pytest.approx(list(expected.positions))
    assert actual.names == expected.names
    assert _optional_vector(actual, "velocities") == _optional_vector(expected, "velocities")
    assert _optional_vector(actual, "accelerations") == _optional_vector(expected, "accelerations")
    assert actual.time == expected.time
    assert actual.move_type == expected.move_type
    assert actual.profile == expected.profile


# --- 1. Pose ---------------------------------------------------------------

_USER_FRAME = Frame([1000.0, 2000.0, 3000.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0])


def test_pose_runscript_matches_backend_with_connected_working_frame(monkeypatch):
    component, errors, _, _ = _load(monkeypatch, "Cf_TesseractPose", _connections("Cf_TesseractPose"))

    pose = component.RunScript(_USER_FRAME, 0.001, "flange")

    expected = pose_from_working_frame(WorkingFrameUserUnits.build(_USER_FRAME, 0.001, "flange"))
    assert errors == []
    _assert_poses_equal(pose, expected)
    assert pose.working_frame == "flange"


def test_pose_runscript_unconnected_working_frame_defaults_to_base_link(monkeypatch):
    component, errors, _, _ = _load(monkeypatch, "Cf_TesseractPose", _connections("Cf_TesseractPose", working_frame=False))

    # A non-empty value is supplied but the parameter is unwired: connection
    # state, not the value, must select the "base_link" default.
    pose = component.RunScript(_USER_FRAME, 0.001, "ignored_because_unconnected")

    expected = pose_from_working_frame(WorkingFrameUserUnits.build(_USER_FRAME, 0.001, "base_link"))
    assert errors == []
    _assert_poses_equal(pose, expected)
    assert pose.working_frame == "base_link"


def test_pose_runscript_none_frame_returns_single_none_without_error(monkeypatch):
    component, errors, _, _ = _load(monkeypatch, "Cf_TesseractPose", _connections("Cf_TesseractPose"))

    assert component.RunScript(None, 0.001, "flange") is None
    assert errors == []


def test_pose_runscript_invalid_scale_surfaces_error_and_returns_none(monkeypatch):
    component, errors, _, _ = _load(monkeypatch, "Cf_TesseractPose", _connections("Cf_TesseractPose"))

    pose = component.RunScript(_USER_FRAME, 0.0, "flange")

    assert pose is None
    assert errors == ["metres_per_user_unit must be an explicit finite positive value, got 0.0."]


# --- 2. CartesianTarget ----------------------------------------------------


def _working_frame_pose():
    return pose_from_working_frame(WorkingFrameUserUnits.build(_USER_FRAME, 0.001, "flange"))


def test_cartesian_runscript_matches_backend_typed_pose(monkeypatch):
    component, errors, _, _ = _load(monkeypatch, "Cf_TesseractCartesianTarget", _connections("Cf_TesseractCartesianTarget"))
    pose = _working_frame_pose()

    target = component.RunScript(pose, "LINEAR", "welding")

    expected = cartesian_target_from_native(pose, move_type_from_name("LINEAR"), "welding")
    assert errors == []
    assert isinstance(target, CartesianTarget)
    _assert_poses_equal(target.pose, expected.pose)
    assert target.move_type == expected.move_type == MoveType.LINEAR
    assert target.profile == expected.profile == "welding"
    assert target.working_frame == "flange"


def test_cartesian_runscript_matches_backend_plain_pose(monkeypatch):
    component, errors, _, _ = _load(monkeypatch, "Cf_TesseractCartesianTarget", _connections("Cf_TesseractCartesianTarget"))
    plain = Pose.from_matrix_position(np.eye(3), np.array([0.1, 0.2, 0.3]))

    target = component.RunScript(plain, "CIRCULAR", "path")

    expected = build_cartesian_target(plain, move_type_from_name("CIRCULAR"), "path")
    assert errors == []
    assert isinstance(target, CartesianTarget)
    assert not hasattr(target, "working_frame")
    _assert_poses_equal(target.pose, expected.pose)
    assert target.move_type == MoveType.CIRCULAR
    assert target.profile == "path"


def test_cartesian_runscript_unconnected_move_type_and_profile_use_defaults(monkeypatch):
    component, errors, _, _ = _load(
        monkeypatch,
        "Cf_TesseractCartesianTarget",
        _connections("Cf_TesseractCartesianTarget", move_type=False, profile=False),
    )

    # Non-default values supplied on unwired inputs must be ignored in favour
    # of the documented FREESPACE / DEFAULT defaults.
    target = component.RunScript(_working_frame_pose(), "LINEAR", "welding")

    assert errors == []
    assert target.move_type == MoveType.FREESPACE
    assert target.profile == "DEFAULT"


def test_cartesian_runscript_none_pose_returns_single_none(monkeypatch):
    component, errors, _, _ = _load(monkeypatch, "Cf_TesseractCartesianTarget", _connections("Cf_TesseractCartesianTarget"))

    assert component.RunScript(None, "FREESPACE", "DEFAULT") is None
    assert errors == []


def test_cartesian_runscript_unknown_move_type_surfaces_error_and_returns_none(monkeypatch):
    component, errors, _, _ = _load(monkeypatch, "Cf_TesseractCartesianTarget", _connections("Cf_TesseractCartesianTarget"))

    target = component.RunScript(_working_frame_pose(), "SPIRAL", "DEFAULT")

    assert target is None
    assert errors == ["Unknown native move type 'SPIRAL'."]


# --- 3. JointTarget --------------------------------------------------------


def test_joint_runscript_matches_backend_with_connected_names(monkeypatch):
    component, errors, _, _ = _load(monkeypatch, "Cf_TesseractJointTarget", _connections("Cf_TesseractJointTarget"))
    positions = [0.1, 0.2, 0.3]
    names = ["j1", "j2", "j3"]

    target = component.RunScript(positions, names, "LINEAR", "welding")

    expected = joint_target_from_native(
        NativeJointPositions.build(positions),
        NativeJointNames.build(names, len(positions)),
        move_type_from_name("LINEAR"),
        "welding",
    )
    assert errors == []
    _assert_joint_targets_equal(target, expected)
    assert target.names == ["j1", "j2", "j3"]
    assert target.move_type == MoveType.LINEAR
    assert target.profile == "welding"


def test_joint_runscript_unconnected_names_move_type_profile_use_defaults(monkeypatch):
    component, errors, _, _ = _load(
        monkeypatch,
        "Cf_TesseractJointTarget",
        _connections("Cf_TesseractJointTarget", joint_names=False, move_type=False, profile=False),
    )

    target = component.RunScript([0.1, 0.2, 0.3], ["ignored", "names", "here"], "LINEAR", "welding")

    expected = joint_target_from_native(
        NativeJointPositions.build([0.1, 0.2, 0.3]),
        None,
        move_type_from_name("FREESPACE"),
        "DEFAULT",
    )
    assert errors == []
    _assert_joint_targets_equal(target, expected)
    assert target.names is None
    assert target.move_type == MoveType.FREESPACE
    assert target.profile == "DEFAULT"


def test_joint_runscript_none_positions_returns_single_none(monkeypatch):
    component, errors, _, _ = _load(monkeypatch, "Cf_TesseractJointTarget", _connections("Cf_TesseractJointTarget"))

    assert component.RunScript(None, [], None, None) is None
    assert errors == []


def test_joint_runscript_empty_positions_surfaces_error_and_returns_none(monkeypatch):
    component, errors, _, _ = _load(monkeypatch, "Cf_TesseractJointTarget", _connections("Cf_TesseractJointTarget"))

    target = component.RunScript([], [], None, None)

    assert target is None
    assert errors == ["Target positions must be a non-empty finite one-dimensional vector."]


# --- 4. StateTarget --------------------------------------------------------


def test_state_runscript_matches_backend_full_dynamics(monkeypatch):
    component, errors, _, _ = _load(monkeypatch, "Cf_TesseractStateTarget", _connections("Cf_TesseractStateTarget"))
    positions = [0.1, 0.2]
    names = ["a", "b"]
    velocities = [1.0, 2.0]
    accelerations = [3.0, 4.0]

    target = component.RunScript(positions, names, velocities, accelerations, 0.5, "LINEAR", "welding")

    expected = state_target_from_native(
        NativeJointPositions.build(positions),
        NativeJointNames.build(names, len(positions)),
        NativeJointVelocities.build(velocities),
        NativeJointAccelerations.build(accelerations),
        NativeTime.build(0.5),
        move_type_from_name("LINEAR"),
        "welding",
    )
    assert errors == []
    _assert_state_targets_equal(target, expected)
    assert target.time == 0.5


def test_state_runscript_unconnected_dynamics_remain_absent(monkeypatch):
    component, errors, _, _ = _load(
        monkeypatch,
        "Cf_TesseractStateTarget",
        _connections(
            "Cf_TesseractStateTarget",
            joint_names=False,
            velocities=False,
            accelerations=False,
            time=False,
            move_type=False,
            profile=False,
        ),
    )

    # Non-empty dynamics values are supplied but every optional is unwired; the
    # component must never manufacture a partial state from ignored values.
    target = component.RunScript([0.1, 0.2], ["a", "b"], [1.0, 2.0], [3.0, 4.0], 0.5, "LINEAR", "welding")

    expected = state_target_from_native(
        NativeJointPositions.build([0.1, 0.2]),
        None,
        None,
        None,
        None,
        move_type_from_name("FREESPACE"),
        "DEFAULT",
    )
    assert errors == []
    _assert_state_targets_equal(target, expected)
    assert target.names is None
    assert target.velocities is None
    assert target.accelerations is None
    assert target.time is None
    assert target.move_type == MoveType.FREESPACE
    assert target.profile == "DEFAULT"


def test_state_runscript_none_positions_returns_single_none(monkeypatch):
    component, errors, _, _ = _load(monkeypatch, "Cf_TesseractStateTarget", _connections("Cf_TesseractStateTarget"))

    assert component.RunScript(None, [], [], [], None, None, None) is None
    assert errors == []


def test_state_runscript_bad_time_surfaces_error_and_returns_none(monkeypatch):
    # Only time is wired, so its negative value is the sole failing quantity.
    component, errors, _, _ = _load(
        monkeypatch,
        "Cf_TesseractStateTarget",
        _connections("Cf_TesseractStateTarget", joint_names=False, velocities=False, accelerations=False),
    )

    target = component.RunScript([0.1, 0.2], [], [], [], -1.0, None, None)

    assert target is None
    assert errors == ["Target time must be finite non-negative seconds, got -1.0."]


# --- 5. MotionProgram (real native robot) ----------------------------------


def test_motion_program_runscript_matches_backend(monkeypatch, tesseract_robot):
    component, errors, _, _ = _load(
        monkeypatch,
        "Cf_TesseractMotionProgram",
        _connections("Cf_TesseractMotionProgram", tcp_frame=False, profile=False),
    )
    targets = [JointTarget([0.0])]

    result = component.RunScript(tesseract_robot, targets, "manipulator", "ignored_tcp", "base", "welding")

    # tcp unconnected -> None -> robot resolves it; profile connected "welding".
    expected = build_motion_program(tesseract_robot, targets, "manipulator", None, "base", "DEFAULT")
    assert errors == []
    assert isinstance(result, tuple) and len(result) == len(_output_names("Cf_TesseractMotionProgram"))
    motion_program, program, joint_names, tcp_frame = result
    assert type(motion_program).__name__ == "MotionProgram"
    assert type(program).__name__ == "CompositeInstruction"
    assert joint_names == list(expected.joint_names) == ["joint1"]
    assert tcp_frame == expected.tcp_frame == "tip"
    # The output joint_names must be a plain list (tree-safe), not a tuple.
    assert isinstance(joint_names, list)


def test_motion_program_runscript_unconnected_working_frame_defaults_to_base_link(monkeypatch, tesseract_robot):
    component, errors, _, _ = _load(
        monkeypatch,
        "Cf_TesseractMotionProgram",
        _connections("Cf_TesseractMotionProgram", tcp_frame=False, working_frame=False, profile=False),
    )

    # The one-joint robot has no "base_link"; the unconnected default surfaces
    # as a backend error that names it, pinning the applied default value.
    result = component.RunScript(tesseract_robot, [JointTarget([0.0])], "manipulator", "", "base", "")

    assert result == (None, None, None, None)
    assert errors == ["Native working frame 'base_link' is not a robot link."]


def test_motion_program_runscript_none_robot_returns_four_none_tuple(monkeypatch, tesseract_robot):
    component, errors, _, _ = _load(monkeypatch, "Cf_TesseractMotionProgram", _connections("Cf_TesseractMotionProgram"))

    assert component.RunScript(None, [JointTarget([0.0])], "manipulator", "", "base", "DEFAULT") == (None, None, None, None)
    assert component.RunScript(tesseract_robot, None, "manipulator", "", "base", "DEFAULT") == (None, None, None, None)
    assert errors == []


def test_motion_program_runscript_unknown_group_surfaces_error_and_returns_four_none(monkeypatch, tesseract_robot):
    # tcp/profile unwired so the unknown group is the sole failing input.
    component, errors, _, _ = _load(
        monkeypatch,
        "Cf_TesseractMotionProgram",
        _connections("Cf_TesseractMotionProgram", tcp_frame=False, profile=False),
    )

    result = component.RunScript(tesseract_robot, [JointTarget([0.0])], "no_such_group", "", "base", "")

    assert result == (None, None, None, None)
    assert len(errors) == 1 and "no_such_group" in errors[0]


# --- 6. DescartesProfile ---------------------------------------------------


def test_descartes_runscript_all_unconnected_returns_native_dictionary(monkeypatch):
    component, errors, _, _ = _load(
        monkeypatch,
        "Cf_TesseractDescartesProfile",
        _connections(
            "Cf_TesseractDescartesProfile",
            profile_names=False,
            enable_collision=False,
            enable_edge_collision=False,
            num_threads=False,
            sample_axis=False,
            sample_resolution=False,
            sample_min=False,
            sample_max=False,
            ik_solver=False,
            use_redundant_joint_solutions=False,
            move_profile=False,
        ),
    )

    profiles = component.RunScript([], False, False, 0, [], 0.0, 0.0, 0.0, "", False, None)

    assert isinstance(profiles, ProfileDictionary)
    assert not isinstance(profiles, tuple)
    assert errors == []


def test_descartes_runscript_duplicate_profile_names_surface_error_and_return_none(monkeypatch):
    component, errors, _, _ = _load(monkeypatch, "Cf_TesseractDescartesProfile", _connections("Cf_TesseractDescartesProfile"))

    profiles = component.RunScript(["p", "p"], None, None, None, None, None, None, None, None, None, None)

    assert profiles is None
    assert errors == ["Descartes profile names must be unique."]


def test_descartes_runscript_sample_bounds_reach_their_own_native_slots(monkeypatch):
    # min > max only errors when sample_min and sample_max each reach their own
    # backend slot; a swapped wiring would invert the comparison and pass.
    component, errors, _, _ = _load(
        monkeypatch,
        "Cf_TesseractDescartesProfile",
        _connections(
            "Cf_TesseractDescartesProfile",
            profile_names=False,
            enable_collision=False,
            enable_edge_collision=False,
            num_threads=False,
            sample_axis=False,
            sample_resolution=False,
            ik_solver=False,
            use_redundant_joint_solutions=False,
            move_profile=False,
        ),
    )

    profiles = component.RunScript([], False, False, 0, [], 0.0, 5.0, 1.0, "", False, None)

    assert profiles is None
    assert errors == ["sample_min must be less than or equal to sample_max."]


# --- 7. RapidProfile -------------------------------------------------------


def test_rapid_profile_runscript_falsey_names_returns_single_none(monkeypatch):
    component, errors, _, _ = _load(monkeypatch, "Cf_TesseractRapidProfile", _connections("Cf_TesseractRapidProfile"))

    assert component.RunScript([], "", "", "", "") is None
    assert errors == []


def test_rapid_profile_runscript_duplicate_names_surface_error_and_return_none(monkeypatch):
    component, errors, _, _ = _load(monkeypatch, "Cf_TesseractRapidProfile", _connections("Cf_TesseractRapidProfile"))

    profiles = component.RunScript(["dup", "dup"], "", "", "", "")

    assert profiles is None
    assert len(errors) == 1


def test_rapid_profile_runscript_binds_each_variable_to_its_named_slot(monkeypatch):
    # A slot swap (e.g. tool<->workobject) would surface here; each RAPID
    # variable must land under its own type-tagged native name.
    component, errors, _, _ = _load(monkeypatch, "Cf_TesseractRapidProfile", _connections("Cf_TesseractRapidProfile"))

    profiles = component.RunScript(["fast", "slow"], "v1000", "z50", "mytool", "mywobj")

    assert errors == []
    assert set(profiles) == {"fast", "slow"}
    for profile in profiles.values():
        assert str(profile.speed) == "v1000"
        assert str(profile.zone) == "z50"
        assert str(profile.tool) == "mytool"
        assert str(profile.wobj) == "mywobj"


def test_rapid_profile_runscript_applies_documented_variable_defaults(monkeypatch):
    component, errors, _, _ = _load(monkeypatch, "Cf_TesseractRapidProfile", _connections("Cf_TesseractRapidProfile"))

    profiles = component.RunScript(["p"], "", "", "", "")

    assert errors == []
    profile = profiles["p"]
    assert str(profile.speed) == "v200"
    assert str(profile.zone) == "z10"
    assert str(profile.tool) == "tool0"
    assert str(profile.wobj) == "wobj0"


# --- 8. Rapid --------------------------------------------------------------


def test_rapid_runscript_none_program_returns_three_none_tuple(monkeypatch):
    component, errors, _, _ = _load(monkeypatch, "Cf_TesseractRapid", _connections("Cf_TesseractRapid"))

    assert component.RunScript(None, [], "mod", "proc") == (None, None, None)
    assert errors == []


def test_rapid_runscript_invalid_program_surfaces_error_and_returns_three_none(monkeypatch):
    component, errors, _, _ = _load(monkeypatch, "Cf_TesseractRapid", _connections("Cf_TesseractRapid"))

    # A non-CompositeInstruction program drives the emitter to fail; the pure
    # adapter must surface the error and return the exact 3-None shape.
    result = component.RunScript(object(), [], "mod", "proc")

    assert result == (None, None, None)
    assert len(errors) == 1


# --- 9. NativeResult -------------------------------------------------------


def test_native_result_runscript_none_returns_ten_none_tuple(monkeypatch):
    component, errors, _, _ = _load(monkeypatch, "Cf_TesseractNativeResult", _connections("Cf_TesseractNativeResult"))

    result = component.RunScript(None)

    assert result == (None,) * len(_output_names("Cf_TesseractNativeResult"))
    assert errors == []


def test_native_result_runscript_invalid_result_surfaces_error_and_returns_ten_none(monkeypatch):
    component, errors, _, _ = _load(monkeypatch, "Cf_TesseractNativeResult", _connections("Cf_TesseractNativeResult"))

    result = component.RunScript(object())

    assert result == (None,) * 10
    assert len(errors) == 1


# --- 10. NativePlan (cache wiring; deep execution needs a live planner) -----


def test_native_plan_runscript_missing_inputs_evicts_cache_and_returns_none(monkeypatch):
    component, errors, sticky, host = _load(monkeypatch, "Cf_TesseractNativePlan", _connections("Cf_TesseractNativePlan"))
    key = "{}::tesseract_native_plan".format(id(host))
    sticky[key] = ("stale-signature", "stale-result")

    result = component.RunScript(None, object(), "FREESPACE", object(), False, False)

    assert result is None
    assert errors == []
    # A missing required input must evict any stale cached result for this key.
    assert key not in sticky


def test_native_plan_runscript_non_bool_compute_surfaces_error_and_evicts_cache(monkeypatch):
    component, errors, sticky, host = _load(monkeypatch, "Cf_TesseractNativePlan", _connections("Cf_TesseractNativePlan"))
    key = "{}::tesseract_native_plan".format(id(host))
    sticky[key] = ("stale-signature", "stale-result")

    # planner/program/profiles are non-None so the guard passes; a non-bool
    # compute drives required_native_plan_bool to raise before any planning.
    result = component.RunScript(object(), object(), "FREESPACE", object(), False, "not-a-bool")

    assert result is None
    assert errors == ["compute must be bool, got str."]
    assert key not in sticky


# --- 11. RobotArtifact (reachable error path; full compile is heavy) --------


def test_robot_artifact_runscript_missing_inputs_returns_two_none(monkeypatch, one_joint_cell):
    component, errors, _, _ = _load(monkeypatch, "Cf_TesseractRobotArtifact", _connections("Cf_TesseractRobotArtifact"))

    assert component.RunScript(None, "u.urdf", "s.srdf", [], [], "", "", "", "") == (None, None)
    assert component.RunScript(one_joint_cell, "", "s.srdf", [], [], "", "", "", "") == (None, None)
    assert component.RunScript(one_joint_cell, "u.urdf", "", [], [], "", "", "", "") == (None, None)
    assert errors == []


def test_robot_artifact_runscript_missing_urdf_file_surfaces_error_and_returns_two_none(monkeypatch, one_joint_cell, tmp_path):
    component, errors, _, _ = _load(monkeypatch, "Cf_TesseractRobotArtifact", _connections("Cf_TesseractRobotArtifact"))
    missing_urdf = str(tmp_path / "absent.urdf")
    missing_srdf = str(tmp_path / "absent.srdf")

    result = component.RunScript(one_joint_cell, missing_urdf, missing_srdf, [], [], "convex_hull", "kdl_lma", "bullet_bvh", "bullet_cast_bvh")

    assert result == (None, None)
    assert len(errors) == 1


def test_robot_artifact_runscript_unknown_selection_surfaces_error_and_returns_two_none(monkeypatch, one_joint_cell, tmp_path):
    component, errors, _, _ = _load(monkeypatch, "Cf_TesseractRobotArtifact", _connections("Cf_TesseractRobotArtifact"))
    urdf = tmp_path / "r.urdf"
    srdf = tmp_path / "r.srdf"
    urdf.write_text("<robot name='r'><link name='base'/></robot>", encoding="utf-8")
    srdf.write_text("<robot name='r'></robot>", encoding="utf-8")

    result = component.RunScript(one_joint_cell, str(urdf), str(srdf), [], [], "bogus_policy", "kdl_lma", "bullet_bvh", "bullet_cast_bvh")

    assert result == (None, None)
    assert len(errors) == 1 and "bogus_policy" in errors[0]


# --- 12. Planner (real client build + clone; caching via sticky) ------------


def test_planner_runscript_missing_inputs_returns_two_none(monkeypatch):
    component, errors, _, _ = _load(monkeypatch, "Cf_TesseractPlanner", _connections("Cf_TesseractPlanner"))

    assert component.RunScript(None, object(), None, [], False, False) == (None, None)
    assert component.RunScript(object(), None, None, [], False, False) == (None, None)
    assert errors == []


def test_planner_runscript_invalid_warmup_pipeline_surfaces_error_and_returns_two_none(monkeypatch):
    component, errors, _, _ = _load(monkeypatch, "Cf_TesseractPlanner", _connections("Cf_TesseractPlanner"))

    # artifact/robot_cell are non-None so the guard passes; a non-string warmup
    # pipeline is rejected before any client is constructed.
    result = component.RunScript(object(), object(), None, [123], False, False)

    assert result == (None, None)
    assert errors == ["warmup_pipelines must contain non-empty pipeline names."]


def test_planner_runscript_warmup_all_with_pipelines_is_rejected(monkeypatch):
    component, errors, _, _ = _load(monkeypatch, "Cf_TesseractPlanner", _connections("Cf_TesseractPlanner"))

    result = component.RunScript(object(), object(), None, ["freespace"], True, False)

    assert result == (None, None)
    assert errors == ["Select warmup_all or warmup_pipelines, not both."]


def test_planner_runscript_builds_planner_and_isolated_native_clone(monkeypatch, tesseract_artifact, one_joint_cell, one_joint_state):
    component, errors, sticky, host = _load(monkeypatch, "Cf_TesseractPlanner", _connections("Cf_TesseractPlanner"))
    try:
        planner, native_robot = component.RunScript(tesseract_artifact, one_joint_cell, one_joint_state, [], False, False)

        assert errors == []
        assert type(planner).__name__ == "TesseractPlanner"
        assert type(native_robot).__name__ == "Robot"
        # The exposed native robot is an isolated clone, not the planner's master.
        assert native_robot is not planner.client.environment
        # The client is cached by artifact identity for reuse across solutions.
        key = "{}::tesseract_planner".format(id(host))
        assert key in sticky
        assert sticky[key][2] == tesseract_artifact.identity.digest
    finally:
        cached = sticky.get("{}::tesseract_planner".format(id(host)))
        if cached is not None:
            cached[0].disconnect()


# --- output-arity contract across every component --------------------------

_GUARD_NONE_SHAPE = {
    "Cf_TesseractPose": (("frame",), None),
    "Cf_TesseractCartesianTarget": (("pose",), None),
    "Cf_TesseractJointTarget": (("positions",), None),
    "Cf_TesseractStateTarget": (("positions",), None),
    "Cf_TesseractMotionProgram": (("native_robot",), (None, None, None, None)),
    "Cf_TesseractNativeResult": (("result",), (None,) * 10),
    "Cf_TesseractRobotArtifact": (("robot_cell",), (None, None)),
    "Cf_TesseractPlanner": (("artifact",), (None, None)),
    "Cf_TesseractRapid": (("program",), (None, None, None)),
}


@pytest.mark.parametrize("directory", sorted(_GUARD_NONE_SHAPE))
def test_guard_none_shape_matches_declared_output_arity(directory):
    expected = _GUARD_NONE_SHAPE[directory][1]
    outputs = _output_names(directory)
    if expected is None:
        assert len(outputs) == 1
    else:
        assert isinstance(expected, tuple) and len(expected) == len(outputs)
