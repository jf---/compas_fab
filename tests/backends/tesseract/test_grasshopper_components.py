import json
from pathlib import Path
import struct

import pytest
import yaml


COMPONENTS = Path(__file__).parents[3] / "src" / "compas_fab" / "ghpython" / "components_cpython"
WORKFLOWS = Path(__file__).parents[3] / ".github" / "workflows"
PYPROJECT = Path(__file__).parents[3] / "pyproject.toml"
PIXI_LOCK = Path(__file__).parents[3] / "pixi.lock"

TESSERACT_USER_OBJECTS = (
    "Cf_TesseractRobotArtifact.ghuser",
    "Cf_TesseractPlanner.ghuser",
    "Cf_TesseractRapidProfile.ghuser",
    "Cf_TesseractRapid.ghuser",
    "Cf_TesseractPose.ghuser",
    "Cf_TesseractCartesianTarget.ghuser",
    "Cf_TesseractJointTarget.ghuser",
    "Cf_TesseractStateTarget.ghuser",
    "Cf_TesseractMotionProgram.ghuser",
    "Cf_TesseractDescartesProfile.ghuser",
    "Cf_TesseractNativePlan.ghuser",
    "Cf_TesseractNativeResult.ghuser",
)


def _component(name: str) -> tuple[str, dict]:
    root = COMPONENTS / name
    return (root.joinpath("code.py").read_text(), json.loads(root.joinpath("metadata.json").read_text()))


def _png_size(path: Path) -> tuple[int, int]:
    content = path.read_bytes()
    assert content[:8] == b"\x89PNG\r\n\x1a\n"
    return struct.unpack(">II", content[16:24])


def test_artifact_component_exposes_explicit_native_selections():
    code, metadata = _component("Cf_TesseractRobotArtifact")
    inputs = [parameter["name"] for parameter in metadata["ghpython"]["inputParameters"]]
    outputs = [parameter["name"] for parameter in metadata["ghpython"]["outputParameters"]]

    assert inputs == [
        "robot_cell",
        "urdf_filename",
        "srdf_filename",
        "resource_roots",
        "planning_groups",
        "collision_mesh_policy",
        "inverse_kinematics",
        "discrete_contact_manager",
        "continuous_contact_manager",
    ]
    assert outputs == ["artifact", "artifact_id"]
    assert "CompasRobotArtifactCompiler" in code
    assert 'default="convex_hull"' in code
    assert 'collision_mesh_policy,\n                    "convex_hull",' in code
    mesh_input = next(parameter for parameter in metadata["ghpython"]["inputParameters"] if parameter["name"] == "collision_mesh_policy")
    assert "defaults to convex_hull" in mesh_input["description"]


def test_planner_component_retains_native_access_and_manages_lifetime():
    code, metadata = _component("Cf_TesseractPlanner")
    inputs = [parameter["name"] for parameter in metadata["ghpython"]["inputParameters"]]
    outputs = [parameter["name"] for parameter in metadata["ghpython"]["outputParameters"]]

    assert inputs == [
        "artifact",
        "robot_cell",
        "robot_cell_state",
        "warmup_pipelines",
        "warmup_all",
        "reload",
    ]
    assert outputs == ["planner", "native_robot"]
    assert "client.clone_robot()" in code
    assert "client.disconnect()" in code
    assert "artifact.identity.digest" in code
    assert "bool(warmup)" not in code


def test_rapid_profile_component_builds_exact_native_profiles():
    code, metadata = _component("Cf_TesseractRapidProfile")
    inputs = [parameter["name"] for parameter in metadata["ghpython"]["inputParameters"]]
    outputs = [parameter["name"] for parameter in metadata["ghpython"]["outputParameters"]]

    assert inputs == ["profile_names", "speed", "zone", "tool", "workobject"]
    assert outputs == ["profiles"]
    assert "RapidProfile" in code
    assert "SpeedName" in code
    assert "ZoneName" in code
    assert "ToolName" in code
    assert "WobjName" in code
    assert "merge_rapid_profile_maps" in code
    assert 'speed or "v200"' in code
    assert 'zone or "z10"' in code
    assert 'tool or "tool0"' in code
    assert 'workobject or "wobj0"' in code


def test_rapid_emitter_component_is_pure_native_adapter():
    code, metadata = _component("Cf_TesseractRapid")
    inputs = [parameter["name"] for parameter in metadata["ghpython"]["inputParameters"]]
    outputs = [parameter["name"] for parameter in metadata["ghpython"]["outputParameters"]]

    assert inputs == ["program", "profile_maps", "module_name", "procedure_name"]
    assert outputs == ["rapid_program", "source", "program_id"]
    assert "TesseractRapidEmitter.emit" in code
    assert "merge_rapid_profile_maps" in code
    assert "RapidEmitterError" in code
    assert ".write(" not in code
    assert "TesseractPlanningRequest" not in code
    assert "TesseractPlanningResult" not in code
    assert "JointTrajectory" not in code


@pytest.mark.parametrize(
    ("component", "inputs", "outputs", "factory"),
    [
        (
            "Cf_TesseractPose",
            ["frame", "metres_per_user_unit", "working_frame"],
            ["pose"],
            "pose_from_working_frame",
        ),
        (
            "Cf_TesseractCartesianTarget",
            ["pose", "move_type", "profile"],
            ["target"],
            "cartesian_target_from_native",
        ),
        (
            "Cf_TesseractJointTarget",
            ["positions", "joint_names", "move_type", "profile"],
            ["target"],
            "joint_target_from_native",
        ),
        (
            "Cf_TesseractStateTarget",
            [
                "positions",
                "joint_names",
                "velocities",
                "accelerations",
                "time",
                "move_type",
                "profile",
            ],
            ["target"],
            "state_target_from_native",
        ),
    ],
)
def test_native_authoring_component_contract(
    component,
    inputs,
    outputs,
    factory,
):
    code, metadata = _component(component)

    assert [item["name"] for item in metadata["ghpython"]["inputParameters"]] == inputs
    assert [item["name"] for item in metadata["ghpython"]["outputParameters"]] == outputs
    assert factory in code
    assert "# r: tesseract-robotics-nanobind==0.35.0.6" in code
    assert "except TesseractBackendError" in code
    assert "except Exception" not in code
    assert _png_size(COMPONENTS / component / "icon.png") == (24, 24)


def test_pose_component_is_the_only_new_geometry_unit_boundary():
    pose_code, _ = _component("Cf_TesseractPose")
    cartesian_code, _ = _component("Cf_TesseractCartesianTarget")

    assert "plane_to_compas_frame" in pose_code
    assert "metres_per_user_unit" in pose_code
    assert "plane_to_compas_frame" not in cartesian_code
    assert "compas.geometry" not in cartesian_code
    assert "compas_rhino.conversions" not in cartesian_code
    assert "WorkingFrameUserUnits.build" in pose_code


def test_joint_components_validate_typed_native_quantities_at_host_boundary():
    joint_code, _ = _component("Cf_TesseractJointTarget")
    state_code, _ = _component("Cf_TesseractStateTarget")

    assert "NativeJointPositions.build" in joint_code
    assert "NativeJointNames.build" in joint_code
    assert "NativeJointPositions.build" in state_code
    assert "NativeJointVelocities.build" in state_code
    assert "NativeJointAccelerations.build" in state_code
    assert "NativeTime.build" in state_code


@pytest.mark.parametrize(
    "component",
    [
        "Cf_TesseractCartesianTarget",
        "Cf_TesseractJointTarget",
        "Cf_TesseractStateTarget",
    ],
)
def test_target_components_expose_all_exact_native_move_types(component):
    code, _ = _component(component)

    assert "move_type_from_name" in code
    assert '"FREESPACE"' in code
    assert '"LINEAR"' in code
    assert '"CIRCULAR"' in code
    assert "ensure_value_list" in code


def test_motion_program_component_preserves_native_authoring_path():
    code, metadata = _component("Cf_TesseractMotionProgram")
    inputs = [item["name"] for item in metadata["ghpython"]["inputParameters"]]
    outputs = [item["name"] for item in metadata["ghpython"]["outputParameters"]]

    assert inputs == [
        "native_robot",
        "targets",
        "group_name",
        "tcp_frame",
        "working_frame",
        "profile",
    ]
    assert outputs == [
        "motion_program",
        "program",
        "joint_names",
        "tcp_frame",
    ]
    assert "build_motion_program" in code
    assert ".move_to(" not in code
    assert ".linear_to(" not in code
    assert ".circular_to(" not in code
    assert "except TesseractBackendError" in code
    assert "except Exception" not in code
    assert _png_size(COMPONENTS / "Cf_TesseractMotionProgram" / "icon.png") == (24, 24)


def test_descartes_profile_component_exposes_complete_native_factory():
    code, metadata = _component("Cf_TesseractDescartesProfile")
    inputs = [item["name"] for item in metadata["ghpython"]["inputParameters"]]
    outputs = [item["name"] for item in metadata["ghpython"]["outputParameters"]]

    assert inputs == [
        "profile_names",
        "enable_collision",
        "enable_edge_collision",
        "num_threads",
        "sample_axis",
        "sample_resolution",
        "sample_min",
        "sample_max",
        "ik_solver",
        "use_redundant_joint_solutions",
        "move_profile",
    ]
    assert outputs == ["profiles"]
    assert "build_descartes_profiles" in code
    assert "create_descartes_pipeline_profiles" not in code
    assert "sample_resolution or" not in code
    assert "sample_min or" not in code
    assert "sample_max or" not in code
    assert "enable_collision or" not in code
    assert "enable_edge_collision or" not in code
    assert "use_redundant_joint_solutions or" not in code
    assert "except TesseractBackendError" in code
    assert "except Exception" not in code
    assert _png_size(COMPONENTS / "Cf_TesseractDescartesProfile" / "icon.png") == (24, 24)


def test_native_plan_component_uses_exact_request_and_observable_cache():
    code, metadata = _component("Cf_TesseractNativePlan")
    inputs = [item["name"] for item in metadata["ghpython"]["inputParameters"]]
    outputs = [item["name"] for item in metadata["ghpython"]["outputParameters"]]

    assert inputs == [
        "planner",
        "program",
        "pipeline",
        "profiles",
        "auto_seed",
        "compute",
    ]
    assert outputs == ["result"]
    assert "NativePlanCall.build" in code
    assert "call.execute()" in code
    assert "signature" in code
    assert "st.pop(key, None)" in code
    assert "plan_motion" not in code
    assert "plan_cartesian_motion" not in code
    assert "except TesseractBackendError" in code
    assert "except Exception" not in code
    assert _png_size(COMPONENTS / "Cf_TesseractNativePlan" / "icon.png") == (24, 24)


def test_native_result_component_retains_exact_objects_and_absence():
    code, metadata = _component("Cf_TesseractNativeResult")
    inputs = [item["name"] for item in metadata["ghpython"]["inputParameters"]]
    outputs = [item["name"] for item in metadata["ghpython"]["outputParameters"]]

    assert inputs == ["result"]
    assert outputs == [
        "request",
        "native_result",
        "raw_program",
        "message",
        "trajectory_points",
        "joint_names",
        "positions",
        "velocities",
        "accelerations",
        "times",
    ]
    assert "TesseractNativeResultView.build" in code
    assert "list_to_tree" in code
    assert "view.request" in code
    assert "view.native_result" in code
    assert "view.raw_program" in code
    assert "JointTrajectory" not in code
    assert "except TesseractBackendError" in code
    assert "except Exception" not in code
    assert _png_size(COMPONENTS / "Cf_TesseractNativeResult" / "icon.png") == (24, 24)


def test_tesseract_components_require_nanobind_distribution_only():
    for name in (
        "Cf_TesseractRobotArtifact",
        "Cf_TesseractPlanner",
        "Cf_TesseractRapidProfile",
        "Cf_TesseractRapid",
        "Cf_TesseractPose",
        "Cf_TesseractCartesianTarget",
        "Cf_TesseractJointTarget",
        "Cf_TesseractStateTarget",
        "Cf_TesseractMotionProgram",
        "Cf_TesseractDescartesProfile",
        "Cf_TesseractNativePlan",
        "Cf_TesseractNativeResult",
    ):
        code, _ = _component(name)
        assert "# r: tesseract-robotics-nanobind==0.35.0.6" in code
        assert ".dev" not in code
        assert "# r: tesseract-python" not in code
        assert "except Exception" not in code
        assert "except ImportError" not in code
        assert _png_size(COMPONENTS / name / "icon.png") == (24, 24)


def test_grasshopper_jobs_pin_rhino_python39_through_pixi():
    pyproject = PYPROJECT.read_text(encoding="utf-8")
    assert '[tool.pixi.feature.rhino.dependencies]\npython = "3.9.*"' in pyproject

    for workflow_name in ("build.yml", "publish_yak.yml"):
        workflow = WORKFLOWS.joinpath(workflow_name).read_text(encoding="utf-8")
        assert "prefix-dev/setup-pixi@v0.9.3" in workflow
        assert "environments: rhino39" in workflow
        assert "locked: true" in workflow
        assert "pixi run -e rhino39 build-gh-components" in workflow
        assert "pip install" not in workflow

    release = WORKFLOWS.joinpath("release.yml").read_text(encoding="utf-8")
    assert "locked: true" in release
    assert "python-version: '3.9'" in release


def test_rhino_lock_contains_released_windows_python39_nanobind_wheel():
    lock = yaml.safe_load(PIXI_LOCK.read_text(encoding="utf-8"))
    packages = lock["environments"]["rhino39"]["packages"]["win-64"]
    locators = [next(iter(package.values())) for package in packages]

    assert any("/python-3.9." in locator for locator in locators)
    assert any("tesseract_robotics_nanobind-0.35.0.6-cp39-cp39-win_amd64.whl" in locator for locator in locators)
    assert all("tesseract_robotics_nanobind-0.35.0.6.dev" not in locator for locator in locators)


def test_windows_ci_requires_all_tesseract_user_objects_before_upload():
    for workflow_name in ("build.yml", "publish_yak.yml", "release.yml"):
        workflow = WORKFLOWS.joinpath(workflow_name).read_text(encoding="utf-8")
        for user_object in TESSERACT_USER_OBJECTS:
            assert user_object in workflow
        assert "Test-Path" in workflow

    build = WORKFLOWS.joinpath("build.yml").read_text(encoding="utf-8")
    assert "path: src/compas_fab/ghpython/components_cpython/ghuser" in build


def test_ci_excludes_only_unsupported_linux_python39_tesseract_cell():
    for workflow_name in ("build.yml", "release.yml"):
        workflow = WORKFLOWS.joinpath(workflow_name).read_text(encoding="utf-8")
        assert "exclude:" in workflow
        assert "os: ubuntu-latest\n            python: '3.9'" in workflow
