import json
from pathlib import Path
import struct

import yaml


COMPONENTS = Path(__file__).parents[3] / "src" / "compas_fab" / "ghpython" / "components_cpython"
WORKFLOWS = Path(__file__).parents[3] / ".github" / "workflows"
PYPROJECT = Path(__file__).parents[3] / "pyproject.toml"
PIXI_LOCK = Path(__file__).parents[3] / "pixi.lock"


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


def test_tesseract_components_require_nanobind_distribution_only():
    for name in (
        "Cf_TesseractRobotArtifact",
        "Cf_TesseractPlanner",
        "Cf_TesseractRapidProfile",
        "Cf_TesseractRapid",
    ):
        code, _ = _component(name)
        assert "# r: tesseract-robotics-nanobind>=0.35.0.6,<0.36" in code
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
        assert "Cf_TesseractRobotArtifact.ghuser" in workflow
        assert "Cf_TesseractPlanner.ghuser" in workflow
        assert "Cf_TesseractRapidProfile.ghuser" in workflow
        assert "Cf_TesseractRapid.ghuser" in workflow
        assert "Test-Path" in workflow

    build = WORKFLOWS.joinpath("build.yml").read_text(encoding="utf-8")
    assert "path: src/compas_fab/ghpython/components_cpython/ghuser" in build


def test_ci_excludes_only_unsupported_linux_python39_tesseract_cell():
    for workflow_name in ("build.yml", "release.yml"):
        workflow = WORKFLOWS.joinpath(workflow_name).read_text(encoding="utf-8")
        assert "exclude:" in workflow
        assert "os: ubuntu-latest\n            python: '3.9'" in workflow
