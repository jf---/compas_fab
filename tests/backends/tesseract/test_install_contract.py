import ast
from importlib.metadata import PackageNotFoundError
from importlib.metadata import version
from pathlib import Path

import pytest
import yaml
from packaging.version import Version
from packaging.requirements import Requirement


SOURCE = Path(__file__).parents[3] / "src" / "compas_fab" / "backends" / "tesseract"
REQUIREMENTS = Path(__file__).parents[3] / "requirements.txt"
PYPROJECT = Path(__file__).parents[3] / "pyproject.toml"
PIXI_LOCK = Path(__file__).parents[3] / "pixi.lock"
TESSERACT_DOC = Path(__file__).parents[3] / "docs" / "backends" / "tesseract.md"


def test_nanobind_runtime_is_installed_as_hard_dependency():
    import tesseract_robotics

    assert tesseract_robotics is not None
    assert Version(version("tesseract-robotics-nanobind")) == Version("0.35.0.6")
    with pytest.raises(PackageNotFoundError):
        version("tesseract-robotics")


def test_public_dependency_requires_released_nanobind_build():
    requirements = REQUIREMENTS.read_text(encoding="utf-8")

    dependency = next(Requirement(line) for line in requirements.splitlines() if line.startswith("tesseract-robotics-nanobind"))

    assert dependency.specifier == Requirement("tesseract-robotics-nanobind==0.35.0.6").specifier
    assert Version("0.35.0.7") not in dependency.specifier
    assert ".dev" not in requirements


def test_documentation_pins_exact_nanobind_build():
    documentation = TESSERACT_DOC.read_text(encoding="utf-8")

    assert '"tesseract-robotics-nanobind==0.35.0.6"' in documentation
    assert "# r: tesseract-robotics-nanobind==0.35.0.6" in documentation
    assert "tesseract-robotics-nanobind>=" not in documentation


def test_pixi_uses_pthreads_blas_beside_nanobind_bundled_openmp():
    pyproject = PYPROJECT.read_text(encoding="utf-8")
    lock = yaml.safe_load(PIXI_LOCK.read_text(encoding="utf-8"))
    openblas_locators = [
        next(iter(package.values()))
        for environment in lock["environments"].values()
        for packages in environment["packages"].values()
        for package in packages
        if "libopenblas-" in next(iter(package.values()))
    ]

    assert 'libopenblas = { version = ">=0.3.30,<0.4", build = "*pthreads*" }' in pyproject
    assert openblas_locators
    assert all("-pthreads_" in locator for locator in openblas_locators)


def test_backend_value_types_remain_python39_compatible():
    incompatible = []
    for path in SOURCE.rglob("*.py"):
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            if not isinstance(node.func, ast.Name) or node.func.id != "dataclass":
                continue
            if any(keyword.arg == "slots" for keyword in node.keywords):
                incompatible.append(str(path.relative_to(SOURCE)))

    assert incompatible == []
    assert version("attrs")


def test_common_tesseract_surface_is_importable_without_all_registry():
    from compas_fab.backends import tesseract

    assert tesseract.CompasRobotArtifactCompiler
    assert tesseract.RobotArtifact
    assert tesseract.RobotArtifactLoader
    assert tesseract.TesseractClient
    assert tesseract.TesseractPlanner
    assert tesseract.TesseractPlanningRequest
    assert not hasattr(tesseract, "__all__")
