import ast
from pathlib import Path


SOURCE = Path(__file__).parents[3] / "src" / "compas_fab" / "backends" / "tesseract"
FORBIDDEN_BACKEND_PREFIXES = (
    "compas_fab.backends.kinematics",
    "compas_fab.backends.pybullet",
    "compas_fab.backends.ros",
)


def test_tesseract_backend_never_imports_fallback_backends():
    violations = []
    for path in SOURCE.rglob("*.py"):
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        modules = [node.module for node in ast.walk(tree) if isinstance(node, ast.ImportFrom) and node.module is not None]
        modules.extend(alias.name for node in ast.walk(tree) if isinstance(node, ast.Import) for alias in node.names)
        for module in modules:
            if module.startswith(FORBIDDEN_BACKEND_PREFIXES):
                violations.append("{} imports {}".format(path.relative_to(SOURCE), module))

    assert violations == []
