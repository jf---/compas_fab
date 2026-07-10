from pathlib import Path


EXAMPLES = Path(__file__).parents[3] / "docs" / "backends" / "tesseract" / "files"
CONVENTIONAL_EXAMPLE = EXAMPLES / "01_compas_plan_motion.py"
NATIVE_EXAMPLE = EXAMPLES / "02_native_program.py"


def test_examples_exercise_convex_hull_loading_default():
    for example in (CONVENTIONAL_EXAMPLE, NATIVE_EXAMPLE):
        source = example.read_text(encoding="utf-8")
        assert "collision_mesh_policy=" not in source


def test_native_example_uses_tesseract_axis_symmetry():
    source = NATIVE_EXAMPLE.read_text(encoding="utf-8")

    assert 'pipeline="DescartesFPipeline"' in source
    assert "TOOL_Z_AXIS = (0.0, 0.0, 1.0)" in source
    assert "TOOL_AXIS_SAMPLE_STEP = radians(30.0)" in source
    assert "sample_axis=TOOL_Z_AXIS" in source
    assert "sample_resolution=TOOL_AXIS_SAMPLE_STEP" in source
    assert "use_redundant_joint_solutions=True" in source
