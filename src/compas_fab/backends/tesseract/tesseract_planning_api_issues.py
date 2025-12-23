"""
Issues found in tesseract_robotics.planning high-level API.

Report for: https://github.com/tesseract-robotics/tesseract_python_nanobind

Run this file to reproduce the issues:
    DYLD_LIBRARY_PATH=/path/to/tesseract/lib python tesseract_planning_api_issues.py
"""

from pathlib import Path


def issue_1_taskcomposer_constructor_error():
    """
    Issue 1: TaskComposer constructor doesn't accept Environment directly.

    The error message is confusing - it says 'createTaskComposerNode' not found
    instead of explaining that TaskComposer expects a TaskComposerPluginFactory.

    Suggested fix: Add type checking in __init__ with helpful error message,
    or add TaskComposer.from_environment(env) factory method.
    """
    print("Issue 1: TaskComposer constructor accepts wrong type without clear error")
    print("  Code that fails:")
    print("    composer = TaskComposer(robot.env)")
    print()
    print("  Actual error:")
    print("    AttributeError: 'Environment' object has no attribute 'createTaskComposerNode'")
    print()
    print("  Expected error:")
    print("    TypeError: TaskComposer expects TaskComposerPluginFactory, got Environment.")
    print("    Use TaskComposer.from_config() instead.")
    print()


def issue_2_config_discovery_silent_failure():
    """
    Issue 2: Config file discovery fails silently without listing tried paths.

    When TESSERACT_TASK_COMPOSER_CONFIG_FILE is not set and no bundled config
    exists, the error doesn't explain what paths were attempted.

    Suggested fix: Include attempted paths in error message.
    """
    import os
    from tesseract_robotics.planning import TaskComposer

    # Clear env var to trigger fallback behavior
    old_val = os.environ.pop("TESSERACT_TASK_COMPOSER_CONFIG_FILE", None)
    old_dir = os.environ.pop("TESSERACT_TASK_COMPOSER_DIR", None)

    try:
        composer = TaskComposer.from_config()
    except ValueError as e:
        print(f"Issue 2: Config discovery error doesn't list attempted paths:")
        print(f"  {e}")
        print(f"  Expected: List of paths that were tried")
        print()
    finally:
        # Restore
        if old_val:
            os.environ["TESSERACT_TASK_COMPOSER_CONFIG_FILE"] = old_val
        if old_dir:
            os.environ["TESSERACT_TASK_COMPOSER_DIR"] = old_dir


def issue_3_plan_freespace_uses_trajopt():
    """
    Issue 3: plan_freespace() uses TrajOpt, not OMPL.

    The method name suggests OMPL-style sampling-based freespace planning,
    but it actually uses TrajOptPipeline (optimization-based).

    Location: composer.py:336
        def plan_freespace(self, ...):
            return self.plan(robot, program, pipeline="TrajOptPipeline", ...)

    Suggested fix: Either rename to plan_trajopt() or add plan_ompl() method.
    """
    print("Issue 3: plan_freespace() misleadingly uses TrajOpt instead of OMPL")
    print("  Current implementation (composer.py:336):")
    print("    def plan_freespace(self, ...):")
    print('        return self.plan(..., pipeline="TrajOptPipeline", ...)')
    print()
    print("  For actual OMPL planning, users must call:")
    print('    composer.plan(robot, program, pipeline="OMPLPipeline")')
    print()


def issue_4_motionprogram_requires_group():
    """
    Issue 4: MotionProgram requires group_name but error is confusing.

    The __init__ requires group_name as first positional arg, but this
    isn't obvious from examples or error message.

    Suggested fix: Better error message or docstring examples.
    """
    from tesseract_robotics.planning import MotionProgram

    try:
        program = MotionProgram()
    except TypeError as e:
        print(f"Issue 4: MotionProgram error doesn't explain what group_name is:")
        print(f"  {e}")
        print(f"  Expected: Explain that group_name is the kinematic group (e.g., 'manipulator')")
        print()


def issue_5_no_ompl_configuration_examples():
    """
    Issue 5: No way to configure OMPL planners via high-level API.

    The high-level API abstracts away planner configuration. Users wanting
    specific OMPL settings (RRT*, PRM, custom parameters) must use low-level API.

    Suggested fix: Add profile configuration or document when to use low-level API.
    """
    print("Issue 5: No OMPL configuration in high-level API")
    print("  Users cannot easily specify:")
    print("    - Specific OMPL planner (RRTConnect, RRT*, PRM, etc.)")
    print("    - Planning time limits")
    print("    - Planner-specific parameters")
    print()
    print("  Must use low-level API instead:")
    print("    from tesseract_robotics.tesseract_motion_planners_ompl import (")
    print("        OMPLMotionPlanner, OMPLRealVectorPlanProfile")
    print("    )")
    print()


def issue_6_tesseract_support_path_discovery():
    """
    Issue 6: Resource locator doesn't auto-discover tesseract_support.

    When loading URDF/SRDF files that reference package:// URIs or relative
    plugin configs, the resource locator fails with cryptic "new_url" errors
    unless TESSERACT_RESOURCE_PATH or TESSERACT_SUPPORT_DIR is manually set.

    Example error output:
        Error: new_url: /path/to/ur5_plugins.yaml
               at line 287 in .../tesseract_common/src/resource_locator.cpp

    The install tree (ws/install/share/tesseract_support) should be discoverable
    automatically based on the tesseract_robotics package location.

    Suggested fix:
    - Auto-detect ws/install/share relative to tesseract_robotics.__file__
    - Or provide clear error message about required env vars
    - Or add Robot.from_files(..., resource_path=...) parameter
    """
    print("Issue 6: tesseract_support path not auto-discovered")
    print("  When loading robots, resource locator fails with cryptic errors:")
    print("    Error: new_url: /path/to/ur5_plugins.yaml")
    print("           at line 287 in .../resource_locator.cpp")
    print()
    print("  Users must manually set environment variables:")
    print("    TESSERACT_RESOURCE_PATH=/path/to/ws/install/share")
    print("    TESSERACT_SUPPORT_DIR=/path/to/ws/install/share/tesseract_support")
    print()
    print("  Suggested fixes:")
    print("    - Auto-detect ws/install/share relative to tesseract_robotics package")
    print("    - Add resource_path parameter to Robot.from_files()")
    print("    - Improve error message to explain required env vars")
    print()


def main():
    """Run all issue demonstrations."""
    print("=" * 70)
    print("tesseract_robotics.planning API Issues")
    print("=" * 70)
    print()

    try:
        issue_1_taskcomposer_constructor_error()
    except Exception as e:
        print(f"Could not run issue 1: {e}\n")

    try:
        issue_2_config_discovery_silent_failure()
    except Exception as e:
        print(f"Could not run issue 2: {e}\n")

    issue_3_plan_freespace_uses_trajopt()

    try:
        issue_4_motionprogram_requires_group()
    except Exception as e:
        print(f"Could not run issue 4: {e}\n")

    issue_5_no_ompl_configuration_examples()
    issue_6_tesseract_support_path_discovery()

    print("=" * 70)
    print("Report these issues to:")
    print("https://github.com/tesseract-robotics/tesseract_python_nanobind/issues")
    print("=" * 70)


if __name__ == "__main__":
    main()
