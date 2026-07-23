import pytest
from tesseract_robotics.tesseract_command_language import ProfileDictionary

from compas_fab.backends.tesseract.errors import InvalidTesseractOmplProfileError
from compas_fab.backends.tesseract.ompl_profiles import build_ompl_profiles


def test_unconnected_options_omit_every_native_argument(mocker):
    expected = ProfileDictionary()
    native = mocker.patch(
        "compas_fab.backends.tesseract.ompl_profiles.create_ompl_default_profiles",
        return_value=expected,
    )

    profiles = build_ompl_profiles(None, None, None, None, None, None, None)

    assert profiles is expected
    native.assert_called_once_with()


def test_supplied_options_are_forwarded_exactly(mocker):
    native = mocker.patch(
        "compas_fab.backends.tesseract.ompl_profiles.create_ompl_default_profiles",
        return_value=ProfileDictionary(),
    )

    build_ompl_profiles(["DEFAULT", "FREESPACE"], 5.0, 3, False, True, 8, 0.25)

    native.assert_called_once_with(
        profile_names=["DEFAULT", "FREESPACE"],
        planning_time=5.0,
        max_solutions=3,
        optimize=False,
        simplify=True,
        num_planners=8,
        planner_range=0.25,
    )


def test_explicit_false_optimize_and_simplify_are_forwarded_not_dropped(mocker):
    native = mocker.patch(
        "compas_fab.backends.tesseract.ompl_profiles.create_ompl_default_profiles",
        return_value=ProfileDictionary(),
    )

    build_ompl_profiles(None, None, None, False, False, None, None)

    native.assert_called_once_with(optimize=False, simplify=False)


def test_default_boundary_builds_exact_native_dictionary():
    profiles = build_ompl_profiles(None, None, None, None, None, None, None)

    assert isinstance(profiles, ProfileDictionary)


@pytest.mark.parametrize("profile_names", [[], [""], ["DEFAULT", "DEFAULT"], [1], "DEFAULT"])
def test_invalid_profile_names_fail(profile_names):
    with pytest.raises(InvalidTesseractOmplProfileError):
        build_ompl_profiles(profile_names, None, None, None, None, None, None)


@pytest.mark.parametrize(
    ("planning_time", "max_solutions", "optimize", "simplify", "num_planners", "planner_range"),
    [
        (0.0, None, None, None, None, None),
        (float("inf"), None, None, None, None, None),
        (None, -2, None, None, None, None),
        (None, True, None, None, None, None),
        (None, None, 1, None, None, None),
        (None, None, None, "yes", None, None),
        (None, None, None, None, 0, None),
        (None, None, None, None, None, -0.1),
    ],
)
def test_invalid_scalar_options_fail(planning_time, max_solutions, optimize, simplify, num_planners, planner_range):
    with pytest.raises(InvalidTesseractOmplProfileError):
        build_ompl_profiles(None, planning_time, max_solutions, optimize, simplify, num_planners, planner_range)
