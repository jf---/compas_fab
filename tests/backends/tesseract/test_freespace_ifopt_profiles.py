import pytest
from tesseract_robotics.tesseract_command_language import ProfileDictionary

from compas_fab.backends.tesseract.errors import InvalidTesseractFreespaceIfoptProfileError
from compas_fab.backends.tesseract.freespace_ifopt_profiles import build_freespace_ifopt_profiles


def test_unconnected_options_omit_every_native_argument(mocker):
    expected = ProfileDictionary()
    native = mocker.patch(
        "compas_fab.backends.tesseract.freespace_ifopt_profiles.create_freespace_ifopt_pipeline_profiles",
        return_value=expected,
    )

    profiles = build_freespace_ifopt_profiles(None, None, None, None, None, None)

    assert profiles is expected
    native.assert_called_once_with()


def test_supplied_options_are_forwarded_exactly(mocker):
    native = mocker.patch(
        "compas_fab.backends.tesseract.freespace_ifopt_profiles.create_freespace_ifopt_pipeline_profiles",
        return_value=ProfileDictionary(),
    )

    build_freespace_ifopt_profiles(["DEFAULT", "FREESPACE"], 8, 5.0, False, 3, 0.25)

    native.assert_called_once_with(
        profile_names=["DEFAULT", "FREESPACE"],
        num_planners=8,
        planning_time=5.0,
        optimize=False,
        max_solutions=3,
        planner_range=0.25,
    )


def test_explicit_false_optimize_is_forwarded_not_dropped(mocker):
    native = mocker.patch(
        "compas_fab.backends.tesseract.freespace_ifopt_profiles.create_freespace_ifopt_pipeline_profiles",
        return_value=ProfileDictionary(),
    )

    build_freespace_ifopt_profiles(None, None, None, False, None, None)

    native.assert_called_once_with(optimize=False)


def test_default_boundary_builds_exact_native_dictionary():
    profiles = build_freespace_ifopt_profiles(None, None, None, None, None, None)

    assert isinstance(profiles, ProfileDictionary)


@pytest.mark.parametrize("profile_names", [[], [""], ["DEFAULT", "DEFAULT"], [1], "DEFAULT"])
def test_invalid_profile_names_fail(profile_names):
    with pytest.raises(InvalidTesseractFreespaceIfoptProfileError):
        build_freespace_ifopt_profiles(profile_names, None, None, None, None, None)


@pytest.mark.parametrize(
    ("num_planners", "planning_time", "optimize", "max_solutions", "planner_range"),
    [
        (0, None, None, None, None),
        (True, None, None, None, None),
        (None, 0.0, None, None, None),
        (None, float("inf"), None, None, None),
        (None, None, 1, None, None),
        (None, None, None, -2, None),
        (None, None, None, None, -0.1),
        (None, None, None, None, float("nan")),
    ],
)
def test_invalid_scalar_options_fail(num_planners, planning_time, optimize, max_solutions, planner_range):
    with pytest.raises(InvalidTesseractFreespaceIfoptProfileError):
        build_freespace_ifopt_profiles(None, num_planners, planning_time, optimize, max_solutions, planner_range)
