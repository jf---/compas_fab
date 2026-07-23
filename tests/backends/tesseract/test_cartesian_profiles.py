import pytest
from tesseract_robotics.tesseract_command_language import ProfileDictionary

from compas_fab.backends.tesseract.cartesian_profiles import build_cartesian_profiles
from compas_fab.backends.tesseract.errors import InvalidTesseractCartesianProfileError


def test_unconnected_options_are_forwarded_as_none(mocker):
    expected = ProfileDictionary()
    native = mocker.patch(
        "compas_fab.backends.tesseract.cartesian_profiles.create_cartesian_pipeline_profiles",
        return_value=expected,
    )

    profiles = build_cartesian_profiles(None, None)

    assert profiles is expected
    native.assert_called_once_with(profile_names=None, num_threads=None)


def test_supplied_options_are_forwarded_exactly(mocker):
    native = mocker.patch(
        "compas_fab.backends.tesseract.cartesian_profiles.create_cartesian_pipeline_profiles",
        return_value=ProfileDictionary(),
    )

    build_cartesian_profiles(["DEFAULT", "CARTESIAN"], 8)

    native.assert_called_once_with(profile_names=["DEFAULT", "CARTESIAN"], num_threads=8)


def test_default_boundary_builds_exact_native_dictionary():
    profiles = build_cartesian_profiles(None, None)

    assert isinstance(profiles, ProfileDictionary)


@pytest.mark.parametrize("profile_names", [[], [""], ["DEFAULT", "DEFAULT"], [1], "DEFAULT"])
def test_invalid_profile_names_fail(profile_names):
    with pytest.raises(InvalidTesseractCartesianProfileError):
        build_cartesian_profiles(profile_names, None)


@pytest.mark.parametrize("num_threads", [True, 0, -1, 1.5, "8"])
def test_invalid_num_threads_fail(num_threads):
    with pytest.raises(InvalidTesseractCartesianProfileError):
        build_cartesian_profiles(None, num_threads)
