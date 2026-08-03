import pytest
from tesseract_robotics.tesseract_command_language import ProfileDictionary

from compas_fab.backends.tesseract.errors import InvalidTesseractTrajOptProfileError
from compas_fab.backends.tesseract.trajopt_profiles import build_trajopt_profiles


def test_unconnected_option_is_forwarded_as_none(mocker):
    expected = ProfileDictionary()
    native = mocker.patch(
        "compas_fab.backends.tesseract.trajopt_profiles.create_trajopt_default_profiles",
        return_value=expected,
    )

    profiles = build_trajopt_profiles(None)

    assert profiles is expected
    native.assert_called_once_with(profile_names=None)


def test_supplied_profile_names_are_forwarded_exactly(mocker):
    native = mocker.patch(
        "compas_fab.backends.tesseract.trajopt_profiles.create_trajopt_default_profiles",
        return_value=ProfileDictionary(),
    )

    build_trajopt_profiles(["DEFAULT", "RASTER"])

    native.assert_called_once_with(profile_names=["DEFAULT", "RASTER"])


def test_default_boundary_builds_exact_native_dictionary():
    profiles = build_trajopt_profiles(None)

    assert isinstance(profiles, ProfileDictionary)


@pytest.mark.parametrize("profile_names", [[], [""], ["DEFAULT", "DEFAULT"], [1], "DEFAULT"])
def test_invalid_profile_names_fail(profile_names):
    with pytest.raises(InvalidTesseractTrajOptProfileError):
        build_trajopt_profiles(profile_names)
