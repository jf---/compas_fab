import pytest
from tesseract_robotics.emitters.rapid import RapidProfile

from compas_fab.backends.tesseract.errors import DuplicateRapidProfileError
from compas_fab.backends.tesseract.errors import InvalidRapidProfileMapError
from compas_fab.backends.tesseract.rapid_profiles import merge_rapid_profile_maps
from compas_fab.backends.tesseract.rapid_profiles import normalize_rapid_profiles


def test_normalize_retains_exact_native_profile_objects():
    profile = RapidProfile()

    normalized = normalize_rapid_profiles({"DEFAULT": profile})

    assert normalized == {"DEFAULT": profile}
    assert normalized["DEFAULT"] is profile


@pytest.mark.parametrize(
    "profiles",
    [
        {"": RapidProfile()},
        {"   ": RapidProfile()},
        {1: RapidProfile()},
        {"P": object()},
        object(),
    ],
)
def test_normalize_rejects_invalid_native_map(profiles):
    with pytest.raises(InvalidRapidProfileMapError):
        normalize_rapid_profiles(profiles)


def test_merge_retains_exact_profile_objects():
    first = RapidProfile()
    second = RapidProfile()

    merged = merge_rapid_profile_maps([{"FIRST": first}, {"SECOND": second}])

    assert merged == {"FIRST": first, "SECOND": second}
    assert merged["FIRST"] is first
    assert merged["SECOND"] is second


def test_merge_rejects_duplicate_exact_tesseract_name():
    with pytest.raises(DuplicateRapidProfileError, match="DEFAULT"):
        merge_rapid_profile_maps(
            [
                {"DEFAULT": RapidProfile()},
                {"DEFAULT": RapidProfile()},
            ]
        )
