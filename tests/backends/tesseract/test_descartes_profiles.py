from math import pi
from math import radians

import numpy as np
import pytest
from tesseract_robotics.tesseract_command_language import ProfileDictionary
from tesseract_robotics.tesseract_motion_planners_descartes import DescartesDefaultMoveProfileD

from compas_fab.backends.tesseract.descartes_profiles import build_descartes_profiles
from compas_fab.backends.tesseract.errors import InvalidTesseractDescartesProfileError


def test_unconnected_options_are_forwarded_as_none(mocker):
    expected = ProfileDictionary()
    native = mocker.patch(
        "compas_fab.backends.tesseract.descartes_profiles.create_descartes_pipeline_profiles",
        return_value=expected,
    )

    profiles = build_descartes_profiles(
        None,
        None,
        None,
        None,
        None,
        None,
        None,
        None,
        None,
        None,
        None,
    )

    assert profiles is expected
    native.assert_called_once_with(
        profile_names=None,
        enable_collision=None,
        enable_edge_collision=None,
        num_threads=None,
        sample_axis=None,
        sample_resolution=None,
        sample_min=None,
        sample_max=None,
        ik_solver=None,
        use_redundant_joint_solutions=None,
        move_profile=None,
    )


def test_full_range_one_degree_redundancy_is_forwarded_exactly(mocker):
    native = mocker.patch(
        "compas_fab.backends.tesseract.descartes_profiles.create_descartes_pipeline_profiles",
        return_value=ProfileDictionary(),
    )

    build_descartes_profiles(
        ["DEFAULT", "CARTESIAN"],
        True,
        False,
        8,
        [0, 0, 1],
        radians(1),
        -pi,
        pi,
        "OPWInvKin",
        True,
        None,
    )

    kwargs = native.call_args.kwargs
    assert kwargs["profile_names"] == ["DEFAULT", "CARTESIAN"]
    assert kwargs["enable_collision"] is True
    assert kwargs["enable_edge_collision"] is False
    assert kwargs["num_threads"] == 8
    np.testing.assert_array_equal(kwargs["sample_axis"], [0.0, 0.0, 1.0])
    assert kwargs["sample_resolution"] == pytest.approx(radians(1))
    assert kwargs["sample_min"] == pytest.approx(-pi)
    assert kwargs["sample_max"] == pytest.approx(pi)
    assert kwargs["ik_solver"] == "OPWInvKin"
    assert kwargs["use_redundant_joint_solutions"] is True
    assert kwargs["move_profile"] is None


def test_default_boundary_builds_exact_native_dictionary():
    profiles = build_descartes_profiles(
        None,
        None,
        None,
        None,
        None,
        None,
        None,
        None,
        None,
        None,
        None,
    )

    assert isinstance(profiles, ProfileDictionary)


@pytest.mark.parametrize(
    "profile_names",
    [[], [""], ["DEFAULT", "DEFAULT"], [1], "DEFAULT"],
)
def test_invalid_profile_names_fail(profile_names):
    with pytest.raises(InvalidTesseractDescartesProfileError):
        build_descartes_profiles(
            profile_names,
            None,
            None,
            None,
            None,
            None,
            None,
            None,
            None,
            None,
            None,
        )


@pytest.mark.parametrize(
    "axis",
    [[0, 0], [0, 0, 0], [0, float("nan"), 1], [[0, 0, 1]], "Z"],
)
def test_invalid_sample_axis_fails(axis):
    with pytest.raises(InvalidTesseractDescartesProfileError):
        build_descartes_profiles(
            None,
            None,
            None,
            None,
            axis,
            None,
            None,
            None,
            None,
            None,
            None,
        )


@pytest.mark.parametrize(
    ("enable_collision", "enable_edge_collision", "num_threads", "redundant"),
    [
        (1, None, None, None),
        (None, 0, None, None),
        (None, None, True, None),
        (None, None, 0, None),
        (None, None, -1, None),
        (None, None, None, 1),
    ],
)
def test_invalid_boolean_or_thread_options_fail(
    enable_collision,
    enable_edge_collision,
    num_threads,
    redundant,
):
    with pytest.raises(InvalidTesseractDescartesProfileError):
        build_descartes_profiles(
            None,
            enable_collision,
            enable_edge_collision,
            num_threads,
            None,
            None,
            None,
            None,
            None,
            redundant,
            None,
        )


@pytest.mark.parametrize(
    ("resolution", "lower", "upper"),
    [
        (0.0, None, None),
        (-1.0, None, None),
        (float("inf"), None, None),
        (None, float("nan"), None),
        (None, None, float("inf")),
        (None, 1.0, -1.0),
        (True, None, None),
    ],
)
def test_invalid_sample_angles_fail(resolution, lower, upper):
    with pytest.raises(InvalidTesseractDescartesProfileError):
        build_descartes_profiles(
            None,
            None,
            None,
            None,
            None,
            resolution,
            lower,
            upper,
            None,
            None,
            None,
        )


@pytest.mark.parametrize("solver", ["", "   ", 1])
def test_invalid_ik_solver_fails(solver):
    with pytest.raises(InvalidTesseractDescartesProfileError):
        build_descartes_profiles(
            None,
            None,
            None,
            None,
            None,
            None,
            None,
            None,
            solver,
            None,
            None,
        )


def test_custom_move_profile_is_forwarded_with_solver_threads(mocker):
    custom = DescartesDefaultMoveProfileD()
    native = mocker.patch(
        "compas_fab.backends.tesseract.descartes_profiles.create_descartes_pipeline_profiles",
        return_value=ProfileDictionary(),
    )

    build_descartes_profiles(
        ["DEFAULT"],
        None,
        None,
        4,
        None,
        None,
        None,
        None,
        None,
        None,
        custom,
    )

    assert native.call_args.kwargs["move_profile"] is custom
    assert native.call_args.kwargs["num_threads"] == 4


@pytest.mark.parametrize(
    "conflicting_index",
    [1, 2, 4, 5, 6, 7, 8, 9],
)
def test_custom_move_profile_conflicts_fail_before_native_call(
    conflicting_index,
):
    values = [None] * 11
    values[conflicting_index] = [0, 0, 1] if conflicting_index == 4 else True
    if conflicting_index in (5, 6, 7):
        values[conflicting_index] = 1.0
    if conflicting_index == 8:
        values[conflicting_index] = "OPWInvKin"
    values[10] = DescartesDefaultMoveProfileD()

    with pytest.raises(
        InvalidTesseractDescartesProfileError,
        match="full override",
    ):
        build_descartes_profiles(*values)


def test_wrong_custom_move_profile_type_fails():
    with pytest.raises(InvalidTesseractDescartesProfileError):
        build_descartes_profiles(
            None,
            None,
            None,
            None,
            None,
            None,
            None,
            None,
            None,
            None,
            object(),
        )
