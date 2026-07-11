from math import inf
from math import nan

from attrs import evolve
import pytest
from tesseract_robotics.planning.profiles import create_freespace_pipeline_profiles

from compas_fab.backends.interfaces.planner_capabilities import ConfigurationTolerancePolicy, PlannerCapabilities, PlannerImplementationId
from compas_fab.backends.interfaces.planner_errors import InvalidPlannerCapabilitiesError, InvalidPlannerOptionsError, PlannerCapabilityError
from compas_fab.backends.interfaces.planner_operation import PlannerOperation
from compas_fab.backends.interfaces.planner_options import OptionIdentityState, PlanMotionLegacyOptions, ResolvedPlannerOptions
from compas_fab.backends.kinematics.options import UnsupportedPlanMotionOptions as AnalyticalOptions
from compas_fab.backends.kinematics.planner import AnalyticalKinematicsPlanner
from compas_fab.backends.kinematics.planner import AnalyticalPyBulletPlanner
from compas_fab.backends.pybullet.options import UnsupportedPlanMotionOptions as PyBulletOptions
from compas_fab.backends.pybullet.planner import PyBulletPlanner
from compas_fab.backends.ros.exceptions import InvalidMoveItPlanMotionOptionsError
from compas_fab.backends.ros.options import MoveItPlanMotionOptions
from compas_fab.backends.ros.planner import MoveItPlanner
from compas_fab.backends.tesseract.options import TesseractPlanOptions
from compas_fab.backends.tesseract.planner import TesseractPlanner


class _TupleSubclass(tuple):
    pass


def test_all_five_exact_declarations_and_adapters() -> None:
    expected = (
        (AnalyticalKinematicsPlanner, "compas_fab.analytical/v1", AnalyticalOptions, {PlannerOperation.INVERSE_KINEMATICS, PlannerOperation.PLAN_CARTESIAN_MOTION}, ConfigurationTolerancePolicy.LEGACY_DEFAULTS),
        (AnalyticalPyBulletPlanner, "compas_fab.analytical_pybullet/v1", AnalyticalOptions, {PlannerOperation.INVERSE_KINEMATICS, PlannerOperation.PLAN_CARTESIAN_MOTION, PlannerOperation.CHECK_COLLISION}, ConfigurationTolerancePolicy.LEGACY_DEFAULTS),
        (PyBulletPlanner, "compas_fab.pybullet/v1", PyBulletOptions, {PlannerOperation.INVERSE_KINEMATICS, PlannerOperation.PLAN_CARTESIAN_MOTION, PlannerOperation.CHECK_COLLISION}, ConfigurationTolerancePolicy.LEGACY_DEFAULTS),
        (MoveItPlanner, "compas_fab.moveit/v1", MoveItPlanMotionOptions, set(PlannerOperation), ConfigurationTolerancePolicy.LEGACY_DEFAULTS),
        (TesseractPlanner, "compas_fab.tesseract/v1", TesseractPlanOptions, {PlannerOperation.INVERSE_KINEMATICS, PlannerOperation.PLAN_MOTION, PlannerOperation.CHECK_COLLISION}, ConfigurationTolerancePolicy.PRESERVE_ABSENT),
    )
    for planner, implementation_id, adapter, operations, tolerance_policy in expected:
        assert planner.implementation_id.value == implementation_id
        assert planner.capabilities.implementation_id == planner.implementation_id
        assert planner.plan_motion_options is adapter
        assert set(planner.capabilities.operations) == operations
        assert planner.capabilities.configuration_tolerance_policy is tolerance_policy


def test_raw_capability_and_option_constructors_revalidate() -> None:
    capabilities = PlannerCapabilities.build(PlannerImplementationId.build("test/v1"), (PlannerOperation.PLAN_MOTION,), ConfigurationTolerancePolicy.LEGACY_DEFAULTS)
    with pytest.raises(InvalidPlannerCapabilitiesError):
        evolve(capabilities, configuration_tolerance_policy="legacy_defaults")
    for invalid_time in (nan, inf, -inf):
        with pytest.raises(InvalidPlannerOptionsError):
            PlanMotionLegacyOptions(None, None, invalid_time)
    verified = ResolvedPlannerOptions.verified({"planner_id": "RRTConnect"})
    with pytest.raises(InvalidPlannerOptionsError):
        evolve(verified, identity_state="verified")


def test_capability_build_snapshots_mutable_operation_sequence() -> None:
    operations = [PlannerOperation.PLAN_MOTION]
    capabilities = PlannerCapabilities.build(
        PlannerImplementationId.build("test/v1"),
        operations,
        ConfigurationTolerancePolicy.LEGACY_DEFAULTS,
    )

    operations.append(PlannerOperation.CHECK_COLLISION)

    assert capabilities.operations == (PlannerOperation.PLAN_MOTION,)


@pytest.mark.parametrize(
    "operations",
    (
        [PlannerOperation.PLAN_MOTION],
        _TupleSubclass((PlannerOperation.PLAN_MOTION,)),
        ("plan_motion",),
        (([],),),
        (PlannerOperation.PLAN_MOTION, PlannerOperation.PLAN_MOTION),
    ),
)
def test_raw_capabilities_reject_mutable_malformed_and_duplicate_operations(operations) -> None:
    with pytest.raises(InvalidPlannerCapabilitiesError):
        PlannerCapabilities(
            PlannerImplementationId.build("test/v1"),
            operations,
            ConfigurationTolerancePolicy.LEGACY_DEFAULTS,
        )


@pytest.mark.parametrize(
    "values",
    (
        [("opaque", object())],
        _TupleSubclass((("opaque", object()),)),
        (["opaque", object()],),
        (_TupleSubclass(("opaque", object())),),
        (("opaque",),),
        (([], object()),),
        (("", object()),),
        (("opaque", object()), ("opaque", object())),
    ),
)
def test_raw_resolved_options_reject_mutable_malformed_and_duplicate_values(values) -> None:
    with pytest.raises(InvalidPlannerOptionsError):
        ResolvedPlannerOptions(values, OptionIdentityState.UNVERIFIABLE, None)


def test_raw_resolved_options_revalidate_digest_rules() -> None:
    verified = ResolvedPlannerOptions.verified({"planner_id": "RRTConnect"})
    with pytest.raises(InvalidPlannerOptionsError):
        evolve(verified, identity_digest="invalid")
    unverifiable = ResolvedPlannerOptions.unverifiable({"opaque": object()})
    with pytest.raises(InvalidPlannerOptionsError):
        evolve(unverifiable, identity_digest="caller-digest")


def test_verified_options_reject_mixed_invalid_key_types_before_sorting() -> None:
    with pytest.raises(InvalidPlannerOptionsError):
        ResolvedPlannerOptions.verified({"valid": 1, 2: 3})


def test_unverifiable_options_snapshot_mapping_and_retain_exact_opaque_value() -> None:
    opaque = []
    source = {"opaque": opaque}
    resolved = ResolvedPlannerOptions.unverifiable(source)

    source["later"] = object()
    opaque.append("retained")

    backend_options = resolved.to_backend_options()
    assert backend_options == {"opaque": ["retained"]}
    assert backend_options["opaque"] is opaque


@pytest.mark.parametrize("planner", (AnalyticalKinematicsPlanner, AnalyticalPyBulletPlanner, PyBulletPlanner))
def test_unsupported_free_motion_adapters_fail_named(planner) -> None:
    with pytest.raises(PlannerCapabilityError):
        planner.plan_motion_options.resolve(PlanMotionLegacyOptions.build(None, None, None), None)


def test_moveit_rejects_noncanonical_and_native_profiles_are_unverifiable() -> None:
    legacy = PlanMotionLegacyOptions.build("RRTConnect", 2, 1.0)
    assert MoveItPlanMotionOptions.resolve(legacy, None).cacheable
    with pytest.raises(InvalidMoveItPlanMotionOptionsError):
        MoveItPlanMotionOptions.resolve(PlanMotionLegacyOptions.build(None, None, None), {"planner_id": object()})
    with pytest.raises(InvalidPlannerOptionsError):
        ResolvedPlannerOptions.verified({"opaque": object()})
    profiles = create_freespace_pipeline_profiles()
    resolved = TesseractPlanOptions.resolve(
        PlanMotionLegacyOptions.build(None, None, None),
        {"profiles": profiles},
    )
    assert resolved.to_backend_options()["profiles"] is profiles
    assert resolved.identity_state is OptionIdentityState.UNVERIFIABLE
    assert not resolved.cacheable


def test_moveit_preserves_base_and_exact_opaque_constraints() -> None:
    empty = PlanMotionLegacyOptions.build(None, None, None)
    base = MoveItPlanMotionOptions.resolve(empty, {"base_link": "world"})
    assert base.to_backend_options()["base_link"] == "world"
    assert base.cacheable
    for name in ("path_constraints", "trajectory_constraints"):
        constraint_tuple = (object(),)
        resolved = MoveItPlanMotionOptions.resolve(empty, {name: constraint_tuple})
        assert resolved.to_backend_options()[name] is constraint_tuple
        assert resolved.identity_state is OptionIdentityState.UNVERIFIABLE
        assert not resolved.cacheable
