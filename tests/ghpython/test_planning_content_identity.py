import pytest
from attrs import evolve
from compas.geometry import Frame
from tesseract_robotics.planning import MoveType
from tesseract_robotics.tesseract_command_language import ProfileDictionary

from compas_fab.identity_verification import IdentityVerification
from compas_fab.ghpython.branch_runtime_identity import TreeRuntimeSnapshot
from compas_fab.ghpython.item_values import Scalar
from compas_fab.ghpython.planning_content_identity import ComputeToken
from compas_fab.ghpython.planning_content_identity import InvalidUnverifiableProfileIdentityError
from compas_fab.ghpython.planning_content_identity import PlanningSharedInputs
from compas_fab.ghpython.planning_content_identity import PlanningTreeIdentity
from compas_fab.ghpython.planning_content_identity import ProfileGeneration
from compas_fab.ghpython.planning_content_identity import UnverifiableProfileIdentity
from compas_fab.ghpython.planning_content_identity import build_branch_plan_requests
from compas_fab.ghpython.tesseract_cartesian_target_series import CartesianTargetSeriesParameters
from compas_fab.ghpython.tesseract_cartesian_target_series import build_cartesian_target_series
from compas_fab.ghpython.tesseract_pose_series import PoseSeriesParameters
from compas_fab.ghpython.tesseract_pose_series import build_pose_series
from compas_fab.ghpython.tesseract_program_series import ProgramSeriesParameters
from compas_fab.ghpython.tesseract_program_series import build_motion_program_series
from compas_fab.ghpython.tree_coordinates import GhPath
from compas_fab.ghpython.tree_coordinates import TreeRootId
from compas_fab.ghpython.tree_values import Tree
from compas_fab.ghpython.tree_values import TreeBranch
from compas_fab.ghpython.tree_values import TreeItem


def _frames() -> Tree[Frame]:
    return Tree.build(
        TreeRootId.build("frames"),
        (
            TreeBranch.build(GhPath.build(0), (TreeItem.value(Frame.worldXY()),)),
            TreeBranch.build(GhPath.build(4, 1), (TreeItem.value(Frame.worldXY()),)),
        ),
    )


def _programs(native_robot):
    poses = build_pose_series(_frames(), PoseSeriesParameters.build(1.0, "base"))
    targets = build_cartesian_target_series(poses, CartesianTargetSeriesParameters.build(Scalar.build(MoveType.LINEAR), Scalar.build("DEFAULT")))
    return build_motion_program_series(native_robot, targets, ProgramSeriesParameters.build("manipulator", None, "base", "DEFAULT"))


def _shared(planner, profiles, *, pipeline="DescartesFPipeline", generation=3, auto_seed=True, token="solve-7"):
    return PlanningSharedInputs.build(
        planner,
        planner.native_artifact_digest,
        planner.native_scene_content_identity,
        pipeline,
        UnverifiableProfileIdentity.build(ProfileGeneration.build(generation)),
        profiles,
        auto_seed,
        ComputeToken.build(token),
    )


def _runtime(planning_identity, programs):
    return TreeRuntimeSnapshot.initial(planning_identity.tree_identity, programs.output.values.root_id)


def test_program_tree_and_shared_native_inputs_bridge_to_branch_requests(tesseract_planner, native_robot) -> None:
    programs = _programs(native_robot)
    profiles = ProfileDictionary()
    shared = _shared(tesseract_planner, profiles)
    planning_identity = PlanningTreeIdentity.build(programs, shared)
    runtime = _runtime(planning_identity, programs)

    requests = build_branch_plan_requests(programs, shared, planning_identity, runtime)

    assert [request.identity.coordinate.path for request in requests] == [GhPath.build(0), GhPath.build(4, 1)]
    assert all(request.call.planner is tesseract_planner for request in requests)
    assert all(request.call.request.profiles is profiles for request in requests)
    assert all(request.content_identity == planning_identity.branch(request.identity.coordinate.path) for request in requests)
    assert all(request.compute_token == ComputeToken.build("solve-7") for request in requests)
    assert all(request.attempt.runtime_identity == request.identity for request in requests)
    assert all(request.attempt.compute_token == request.compute_token for request in requests)
    assert planning_identity.verification is IdentityVerification.UNVERIFIABLE


def test_new_token_changes_attempt_not_content_identity(tesseract_planner, native_robot) -> None:
    programs = _programs(native_robot)
    profiles = ProfileDictionary()

    def build_request(token):
        shared = _shared(tesseract_planner, profiles, token=token)
        planning_identity = PlanningTreeIdentity.build(programs, shared)
        runtime = _runtime(planning_identity, programs)
        return build_branch_plan_requests(programs, shared, planning_identity, runtime)[0]

    first = build_request("attempt-a")
    repeated = build_request("attempt-a")
    second = build_request("attempt-b")

    assert first.attempt == repeated.attempt
    assert first.attempt.id != second.attempt.id
    assert first.content_identity == second.content_identity


def test_planning_branch_identity_binds_native_digest_while_authoring_is_deterministic(tesseract_planner, native_robot) -> None:
    poses = build_pose_series(_frames(), PoseSeriesParameters.build(1.0, "base"))
    targets = build_cartesian_target_series(poses, CartesianTargetSeriesParameters.build(Scalar.build(MoveType.LINEAR), Scalar.build("DEFAULT")))
    program_parameters = ProgramSeriesParameters.build("manipulator", None, "base", "DEFAULT")
    programs_a = build_motion_program_series(native_robot, targets, program_parameters)
    programs_b = build_motion_program_series(native_robot, targets, program_parameters)
    profiles = ProfileDictionary()

    assert programs_a.identity == programs_b.identity
    left = PlanningTreeIdentity.build(programs_a, _shared(tesseract_planner, profiles))
    right = PlanningTreeIdentity.build(programs_b, _shared(tesseract_planner, profiles))
    assert left.tree_identity != right.tree_identity
    assert left.branch(GhPath.build(0)) != right.branch(GhPath.build(0))


def test_planning_identity_is_unverifiable_and_requires_fresh_token(tesseract_planner, native_robot) -> None:
    programs = _programs(native_robot)
    planning_identity = PlanningTreeIdentity.build(programs, _shared(tesseract_planner, ProfileDictionary()))

    assert planning_identity.verification is IdentityVerification.UNVERIFIABLE
    assert planning_identity.tree_identity.requires_fresh_compute_token is True
    assert planning_identity.tree_identity.reusable_from_content_cache is False


def test_unverifiable_profile_identity_cannot_be_forged_verified() -> None:
    identity = UnverifiableProfileIdentity.build(ProfileGeneration.build(0))
    assert identity.verification is IdentityVerification.UNVERIFIABLE
    with pytest.raises(InvalidUnverifiableProfileIdentityError):
        evolve(identity, verification=IdentityVerification.VERIFIED)


def test_compute_token_never_enters_content_identity(tesseract_planner, native_robot) -> None:
    programs = _programs(native_robot)
    profiles = ProfileDictionary()
    left = PlanningTreeIdentity.build(programs, _shared(tesseract_planner, profiles, token="solve-a"))
    right = PlanningTreeIdentity.build(programs, _shared(tesseract_planner, profiles, token="solve-b"))

    assert left.tree_identity == right.tree_identity


@pytest.mark.parametrize("field", ["pipeline", "generation", "auto_seed"])
def test_shared_input_changes_alter_planning_identity(tesseract_planner, native_robot, field) -> None:
    programs = _programs(native_robot)
    profiles = ProfileDictionary()
    baseline = PlanningTreeIdentity.build(programs, _shared(tesseract_planner, profiles))
    overrides = {
        "pipeline": {"pipeline": "DescartesDPipeline"},
        "generation": {"generation": 4},
        "auto_seed": {"auto_seed": False},
    }[field]
    changed = PlanningTreeIdentity.build(programs, _shared(tesseract_planner, profiles, **overrides))

    assert changed.tree_identity != baseline.tree_identity


def test_direct_scene_mutation_alters_planning_identity_and_scene_verification(tesseract_client, tesseract_planner, native_robot) -> None:
    programs = _programs(native_robot)
    profiles = ProfileDictionary()
    before = PlanningTreeIdentity.build(programs, _shared(tesseract_planner, profiles))
    assert tesseract_planner.native_scene_content_identity.verification is IdentityVerification.VERIFIED

    tesseract_client._mark_native_scene_changed()

    assert tesseract_planner.native_scene_content_identity.verification is IdentityVerification.UNVERIFIABLE
    after = PlanningTreeIdentity.build(programs, _shared(tesseract_planner, profiles))
    assert after.tree_identity != before.tree_identity
