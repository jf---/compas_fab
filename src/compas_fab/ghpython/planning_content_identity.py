"""Planning-tree content identity and the runtime branch-request bridge.

Baseline planning identity is `UNVERIFIABLE`: Tesseract 0.35.0.6 exposes no
canonical `ProfileDictionary` serialization, so an exact profile dictionary can
never be content-bound and a verified profile path is deferred. The content
identity (`PlanningTreeIdentity.tree_identity`) derives the program identity plus
the artifact digest, complete scene identity (verification and direct generation),
pipeline, unverifiable profile generation, auto-seed, and planning schema version,
and binds the exact per-branch `native_program_digest` as branch evidence.

Unlike the reproducible authoring identity (`ProgramSeriesBuild.identity`, which
excludes the UUID-bearing native serialization so identical authoring is stable),
the planning branch identity DOES bind the native digest per the spec. This is
sound precisely because planning identity is UNVERIFIABLE and fresh-compute-token
gated and asserts no cross-build equality. A `ComputeToken` is runtime edge
evidence and never enters the content identity; it distinguishes plan attempts
without altering what content was planned.
"""

from __future__ import annotations

from hashlib import sha256
from typing import List
from typing import Optional
from typing import Tuple
from typing import cast

from attrs import define
from attrs import field
from tesseract_robotics.tesseract_command_language import ProfileDictionary

from compas_fab.backends.tesseract.native_plan import NativePlanCall
from compas_fab.backends.tesseract.native_program_builder import NativeProgramBuild
from compas_fab.backends.tesseract.planner import TesseractPlanner
from compas_fab.backends.tesseract.scene_identity import NativeSceneContentIdentity
from compas_fab.ghpython.branch_runtime_identity import BranchRuntimeIdentity
from compas_fab.ghpython.branch_runtime_identity import TreeRuntimeSnapshot
from compas_fab.ghpython.component_identity import CanonicalField
from compas_fab.ghpython.tesseract_program_series import ProgramSeriesBuild
from compas_fab.ghpython.tree_coordinates import GhPath
from compas_fab.ghpython.tree_identity import BranchContentDigest
from compas_fab.ghpython.tree_identity import StageBranchEvidence
from compas_fab.ghpython.tree_identity import StageParameter
from compas_fab.ghpython.tree_identity import StagePriorBinding
from compas_fab.ghpython.tree_identity import StageTreeIdentity
from compas_fab.identity_verification import IdentityVerification

PLANNING_TREE_SCHEMA = "compas_fab.tesseract.planning_tree_identity/v1"
PLANNING_SCHEMA_VERSION = "1"
PLAN_ATTEMPT_SCHEMA = "compas_fab.tesseract.branch_plan_attempt/v1"
_SHA256_HEX_LENGTH = sha256().digest_size * 2
_PLANNING_FACTORY_TOKEN = object()
_NATIVE_PROGRAM_DIGEST_FIELD = "native_program_digest"
_ABSENT_PROGRAM_FIELD = "program"
_ABSENT_PROGRAM_MARKER = "absent"


class PlanningContentIdentityError(ValueError):
    """Base failure for planning content identity and branch requests."""


class InvalidProfileGenerationError(PlanningContentIdentityError):
    """Raised when a profile generation is not a non-negative integer."""


class InvalidComputeTokenError(PlanningContentIdentityError):
    """Raised when a runtime compute token is malformed."""


class InvalidUnverifiableProfileIdentityError(PlanningContentIdentityError):
    """Raised when a profile identity is forged or is not unverifiable."""


class InvalidPlanningSharedInputsError(PlanningContentIdentityError):
    """Raised when shared native planning inputs disagree with the planner."""


class InvalidPlanningTreeIdentityError(PlanningContentIdentityError):
    """Raised when a planning tree identity is malformed."""


class InvalidBranchPlanAttemptError(PlanningContentIdentityError):
    """Raised when a branch plan attempt is forged or inconsistent."""


class InvalidBranchPlanRequestError(PlanningContentIdentityError):
    """Raised when a branch plan request is forged or inconsistent."""


class MismatchedPlanningRuntimeError(PlanningContentIdentityError):
    """Raised when a runtime snapshot does not match the planning content."""


def _part(payload: bytes) -> bytes:
    return len(payload).to_bytes(8, "big") + payload


def _valid_digest(value: object) -> bool:
    return type(value) is str and len(value) == _SHA256_HEX_LENGTH and value == value.lower() and all(character in "0123456789abcdef" for character in value)


def _path_bytes(path: GhPath) -> bytes:
    return b"".join(_part(str(index).encode("ascii")) for index in path.indices)


@define(frozen=True, slots=True)
class ProfileGeneration:
    """Monotonic epoch of an explicit externally supplied profile dictionary."""

    value: int

    @classmethod
    def build(cls, value: int) -> "ProfileGeneration":
        return cls(value)

    def __attrs_post_init__(self) -> None:
        if type(self.value) is not int or self.value < 0:
            raise InvalidProfileGenerationError("Profile generation must be an exact non-negative integer.")

    def next(self) -> "ProfileGeneration":
        """Return the next explicit profile epoch."""
        return ProfileGeneration.build(self.value + 1)


@define(frozen=True, slots=True)
class ComputeToken:
    """Opaque runtime edge evidence; never part of any content identity."""

    value: str

    @classmethod
    def build(cls, value: str) -> "ComputeToken":
        return cls(value)

    def __attrs_post_init__(self) -> None:
        if type(self.value) is not str or not self.value or self.value != self.value.strip():
            raise InvalidComputeTokenError("Compute token must be non-empty canonical text.")


@define(frozen=True, slots=True)
class UnverifiableProfileIdentity:
    """Exact profile epoch that is always UNVERIFIABLE and never cache-reusable."""

    generation: ProfileGeneration
    verification: IdentityVerification
    _factory_token: Optional[object] = field(default=None, eq=False, repr=False)

    @classmethod
    def build(cls, generation: ProfileGeneration) -> "UnverifiableProfileIdentity":
        if type(generation) is not ProfileGeneration:
            raise InvalidUnverifiableProfileIdentityError("Unverifiable profile identity requires an exact ProfileGeneration.")
        return cls(generation, IdentityVerification.UNVERIFIABLE, _PLANNING_FACTORY_TOKEN)

    def __attrs_post_init__(self) -> None:
        if type(self.generation) is not ProfileGeneration or self.verification is not IdentityVerification.UNVERIFIABLE or self._factory_token is not _PLANNING_FACTORY_TOKEN:
            raise InvalidUnverifiableProfileIdentityError("Profile identity must be factory-built and remain unverifiable.")


@define(frozen=True, slots=True)
class PlanningSharedInputs:
    """Exact native inputs shared by every branch plan in one solve.

    Retains the exact shared planner and profile dictionary objects for the
    runtime edge; the artifact digest and scene identity are the content snapshot
    that must match the planner at build time.
    """

    planner: TesseractPlanner = field(eq=False)
    artifact_digest: str
    scene_identity: NativeSceneContentIdentity
    pipeline: str
    profile_identity: UnverifiableProfileIdentity
    profiles: ProfileDictionary = field(eq=False)
    auto_seed: bool
    compute_token: ComputeToken
    _factory_token: Optional[object] = field(default=None, eq=False, repr=False)

    @classmethod
    def build(
        cls,
        planner: TesseractPlanner,
        artifact_digest: str,
        scene_identity: NativeSceneContentIdentity,
        pipeline: str,
        profile_identity: UnverifiableProfileIdentity,
        profiles: ProfileDictionary,
        auto_seed: bool,
        compute_token: ComputeToken,
    ) -> "PlanningSharedInputs":
        if not isinstance(planner, TesseractPlanner):
            raise InvalidPlanningSharedInputsError("Planning shared inputs require an exact TesseractPlanner.")
        if not _valid_digest(artifact_digest):
            raise InvalidPlanningSharedInputsError("Planning shared inputs require a lowercase SHA-256 artifact digest.")
        if type(scene_identity) is not NativeSceneContentIdentity:
            raise InvalidPlanningSharedInputsError("Planning shared inputs require an exact NativeSceneContentIdentity.")
        if type(pipeline) is not str or not pipeline.strip():
            raise InvalidPlanningSharedInputsError("Planning shared inputs require an exact pipeline name.")
        if type(profile_identity) is not UnverifiableProfileIdentity:
            raise InvalidPlanningSharedInputsError("Planning shared inputs require an exact UnverifiableProfileIdentity.")
        if not isinstance(profiles, ProfileDictionary):
            raise InvalidPlanningSharedInputsError("Planning shared inputs require an exact ProfileDictionary.")
        if type(auto_seed) is not bool:
            raise InvalidPlanningSharedInputsError("Planning shared inputs require an exact auto_seed bool.")
        if type(compute_token) is not ComputeToken:
            raise InvalidPlanningSharedInputsError("Planning shared inputs require an exact ComputeToken.")
        if artifact_digest != planner.native_artifact_digest or scene_identity != planner.native_scene_content_identity:
            raise InvalidPlanningSharedInputsError("Planning shared inputs must snapshot the planner's exact artifact and scene identity.")
        return cls(planner, artifact_digest, scene_identity, pipeline, profile_identity, profiles, auto_seed, compute_token, _PLANNING_FACTORY_TOKEN)

    def __attrs_post_init__(self) -> None:
        if (
            not isinstance(self.planner, TesseractPlanner)
            or not _valid_digest(self.artifact_digest)
            or type(self.scene_identity) is not NativeSceneContentIdentity
            or type(self.pipeline) is not str
            or not self.pipeline.strip()
            or type(self.profile_identity) is not UnverifiableProfileIdentity
            or not isinstance(self.profiles, ProfileDictionary)
            or type(self.auto_seed) is not bool
            or type(self.compute_token) is not ComputeToken
            or self._factory_token is not _PLANNING_FACTORY_TOKEN
        ):
            raise InvalidPlanningSharedInputsError("Planning shared input fields are inconsistent.")


def _program_branch_evidence(programs: ProgramSeriesBuild) -> Tuple[StageBranchEvidence, ...]:
    """Bind the exact per-branch native program digest into planning branch identity.

    Per the spec, planning branch identity uses the existing `native_program_digest`.
    Its cross-build nondeterminism (UUID-bearing native serialization) is acceptable
    here by design: every planning identity is already UNVERIFIABLE and fresh-compute-
    token gated, and no cross-build equality is asserted for it.
    """
    evidence: List[StageBranchEvidence] = []
    for branch, digest in zip(programs.output.values.branches, programs.native_program_digests):
        if digest is None:
            evidence_field = CanonicalField.text(_ABSENT_PROGRAM_FIELD, _ABSENT_PROGRAM_MARKER)
        else:
            evidence_field = CanonicalField.text(_NATIVE_PROGRAM_DIGEST_FIELD, digest.digest)
        evidence.append(StageBranchEvidence.build(branch.path, (evidence_field,)))
    return tuple(evidence)


def _planning_parameters(shared: PlanningSharedInputs) -> Tuple[StageParameter, ...]:
    return (
        StageParameter.text("artifact_digest", shared.artifact_digest),
        StageParameter.text("scene_digest", shared.scene_identity.digest),
        StageParameter.text("scene_verification", shared.scene_identity.verification.value),
        StageParameter.text("scene_direct_generation", str(shared.scene_identity.direct_generation.value)),
        StageParameter.text("pipeline", shared.pipeline),
        StageParameter.text("profile_generation", str(shared.profile_identity.generation.value)),
        StageParameter.bool("auto_seed", shared.auto_seed),
        StageParameter.text("planning_schema_version", PLANNING_SCHEMA_VERSION),
    )


@define(frozen=True, slots=True)
class PlanningTreeIdentity:
    """Unverifiable planning content identity retaining shared runtime objects."""

    tree_identity: StageTreeIdentity
    shared: PlanningSharedInputs = field(eq=False)
    _factory_token: Optional[object] = field(default=None, eq=False, repr=False)

    @classmethod
    def build(cls, programs: ProgramSeriesBuild, shared: PlanningSharedInputs) -> "PlanningTreeIdentity":
        if type(programs) is not ProgramSeriesBuild:
            raise InvalidPlanningTreeIdentityError("Planning identity requires an exact ProgramSeriesBuild.")
        if type(shared) is not PlanningSharedInputs:
            raise InvalidPlanningTreeIdentityError("Planning identity requires exact PlanningSharedInputs.")
        tree_identity = StageTreeIdentity.build(
            programs.identity,
            PLANNING_TREE_SCHEMA,
            _planning_parameters(shared),
            programs.output.values.topology,
            verification=IdentityVerification.UNVERIFIABLE,
            prior_binding=StagePriorBinding.build("programs", programs.output.values.root_id, programs.identity),
            branch_evidence=_program_branch_evidence(programs),
        )
        return cls(tree_identity, shared, _PLANNING_FACTORY_TOKEN)

    def __attrs_post_init__(self) -> None:
        if (
            type(self.tree_identity) is not StageTreeIdentity
            or self.tree_identity.builder_schema != PLANNING_TREE_SCHEMA
            or self.tree_identity.verification is not IdentityVerification.UNVERIFIABLE
            or self.tree_identity.parameters != _planning_parameters(self.shared)
            or self._factory_token is not _PLANNING_FACTORY_TOKEN
        ):
            raise InvalidPlanningTreeIdentityError("Planning tree identity fields are inconsistent.")

    @property
    def verification(self) -> IdentityVerification:
        """Return the baseline planning verification, always UNVERIFIABLE."""
        return self.tree_identity.verification

    def branch(self, path: GhPath) -> BranchContentDigest:
        """Return the content digest for one exact program branch path."""
        return self.tree_identity.branch(path)


@define(frozen=True, slots=True)
class BranchPlanAttemptId:
    """Content-addressed identity of one runtime branch attempt plus its token."""

    value: str

    @classmethod
    def build(cls, runtime_identity: BranchRuntimeIdentity, compute_token: ComputeToken) -> "BranchPlanAttemptId":
        if type(runtime_identity) is not BranchRuntimeIdentity:
            raise InvalidBranchPlanAttemptError("Branch plan attempt id requires an exact BranchRuntimeIdentity.")
        if type(compute_token) is not ComputeToken:
            raise InvalidBranchPlanAttemptError("Branch plan attempt id requires an exact ComputeToken.")
        payload = b"".join(
            (
                _part(PLAN_ATTEMPT_SCHEMA.encode("utf-8")),
                _part(runtime_identity.coordinate.root_id.value.encode("utf-8")),
                _part(_path_bytes(runtime_identity.coordinate.path)),
                _part(runtime_identity.content_digest.value.encode("ascii")),
                _part(str(runtime_identity.solve_generation.value).encode("ascii")),
                _part(str(runtime_identity.request_generation.value).encode("ascii")),
                _part(compute_token.value.encode("utf-8")),
            )
        )
        return cls(sha256(payload).hexdigest())

    def __attrs_post_init__(self) -> None:
        if not _valid_digest(self.value):
            raise InvalidBranchPlanAttemptError("Branch plan attempt id must be lowercase SHA-256 hex.")


@define(frozen=True, slots=True)
class BranchPlanAttempt:
    """One runtime branch attempt bound to an exact compute token."""

    id: BranchPlanAttemptId
    runtime_identity: BranchRuntimeIdentity
    compute_token: ComputeToken
    _factory_token: Optional[object] = field(default=None, eq=False, repr=False)

    @classmethod
    def build(cls, runtime_identity: BranchRuntimeIdentity, compute_token: ComputeToken) -> "BranchPlanAttempt":
        if type(runtime_identity) is not BranchRuntimeIdentity:
            raise InvalidBranchPlanAttemptError("Branch plan attempt requires an exact BranchRuntimeIdentity.")
        if type(compute_token) is not ComputeToken:
            raise InvalidBranchPlanAttemptError("Branch plan attempt requires an exact ComputeToken.")
        return cls(BranchPlanAttemptId.build(runtime_identity, compute_token), runtime_identity, compute_token, _PLANNING_FACTORY_TOKEN)

    def __attrs_post_init__(self) -> None:
        if (
            type(self.id) is not BranchPlanAttemptId
            or type(self.runtime_identity) is not BranchRuntimeIdentity
            or type(self.compute_token) is not ComputeToken
            or self.id != BranchPlanAttemptId.build(self.runtime_identity, self.compute_token)
            or self._factory_token is not _PLANNING_FACTORY_TOKEN
        ):
            raise InvalidBranchPlanAttemptError("Branch plan attempt fields are inconsistent.")


@define(frozen=True, slots=True)
class BranchPlanRequest:
    """One exact native branch plan call bound to content and runtime identity."""

    attempt: BranchPlanAttempt
    content_identity: BranchContentDigest
    call: NativePlanCall
    _factory_token: Optional[object] = field(default=None, eq=False, repr=False)

    @classmethod
    def build(cls, attempt: BranchPlanAttempt, content_identity: BranchContentDigest, call: NativePlanCall) -> "BranchPlanRequest":
        return cls(attempt, content_identity, call, _PLANNING_FACTORY_TOKEN)

    def __attrs_post_init__(self) -> None:
        if (
            type(self.attempt) is not BranchPlanAttempt
            or type(self.content_identity) is not BranchContentDigest
            or not isinstance(self.call, NativePlanCall)
            or self._factory_token is not _PLANNING_FACTORY_TOKEN
        ):
            raise InvalidBranchPlanRequestError("Branch plan request fields are inconsistent.")

    @property
    def identity(self) -> BranchRuntimeIdentity:
        """Return the runtime identity of the attempt this request executes."""
        return self.attempt.runtime_identity

    @property
    def compute_token(self) -> ComputeToken:
        """Return the runtime compute token of the attempt this request executes."""
        return self.attempt.compute_token


def build_branch_plan_requests(
    programs: ProgramSeriesBuild,
    shared: PlanningSharedInputs,
    planning_identity: PlanningTreeIdentity,
    runtime: TreeRuntimeSnapshot,
) -> List[BranchPlanRequest]:
    """Bind every valid program branch to one exact native plan call and attempt."""
    if type(programs) is not ProgramSeriesBuild:
        raise InvalidBranchPlanRequestError("Branch plan requests require an exact ProgramSeriesBuild.")
    if type(shared) is not PlanningSharedInputs:
        raise InvalidBranchPlanRequestError("Branch plan requests require exact PlanningSharedInputs.")
    if type(planning_identity) is not PlanningTreeIdentity:
        raise InvalidBranchPlanRequestError("Branch plan requests require an exact PlanningTreeIdentity.")
    if type(runtime) is not TreeRuntimeSnapshot:
        raise InvalidBranchPlanRequestError("Branch plan requests require an exact TreeRuntimeSnapshot.")
    if runtime.content != planning_identity.tree_identity:
        raise MismatchedPlanningRuntimeError("Runtime snapshot content does not match the planning tree identity.")
    requests: List[BranchPlanRequest] = []
    for branch in programs.output.values.branches:
        item = branch.items[0]
        if item.is_null:
            continue
        path = branch.path
        program = cast(NativeProgramBuild, item.item)
        content_identity = planning_identity.branch(path)
        runtime_identity = runtime.branch(path)
        if runtime_identity.content_digest != content_identity:
            raise MismatchedPlanningRuntimeError("Runtime branch digest does not match the planning branch identity.")
        call = NativePlanCall.build(shared.planner, program.composite_instruction, shared.pipeline, shared.profiles, shared.auto_seed)
        attempt = BranchPlanAttempt.build(runtime_identity, shared.compute_token)
        requests.append(BranchPlanRequest.build(attempt, content_identity, call))
    return requests
