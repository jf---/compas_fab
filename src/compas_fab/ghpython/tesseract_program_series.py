"""Exact per-branch native motion program series with digest provenance.

The cross-build content identity (`ProgramSeriesBuild.identity`) derives only the
deterministic, root-free authoring inputs (target identity, program parameters,
reduced topology, and root-free source map) so identical authoring yields an
identical identity regardless of runtime routing roots. The native
`composite_instruction_to_binary` serialization embeds per-instruction UUIDs and
therefore is NOT reproducible across independent builds; it is retained as
per-branch integrity provenance (`native_program_digests`) and re-derived in
`__attrs_post_init__` so a post-build native program swap or mutation fails loudly
without polluting the reproducible content identity.
"""

from __future__ import annotations

from typing import List
from typing import Optional
from typing import Tuple
from typing import Union
from typing import cast

from attrs import define
from attrs import field
from tesseract_robotics.planning import Robot

from compas_fab.backends.tesseract.native_program_builder import NativeProgramBuild
from compas_fab.backends.tesseract.native_program_builder import build_motion_program
from compas_fab.backends.tesseract.native_program_identity import NativeProgramIdentity
from compas_fab.backends.tesseract.native_program_identity import native_program_digest
from compas_fab.ghpython.tesseract_cartesian_target_series import CartesianTargetSeriesBuild
from compas_fab.ghpython.tesseract_joint_target_series import JointTargetSeriesBuild
from compas_fab.ghpython.tesseract_series_outcome import SeriesDiagnostic
from compas_fab.ghpython.tesseract_series_outcome import SeriesItemStatus
from compas_fab.ghpython.tesseract_state_target_series import StateTargetSeriesBuild
from compas_fab.ghpython.tree_coordinates import BranchCoordinate
from compas_fab.ghpython.tree_coordinates import ItemIndex
from compas_fab.ghpython.tree_coordinates import TreeCoordinate
from compas_fab.ghpython.tree_diagnostics import BranchDiagnosticMap
from compas_fab.ghpython.tree_diagnostics import ReducedTopologyOutput
from compas_fab.ghpython.tree_diagnostics import SourceCoordinateEntry
from compas_fab.ghpython.tree_diagnostics import SourceCoordinateMap
from compas_fab.ghpython.tree_identity import SourceCoordinateRequirement
from compas_fab.ghpython.tree_identity import StageParameter
from compas_fab.ghpython.tree_identity import StagePriorBinding
from compas_fab.ghpython.tree_identity import StageTreeIdentity
from compas_fab.ghpython.tree_identity import _register_audited_stage_schema
from compas_fab.ghpython.tree_values import Tree
from compas_fab.ghpython.tree_values import TreeBranch
from compas_fab.ghpython.tree_values import TreeItem

PROGRAM_SERIES_SCHEMA = "compas_fab.tesseract.motion_program_series/v1"
_PROGRAM_BUILD_TOKEN = object()

NativeTargetSeriesBuild = Union[CartesianTargetSeriesBuild, JointTargetSeriesBuild, StateTargetSeriesBuild]
_NATIVE_TARGET_SERIES_TYPES = (CartesianTargetSeriesBuild, JointTargetSeriesBuild, StateTargetSeriesBuild)


class ProgramSeriesError(ValueError):
    """Base failure for native motion program series."""


class InvalidProgramSeriesParametersError(ProgramSeriesError):
    """Raised when group/TCP/working/profile parameters are malformed."""


class InvalidProgramSeriesBuildError(ProgramSeriesError):
    """Raised when retained program-series surfaces or native digests disagree."""


@define(frozen=True, slots=True)
class ProgramSeriesParameters:
    """Exact native group, optional TCP, working frame, and program profile."""

    group: str
    tcp_frame: Optional[str]
    working_frame: str
    profile: str

    @classmethod
    def build(cls, group: str, tcp_frame: Optional[str], working_frame: str, profile: str) -> "ProgramSeriesParameters":
        return cls(group, tcp_frame, working_frame, profile)

    def __attrs_post_init__(self) -> None:
        if (
            type(self.group) is not str
            or not self.group.strip()
            or (self.tcp_frame is not None and (type(self.tcp_frame) is not str or not self.tcp_frame.strip()))
            or type(self.working_frame) is not str
            or not self.working_frame.strip()
            or type(self.profile) is not str
            or not self.profile.strip()
        ):
            raise InvalidProgramSeriesParametersError("Program series requires exact group, working frame, profile, and optional TCP.")


@define(frozen=True, slots=True)
class ProgramSeriesBuild:
    """One native program per branch with target and native-digest provenance."""

    targets: NativeTargetSeriesBuild
    parameters: ProgramSeriesParameters
    output: ReducedTopologyOutput[NativeProgramBuild, SeriesItemStatus, SeriesDiagnostic]
    native_program_digests: Tuple[Optional[NativeProgramIdentity], ...]
    identity: StageTreeIdentity
    _factory_token: Optional[object] = field(default=None, eq=False, repr=False)

    def __attrs_post_init__(self) -> None:
        if (
            not isinstance(self.targets, _NATIVE_TARGET_SERIES_TYPES)
            or type(self.parameters) is not ProgramSeriesParameters
            or type(self.output) is not ReducedTopologyOutput
            or type(self.native_program_digests) is not tuple
            or type(self.identity) is not StageTreeIdentity
            or self.identity.builder_schema != PROGRAM_SERIES_SCHEMA
            or self.identity.parameters != _stage_parameters(self.parameters)
            or self.identity.topology != self.output.values.topology
            or self.identity.source_coordinates.entries != self.output.source_coordinates.entries
            or self.native_program_digests != _native_program_digests(self.output.values)
            or self._factory_token is not _PROGRAM_BUILD_TOKEN
        ):
            raise InvalidProgramSeriesBuildError("Program series fields disagree with retained targets, native digests, or identity.")


def _stage_parameters(parameters: ProgramSeriesParameters) -> Tuple[StageParameter, ...]:
    values = [
        StageParameter.text("group", parameters.group),
        StageParameter.text("tcp_presence", "present" if parameters.tcp_frame is not None else "absent"),
        StageParameter.text("working_frame", parameters.working_frame),
        StageParameter.text("profile", parameters.profile),
    ]
    if parameters.tcp_frame is not None:
        values.append(StageParameter.text("tcp_frame", parameters.tcp_frame))
    return tuple(values)


def _native_program_digests(values: Tree[NativeProgramBuild]) -> Tuple[Optional[NativeProgramIdentity], ...]:
    """Re-derive one native program identity per branch from retained programs."""
    digests: List[Optional[NativeProgramIdentity]] = []
    for branch in values.branches:
        item = branch.items[0]
        if item.is_null:
            digests.append(None)
        else:
            digests.append(native_program_digest(cast(NativeProgramBuild, item.item).composite_instruction))
    return tuple(digests)


def build_motion_program_series(
    native_robot: Robot,
    targets: NativeTargetSeriesBuild,
    parameters: ProgramSeriesParameters,
) -> ProgramSeriesBuild:
    """Reduce each target branch to one exact native program with digest provenance."""
    if not isinstance(native_robot, Robot):
        raise InvalidProgramSeriesBuildError("Program series requires an exact native Robot.")
    if not isinstance(targets, _NATIVE_TARGET_SERIES_TYPES):
        raise InvalidProgramSeriesBuildError("Program series requires an exact native target series build.")
    if type(parameters) is not ProgramSeriesParameters:
        raise InvalidProgramSeriesBuildError("Program series requires exact ProgramSeriesParameters.")
    targets.require_current()

    target_tree: Tree[object] = cast(Tree[object], targets.output.values)
    root = target_tree.root_id
    value_branches: List[TreeBranch[NativeProgramBuild]] = []
    status_branches: List[TreeBranch[SeriesItemStatus]] = []
    item_diagnostic_branches: List[TreeBranch[SeriesDiagnostic]] = []
    branch_diagnostics: List[Tuple[BranchCoordinate, SeriesDiagnostic]] = []
    source_entries: List[SourceCoordinateEntry] = []
    digests: List[Optional[NativeProgramIdentity]] = []

    for branch in target_tree.branches:
        path = branch.path
        output_coordinate = TreeCoordinate.build(BranchCoordinate.build(root, path), ItemIndex.build(0))
        native_targets: List[object] = []
        item_diagnostics: List[TreeItem[SeriesDiagnostic]] = []
        for index, item in enumerate(branch.items):
            source_coordinate = TreeCoordinate.build(BranchCoordinate.build(root, path), ItemIndex.build(index))
            if item.is_null:
                diagnostic = SeriesDiagnostic.build("null_program_target", "Program branch input contains an explicit null target slot.", source_coordinate)
                item_diagnostics.append(TreeItem.value(diagnostic))
            else:
                native_targets.append(item.item)
                item_diagnostics.append(TreeItem.null())
        item_diagnostic_branches.append(TreeBranch.build(path, tuple(item_diagnostics)))

        if branch.items:
            source_entries.append(
                SourceCoordinateEntry.build(
                    output_coordinate,
                    tuple(TreeCoordinate.build(BranchCoordinate.build(root, path), ItemIndex.build(index)) for index in range(len(branch.items))),
                )
            )
        else:
            source_entries.append(SourceCoordinateEntry.from_empty_branch(output_coordinate, BranchCoordinate.build(root, path)))

        if not branch.items or len(native_targets) != len(branch.items):
            value_branches.append(TreeBranch.build(path, (TreeItem.null(),)))
            status_branches.append(TreeBranch.build(path, (TreeItem.null(),)))
            code, message = (
                ("empty_program_branch", "Program branch has no ordered targets to compose.")
                if not branch.items
                else ("null_program_targets", "Program branch contains at least one null target and cannot compose.")
            )
            branch_diagnostics.append((BranchCoordinate.build(root, path), SeriesDiagnostic.build(code, message, output_coordinate)))
            digests.append(None)
            continue

        program = build_motion_program(
            native_robot,
            native_targets,
            parameters.group,
            parameters.tcp_frame,
            parameters.working_frame,
            parameters.profile,
        )
        value_branches.append(TreeBranch.build(path, (TreeItem.value(program),)))
        status_branches.append(TreeBranch.build(path, (TreeItem.value(SeriesItemStatus.VALID),)))
        digests.append(native_program_digest(program.composite_instruction))

    values_tree = Tree.build(root, tuple(value_branches))
    output = ReducedTopologyOutput.build(
        values_tree,
        Tree.build(root, tuple(status_branches)),
        Tree.build(root, tuple(item_diagnostic_branches)),
        BranchDiagnosticMap.build(tuple(branch_diagnostics)),
        SourceCoordinateMap.build(tuple(source_entries)),
    )
    identity = StageTreeIdentity.build(
        targets.identity,
        PROGRAM_SERIES_SCHEMA,
        _stage_parameters(parameters),
        values_tree.topology,
        output.source_coordinates,
        authority=_PROGRAM_STAGE_AUTHORITY,
        prior_binding=StagePriorBinding.build("targets", root, targets.identity),
    )
    return ProgramSeriesBuild(targets, parameters, output, tuple(digests), identity, _PROGRAM_BUILD_TOKEN)


_PROGRAM_STAGE_AUTHORITY = _register_audited_stage_schema(
    PROGRAM_SERIES_SCHEMA,
    build_motion_program_series,
    SourceCoordinateRequirement.COMPLETE_OUTPUTS,
)
