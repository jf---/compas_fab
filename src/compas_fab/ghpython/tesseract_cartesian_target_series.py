"""Exact branch-local Cartesian target series from native poses."""

from __future__ import annotations

from typing import Optional
from typing import Tuple
from typing import Union
from typing import cast

from attrs import define
from attrs import field
from tesseract_robotics.planning import MoveType

from compas_fab.backends.tesseract.native_targets import WorkingFrameCartesianTarget
from compas_fab.backends.tesseract.native_targets import cartesian_target_from_native
from compas_fab.ghpython.item_values import ItemShape
from compas_fab.ghpython.item_values import Scalar
from compas_fab.ghpython.port_semantics import BranchSemantics
from compas_fab.ghpython.port_semantics import PortSemantics
from compas_fab.ghpython.port_semantics import TopologyRole
from compas_fab.ghpython.tesseract_pose_series import PoseSeriesBuild
from compas_fab.ghpython.tesseract_pose_series import PoseSeriesSourceChangedError
from compas_fab.ghpython.tesseract_series_outcome import SeriesDiagnostic
from compas_fab.ghpython.tesseract_series_outcome import SeriesItemStatus
from compas_fab.ghpython.tesseract_series_outcome import SeriesTopologyOutput
from compas_fab.ghpython.tree_coordinates import BranchCoordinate
from compas_fab.ghpython.tree_coordinates import ItemIndex
from compas_fab.ghpython.tree_coordinates import TreeCoordinate
from compas_fab.ghpython.tree_diagnostics import SourceCoordinateEntry
from compas_fab.ghpython.tree_diagnostics import SourceCoordinateMap
from compas_fab.ghpython.tree_identity import TEXT_CODEC
from compas_fab.ghpython.tree_identity import ExactItemCodec
from compas_fab.ghpython.tree_identity import SourceCoordinateRequirement
from compas_fab.ghpython.tree_identity import SourceTreeIdentity
from compas_fab.ghpython.tree_identity import StageParameter
from compas_fab.ghpython.tree_identity import StagePriorBinding
from compas_fab.ghpython.tree_identity import StageTreeIdentity
from compas_fab.ghpython.tree_identity import _register_audited_stage_schema
from compas_fab.ghpython.tree_matching import MatchedTree
from compas_fab.ghpython.tree_matching import MatchInput
from compas_fab.ghpython.tree_matching import MatchPolicy
from compas_fab.ghpython.tree_matching import match_inputs
from compas_fab.ghpython.tree_values import Tree
from compas_fab.ghpython.tree_values import TreeBranch
from compas_fab.ghpython.tree_values import TreeItem

CARTESIAN_TARGET_SERIES_SCHEMA = "compas_fab.tesseract.cartesian_target_series/v2"
_CARTESIAN_BUILD_TOKEN = object()
_SCALAR_SEMANTICS = PortSemantics.build(TopologyRole.TREE, BranchSemantics.ELEMENTWISE, ItemShape.scalar())
_MOVE_TYPE_CODEC = ExactItemCodec.build("tesseract-move-type/v1", MoveType, lambda value: value.name.encode("ascii"))

MoveSeriesInput = Union[Tree[MoveType], Scalar[MoveType]]
ProfileSeriesInput = Union[Tree[str], Scalar[str]]


class CartesianTargetSeriesError(ValueError):
    """Base failure for Cartesian target series."""


class InvalidCartesianTargetSeriesParametersError(CartesianTargetSeriesError):
    """Raised when move/profile inputs lack exact declared roles."""


class InvalidCartesianTargetSeriesBuildError(CartesianTargetSeriesError):
    """Raised when retained target-series surfaces disagree."""


class NativeSeriesValueChangedError(CartesianTargetSeriesError):
    """Raised when mutable native authoring content changed between stages."""


@define(frozen=True, slots=True)
class CartesianTargetSeriesParameters:
    """Exact tree-or-explicit-scalar move/profile inputs."""

    move_types: MoveSeriesInput
    profiles: ProfileSeriesInput

    @classmethod
    def build(cls, move_types: MoveSeriesInput, profiles: ProfileSeriesInput) -> "CartesianTargetSeriesParameters":
        return cls(move_types, profiles)

    def __attrs_post_init__(self) -> None:
        if type(self.move_types) is Scalar:
            if type(self.move_types.value) is not MoveType:
                raise InvalidCartesianTargetSeriesParametersError("Move scalar must contain exact MoveType.")
        elif type(self.move_types) is not Tree:
            raise InvalidCartesianTargetSeriesParametersError("Move input must be exact tree or explicit Scalar.")
        if type(self.profiles) is Scalar:
            if type(self.profiles.value) is not str or not self.profiles.value.strip():
                raise InvalidCartesianTargetSeriesParametersError("Profile scalar must contain exact non-empty text.")
        elif type(self.profiles) is not Tree:
            raise InvalidCartesianTargetSeriesParametersError("Profile input must be exact tree or explicit Scalar.")


@define(frozen=True, slots=True)
class CartesianTargetSeriesBuild:
    """Exact native targets with complete pose/move/profile provenance."""

    poses: PoseSeriesBuild
    parameters: CartesianTargetSeriesParameters
    output: SeriesTopologyOutput[WorkingFrameCartesianTarget]
    move_identity: Optional[SourceTreeIdentity]
    profile_identity: Optional[SourceTreeIdentity]
    identity: StageTreeIdentity
    _factory_token: Optional[object] = field(default=None, eq=False, repr=False)

    def __attrs_post_init__(self) -> None:
        expected_move = (
            SourceTreeIdentity.build(self.parameters.move_types, _MOVE_TYPE_CODEC, _SCALAR_SEMANTICS)
            if type(self.parameters.move_types) is Tree
            else None
        )
        expected_profile = (
            SourceTreeIdentity.build(self.parameters.profiles, TEXT_CODEC, _SCALAR_SEMANTICS)
            if type(self.parameters.profiles) is Tree
            else None
        )
        expected_bindings: list[Union[SourceTreeIdentity, StageTreeIdentity]] = [self.poses.identity]
        if expected_move is not None:
            expected_bindings.append(expected_move)
        if expected_profile is not None:
            expected_bindings.append(expected_profile)
        if (
            type(self.poses) is not PoseSeriesBuild
            or type(self.parameters) is not CartesianTargetSeriesParameters
            or type(self.output) is not SeriesTopologyOutput
            or (self.move_identity is not None and type(self.move_identity) is not SourceTreeIdentity)
            or (self.profile_identity is not None and type(self.profile_identity) is not SourceTreeIdentity)
            or type(self.identity) is not StageTreeIdentity
            or self.output.values.topology != self.identity.topology
            or self.move_identity != expected_move
            or self.profile_identity != expected_profile
            or self.identity.builder_schema != CARTESIAN_TARGET_SERIES_SCHEMA
            or self.identity.parameters != _stage_parameters(self.parameters, expected_move, expected_profile)
            or tuple(binding.identity for binding in self.identity.prior_bindings) != tuple(expected_bindings)
            or self.identity.source_coordinates.entries != self.output.source_coordinates.entries
            or self._factory_token is not _CARTESIAN_BUILD_TOKEN
        ):
            raise InvalidCartesianTargetSeriesBuildError("Cartesian target series fields are inconsistent.")

    def require_current(self) -> None:
        """Reject upstream or retained target mutation before program reduction."""
        try:
            self.poses.require_current()
        except PoseSeriesSourceChangedError as error:
            raise NativeSeriesValueChangedError(str(error)) from error
        if type(self.parameters.move_types) is Tree:
            current_move = SourceTreeIdentity.build(self.parameters.move_types, _MOVE_TYPE_CODEC, _SCALAR_SEMANTICS)
            if current_move != self.move_identity:
                raise NativeSeriesValueChangedError("Move-type tree changed after target construction.")
        if type(self.parameters.profiles) is Tree:
            current_profile = SourceTreeIdentity.build(self.parameters.profiles, TEXT_CODEC, _SCALAR_SEMANTICS)
            if current_profile != self.profile_identity:
                raise NativeSeriesValueChangedError("Profile tree changed after target construction.")
        matched = _match(cast(Tree[object], self.poses.output.values), self.parameters)
        for matched_branch, output_branch in zip(matched.branches, self.output.values.branches):
            for row, output_item in zip(matched_branch.rows, output_branch.items):
                if any(item.is_null for item in row.items):
                    continue
                target = cast(WorkingFrameCartesianTarget, output_item.item)
                pose = row.items[0].item
                move_type = row.items[1].item
                profile = row.items[2].item
                if target.pose is not pose or target.move_type is not move_type or target.profile != profile:
                    raise NativeSeriesValueChangedError("Native Cartesian target changed after series construction.")


def _input(name: str, value: object) -> MatchInput:
    if type(value) is Tree:
        return MatchInput.tree(name, value)
    if type(value) is Scalar:
        return MatchInput.scalar(name, value)
    raise InvalidCartesianTargetSeriesParametersError("Matched target input must be exact tree or Scalar.")


def _match(poses: Tree[object], parameters: CartesianTargetSeriesParameters) -> MatchedTree:
    inputs = (
        MatchInput.tree("pose", poses),
        _input("move_type", parameters.move_types),
        _input("profile", parameters.profiles),
    )
    return match_inputs(inputs, MatchPolicy.build(tuple(value.name for value in inputs), tuple(value.role for value in inputs)))


def _stage_parameters(
    parameters: CartesianTargetSeriesParameters,
    move_identity: Optional[SourceTreeIdentity],
    profile_identity: Optional[SourceTreeIdentity],
) -> Tuple[StageParameter, ...]:
    values = [
        StageParameter.text("move_input_role", "tree" if move_identity is not None else "broadcastable_scalar"),
        StageParameter.text("profile_input_role", "tree" if profile_identity is not None else "broadcastable_scalar"),
        StageParameter.text("match_policy", "exact_branch_local_zip/v1"),
    ]
    if type(parameters.move_types) is Scalar:
        values.append(StageParameter.text("move_type_scalar", parameters.move_types.value.name))
    if type(parameters.profiles) is Scalar:
        values.append(StageParameter.text("profile_scalar", parameters.profiles.value))
    return tuple(values)


def build_cartesian_target_series(
    poses: PoseSeriesBuild,
    parameters: CartesianTargetSeriesParameters,
) -> CartesianTargetSeriesBuild:
    """Build exact native targets under explicit branch-local matching."""
    if type(poses) is not PoseSeriesBuild or type(parameters) is not CartesianTargetSeriesParameters:
        raise InvalidCartesianTargetSeriesBuildError("Cartesian target series requires exact pose build and parameters.")
    try:
        poses.require_current()
    except PoseSeriesSourceChangedError as error:
        raise NativeSeriesValueChangedError(str(error)) from error
    move_identity = (
        SourceTreeIdentity.build(parameters.move_types, _MOVE_TYPE_CODEC, _SCALAR_SEMANTICS)
        if type(parameters.move_types) is Tree
        else None
    )
    profile_identity = (
        SourceTreeIdentity.build(parameters.profiles, TEXT_CODEC, _SCALAR_SEMANTICS)
        if type(parameters.profiles) is Tree
        else None
    )
    matched = _match(cast(Tree[object], poses.output.values), parameters)
    value_branches = []
    status_branches = []
    diagnostic_branches = []
    source_entries = []
    for branch in matched.branches:
        values: list[TreeItem[WorkingFrameCartesianTarget]] = []
        statuses: list[TreeItem[SeriesItemStatus]] = []
        diagnostics: list[TreeItem[SeriesDiagnostic]] = []
        for index, row in enumerate(branch.rows):
            output_coordinate = TreeCoordinate.build(
                BranchCoordinate.build(poses.output.values.root_id, branch.path),
                ItemIndex.build(index),
            )
            sources = [
                TreeCoordinate.build(BranchCoordinate.build(poses.output.values.root_id, branch.path), ItemIndex.build(index))
            ]
            if type(parameters.move_types) is Tree:
                sources.append(TreeCoordinate.build(BranchCoordinate.build(parameters.move_types.root_id, branch.path), ItemIndex.build(index)))
            if type(parameters.profiles) is Tree:
                sources.append(TreeCoordinate.build(BranchCoordinate.build(parameters.profiles.root_id, branch.path), ItemIndex.build(index)))
            source_entries.append(SourceCoordinateEntry.build(output_coordinate, tuple(sources)))
            if any(item.is_null for item in row.items):
                values.append(TreeItem.null())
                statuses.append(TreeItem.value(SeriesItemStatus.INVALID))
                diagnostics.append(TreeItem.value(SeriesDiagnostic.build("null_target_input", "Cartesian target input contains an explicit null slot.", output_coordinate)))
                continue
            pose = row.items[0].item
            move_type = row.items[1].item
            profile = row.items[2].item
            values.append(TreeItem.value(cartesian_target_from_native(pose, move_type, profile)))  # type: ignore[arg-type]
            statuses.append(TreeItem.value(SeriesItemStatus.VALID))
            diagnostics.append(TreeItem.null())
        value_branches.append(TreeBranch.build(branch.path, tuple(values)))
        status_branches.append(TreeBranch.build(branch.path, tuple(statuses)))
        diagnostic_branches.append(TreeBranch.build(branch.path, tuple(diagnostics)))
    values_tree = Tree.build(poses.output.values.root_id, tuple(value_branches))
    status_tree = Tree.build(poses.output.values.root_id, tuple(status_branches))
    diagnostics_tree = Tree.build(poses.output.values.root_id, tuple(diagnostic_branches))
    source_map = SourceCoordinateMap.build(tuple(source_entries))
    output = SeriesTopologyOutput.build(values_tree, status_tree, diagnostics_tree, source_map)
    parameters_identity = _stage_parameters(parameters, move_identity, profile_identity)
    bindings = [StagePriorBinding.build("pose", poses.output.values.root_id, poses.identity)]
    if move_identity is not None and type(parameters.move_types) is Tree:
        bindings.append(StagePriorBinding.build("move_type", parameters.move_types.root_id, move_identity))
    if profile_identity is not None and type(parameters.profiles) is Tree:
        bindings.append(StagePriorBinding.build("profile", parameters.profiles.root_id, profile_identity))
    identity = StageTreeIdentity.build(
        poses.identity,
        CARTESIAN_TARGET_SERIES_SCHEMA,
        parameters_identity,
        values_tree.topology,
        source_map,
        authority=_CARTESIAN_STAGE_AUTHORITY,
        prior_binding=bindings[0],
        additional_priors=tuple(bindings[1:]),
    )
    return CartesianTargetSeriesBuild(
        poses,
        parameters,
        output,
        move_identity,
        profile_identity,
        identity,
        _CARTESIAN_BUILD_TOKEN,
    )


_CARTESIAN_STAGE_AUTHORITY = _register_audited_stage_schema(
    CARTESIAN_TARGET_SERIES_SCHEMA,
    build_cartesian_target_series,
    SourceCoordinateRequirement.COMPLETE_OUTPUTS,
)
