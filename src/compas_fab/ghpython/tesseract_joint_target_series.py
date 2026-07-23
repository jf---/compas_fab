"""Exact group-tagged joint target series."""

from __future__ import annotations

from typing import Optional
from typing import Tuple
from typing import cast

import numpy as np
from attrs import define
from attrs import field
from tesseract_robotics.planning import JointTarget

from compas_fab.backends.tesseract.native_quantities import NativeJointNames
from compas_fab.backends.tesseract.native_quantities import NativeJointPositions
from compas_fab.backends.tesseract.native_targets import joint_target_from_native
from compas_fab.ghpython.group_shape import GroupFixedVector
from compas_fab.ghpython.group_shape import GroupShape
from compas_fab.ghpython.group_shape import GroupVectorQuantity
from compas_fab.ghpython.item_values import Scalar
from compas_fab.ghpython.optional_tree_input import OptionalTreeInput
from compas_fab.ghpython.tesseract_series_outcome import SeriesDiagnostic
from compas_fab.ghpython.tesseract_series_outcome import SeriesItemStatus
from compas_fab.ghpython.tesseract_series_outcome import SeriesTopologyOutput
from compas_fab.ghpython.tesseract_vector_target_common import VectorTargetInputError
from compas_fab.ghpython.tesseract_vector_target_common import VectorTargetSeriesParameters
from compas_fab.ghpython.tesseract_vector_target_common import match_input
from compas_fab.ghpython.tesseract_vector_target_common import move_identity
from compas_fab.ghpython.tesseract_vector_target_common import profile_identity
from compas_fab.ghpython.tesseract_vector_target_common import validate_vector_tree
from compas_fab.ghpython.tesseract_vector_target_common import vector_identity
from compas_fab.ghpython.tree_coordinates import BranchCoordinate
from compas_fab.ghpython.tree_coordinates import ItemIndex
from compas_fab.ghpython.tree_coordinates import TreeCoordinate
from compas_fab.ghpython.tree_diagnostics import SourceCoordinateEntry
from compas_fab.ghpython.tree_diagnostics import SourceCoordinateMap
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

JOINT_TARGET_SERIES_SCHEMA = "compas_fab.tesseract.joint_target_series/v1"
_JOINT_BUILD_TOKEN = object()


class JointTargetSeriesError(ValueError):
    """Base failure for joint target series."""


class InvalidJointTargetSeriesBuildError(JointTargetSeriesError):
    """Raised when retained joint target fields disagree."""


class JointTargetSeriesValueChangedError(JointTargetSeriesError):
    """Raised when mutable native target inputs changed between stages."""


@define(frozen=True, slots=True)
class JointTargetSeriesBuild:
    """Exact joint targets with group, optional-name, and stage provenance."""

    group_shape: GroupShape
    positions: Tree[GroupFixedVector]
    names_input: OptionalTreeInput[GroupFixedVector]
    parameters: VectorTargetSeriesParameters
    output: SeriesTopologyOutput[JointTarget]
    position_identity: SourceTreeIdentity
    names_identity: Optional[SourceTreeIdentity]
    move_identity: Optional[SourceTreeIdentity]
    profile_identity: Optional[SourceTreeIdentity]
    identity: StageTreeIdentity
    _factory_token: Optional[object] = field(default=None, eq=False, repr=False)

    def __attrs_post_init__(self) -> None:
        if (
            type(self.group_shape) is not GroupShape
            or type(self.positions) is not Tree
            or type(self.names_input) is not OptionalTreeInput
            or type(self.parameters) is not VectorTargetSeriesParameters
            or type(self.output) is not SeriesTopologyOutput
            or type(self.position_identity) is not SourceTreeIdentity
            or type(self.identity) is not StageTreeIdentity
            or self._factory_token is not _JOINT_BUILD_TOKEN
        ):
            raise InvalidJointTargetSeriesBuildError("Joint target series fields are inconsistent.")
        try:
            expected_position = vector_identity(self.positions, self.group_shape, GroupVectorQuantity.POSITION)
            expected_names = _names_identity(self.names_input, self.group_shape)
        except VectorTargetInputError as error:
            raise InvalidJointTargetSeriesBuildError("Joint target series group tags disagree with retained vectors.") from error
        if (
            self.position_identity != expected_position
            or self.names_identity != expected_names
            or self.move_identity != move_identity(self.parameters.move_types)
            or self.profile_identity != profile_identity(self.parameters.profiles)
            or self.identity.builder_schema != JOINT_TARGET_SERIES_SCHEMA
            or self.identity.parameters != _stage_parameters(self.group_shape, self.names_input, self.parameters, self.move_identity, self.profile_identity)
            or self.identity.source_coordinates.entries != self.output.source_coordinates.entries
            or self.output.values.topology != self.identity.topology
        ):
            raise InvalidJointTargetSeriesBuildError("Joint target series fields are inconsistent.")

    def require_current(self) -> None:
        """Reject changed source trees or mutable target fields."""
        try:
            current_positions = vector_identity(self.positions, self.group_shape, GroupVectorQuantity.POSITION)
            current_names = _names_identity(self.names_input, self.group_shape)
        except VectorTargetInputError as error:
            raise JointTargetSeriesValueChangedError(str(error)) from error
        if current_positions != self.position_identity or current_names != self.names_identity:
            raise JointTargetSeriesValueChangedError("Joint target vector inputs changed after construction.")
        if move_identity(self.parameters.move_types) != self.move_identity or profile_identity(self.parameters.profiles) != self.profile_identity:
            raise JointTargetSeriesValueChangedError("Joint target move/profile inputs changed after construction.")
        matched = _match(self.positions, self.names_input, self.parameters)
        for matched_branch, output_branch in zip(matched.branches, self.output.values.branches):
            for row, output_item in zip(matched_branch.rows, output_branch.items):
                if any(item.is_null for item in row.items):
                    continue
                target = cast(JointTarget, output_item.item)
                position = cast(GroupFixedVector, row.items[0].item)
                offset = 1
                names = None
                if self.names_input.is_present:
                    names = cast(GroupFixedVector, row.items[1].item)
                    offset = 2
                move_type = row.items[offset].item
                profile = row.items[offset + 1].item
                expected_names = None if names is None else list(cast(Tuple[str, ...], names.vector.values))
                if (
                    not np.array_equal(target.positions, np.asarray(position.vector.values, dtype=np.float64))
                    or target.names != expected_names
                    or target.move_type is not move_type
                    or target.profile != profile
                ):
                    raise JointTargetSeriesValueChangedError("Native JointTarget changed after series construction.")


def _names_identity(
    names_input: OptionalTreeInput[GroupFixedVector],
    group_shape: GroupShape,
) -> Optional[SourceTreeIdentity]:
    if names_input.is_absent:
        return None
    tree = names_input.require_present()
    return vector_identity(tree, group_shape, GroupVectorQuantity.NAME)


def _match(
    positions: Tree[GroupFixedVector],
    names_input: OptionalTreeInput[GroupFixedVector],
    parameters: VectorTargetSeriesParameters,
) -> MatchedTree:
    inputs = [MatchInput.tree("positions", positions)]
    if names_input.is_present:
        inputs.append(MatchInput.tree("names", names_input.require_present()))
    inputs.extend((match_input("move_type", parameters.move_types), match_input("profile", parameters.profiles)))
    exact = tuple(inputs)
    return match_inputs(exact, MatchPolicy.build(tuple(value.name for value in exact), tuple(value.role for value in exact)))


def _stage_parameters(
    group_shape: GroupShape,
    names_input: OptionalTreeInput[GroupFixedVector],
    parameters: VectorTargetSeriesParameters,
    move_id: Optional[SourceTreeIdentity],
    profile_id: Optional[SourceTreeIdentity],
) -> Tuple[StageParameter, ...]:
    values = [
        StageParameter.text("group_shape", group_shape.canonical_bytes().hex()),
        StageParameter.text("names_presence", "present" if names_input.is_present else "absent"),
        StageParameter.text("move_input_role", "tree" if move_id is not None else "broadcastable_scalar"),
        StageParameter.text("profile_input_role", "tree" if profile_id is not None else "broadcastable_scalar"),
        StageParameter.text("match_policy", "exact_branch_local_zip/v1"),
    ]
    if type(parameters.move_types) is Scalar:
        values.append(StageParameter.text("move_type_scalar", parameters.move_types.value.name))
    if type(parameters.profiles) is Scalar:
        values.append(StageParameter.text("profile_scalar", parameters.profiles.value))
    return tuple(values)


def build_joint_target_series(
    group_shape: GroupShape,
    positions: Tree[GroupFixedVector],
    names_input: OptionalTreeInput[GroupFixedVector],
    parameters: VectorTargetSeriesParameters,
) -> JointTargetSeriesBuild:
    """Build exact native JointTargets after complete group validation."""
    if type(group_shape) is not GroupShape or type(positions) is not Tree or type(names_input) is not OptionalTreeInput or type(parameters) is not VectorTargetSeriesParameters:
        raise InvalidJointTargetSeriesBuildError("Joint target series requires exact typed inputs.")
    validate_vector_tree(positions, group_shape, GroupVectorQuantity.POSITION)
    position_id = vector_identity(positions, group_shape, GroupVectorQuantity.POSITION)
    names_id = _names_identity(names_input, group_shape)
    move_id = move_identity(parameters.move_types)
    profile_id = profile_identity(parameters.profiles)
    matched = _match(positions, names_input, parameters)
    value_branches = []
    status_branches = []
    diagnostic_branches = []
    entries = []
    for branch in matched.branches:
        values: list[TreeItem[JointTarget]] = []
        statuses: list[TreeItem[SeriesItemStatus]] = []
        diagnostics: list[TreeItem[SeriesDiagnostic]] = []
        for index, row in enumerate(branch.rows):
            output_coordinate = TreeCoordinate.build(BranchCoordinate.build(positions.root_id, branch.path), ItemIndex.build(index))
            sources = [TreeCoordinate.build(BranchCoordinate.build(positions.root_id, branch.path), ItemIndex.build(index))]
            if names_input.is_present:
                names_tree = names_input.require_present()
                sources.append(TreeCoordinate.build(BranchCoordinate.build(names_tree.root_id, branch.path), ItemIndex.build(index)))
            if type(parameters.move_types) is Tree:
                sources.append(TreeCoordinate.build(BranchCoordinate.build(parameters.move_types.root_id, branch.path), ItemIndex.build(index)))
            if type(parameters.profiles) is Tree:
                sources.append(TreeCoordinate.build(BranchCoordinate.build(parameters.profiles.root_id, branch.path), ItemIndex.build(index)))
            entries.append(SourceCoordinateEntry.build(output_coordinate, tuple(sources)))
            if any(item.is_null for item in row.items):
                values.append(TreeItem.null())
                statuses.append(TreeItem.value(SeriesItemStatus.INVALID))
                diagnostics.append(TreeItem.value(SeriesDiagnostic.build("null_joint_target_input", "Joint target input contains an explicit null slot.", output_coordinate)))
                continue
            position = cast(GroupFixedVector, row.items[0].item)
            offset = 1
            native_names = None
            if names_input.is_present:
                names = cast(GroupFixedVector, row.items[1].item)
                native_names = NativeJointNames.build(names.vector.values, group_shape.dof.value)
                offset = 2
            target = joint_target_from_native(
                NativeJointPositions.build(position.vector.values),
                native_names,
                row.items[offset].item,  # type: ignore[arg-type]
                row.items[offset + 1].item,  # type: ignore[arg-type]
            )
            values.append(TreeItem.value(target))
            statuses.append(TreeItem.value(SeriesItemStatus.VALID))
            diagnostics.append(TreeItem.null())
        value_branches.append(TreeBranch.build(branch.path, tuple(values)))
        status_branches.append(TreeBranch.build(branch.path, tuple(statuses)))
        diagnostic_branches.append(TreeBranch.build(branch.path, tuple(diagnostics)))
    values_tree = Tree.build(positions.root_id, tuple(value_branches))
    source_map = SourceCoordinateMap.build(tuple(entries))
    output = SeriesTopologyOutput.build(
        values_tree,
        Tree.build(positions.root_id, tuple(status_branches)),
        Tree.build(positions.root_id, tuple(diagnostic_branches)),
        source_map,
    )
    bindings = [StagePriorBinding.build("positions", positions.root_id, position_id)]
    if names_id is not None:
        bindings.append(StagePriorBinding.build("names", names_input.require_present().root_id, names_id))
    if move_id is not None and type(parameters.move_types) is Tree:
        bindings.append(StagePriorBinding.build("move_type", parameters.move_types.root_id, move_id))
    if profile_id is not None and type(parameters.profiles) is Tree:
        bindings.append(StagePriorBinding.build("profile", parameters.profiles.root_id, profile_id))
    identity = StageTreeIdentity.build(
        position_id,
        JOINT_TARGET_SERIES_SCHEMA,
        _stage_parameters(group_shape, names_input, parameters, move_id, profile_id),
        values_tree.topology,
        source_map,
        authority=_JOINT_STAGE_AUTHORITY,
        prior_binding=bindings[0],
        additional_priors=tuple(bindings[1:]),
    )
    return JointTargetSeriesBuild(
        group_shape,
        positions,
        names_input,
        parameters,
        output,
        position_id,
        names_id,
        move_id,
        profile_id,
        identity,
        _JOINT_BUILD_TOKEN,
    )


_JOINT_STAGE_AUTHORITY = _register_audited_stage_schema(
    JOINT_TARGET_SERIES_SCHEMA,
    build_joint_target_series,
    SourceCoordinateRequirement.COMPLETE_OUTPUTS,
)
