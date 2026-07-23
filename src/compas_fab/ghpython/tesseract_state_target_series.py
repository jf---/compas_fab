"""Exact group-tagged state target series with optional dynamics and time."""

from __future__ import annotations

from math import isfinite
from typing import Optional
from typing import Tuple
from typing import cast

import numpy as np
from attrs import define
from attrs import field
from numpy.typing import ArrayLike
from tesseract_robotics.planning import StateTarget

from compas_fab.backends.tesseract.native_quantities import NativeJointAccelerations
from compas_fab.backends.tesseract.native_quantities import NativeJointNames
from compas_fab.backends.tesseract.native_quantities import NativeJointPositions
from compas_fab.backends.tesseract.native_quantities import NativeJointVelocities
from compas_fab.backends.tesseract.native_quantities import NativeTime
from compas_fab.backends.tesseract.native_targets import state_target_from_native
from compas_fab.ghpython.group_shape import GroupFixedVector
from compas_fab.ghpython.group_shape import GroupShape
from compas_fab.ghpython.group_shape import GroupVectorQuantity
from compas_fab.ghpython.item_values import ItemShape
from compas_fab.ghpython.item_values import Scalar
from compas_fab.ghpython.optional_tree_input import OptionalTreeInput
from compas_fab.ghpython.port_semantics import BranchSemantics
from compas_fab.ghpython.port_semantics import PortSemantics
from compas_fab.ghpython.port_semantics import TopologyRole
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
from compas_fab.ghpython.tree_identity import FLOAT64_CODEC
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

STATE_TARGET_SERIES_SCHEMA = "compas_fab.tesseract.state_target_series/v1"
_STATE_BUILD_TOKEN = object()
_TIME_SEMANTICS = PortSemantics.build(TopologyRole.TREE, BranchSemantics.ELEMENTWISE, ItemShape.scalar())


class StateTargetSeriesError(ValueError):
    """Base failure for state target series."""


class InvalidStateTargetSeriesBuildError(StateTargetSeriesError):
    """Raised when retained state target fields disagree."""


class InvalidStateTimeInputError(StateTargetSeriesError):
    """Raised before native work when a time tree carries a non-scalar value."""


class StateTargetSeriesValueChangedError(StateTargetSeriesError):
    """Raised when mutable native target inputs changed between stages."""


@define(frozen=True, slots=True)
class StateTargetSeriesBuild:
    """Exact state targets with group, optional dynamics/time, and provenance."""

    group_shape: GroupShape
    positions: Tree[GroupFixedVector]
    names_input: OptionalTreeInput[GroupFixedVector]
    velocities_input: OptionalTreeInput[GroupFixedVector]
    accelerations_input: OptionalTreeInput[GroupFixedVector]
    time_input: OptionalTreeInput[float]
    parameters: VectorTargetSeriesParameters
    output: SeriesTopologyOutput[StateTarget]
    position_identity: SourceTreeIdentity
    names_identity: Optional[SourceTreeIdentity]
    velocities_identity: Optional[SourceTreeIdentity]
    accelerations_identity: Optional[SourceTreeIdentity]
    time_identity: Optional[SourceTreeIdentity]
    move_identity: Optional[SourceTreeIdentity]
    profile_identity: Optional[SourceTreeIdentity]
    identity: StageTreeIdentity
    _factory_token: Optional[object] = field(default=None, eq=False, repr=False)

    def __attrs_post_init__(self) -> None:
        if (
            type(self.group_shape) is not GroupShape
            or type(self.positions) is not Tree
            or type(self.names_input) is not OptionalTreeInput
            or type(self.velocities_input) is not OptionalTreeInput
            or type(self.accelerations_input) is not OptionalTreeInput
            or type(self.time_input) is not OptionalTreeInput
            or type(self.parameters) is not VectorTargetSeriesParameters
            or type(self.output) is not SeriesTopologyOutput
            or type(self.position_identity) is not SourceTreeIdentity
            or type(self.identity) is not StageTreeIdentity
            or self._factory_token is not _STATE_BUILD_TOKEN
        ):
            raise InvalidStateTargetSeriesBuildError("State target series fields are inconsistent.")
        try:
            expected_position = vector_identity(self.positions, self.group_shape, GroupVectorQuantity.POSITION)
            expected_names = _vector_optional_identity(self.names_input, self.group_shape, GroupVectorQuantity.NAME)
            expected_velocities = _vector_optional_identity(self.velocities_input, self.group_shape, GroupVectorQuantity.VELOCITY)
            expected_accelerations = _vector_optional_identity(self.accelerations_input, self.group_shape, GroupVectorQuantity.ACCELERATION)
            expected_time = _time_optional_identity(self.time_input)
        except (VectorTargetInputError, InvalidStateTimeInputError) as error:
            raise InvalidStateTargetSeriesBuildError("State target group tags disagree with retained vectors.") from error
        expected_parameters = _stage_parameters(
            self.names_input,
            self.velocities_input,
            self.accelerations_input,
            self.time_input,
            self.group_shape,
            self.parameters,
            self.move_identity,
            self.profile_identity,
        )
        if (
            self.position_identity != expected_position
            or self.names_identity != expected_names
            or self.velocities_identity != expected_velocities
            or self.accelerations_identity != expected_accelerations
            or self.time_identity != expected_time
            or self.move_identity != move_identity(self.parameters.move_types)
            or self.profile_identity != profile_identity(self.parameters.profiles)
            or self.identity.builder_schema != STATE_TARGET_SERIES_SCHEMA
            or self.identity.parameters != expected_parameters
            or self.identity.source_coordinates.entries != self.output.source_coordinates.entries
            or self.output.values.topology != self.identity.topology
        ):
            raise InvalidStateTargetSeriesBuildError("State target series fields are inconsistent.")

    def require_current(self) -> None:
        """Reject changed source trees or mutable native target fields."""
        try:
            current_position = vector_identity(self.positions, self.group_shape, GroupVectorQuantity.POSITION)
            current_names = _vector_optional_identity(self.names_input, self.group_shape, GroupVectorQuantity.NAME)
            current_velocities = _vector_optional_identity(self.velocities_input, self.group_shape, GroupVectorQuantity.VELOCITY)
            current_accelerations = _vector_optional_identity(self.accelerations_input, self.group_shape, GroupVectorQuantity.ACCELERATION)
            current_time = _time_optional_identity(self.time_input)
        except (VectorTargetInputError, InvalidStateTimeInputError) as error:
            raise StateTargetSeriesValueChangedError(str(error)) from error
        if (
            current_position != self.position_identity
            or current_names != self.names_identity
            or current_velocities != self.velocities_identity
            or current_accelerations != self.accelerations_identity
            or current_time != self.time_identity
            or move_identity(self.parameters.move_types) != self.move_identity
            or profile_identity(self.parameters.profiles) != self.profile_identity
        ):
            raise StateTargetSeriesValueChangedError("State target inputs changed after series construction.")
        present_optionals = _present_optionals(self.names_input, self.velocities_input, self.accelerations_input, self.time_input)
        matched = _match(self.positions, present_optionals, self.parameters)
        by_role = {role: offset for offset, (role, _tree) in enumerate(present_optionals, start=1)}
        move_index = 1 + len(present_optionals)
        profile_index = move_index + 1
        for matched_branch, output_branch in zip(matched.branches, self.output.values.branches):
            for row, output_item in zip(matched_branch.rows, output_branch.items):
                if any(item.is_null for item in row.items):
                    continue
                target = cast(StateTarget, output_item.item)
                position = cast(GroupFixedVector, row.items[0].item)
                expected_names = None
                if "names" in by_role:
                    expected_names = list(cast(Tuple[str, ...], cast(GroupFixedVector, row.items[by_role["names"]].item).vector.values))
                expected_time = None if "time" not in by_role else cast(float, row.items[by_role["time"]].item)
                if (
                    not np.array_equal(target.positions, np.asarray(position.vector.values, dtype=np.float64))
                    or target.names != expected_names
                    or not _optional_vector_equal(target.velocities, row.items, by_role, "velocities")
                    or not _optional_vector_equal(target.accelerations, row.items, by_role, "accelerations")
                    or target.time != expected_time
                    or target.move_type is not row.items[move_index].item
                    or target.profile != row.items[profile_index].item
                ):
                    raise StateTargetSeriesValueChangedError("Native StateTarget changed after series construction.")


def _vector_optional_identity(
    optional: OptionalTreeInput[GroupFixedVector],
    group_shape: GroupShape,
    quantity: GroupVectorQuantity,
) -> Optional[SourceTreeIdentity]:
    if optional.is_absent:
        return None
    return vector_identity(optional.require_present(), group_shape, quantity)


def _time_optional_identity(optional: OptionalTreeInput[float]) -> Optional[SourceTreeIdentity]:
    if optional.is_absent:
        return None
    tree = optional.require_present()
    validate_time_tree(tree)
    return SourceTreeIdentity.build(tree, FLOAT64_CODEC, _TIME_SEMANTICS)


def validate_time_tree(tree: Tree[float]) -> None:
    """Fail before native work on any non-null time item that is not finite seconds."""
    if type(tree) is not Tree:
        raise InvalidStateTimeInputError("State time input requires an exact Tree value.")
    for branch in tree.branches:
        for item in branch.items:
            if item.is_null:
                continue
            value = item.item
            if type(value) is not float or not isfinite(value) or value < 0.0:
                raise InvalidStateTimeInputError("Every state time item must be finite non-negative seconds.")


def _optional_vector_equal(
    actual: Optional[ArrayLike],
    row_items: Tuple[TreeItem[object], ...],
    by_role: dict[str, int],
    role: str,
) -> bool:
    """Compare one retained optional native vector against its matched source values."""
    if role not in by_role:
        return actual is None
    expected = np.asarray(cast(GroupFixedVector, row_items[by_role[role]].item).vector.values, dtype=np.float64)
    return actual is not None and np.array_equal(actual, expected)


def _present_optionals(
    names_input: OptionalTreeInput[GroupFixedVector],
    velocities_input: OptionalTreeInput[GroupFixedVector],
    accelerations_input: OptionalTreeInput[GroupFixedVector],
    time_input: OptionalTreeInput[float],
) -> Tuple[Tuple[str, Tree[object]], ...]:
    """Return each present optional's role and tree in fixed native order."""
    present: list[Tuple[str, Tree[object]]] = []
    for role, optional in (
        ("names", cast(OptionalTreeInput[object], names_input)),
        ("velocities", cast(OptionalTreeInput[object], velocities_input)),
        ("accelerations", cast(OptionalTreeInput[object], accelerations_input)),
        ("time", cast(OptionalTreeInput[object], time_input)),
    ):
        if optional.is_present:
            present.append((role, optional.require_present()))
    return tuple(present)


def _match(
    positions: Tree[GroupFixedVector],
    present_optionals: Tuple[Tuple[str, Tree[object]], ...],
    parameters: VectorTargetSeriesParameters,
) -> MatchedTree:
    inputs = [MatchInput.tree("positions", positions)]
    for role, tree in present_optionals:
        inputs.append(MatchInput.tree(role, tree))
    inputs.extend((match_input("move_type", parameters.move_types), match_input("profile", parameters.profiles)))
    exact = tuple(inputs)
    return match_inputs(exact, MatchPolicy.build(tuple(value.name for value in exact), tuple(value.role for value in exact)))


def _stage_parameters(
    names_input: OptionalTreeInput[GroupFixedVector],
    velocities_input: OptionalTreeInput[GroupFixedVector],
    accelerations_input: OptionalTreeInput[GroupFixedVector],
    time_input: OptionalTreeInput[float],
    group_shape: GroupShape,
    parameters: VectorTargetSeriesParameters,
    move_id: Optional[SourceTreeIdentity],
    profile_id: Optional[SourceTreeIdentity],
) -> Tuple[StageParameter, ...]:
    values = [
        StageParameter.text("group_shape", group_shape.canonical_bytes().hex()),
        StageParameter.text("names_presence", "present" if names_input.is_present else "absent"),
        StageParameter.text("velocities_presence", "present" if velocities_input.is_present else "absent"),
        StageParameter.text("accelerations_presence", "present" if accelerations_input.is_present else "absent"),
        StageParameter.text("time_presence", "present" if time_input.is_present else "absent"),
        StageParameter.text("move_input_role", "tree" if move_id is not None else "broadcastable_scalar"),
        StageParameter.text("profile_input_role", "tree" if profile_id is not None else "broadcastable_scalar"),
        StageParameter.text("match_policy", "exact_branch_local_zip/v1"),
    ]
    if type(parameters.move_types) is Scalar:
        values.append(StageParameter.text("move_type_scalar", parameters.move_types.value.name))
    if type(parameters.profiles) is Scalar:
        values.append(StageParameter.text("profile_scalar", parameters.profiles.value))
    return tuple(values)


def build_state_target_series(
    group_shape: GroupShape,
    positions: Tree[GroupFixedVector],
    names_input: OptionalTreeInput[GroupFixedVector],
    velocities_input: OptionalTreeInput[GroupFixedVector],
    accelerations_input: OptionalTreeInput[GroupFixedVector],
    time_input: OptionalTreeInput[float],
    parameters: VectorTargetSeriesParameters,
) -> StateTargetSeriesBuild:
    """Build exact native StateTargets after complete group and time validation."""
    if (
        type(group_shape) is not GroupShape
        or type(positions) is not Tree
        or type(names_input) is not OptionalTreeInput
        or type(velocities_input) is not OptionalTreeInput
        or type(accelerations_input) is not OptionalTreeInput
        or type(time_input) is not OptionalTreeInput
        or type(parameters) is not VectorTargetSeriesParameters
    ):
        raise InvalidStateTargetSeriesBuildError("State target series requires exact typed inputs.")
    validate_vector_tree(positions, group_shape, GroupVectorQuantity.POSITION)
    position_id = vector_identity(positions, group_shape, GroupVectorQuantity.POSITION)
    names_id = _vector_optional_identity(names_input, group_shape, GroupVectorQuantity.NAME)
    velocities_id = _vector_optional_identity(velocities_input, group_shape, GroupVectorQuantity.VELOCITY)
    accelerations_id = _vector_optional_identity(accelerations_input, group_shape, GroupVectorQuantity.ACCELERATION)
    time_id = _time_optional_identity(time_input)
    move_id = move_identity(parameters.move_types)
    profile_id = profile_identity(parameters.profiles)
    present_optionals = _present_optionals(names_input, velocities_input, accelerations_input, time_input)
    matched = _match(positions, present_optionals, parameters)
    value_branches = []
    status_branches = []
    diagnostic_branches = []
    entries = []
    for branch in matched.branches:
        values: list[TreeItem[StateTarget]] = []
        statuses: list[TreeItem[SeriesItemStatus]] = []
        diagnostics: list[TreeItem[SeriesDiagnostic]] = []
        for index, row in enumerate(branch.rows):
            output_coordinate = TreeCoordinate.build(BranchCoordinate.build(positions.root_id, branch.path), ItemIndex.build(index))
            sources = [TreeCoordinate.build(BranchCoordinate.build(positions.root_id, branch.path), ItemIndex.build(index))]
            for _role, tree in present_optionals:
                sources.append(TreeCoordinate.build(BranchCoordinate.build(tree.root_id, branch.path), ItemIndex.build(index)))
            if type(parameters.move_types) is Tree:
                sources.append(TreeCoordinate.build(BranchCoordinate.build(parameters.move_types.root_id, branch.path), ItemIndex.build(index)))
            if type(parameters.profiles) is Tree:
                sources.append(TreeCoordinate.build(BranchCoordinate.build(parameters.profiles.root_id, branch.path), ItemIndex.build(index)))
            entries.append(SourceCoordinateEntry.build(output_coordinate, tuple(sources)))
            if any(item.is_null for item in row.items):
                values.append(TreeItem.null())
                statuses.append(TreeItem.value(SeriesItemStatus.INVALID))
                diagnostics.append(TreeItem.value(SeriesDiagnostic.build("null_state_target_input", "State target input contains an explicit null slot.", output_coordinate)))
                continue
            values.append(TreeItem.value(_native_state_target(row.items, present_optionals, group_shape)))
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
    bindings = _bindings(positions, present_optionals, position_id, names_id, velocities_id, accelerations_id, time_id, parameters, move_id, profile_id)
    identity = StageTreeIdentity.build(
        position_id,
        STATE_TARGET_SERIES_SCHEMA,
        _stage_parameters(names_input, velocities_input, accelerations_input, time_input, group_shape, parameters, move_id, profile_id),
        values_tree.topology,
        source_map,
        authority=_STATE_STAGE_AUTHORITY,
        prior_binding=bindings[0],
        additional_priors=tuple(bindings[1:]),
    )
    return StateTargetSeriesBuild(
        group_shape,
        positions,
        names_input,
        velocities_input,
        accelerations_input,
        time_input,
        parameters,
        output,
        position_id,
        names_id,
        velocities_id,
        accelerations_id,
        time_id,
        move_id,
        profile_id,
        identity,
        _STATE_BUILD_TOKEN,
    )


def _native_state_target(
    row_items: Tuple[TreeItem[object], ...],
    present_optionals: Tuple[Tuple[str, Tree[object]], ...],
    group_shape: GroupShape,
) -> StateTarget:
    """Build one exact native StateTarget from a complete non-null matched row."""
    by_role = {"positions": 0}
    for offset, (role, _tree) in enumerate(present_optionals, start=1):
        by_role[role] = offset
    move_index = 1 + len(present_optionals)
    profile_index = move_index + 1
    position = cast(GroupFixedVector, row_items[0].item)
    native_names = None
    if "names" in by_role:
        native_names = NativeJointNames.build(cast(GroupFixedVector, row_items[by_role["names"]].item).vector.values, group_shape.dof.value)
    native_velocities = None
    if "velocities" in by_role:
        native_velocities = NativeJointVelocities.build(cast(GroupFixedVector, row_items[by_role["velocities"]].item).vector.values)
    native_accelerations = None
    if "accelerations" in by_role:
        native_accelerations = NativeJointAccelerations.build(cast(GroupFixedVector, row_items[by_role["accelerations"]].item).vector.values)
    native_time = None
    if "time" in by_role:
        native_time = NativeTime.build(cast(float, row_items[by_role["time"]].item))
    return state_target_from_native(
        NativeJointPositions.build(position.vector.values),
        native_names,
        native_velocities,
        native_accelerations,
        native_time,
        row_items[move_index].item,  # type: ignore[arg-type]
        row_items[profile_index].item,  # type: ignore[arg-type]
    )


def _bindings(
    positions: Tree[GroupFixedVector],
    present_optionals: Tuple[Tuple[str, Tree[object]], ...],
    position_id: SourceTreeIdentity,
    names_id: Optional[SourceTreeIdentity],
    velocities_id: Optional[SourceTreeIdentity],
    accelerations_id: Optional[SourceTreeIdentity],
    time_id: Optional[SourceTreeIdentity],
    parameters: VectorTargetSeriesParameters,
    move_id: Optional[SourceTreeIdentity],
    profile_id: Optional[SourceTreeIdentity],
) -> list[StagePriorBinding]:
    identities = {"names": names_id, "velocities": velocities_id, "accelerations": accelerations_id, "time": time_id}
    bindings = [StagePriorBinding.build("positions", positions.root_id, position_id)]
    for role, tree in present_optionals:
        identity = identities[role]
        if identity is None:
            continue
        bindings.append(StagePriorBinding.build(role, tree.root_id, identity))
    if move_id is not None and type(parameters.move_types) is Tree:
        bindings.append(StagePriorBinding.build("move_type", parameters.move_types.root_id, move_id))
    if profile_id is not None and type(parameters.profiles) is Tree:
        bindings.append(StagePriorBinding.build("profile", parameters.profiles.root_id, profile_id))
    return bindings


_STATE_STAGE_AUTHORITY = _register_audited_stage_schema(
    STATE_TARGET_SERIES_SCHEMA,
    build_state_target_series,
    SourceCoordinateRequirement.COMPLETE_OUTPUTS,
)
