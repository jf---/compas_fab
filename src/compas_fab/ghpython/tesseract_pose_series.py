"""Topology-preserving COMPAS frame to exact native pose series."""

from __future__ import annotations

from math import isfinite
from typing import Optional
from typing import cast

import numpy as np
from attrs import define
from attrs import field
from compas.geometry import Frame  # type: ignore[import-untyped]

from compas_fab.backends.tesseract.frames import MetersPerUserUnit
from compas_fab.backends.tesseract.native_pose import WorkingFrameName
from compas_fab.backends.tesseract.native_pose import WorkingFramePose
from compas_fab.backends.tesseract.native_pose import WorkingFrameUserUnits
from compas_fab.backends.tesseract.native_pose import pose_from_working_frame
from compas_fab.ghpython.item_values import ItemShape
from compas_fab.ghpython.item_values import ShapeTag
from compas_fab.ghpython.port_semantics import BranchSemantics
from compas_fab.ghpython.port_semantics import PortSemantics
from compas_fab.ghpython.port_semantics import TopologyRole
from compas_fab.ghpython.tesseract_series_outcome import SeriesDiagnostic
from compas_fab.ghpython.tesseract_series_outcome import SeriesItemStatus
from compas_fab.ghpython.tesseract_series_outcome import SeriesTopologyOutput
from compas_fab.ghpython.tree_coordinates import BranchCoordinate
from compas_fab.ghpython.tree_coordinates import GhPath
from compas_fab.ghpython.tree_coordinates import ItemIndex
from compas_fab.ghpython.tree_coordinates import TreeCoordinate
from compas_fab.ghpython.tree_coordinates import TreeRootId
from compas_fab.ghpython.tree_diagnostics import SourceCoordinateEntry
from compas_fab.ghpython.tree_diagnostics import SourceCoordinateMap
from compas_fab.ghpython.tree_identity import FRAME_CODEC
from compas_fab.ghpython.tree_identity import SourceCoordinateRequirement
from compas_fab.ghpython.tree_identity import SourceTreeIdentity
from compas_fab.ghpython.tree_identity import StageParameter
from compas_fab.ghpython.tree_identity import StagePriorBinding
from compas_fab.ghpython.tree_identity import StageTreeIdentity
from compas_fab.ghpython.tree_identity import _register_audited_stage_schema
from compas_fab.ghpython.tree_values import Tree
from compas_fab.ghpython.tree_values import TreeBranch
from compas_fab.ghpython.tree_values import TreeItem

POSE_SERIES_SCHEMA = "compas_fab.tesseract.pose_series/v2"
_POSE_BUILD_TOKEN = object()
_FRAME_SEMANTICS = PortSemantics.build(
    TopologyRole.TREE,
    BranchSemantics.ELEMENTWISE,
    ItemShape.domain_atomic(ShapeTag.build("compas.geometry.Frame")),
)


class PoseSeriesError(ValueError):
    """Base failure for exact pose-series construction."""


class InvalidPoseSeriesParametersError(PoseSeriesError):
    """Raised when scale or working-frame parameters are malformed."""


class InvalidPoseSeriesBuildError(PoseSeriesError):
    """Raised when retained pose-series fields disagree."""


class PoseSeriesSourceChangedError(PoseSeriesError):
    """Raised when mutable source/native pose content changed after build."""


@define(frozen=True, slots=True)
class PoseSeriesParameters:
    """Shared exact scale and working-frame authoring inputs."""

    metres_per_user_unit: MetersPerUserUnit
    working_frame: WorkingFrameName

    @classmethod
    def build(cls, metres_per_user_unit: object, working_frame: object) -> "PoseSeriesParameters":
        if (
            isinstance(metres_per_user_unit, bool)
            or not isinstance(metres_per_user_unit, (int, float))
            or not isfinite(metres_per_user_unit)
            or metres_per_user_unit <= 0.0
            or type(working_frame) is not str
            or not working_frame.strip()
        ):
            raise InvalidPoseSeriesParametersError("Pose series requires finite positive scale and exact working frame.")
        return cls(MetersPerUserUnit(float(metres_per_user_unit)), WorkingFrameName(working_frame))

    def __attrs_post_init__(self) -> None:
        if (
            type(self.metres_per_user_unit) is not float
            or not isfinite(self.metres_per_user_unit)
            or self.metres_per_user_unit <= 0.0
            or type(self.working_frame) is not str
            or not self.working_frame.strip()
        ):
            raise InvalidPoseSeriesParametersError("Stored pose series parameters are invalid.")


@define(frozen=True, slots=True)
class PoseSeriesBuild:
    """Exact pose output, source values, and verified stage provenance."""

    source_frames: Tree[Frame]
    parameters: PoseSeriesParameters
    output: SeriesTopologyOutput[WorkingFramePose]
    source_identity: SourceTreeIdentity
    identity: StageTreeIdentity
    _factory_token: Optional[object] = field(default=None, eq=False, repr=False)

    def __attrs_post_init__(self) -> None:
        expected_parameters = (
            StageParameter.f64("metres_per_user_unit", self.parameters.metres_per_user_unit),
            StageParameter.text("working_frame", self.parameters.working_frame),
        )
        valid = (
            type(self.source_frames) is Tree
            and type(self.parameters) is PoseSeriesParameters
            and type(self.output) is SeriesTopologyOutput
            and type(self.source_identity) is SourceTreeIdentity
            and type(self.identity) is StageTreeIdentity
            and self.output.values.topology == self.identity.topology
            and self.source_identity.digest == self.identity.prior_digest
            and SourceTreeIdentity.build(self.source_frames, FRAME_CODEC, _FRAME_SEMANTICS) == self.source_identity
            and self.identity.builder_schema == POSE_SERIES_SCHEMA
            and self.identity.parameters == expected_parameters
            and self.identity.source_coordinates.entries == self.output.source_coordinates.entries
            and len(self.identity.prior_bindings) == 1
            and self.identity.prior_bindings[0].role == "frames"
            and self.identity.prior_bindings[0].root_id == self.source_frames.root_id
            and self.identity.prior_bindings[0].identity == self.source_identity
            and self._factory_token is _POSE_BUILD_TOKEN
        )
        if not valid:
            raise InvalidPoseSeriesBuildError("Pose series must retain exact source, output, and stage identity.")

    def require_current(self) -> None:
        """Reject mutable source or native pose drift before a consumer stage."""
        if SourceTreeIdentity.build(self.source_frames, FRAME_CODEC, _FRAME_SEMANTICS) != self.source_identity:
            raise PoseSeriesSourceChangedError("Pose source frames changed after series construction.")
        for source_branch, output_branch in zip(self.source_frames.branches, self.output.values.branches):
            for source_item, output_item in zip(source_branch.items, output_branch.items):
                if source_item.is_null:
                    continue
                expected = pose_from_working_frame(
                    WorkingFrameUserUnits.build(
                        cast(Frame, source_item.item),
                        self.parameters.metres_per_user_unit,
                        self.parameters.working_frame,
                    )
                )
                actual = cast(WorkingFramePose, output_item.item)
                if not np.array_equal(np.asarray(actual.matrix), np.asarray(expected.matrix)):
                    raise PoseSeriesSourceChangedError("Native pose changed after series construction.")


def _coordinate(root_id: TreeRootId, path: GhPath, index: int) -> TreeCoordinate:
    return TreeCoordinate.build(BranchCoordinate.build(root_id, path), ItemIndex.build(index))


def build_pose_series(
    frames: Tree[Frame],
    parameters: PoseSeriesParameters,
) -> PoseSeriesBuild:
    """Build one exact pose per frame slot without flattening or null removal."""
    if type(frames) is not Tree or type(parameters) is not PoseSeriesParameters:
        raise InvalidPoseSeriesBuildError("Pose series requires exact frame tree and parameters.")
    value_branches = []
    status_branches = []
    diagnostic_branches = []
    source_entries = []
    for branch in frames.branches:
        values: list[TreeItem[WorkingFramePose]] = []
        statuses: list[TreeItem[SeriesItemStatus]] = []
        diagnostics: list[TreeItem[SeriesDiagnostic]] = []
        for index, item in enumerate(branch.items):
            coordinate = _coordinate(frames.root_id, branch.path, index)
            source_entries.append(SourceCoordinateEntry.build(coordinate, (coordinate,)))
            if item.is_null:
                values.append(TreeItem.null())
                statuses.append(TreeItem.value(SeriesItemStatus.INVALID))
                diagnostics.append(TreeItem.value(SeriesDiagnostic.build("null_frame", "Frame input is an explicit null slot.", coordinate)))
                continue
            value = WorkingFrameUserUnits.build(
                cast(Frame, item.item),
                parameters.metres_per_user_unit,
                parameters.working_frame,
            )
            values.append(TreeItem.value(pose_from_working_frame(value)))
            statuses.append(TreeItem.value(SeriesItemStatus.VALID))
            diagnostics.append(TreeItem.null())
        value_branches.append(TreeBranch.build(branch.path, tuple(values)))
        status_branches.append(TreeBranch.build(branch.path, tuple(statuses)))
        diagnostic_branches.append(TreeBranch.build(branch.path, tuple(diagnostics)))
    values_tree = Tree.build(frames.root_id, tuple(value_branches))
    status_tree = Tree.build(frames.root_id, tuple(status_branches))
    diagnostics_tree = Tree.build(frames.root_id, tuple(diagnostic_branches))
    source_map = SourceCoordinateMap.build(tuple(source_entries))
    output = SeriesTopologyOutput.build(values_tree, status_tree, diagnostics_tree, source_map)
    source_identity = SourceTreeIdentity.build(frames, FRAME_CODEC, _FRAME_SEMANTICS)
    identity = StageTreeIdentity.build(
        source_identity,
        POSE_SERIES_SCHEMA,
        (
            StageParameter.f64("metres_per_user_unit", parameters.metres_per_user_unit),
            StageParameter.text("working_frame", parameters.working_frame),
        ),
        values_tree.topology,
        source_map,
        authority=_POSE_STAGE_AUTHORITY,
        prior_binding=StagePriorBinding.build("frames", frames.root_id, source_identity),
    )
    return PoseSeriesBuild(frames, parameters, output, source_identity, identity, _POSE_BUILD_TOKEN)


_POSE_STAGE_AUTHORITY = _register_audited_stage_schema(
    POSE_SERIES_SCHEMA,
    build_pose_series,
    SourceCoordinateRequirement.COMPLETE_OUTPUTS,
)
