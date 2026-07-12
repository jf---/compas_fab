from compas.geometry import Frame
from attrs import evolve
import pytest

from compas_fab.ghpython.tesseract_pose_series import PoseSeriesParameters
from compas_fab.ghpython.tesseract_pose_series import InvalidPoseSeriesBuildError
from compas_fab.ghpython.tesseract_pose_series import build_pose_series
from compas_fab.ghpython.tesseract_series_outcome import SeriesItemStatus
from compas_fab.ghpython.tesseract_series_outcome import InvalidSeriesTopologyOutputError
from compas_fab.ghpython.tree_coordinates import GhPath
from compas_fab.ghpython.tree_coordinates import TreeRootId
from compas_fab.ghpython.tree_identity import IdentityVerification
from compas_fab.ghpython.tree_values import Tree
from compas_fab.ghpython.tree_values import TreeBranch
from compas_fab.ghpython.tree_values import TreeItem
from compas_fab.backends.tesseract.frames import MetersPerUserUnit


def _frames(root: str = "frames") -> Tree[Frame]:
    return Tree.build(
        TreeRootId.build(root),
        (
            TreeBranch.build(GhPath.build(0), (TreeItem.value(Frame.worldXY()), TreeItem.null())),
            TreeBranch.build(GhPath.build(4, 1), (TreeItem.value(Frame([1000, 0, 0], [1, 0, 0], [0, 1, 0])),)),
        ),
    )


def test_pose_series_preserves_paths_nulls_units_and_provenance() -> None:
    built = build_pose_series(_frames(), PoseSeriesParameters.build(0.001, "base_link"))

    assert built.output.values.topology == _frames().topology
    assert built.output.status.branches[0].items[0].item is SeriesItemStatus.VALID
    assert built.output.status.branches[0].items[1].item is SeriesItemStatus.INVALID
    assert built.output.values.branches[0].items[1].is_null
    assert built.output.values.branches[1].items[0].item.translation.tolist() == [1.0, 0.0, 0.0]
    assert built.identity.verification is IdentityVerification.VERIFIED
    assert len(built.output.source_coordinates.entries) == 3


def test_pose_series_identity_ignores_runtime_root() -> None:
    parameters = PoseSeriesParameters.build(0.001, "base_link")
    assert build_pose_series(_frames("document-a"), parameters).identity == build_pose_series(_frames("document-b"), parameters).identity


def test_pose_series_retains_typed_unit_scale() -> None:
    parameters = PoseSeriesParameters.build(0.001, "base_link")
    assert parameters.metres_per_user_unit == MetersPerUserUnit(0.001)


def test_pose_series_raw_constructor_cannot_swap_parameters() -> None:
    built = build_pose_series(_frames(), PoseSeriesParameters.build(0.001, "base_link"))
    with pytest.raises(InvalidPoseSeriesBuildError):
        evolve(built, parameters=PoseSeriesParameters.build(1.0, "base_link"))


def test_series_output_raw_constructor_cannot_reroute_status_root() -> None:
    built = build_pose_series(_frames(), PoseSeriesParameters.build(0.001, "base_link"))
    rerouted_status = Tree.build(TreeRootId.build("other"), built.output.status.branches)
    with pytest.raises(InvalidSeriesTopologyOutputError):
        evolve(built.output, status=rerouted_status)
