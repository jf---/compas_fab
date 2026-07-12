import pytest
import numpy as np
from attrs import evolve
from compas.geometry import Frame
from tesseract_robotics.planning import MoveType

from compas_fab.ghpython.item_values import Scalar
from compas_fab.ghpython.tesseract_cartesian_target_series import CartesianTargetSeriesParameters
from compas_fab.ghpython.tesseract_cartesian_target_series import InvalidCartesianTargetSeriesBuildError
from compas_fab.ghpython.tesseract_cartesian_target_series import NativeSeriesValueChangedError
from compas_fab.ghpython.tesseract_cartesian_target_series import build_cartesian_target_series
from compas_fab.ghpython.tesseract_pose_series import PoseSeriesParameters
from compas_fab.ghpython.tesseract_pose_series import build_pose_series
from compas_fab.ghpython.tree_coordinates import GhPath
from compas_fab.ghpython.tree_coordinates import TreeRootId
from compas_fab.ghpython.tree_matching import ImplicitSingletonBroadcastError
from compas_fab.ghpython.tree_values import Tree
from compas_fab.ghpython.tree_values import TreeBranch
from compas_fab.ghpython.tree_values import TreeItem


def _frames() -> Tree[Frame]:
    return Tree.build(
        TreeRootId.build("frames"),
        (TreeBranch.build(GhPath.build(0), (TreeItem.value(Frame.worldXY()), TreeItem.value(Frame([1, 0, 0], [1, 0, 0], [0, 1, 0])))),),
    )


def test_cartesian_target_series_retains_exact_pose_objects() -> None:
    poses = build_pose_series(_frames(), PoseSeriesParameters.build(1.0, "base_link"))
    targets = build_cartesian_target_series(
        poses,
        CartesianTargetSeriesParameters.build(Scalar.build(MoveType.LINEAR), Scalar.build("weld")),
    )

    for pose_item, target_item in zip(poses.output.values.branches[0].items, targets.output.values.branches[0].items):
        assert target_item.item.pose is pose_item.item
        assert target_item.item.move_type is MoveType.LINEAR
        assert target_item.item.profile == "weld"
    assert len(targets.output.source_coordinates.entries) == 2


def test_one_item_move_tree_does_not_broadcast() -> None:
    poses = build_pose_series(_frames(), PoseSeriesParameters.build(1.0, "base_link"))
    moves = Tree.build(
        TreeRootId.build("moves"),
        (TreeBranch.build(GhPath.build(0), (TreeItem.value(MoveType.LINEAR),)),),
    )

    with pytest.raises(ImplicitSingletonBroadcastError):
        build_cartesian_target_series(poses, CartesianTargetSeriesParameters.build(moves, Scalar.build("DEFAULT")))


def test_pose_mutation_is_rejected_before_target_construction() -> None:
    poses = build_pose_series(_frames(), PoseSeriesParameters.build(1.0, "base_link"))
    poses.output.values.branches[0].items[0].item.translate(np.asarray([1.0, 0.0, 0.0], dtype=np.float64))

    with pytest.raises(NativeSeriesValueChangedError):
        build_cartesian_target_series(
            poses,
            CartesianTargetSeriesParameters.build(Scalar.build(MoveType.LINEAR), Scalar.build("DEFAULT")),
        )


def test_cartesian_target_series_raw_constructor_cannot_swap_parameters() -> None:
    poses = build_pose_series(_frames(), PoseSeriesParameters.build(1.0, "base_link"))
    built = build_cartesian_target_series(
        poses,
        CartesianTargetSeriesParameters.build(Scalar.build(MoveType.LINEAR), Scalar.build("DEFAULT")),
    )
    with pytest.raises(InvalidCartesianTargetSeriesBuildError):
        evolve(
            built,
            parameters=CartesianTargetSeriesParameters.build(Scalar.build(MoveType.FREESPACE), Scalar.build("DEFAULT")),
        )
