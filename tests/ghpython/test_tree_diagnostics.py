from __future__ import annotations

from typing import Tuple

import pytest

from compas_fab.ghpython.tree_coordinates import BranchCoordinate
from compas_fab.ghpython.tree_coordinates import GhPath
from compas_fab.ghpython.tree_coordinates import ItemIndex
from compas_fab.ghpython.tree_coordinates import TreeCoordinate
from compas_fab.ghpython.tree_coordinates import TreeRootId
from compas_fab.ghpython.tree_diagnostics import BranchDiagnosticMap
from compas_fab.ghpython.tree_diagnostics import DuplicateBranchDiagnosticError
from compas_fab.ghpython.tree_diagnostics import DuplicateSourceOutputError
from compas_fab.ghpython.tree_diagnostics import EmptySourceCoordinateError
from compas_fab.ghpython.tree_diagnostics import InvalidReducedTopologyOutputError
from compas_fab.ghpython.tree_diagnostics import InvalidSourceCoordinateEntryError
from compas_fab.ghpython.tree_diagnostics import InvalidTopologyOutputError
from compas_fab.ghpython.tree_diagnostics import ReducedTopologyOutput
from compas_fab.ghpython.tree_diagnostics import SourceCoordinateEntry
from compas_fab.ghpython.tree_diagnostics import SourceCoordinateMap
from compas_fab.ghpython.tree_diagnostics import TopologyOutput
from compas_fab.ghpython.tree_values import Tree
from compas_fab.ghpython.tree_values import TreeBranch
from compas_fab.ghpython.tree_values import TreeItem


def tree(root: str, branches: Tuple[Tuple[Tuple[int, ...], Tuple[object | None, ...]], ...]) -> Tree[object]:
    return Tree.build(
        TreeRootId.build(root),
        tuple(
            TreeBranch.build(
                GhPath.build(*path),
                tuple(TreeItem.null() if item is None else TreeItem.value(item) for item in items),
            )
            for path, items in branches
        ),
    )


def coordinate(root: str, path: Tuple[int, ...], index: int) -> TreeCoordinate:
    return TreeCoordinate.build(
        BranchCoordinate.build(TreeRootId.build(root), GhPath.build(*path)),
        ItemIndex.build(index),
    )


def test_reduction_maps_one_output_to_ordered_nonempty_sources() -> None:
    output = coordinate("out", (4,), 0)
    sources = tuple(coordinate("in", (4,), index) for index in range(3))

    mapping = SourceCoordinateMap.build((SourceCoordinateEntry.build(output, sources),))

    assert mapping.sources_for(output) == sources
    with pytest.raises(DuplicateSourceOutputError):
        SourceCoordinateMap.build((SourceCoordinateEntry.build(output, sources), SourceCoordinateEntry.build(output, sources)))
    with pytest.raises(EmptySourceCoordinateError):
        SourceCoordinateEntry.build(output, ())


def test_source_coordinate_entry_rejects_raw_container_bypass() -> None:
    output = coordinate("out", (0,), 0)
    source = coordinate("in", (0,), 0)

    with pytest.raises(InvalidSourceCoordinateEntryError):
        SourceCoordinateEntry(output, [source])  # type: ignore[arg-type]


def test_source_coordinate_identity_bytes_exclude_runtime_roots() -> None:
    left = SourceCoordinateMap.build((SourceCoordinateEntry.build(coordinate("out-a", (2,), 0), (coordinate("in-a", (2,), 1),)),))
    right = SourceCoordinateMap.build((SourceCoordinateEntry.build(coordinate("out-b", (2,), 0), (coordinate("in-b", (2,), 1),)),))

    assert left.root_free_identity_bytes() == right.root_free_identity_bytes()


def test_topology_output_requires_identical_value_and_status_topology() -> None:
    values = tree(
        "values",
        (
            ((0,), ("a", None)),
            ((2,), ()),
        ),
    )
    status = tree(
        "status",
        (
            ((0,), ("ok", None)),
            ((2,), ()),
        ),
    )
    output = TopologyOutput.build(values, status)

    assert output.values is values
    assert output.status is status
    with pytest.raises(InvalidTopologyOutputError):
        TopologyOutput.build(
            values,
            tree(
                "status",
                (
                    ((0,), ("ok",)),
                    ((2,), ()),
                ),
            ),
        )
    with pytest.raises(InvalidTopologyOutputError):
        TopologyOutput(values, tree("status", (((0,), ("ok",)),)))


def test_branch_diagnostics_retain_empty_branch_failures() -> None:
    empty_branch = BranchCoordinate.build(TreeRootId.build("source"), GhPath.build(7))
    diagnostics = BranchDiagnosticMap.build(((empty_branch, "empty ordered series"),))

    assert diagnostics.diagnostic_for(empty_branch) == "empty ordered series"
    with pytest.raises(DuplicateBranchDiagnosticError):
        BranchDiagnosticMap.build(((empty_branch, "first"), (empty_branch, "second")))


def test_reduced_output_has_one_aggregate_and_source_shaped_diagnostics() -> None:
    values = tree("out-values", (((0,), ("program",)), ((7,), (None,))))
    status = tree("out-status", (((0,), ("ok",)), ((7,), (None,))))
    item_diagnostics = tree("source", (((0,), (None, "bad", None)), ((7,), ())))
    empty_branch = BranchCoordinate.build(TreeRootId.build("source"), GhPath.build(7))
    branch_diagnostics = BranchDiagnosticMap.build(((empty_branch, "empty ordered series"),))
    output_coordinate = coordinate("out-values", (0,), 0)
    sources = tuple(coordinate("source", (0,), index) for index in range(3))
    source_coordinates = SourceCoordinateMap.build((SourceCoordinateEntry.build(output_coordinate, sources),))

    output = ReducedTopologyOutput.build(
        values,
        status,
        item_diagnostics,
        branch_diagnostics,
        source_coordinates,
    )

    assert output.values is values
    assert output.status is status
    assert output.item_diagnostics is item_diagnostics
    assert output.source_coordinates.sources_for(output_coordinate) == sources
    assert output.branch_diagnostics.diagnostic_for(empty_branch) == "empty ordered series"


@pytest.mark.parametrize(
    ("values", "status", "item_diagnostics", "source_coordinates"),
    (
        (
            tree("out", (((0,), ("a", "b")),)),
            tree("status", (((0,), ("ok", "ok")),)),
            tree("source", (((0,), (None, None)),)),
            SourceCoordinateMap.build((SourceCoordinateEntry.build(coordinate("out", (0,), 0), (coordinate("source", (0,), 0),)),)),
        ),
        (
            tree("out", (((0,), ("a",)),)),
            tree("status", (((0,), ("ok",)),)),
            tree("source", (((1,), (None,)),)),
            SourceCoordinateMap.build((SourceCoordinateEntry.build(coordinate("out", (0,), 0), (coordinate("source", (1,), 0),)),)),
        ),
        (
            tree("out", (((0,), ("a",)),)),
            tree("status", (((0,), ("ok",)),)),
            tree("source", (((0,), (None, None)),)),
            SourceCoordinateMap.build((SourceCoordinateEntry.build(coordinate("out", (0,), 0), (coordinate("source", (0,), 0),)),)),
        ),
    ),
)
def test_reduced_output_rejects_cardinality_shape_and_incomplete_mapping(
    values: Tree[object],
    status: Tree[object],
    item_diagnostics: Tree[object],
    source_coordinates: SourceCoordinateMap,
) -> None:
    with pytest.raises(InvalidReducedTopologyOutputError):
        ReducedTopologyOutput.build(
            values,
            status,
            item_diagnostics,
            BranchDiagnosticMap.build(()),
            source_coordinates,
        )


def test_reduced_output_rejects_raw_bypass() -> None:
    values = tree("out", (((0,), ("a", "b")),))
    status = tree("status", (((0,), ("ok", "ok")),))
    item_diagnostics = tree("source", (((0,), (None, None)),))

    with pytest.raises(InvalidReducedTopologyOutputError):
        ReducedTopologyOutput(
            values,
            status,
            item_diagnostics,
            BranchDiagnosticMap.build(()),
            SourceCoordinateMap.build(()),
        )
