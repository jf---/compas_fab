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
from compas_fab.ghpython.tree_diagnostics import NonCanonicalSourceCoordinateOrderError
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


def test_source_coordinate_map_keys_outputs_without_runtime_roots() -> None:
    sources = (coordinate("source", (4,), 0),)

    with pytest.raises(DuplicateSourceOutputError):
        SourceCoordinateMap.build(
            (
                SourceCoordinateEntry.build(coordinate("output-a", (4,), 0), sources),
                SourceCoordinateEntry.build(coordinate("output-b", (4,), 0), sources),
            )
        )

    with pytest.raises(DuplicateSourceOutputError):
        SourceCoordinateMap(
            (
                SourceCoordinateEntry.build(coordinate("output-a", (4,), 0), sources),
                SourceCoordinateEntry.build(coordinate("output-b", (4,), 0), sources),
            )
        )


def test_source_coordinate_map_requires_canonical_root_free_output_order() -> None:
    first = SourceCoordinateEntry.build(
        coordinate("output", (0,), 1),
        (coordinate("source-b", (9,), 1), coordinate("source-a", (9,), 0)),
    )
    second = SourceCoordinateEntry.build(coordinate("output", (1,), 0), (coordinate("source", (1,), 0),))

    mapping = SourceCoordinateMap.build((first, second))

    assert mapping.entries == (first, second)
    assert mapping.sources_for(first.output) == first.sources
    with pytest.raises(NonCanonicalSourceCoordinateOrderError):
        SourceCoordinateMap.build((second, first))
    with pytest.raises(NonCanonicalSourceCoordinateOrderError):
        SourceCoordinateMap((second, first))


def test_source_coordinate_entry_rejects_raw_container_bypass() -> None:
    output = coordinate("out", (0,), 0)
    source = coordinate("in", (0,), 0)

    with pytest.raises(InvalidSourceCoordinateEntryError):
        SourceCoordinateEntry(output, [source])  # type: ignore[arg-type]


def test_empty_source_branch_provenance_is_explicit_root_free_and_mutually_exclusive() -> None:
    output = coordinate("out-a", (2,), 0)
    empty_branch = BranchCoordinate.build(TreeRootId.build("source-a"), GhPath.build(2))
    rerooted = SourceCoordinateEntry.from_empty_branch(
        coordinate("out-b", (2,), 0),
        BranchCoordinate.build(TreeRootId.build("source-b"), GhPath.build(2)),
    )

    entry = SourceCoordinateEntry.from_empty_branch(output, empty_branch)

    assert entry.sources == ()
    assert entry.empty_source_branch == empty_branch
    assert entry.root_free_identity_bytes() == rerooted.root_free_identity_bytes()
    assert b"empty_branch" in entry.root_free_identity_bytes()
    with pytest.raises(EmptySourceCoordinateError):
        SourceCoordinateEntry(output, ())
    with pytest.raises(InvalidSourceCoordinateEntryError):
        SourceCoordinateEntry(output, (coordinate("source", (2,), 0),), empty_branch)


def test_source_coordinate_map_applies_uniqueness_and_order_across_provenance_forms() -> None:
    item_entry = SourceCoordinateEntry.build(
        coordinate("out", (0,), 0),
        (coordinate("source", (0,), 0),),
    )
    empty_entry = SourceCoordinateEntry.from_empty_branch(
        coordinate("out", (1,), 0),
        BranchCoordinate.build(TreeRootId.build("source"), GhPath.build(1)),
    )

    assert SourceCoordinateMap.build((item_entry, empty_entry)).entries == (item_entry, empty_entry)
    with pytest.raises(DuplicateSourceOutputError):
        SourceCoordinateMap.build(
            (
                item_entry,
                SourceCoordinateEntry.from_empty_branch(
                    coordinate("other-root", (0,), 0),
                    BranchCoordinate.build(TreeRootId.build("source"), GhPath.build(1)),
                ),
            )
        )
    with pytest.raises(NonCanonicalSourceCoordinateOrderError):
        SourceCoordinateMap.build((empty_entry, item_entry))


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
    empty_output_coordinate = coordinate("out-values", (7,), 0)
    sources = tuple(coordinate("source", (0,), index) for index in range(3))
    source_coordinates = SourceCoordinateMap.build(
        (
            SourceCoordinateEntry.build(output_coordinate, sources),
            SourceCoordinateEntry.from_empty_branch(empty_output_coordinate, empty_branch),
        )
    )

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
    assert output.source_coordinates.sources_for(empty_output_coordinate) == ()
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
