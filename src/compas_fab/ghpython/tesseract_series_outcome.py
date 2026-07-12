"""Shared item outcome surfaces for native authoring series."""

from __future__ import annotations

from enum import Enum
from typing import Generic
from typing import Tuple
from typing import TypeVar

from attrs import define

from compas_fab.ghpython.tree_coordinates import TreeCoordinate
from compas_fab.ghpython.tree_diagnostics import SourceCoordinateMap
from compas_fab.ghpython.tree_values import Tree

T = TypeVar("T")


class SeriesOutcomeError(ValueError):
    """Base failure for native-series output surfaces."""


class InvalidSeriesTopologyOutputError(SeriesOutcomeError):
    """Raised when value, status, diagnostic, and provenance shapes disagree."""


class SeriesItemStatus(Enum):
    VALID = "valid"
    INVALID = "invalid"


@define(frozen=True, slots=True)
class SeriesDiagnostic:
    """One named item-local authoring failure."""

    code: str
    message: str
    coordinate: TreeCoordinate

    @classmethod
    def build(cls, code: str, message: str, coordinate: TreeCoordinate) -> "SeriesDiagnostic":
        return cls(code, message, coordinate)

    def __attrs_post_init__(self) -> None:
        if (
            type(self.code) is not str
            or not self.code
            or self.code != self.code.strip()
            or type(self.message) is not str
            or not self.message
            or type(self.coordinate) is not TreeCoordinate
        ):
            raise InvalidSeriesTopologyOutputError("Series diagnostic requires a code, message, and exact coordinate.")


@define(frozen=True, slots=True)
class SeriesTopologyOutput(Generic[T]):
    """Exact values plus status, source-shaped diagnostics, and provenance."""

    values: Tree[T]
    status: Tree[SeriesItemStatus]
    item_diagnostics: Tree[SeriesDiagnostic]
    source_coordinates: SourceCoordinateMap

    @classmethod
    def build(
        cls,
        values: Tree[T],
        status: Tree[SeriesItemStatus],
        item_diagnostics: Tree[SeriesDiagnostic],
        source_coordinates: SourceCoordinateMap,
    ) -> "SeriesTopologyOutput[T]":
        return cls(values, status, item_diagnostics, source_coordinates)

    def __attrs_post_init__(self) -> None:
        if (
            type(self.values) is not Tree
            or type(self.status) is not Tree
            or type(self.item_diagnostics) is not Tree
            or type(self.source_coordinates) is not SourceCoordinateMap
        ):
            raise InvalidSeriesTopologyOutputError("Series output requires exact typed tree surfaces.")
        expected = tuple((branch.path, len(branch.items)) for branch in self.values.branches)
        status = tuple((branch.path, len(branch.items)) for branch in self.status.branches)
        diagnostics = tuple((branch.path, len(branch.items)) for branch in self.item_diagnostics.branches)
        if expected != status or expected != diagnostics:
            raise InvalidSeriesTopologyOutputError("Series value, status, and diagnostic paths/counts must match exactly.")
        if self.values.root_id != self.status.root_id or self.values.root_id != self.item_diagnostics.root_id:
            raise InvalidSeriesTopologyOutputError("Series value, status, and diagnostic runtime roots must match exactly.")
        expected_coordinates: Tuple[Tuple[Tuple[int, ...], int], ...] = tuple(
            (branch.path.canonical_key(), index)
            for branch in self.values.branches
            for index in range(len(branch.items))
        )
        mapped = tuple(
            (entry.output.branch.path.canonical_key(), entry.output.item_index.value)
            for entry in self.source_coordinates.entries
        )
        if mapped != expected_coordinates:
            raise InvalidSeriesTopologyOutputError("Series source coordinates must cover every output slot exactly once.")
        if any(entry.output.branch.root_id != self.values.root_id for entry in self.source_coordinates.entries):
            raise InvalidSeriesTopologyOutputError("Series provenance outputs must use the exact values runtime root.")
