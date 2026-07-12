"""Typed topology, diagnostic, and source-coordinate output surfaces."""

from __future__ import annotations

from typing import Generic
from typing import Optional
from typing import Tuple
from typing import TypeVar

from attrs import define

from compas_fab.ghpython.tree_coordinates import BranchCoordinate
from compas_fab.ghpython.tree_coordinates import GhPath
from compas_fab.ghpython.tree_coordinates import ItemIndex
from compas_fab.ghpython.tree_coordinates import TreeCoordinate
from compas_fab.ghpython.tree_values import Tree

ValueT = TypeVar("ValueT")
StatusT = TypeVar("StatusT")
DiagnosticT = TypeVar("DiagnosticT")


class TreeDiagnosticContractError(ValueError):
    """Base failure for diagnostic and topology surfaces."""


class InvalidSourceCoordinateEntryError(TreeDiagnosticContractError):
    """Raised when a source-coordinate entry is malformed."""


class EmptySourceCoordinateError(TreeDiagnosticContractError):
    """Raised when an output has no source item coordinates."""


class InvalidSourceCoordinateMapError(TreeDiagnosticContractError):
    """Raised when a source-coordinate map is malformed."""


class DuplicateSourceOutputError(TreeDiagnosticContractError):
    """Raised when an output coordinate occurs more than once."""


class NonCanonicalSourceCoordinateOrderError(InvalidSourceCoordinateMapError):
    """Raised when source-coordinate entries are not in root-free host order."""


class InvalidBranchDiagnosticMapError(TreeDiagnosticContractError):
    """Raised when a branch-diagnostic map is malformed."""


class DuplicateBranchDiagnosticError(TreeDiagnosticContractError):
    """Raised when one branch has multiple branch diagnostics."""


class UnknownBranchDiagnosticError(TreeDiagnosticContractError):
    """Raised when no diagnostic exists for a requested branch."""


class UnknownSourceOutputError(TreeDiagnosticContractError):
    """Raised when no source entry exists for a requested output."""


class InvalidTopologyOutputError(TreeDiagnosticContractError):
    """Raised when parallel value and status topologies differ."""


class InvalidReducedTopologyOutputError(TreeDiagnosticContractError):
    """Raised when a reduced output violates aggregate topology."""


def _part(payload: bytes) -> bytes:
    return len(payload).to_bytes(8, "big") + payload


def _integer_bytes(value: int) -> bytes:
    return str(value).encode("ascii")


def _coordinate_identity_bytes(coordinate: TreeCoordinate) -> bytes:
    path = b"".join(_part(_integer_bytes(index)) for index in coordinate.branch.path.indices)
    return _part(path) + _part(_integer_bytes(coordinate.item_index.value))


def _branch_coordinate_identity_bytes(coordinate: BranchCoordinate) -> bytes:
    return b"".join(_part(_integer_bytes(index)) for index in coordinate.path.indices)


def _coordinate_key(coordinate: TreeCoordinate) -> Tuple[Tuple[int, ...], int]:
    return coordinate.branch.path.canonical_key(), coordinate.item_index.value


@define(frozen=True, slots=True)
class SourceCoordinateEntry:
    """One output attributed to ordered items or one exact empty source branch."""

    output: TreeCoordinate
    sources: Tuple[TreeCoordinate, ...]
    empty_source_branch: Optional[BranchCoordinate] = None

    @classmethod
    def build(cls, output: TreeCoordinate, sources: Tuple[TreeCoordinate, ...]) -> "SourceCoordinateEntry":
        return cls(output, sources)

    @classmethod
    def from_empty_branch(cls, output: TreeCoordinate, source_branch: BranchCoordinate) -> "SourceCoordinateEntry":
        """Attribute one output to a deterministic reduction over an empty branch."""
        return cls(output, (), source_branch)

    def __attrs_post_init__(self) -> None:
        if type(self.output) is not TreeCoordinate or type(self.sources) is not tuple:
            raise InvalidSourceCoordinateEntryError("Source entry requires an exact output and source tuple.")
        if any(type(source) is not TreeCoordinate for source in self.sources):
            raise InvalidSourceCoordinateEntryError("Source entry coordinates must be exact TreeCoordinate values.")
        if self.empty_source_branch is not None and type(self.empty_source_branch) is not BranchCoordinate:
            raise InvalidSourceCoordinateEntryError("Empty-branch provenance requires an exact BranchCoordinate value.")
        if self.sources and self.empty_source_branch is not None:
            raise InvalidSourceCoordinateEntryError("Source entry requires exactly one provenance form.")
        if not self.sources and self.empty_source_branch is None:
            raise EmptySourceCoordinateError("Every mapped output requires at least one source coordinate.")

    def root_free_identity_bytes(self) -> bytes:
        """Encode the provenance kind and coordinates without runtime roots."""
        output = _part(_coordinate_identity_bytes(self.output))
        if self.empty_source_branch is not None:
            return _part(b"empty_branch") + output + _part(_branch_coordinate_identity_bytes(self.empty_source_branch))
        sources = b"".join(_part(_coordinate_identity_bytes(source)) for source in self.sources)
        return _part(b"items") + output + _part(sources)


@define(frozen=True, slots=True)
class SourceCoordinateMap:
    """Immutable output-to-ordered-sources mapping."""

    entries: Tuple[SourceCoordinateEntry, ...]

    @classmethod
    def build(cls, entries: Tuple[SourceCoordinateEntry, ...]) -> "SourceCoordinateMap":
        return cls(entries)

    def __attrs_post_init__(self) -> None:
        if type(self.entries) is not tuple or any(type(entry) is not SourceCoordinateEntry for entry in self.entries):
            raise InvalidSourceCoordinateMapError("Source-coordinate map requires an exact entry tuple.")
        output_keys = tuple(_coordinate_key(entry.output) for entry in self.entries)
        if len(set(output_keys)) != len(output_keys):
            raise DuplicateSourceOutputError("Root-free source-coordinate output keys must be unique.")
        if output_keys != tuple(sorted(output_keys)):
            raise NonCanonicalSourceCoordinateOrderError("Source-coordinate entries must retain canonical root-free output order.")

    def sources_for(self, output: TreeCoordinate) -> Tuple[TreeCoordinate, ...]:
        """Return ordered sources for one exact runtime output coordinate."""
        if type(output) is not TreeCoordinate:
            raise UnknownSourceOutputError("Source lookup requires an exact TreeCoordinate value.")
        for entry in self.entries:
            if entry.output == output:
                return entry.sources
        raise UnknownSourceOutputError("No sources are recorded for the requested output coordinate.")

    def root_free_identity_bytes(self) -> bytes:
        """Encode mapping direction and order without runtime roots."""
        return b"".join(_part(entry.root_free_identity_bytes()) for entry in self.entries)


@define(frozen=True, slots=True)
class BranchDiagnosticMap(Generic[DiagnosticT]):
    """Diagnostics that truthfully apply to whole branches only."""

    entries: Tuple[Tuple[BranchCoordinate, DiagnosticT], ...]

    @classmethod
    def build(cls, entries: Tuple[Tuple[BranchCoordinate, DiagnosticT], ...]) -> "BranchDiagnosticMap[DiagnosticT]":
        return cls(entries)

    def __attrs_post_init__(self) -> None:
        if type(self.entries) is not tuple:
            raise InvalidBranchDiagnosticMapError("Branch diagnostics require an exact tuple.")
        for entry in self.entries:
            if type(entry) is not tuple or len(entry) != 2 or type(entry[0]) is not BranchCoordinate:
                raise InvalidBranchDiagnosticMapError("Each branch diagnostic requires one exact branch coordinate.")
        coordinates = tuple(entry[0] for entry in self.entries)
        if len(set(coordinates)) != len(coordinates):
            raise DuplicateBranchDiagnosticError("Branch-diagnostic coordinates must be unique.")

    def diagnostic_for(self, coordinate: BranchCoordinate) -> DiagnosticT:
        """Return the diagnostic recorded at one branch coordinate."""
        if type(coordinate) is not BranchCoordinate:
            raise UnknownBranchDiagnosticError("Branch diagnostic lookup requires an exact coordinate.")
        for candidate, diagnostic in self.entries:
            if candidate == coordinate:
                return diagnostic
        raise UnknownBranchDiagnosticError("No diagnostic is recorded for the requested branch.")


@define(frozen=True, slots=True)
class TopologyOutput(Generic[ValueT, StatusT]):
    """Values and statuses with exactly identical output topology."""

    values: Tree[ValueT]
    status: Tree[StatusT]

    @classmethod
    def build(cls, values: Tree[ValueT], status: Tree[StatusT]) -> "TopologyOutput[ValueT, StatusT]":
        return cls(values, status)

    def __attrs_post_init__(self) -> None:
        if type(self.values) is not Tree or type(self.status) is not Tree:
            raise InvalidTopologyOutputError("Topology output requires exact value and status trees.")
        if self.values.topology != self.status.topology:
            raise InvalidTopologyOutputError("Value and status trees must have identical topology and null slots.")


@define(frozen=True, slots=True)
class ReducedTopologyOutput(Generic[ValueT, StatusT, DiagnosticT]):
    """One aggregate output per source branch plus source-shaped diagnostics."""

    values: Tree[ValueT]
    status: Tree[StatusT]
    item_diagnostics: Tree[DiagnosticT]
    branch_diagnostics: BranchDiagnosticMap[DiagnosticT]
    source_coordinates: SourceCoordinateMap

    @classmethod
    def build(
        cls,
        values: Tree[ValueT],
        status: Tree[StatusT],
        item_diagnostics: Tree[DiagnosticT],
        branch_diagnostics: BranchDiagnosticMap[DiagnosticT],
        source_coordinates: SourceCoordinateMap,
    ) -> "ReducedTopologyOutput[ValueT, StatusT, DiagnosticT]":
        return cls(values, status, item_diagnostics, branch_diagnostics, source_coordinates)

    def __attrs_post_init__(self) -> None:
        if (
            type(self.values) is not Tree
            or type(self.status) is not Tree
            or type(self.item_diagnostics) is not Tree
            or type(self.branch_diagnostics) is not BranchDiagnosticMap
            or type(self.source_coordinates) is not SourceCoordinateMap
        ):
            raise InvalidReducedTopologyOutputError("Reduced output requires exact typed output surfaces.")
        if self.values.topology != self.status.topology:
            raise InvalidReducedTopologyOutputError("Reduced value and status topologies must be identical.")
        if any(len(branch.items) != 1 for branch in self.values.branches):
            raise InvalidReducedTopologyOutputError("Reduced output requires exactly one aggregate item per branch.")
        output_paths = tuple(branch.path for branch in self.values.branches)
        source_paths = tuple(branch.path for branch in self.item_diagnostics.branches)
        if output_paths != source_paths:
            raise InvalidReducedTopologyOutputError("Reduced outputs and source-shaped diagnostics require equal paths.")
        self._validate_branch_diagnostics(source_paths)
        self._validate_source_coordinates()

    def _validate_branch_diagnostics(self, source_paths: Tuple[GhPath, ...]) -> None:
        source_root = self.item_diagnostics.root_id
        for coordinate, _diagnostic in self.branch_diagnostics.entries:
            if coordinate.root_id != source_root or coordinate.path not in source_paths:
                raise InvalidReducedTopologyOutputError("Branch diagnostics must address exact branches in the source-shaped diagnostic tree.")

    def _validate_source_coordinates(self) -> None:
        expected_entries = []
        for output_branch, source_branch in zip(self.values.branches, self.item_diagnostics.branches):
            output = TreeCoordinate.build(
                BranchCoordinate.build(self.values.root_id, output_branch.path),
                ItemIndex.build(0),
            )
            if not source_branch.items:
                expected_entries.append(
                    SourceCoordinateEntry.from_empty_branch(
                        output,
                        BranchCoordinate.build(self.item_diagnostics.root_id, source_branch.path),
                    )
                )
                continue
            sources = tuple(
                TreeCoordinate.build(
                    BranchCoordinate.build(self.item_diagnostics.root_id, source_branch.path),
                    ItemIndex.build(index),
                )
                for index in range(len(source_branch.items))
            )
            expected_entries.append(SourceCoordinateEntry.build(output, sources))
        if self.source_coordinates.entries != tuple(expected_entries):
            raise InvalidReducedTopologyOutputError("Reduced source mapping must cover every source branch completely and in order.")
