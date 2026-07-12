"""One tagged reversible path codec for every cardinality expansion."""

from __future__ import annotations

from typing import Tuple
from typing import Union

from attrs import define

from compas_fab.ghpython.tree_coordinates import BranchCoordinate
from compas_fab.ghpython.tree_coordinates import GhPath
from compas_fab.ghpython.tree_coordinates import ItemIndex
from compas_fab.ghpython.tree_coordinates import TreeCoordinate
from compas_fab.ghpython.tree_coordinates import TreeRootId

_CROSS_PRODUCT_TAG = 0
_EXPANDED_BRANCH_TAG = 1
_MAX_UTF8_BYTE = 255


class ExpansionCodecError(ValueError):
    """Base failure for expanded coordinate values and paths."""


class InvalidRequestOrdinalError(ExpansionCodecError):
    """Raised when a request ordinal is invalid."""


class InvalidResultOrdinalError(ExpansionCodecError):
    """Raised when a result ordinal is invalid."""


class InvalidExpansionCoordinateError(ExpansionCodecError):
    """Raised when an expanded coordinate is structurally invalid."""


class InvalidExpansionPathError(ExpansionCodecError):
    """Raised when a tagged expanded path is malformed or incomplete."""


@define(frozen=True, slots=True)
class RequestOrdinal:
    """Exact zero-based request position within one source branch."""

    value: int

    @classmethod
    def build(cls, value: int) -> "RequestOrdinal":
        return cls(value)

    def __attrs_post_init__(self) -> None:
        if type(self.value) is not int or self.value < 0:
            raise InvalidRequestOrdinalError("Request ordinal must be an exact non-negative integer.")


@define(frozen=True, slots=True)
class ResultOrdinal:
    """Exact zero-based result position within one request."""

    value: int

    @classmethod
    def build(cls, value: int) -> "ResultOrdinal":
        return cls(value)

    def __attrs_post_init__(self) -> None:
        if type(self.value) is not int or self.value < 0:
            raise InvalidResultOrdinalError("Result ordinal must be an exact non-negative integer.")


@define(frozen=True, slots=True)
class CrossProductCoordinate:
    """Exact left/right item coordinates for one explicit product pair."""

    left: TreeCoordinate
    right: TreeCoordinate

    @classmethod
    def build(cls, left: TreeCoordinate, right: TreeCoordinate) -> "CrossProductCoordinate":
        return cls(left, right)

    def __attrs_post_init__(self) -> None:
        if type(self.left) is not TreeCoordinate or type(self.right) is not TreeCoordinate:
            raise InvalidExpansionCoordinateError("Cross-product coordinate requires two exact item coordinates.")


@define(frozen=True, slots=True)
class ExpandedBranchCoordinate:
    """Reserved result branch coordinate beneath one exact source branch."""

    source_branch: BranchCoordinate
    request_ordinal: RequestOrdinal
    result_ordinal: ResultOrdinal

    @classmethod
    def build(
        cls,
        source_branch: BranchCoordinate,
        request_ordinal: RequestOrdinal,
        result_ordinal: ResultOrdinal,
    ) -> "ExpandedBranchCoordinate":
        return cls(source_branch, request_ordinal, result_ordinal)

    def __attrs_post_init__(self) -> None:
        if type(self.source_branch) is not BranchCoordinate or type(self.request_ordinal) is not RequestOrdinal or type(self.result_ordinal) is not ResultOrdinal:
            raise InvalidExpansionCoordinateError("Expanded branch coordinate requires an exact source branch and request/result ordinals.")


ExpansionCoordinate = Union[CrossProductCoordinate, ExpandedBranchCoordinate]


def _encode_root(root_id: TreeRootId) -> Tuple[int, ...]:
    payload = root_id.value.encode("utf-8")
    return (len(payload), *(byte for byte in payload))


def _encode_branch(branch: BranchCoordinate) -> Tuple[int, ...]:
    return (*_encode_root(branch.root_id), len(branch.path.indices), *branch.path.indices)


class _PathReader:
    def __init__(self, values: Tuple[int, ...]) -> None:
        self._values = values
        self._index = 0

    def read(self, label: str) -> int:
        if self._index >= len(self._values):
            raise InvalidExpansionPathError("Expanded path ended before {0}.".format(label))
        value = self._values[self._index]
        self._index += 1
        return value

    def read_counted(self, label: str, *, allow_empty: bool) -> Tuple[int, ...]:
        count = self.read("{0} count".format(label))
        if count < 0 or (not allow_empty and count == 0):
            raise InvalidExpansionPathError("Expanded path contains an invalid {0} count.".format(label))
        if self._index + count > len(self._values):
            raise InvalidExpansionPathError("Expanded path truncates the declared {0}.".format(label))
        values = self._values[self._index : self._index + count]
        self._index += count
        return values

    def read_root(self) -> TreeRootId:
        payload = self.read_counted("root UTF-8 payload", allow_empty=False)
        if any(byte > _MAX_UTF8_BYTE for byte in payload):
            raise InvalidExpansionPathError("Expanded path root payload contains a non-byte segment.")
        try:
            value = bytes(payload).decode("utf-8")
        except UnicodeDecodeError as error:
            raise InvalidExpansionPathError("Expanded path root payload is not canonical UTF-8.") from error
        try:
            return TreeRootId.build(value)
        except ValueError as error:
            raise InvalidExpansionPathError("Expanded path contains an invalid tree root.") from error

    def read_branch(self) -> BranchCoordinate:
        root_id = self.read_root()
        path = self.read_counted("path", allow_empty=False)
        try:
            return BranchCoordinate.build(root_id, GhPath.build(*path))
        except ValueError as error:
            raise InvalidExpansionPathError("Expanded path contains an invalid source path.") from error

    def finish(self) -> None:
        if self._index != len(self._values):
            raise InvalidExpansionPathError("Expanded path contains trailing segments.")


class ExpansionPathCodec:
    """Encode both expansion coordinate kinds into disjoint exact paths."""

    @staticmethod
    def encode(coordinate: ExpansionCoordinate) -> GhPath:
        if type(coordinate) is CrossProductCoordinate:
            return GhPath.build(
                _CROSS_PRODUCT_TAG,
                *_encode_branch(coordinate.left.branch),
                coordinate.left.item_index.value,
                *_encode_branch(coordinate.right.branch),
                coordinate.right.item_index.value,
            )
        if type(coordinate) is ExpandedBranchCoordinate:
            return GhPath.build(
                _EXPANDED_BRANCH_TAG,
                *_encode_branch(coordinate.source_branch),
                coordinate.request_ordinal.value,
                coordinate.result_ordinal.value,
            )
        raise InvalidExpansionCoordinateError("Expansion codec requires an exact declared coordinate kind.")

    @staticmethod
    def decode(path: GhPath) -> ExpansionCoordinate:
        if type(path) is not GhPath:
            raise InvalidExpansionPathError("Expansion decoder requires an exact GhPath value.")
        reader = _PathReader(path.indices)
        tag = reader.read("coordinate-kind tag")
        if tag == _CROSS_PRODUCT_TAG:
            left_branch = reader.read_branch()
            left_index = reader.read("left item ordinal")
            right_branch = reader.read_branch()
            right_index = reader.read("right item ordinal")
            reader.finish()
            try:
                return CrossProductCoordinate.build(
                    TreeCoordinate.build(left_branch, ItemIndex.build(left_index)),
                    TreeCoordinate.build(right_branch, ItemIndex.build(right_index)),
                )
            except ValueError as error:
                raise InvalidExpansionPathError("Expanded path contains an invalid item ordinal.") from error
        if tag == _EXPANDED_BRANCH_TAG:
            source_branch = reader.read_branch()
            request_ordinal = reader.read("request ordinal")
            result_ordinal = reader.read("result ordinal")
            reader.finish()
            try:
                return ExpandedBranchCoordinate.build(
                    source_branch,
                    RequestOrdinal.build(request_ordinal),
                    ResultOrdinal.build(result_ordinal),
                )
            except ValueError as error:
                raise InvalidExpansionPathError("Expanded path contains an invalid result ordinal.") from error
        raise InvalidExpansionPathError("Expanded path contains an unknown coordinate-kind tag.")
