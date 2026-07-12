"""Canonical host-neutral wire encoding for exact Grasshopper trees."""

from __future__ import annotations

import base64
import binascii
import json
from typing import Callable
from typing import Dict
from typing import Generic
from typing import List
from typing import Optional
from typing import Tuple
from typing import Type
from typing import TypeVar
from typing import cast

from attrs import define

from compas_fab.ghpython.tree_coordinates import GhPath
from compas_fab.ghpython.tree_coordinates import TreeRootId
from compas_fab.ghpython.tree_errors import DuplicateTreePathError
from compas_fab.ghpython.tree_errors import NonCanonicalTreeOrderError
from compas_fab.ghpython.tree_values import Tree
from compas_fab.ghpython.tree_values import TreeBranch
from compas_fab.ghpython.tree_values import TreeItem

T = TypeVar("T")
TREE_WIRE_SCHEMA = "compas_fab.gh_tree/v1"


class TreeCodecError(ValueError):
    """Base failure for canonical tree wire encoding."""


class InvalidEncodedSlotError(TreeCodecError):
    """Raised when an encoded slot violates null/payload structure."""


class InvalidEncodedBranchError(TreeCodecError):
    """Raised when an encoded branch has an invalid path or slot tuple."""


class InvalidEncodedTreeError(TreeCodecError):
    """Raised when the runtime envelope or canonical JSON has a wrong type."""


class InvalidTreeValueCodecError(TreeCodecError):
    """Raised when a value codec or one of its exact values is invalid."""


class MalformedTreeJsonError(TreeCodecError):
    """Raised when the wire document is not the exact declared JSON shape."""


class TreeSchemaError(TreeCodecError):
    """Raised when the wire document has an unknown tree schema."""


class MalformedTreePayloadError(TreeCodecError):
    """Raised when a non-null slot is not canonical base64."""


class NonCanonicalTreeJsonError(TreeCodecError):
    """Raised when decoded content does not reproduce the original JSON bytes."""


@define(frozen=True, slots=True)
class EncodedSlot:
    """One exact null marker or non-null byte payload."""

    payload: Optional[bytes]
    is_null: bool

    @classmethod
    def value(cls, payload: bytes) -> "EncodedSlot":
        return cls(payload, False)

    @classmethod
    def null(cls) -> "EncodedSlot":
        return cls(None, True)

    def __attrs_post_init__(self) -> None:
        if type(self.is_null) is not bool:
            raise InvalidEncodedSlotError("Encoded slot null state must be exact bool.")
        if self.is_null:
            if self.payload is not None:
                raise InvalidEncodedSlotError("Encoded null slot must not contain a payload.")
        elif type(self.payload) is not bytes:
            raise InvalidEncodedSlotError("Encoded value slot must contain exact bytes, including empty bytes.")


def _valid_encoded_slot(slot: object) -> bool:
    if type(slot) is not EncodedSlot or type(slot.is_null) is not bool:
        return False
    return (slot.is_null and slot.payload is None) or (not slot.is_null and type(slot.payload) is bytes)


@define(frozen=True, slots=True)
class EncodedBranch:
    """One canonical primitive path and its ordered encoded slots."""

    path: Tuple[int, ...]
    slots: Tuple[EncodedSlot, ...]

    @classmethod
    def build(cls, path: Tuple[int, ...], slots: Tuple[EncodedSlot, ...]) -> "EncodedBranch":
        return cls(path, slots)

    def __attrs_post_init__(self) -> None:
        if type(self.path) is not tuple or not self.path:
            raise InvalidEncodedBranchError("Encoded branch path must be a non-empty exact tuple.")
        if any(type(index) is not int or index < 0 for index in self.path):
            raise InvalidEncodedBranchError("Encoded branch path segments must be exact non-negative integers.")
        if type(self.slots) is not tuple or any(not _valid_encoded_slot(slot) for slot in self.slots):
            raise InvalidEncodedBranchError("Encoded branch slots must be an exact tuple of valid EncodedSlot values.")


@define(frozen=True, slots=True)
class EncodedTree:
    """Canonical content bytes plus a separate runtime-routing envelope."""

    root_id: TreeRootId
    canonical_json: bytes

    @classmethod
    def build(cls, root_id: TreeRootId, canonical_json: bytes) -> "EncodedTree":
        return cls(root_id, canonical_json)

    def __attrs_post_init__(self) -> None:
        if type(self.root_id) is not TreeRootId or type(self.canonical_json) is not bytes:
            raise InvalidEncodedTreeError("Encoded tree requires an exact runtime root and JSON byte payload.")
        _validated_runtime_root(self.root_id)


def _validated_runtime_root(root_id: TreeRootId) -> TreeRootId:
    try:
        return TreeRootId.build(root_id.value)
    except (AttributeError, ValueError) as error:
        raise InvalidEncodedTreeError("Encoded tree runtime root must remain a valid exact routing value.") from error


def _validated_tree_for_encoding(tree: Tree[T]) -> Tree[T]:
    try:
        root_id = _validated_runtime_root(tree.root_id)
        if type(tree.branches) is not tuple:
            raise InvalidEncodedTreeError("Tree producer branches must remain an exact tuple.")
        branches: List[TreeBranch[T]] = []
        for branch in tree.branches:
            if type(branch) is not TreeBranch or type(branch.path) is not GhPath or type(branch.items) is not tuple:
                raise InvalidEncodedTreeError("Tree producer branches must remain exact branch values.")
            path = GhPath.build(*branch.path.indices)
            items: List[TreeItem[T]] = []
            for item in branch.items:
                if type(item) is not TreeItem:
                    raise InvalidEncodedTreeError("Tree producer items must remain exact TreeItem values.")
                if type(item.is_null) is not bool or item.is_null != (item.item is None):
                    raise InvalidEncodedTreeError("Tree producer items must retain exact null/value state.")
                if item.is_null:
                    items.append(TreeItem.null())
                else:
                    items.append(TreeItem.value(cast(T, item.item)))
            branches.append(TreeBranch.build(path, tuple(items)))
        return Tree.build(root_id, tuple(branches))
    except InvalidEncodedTreeError:
        raise
    except (AttributeError, TypeError, ValueError) as error:
        raise InvalidEncodedTreeError("Tree producer contains invalid nested topology or item state.") from error


@define(frozen=True, slots=True)
class TreeValueCodec(Generic[T]):
    """Exact-type reversible byte codec for one tree value domain."""

    item_type: Type[T]
    encoder: Callable[[T], bytes]
    decoder: Callable[[bytes], T]

    @classmethod
    def build(
        cls,
        item_type: Type[T],
        encoder: Callable[[T], bytes],
        decoder: Callable[[bytes], T],
    ) -> "TreeValueCodec[T]":
        return cls(item_type, encoder, decoder)

    def __attrs_post_init__(self) -> None:
        if not isinstance(self.item_type, type) or not callable(self.encoder) or not callable(self.decoder):
            raise InvalidTreeValueCodecError("Tree value codec requires an exact type and callable encoder/decoder.")

    def encode(self, value: T) -> bytes:
        """Encode one exact domain value to exact bytes."""
        if type(value) is not self.item_type:
            raise InvalidTreeValueCodecError("Tree value does not match the codec's exact item type.")
        try:
            payload = self.encoder(value)
        except (OverflowError, TypeError, UnicodeError, ValueError) as error:
            raise InvalidTreeValueCodecError("Tree value encoder rejected the value.") from error
        if type(payload) is not bytes:
            raise InvalidTreeValueCodecError("Tree value encoder must return exact bytes.")
        return payload

    def decode(self, payload: bytes) -> T:
        """Decode exact bytes and require the codec's exact domain type."""
        if type(payload) is not bytes:
            raise InvalidTreeValueCodecError("Tree value decoder requires exact bytes.")
        try:
            value = self.decoder(payload)
        except (OverflowError, TypeError, UnicodeError, ValueError) as error:
            raise InvalidTreeValueCodecError("Tree value decoder rejected the payload.") from error
        if type(value) is not self.item_type:
            raise InvalidTreeValueCodecError("Tree value decoder returned the wrong exact item type.")
        return value


class _DuplicateObjectKeyError(ValueError):
    pass


def _object_without_duplicate_keys(pairs: List[Tuple[str, object]]) -> Dict[str, object]:
    result: Dict[str, object] = {}
    for key, value in pairs:
        if key in result:
            raise _DuplicateObjectKeyError(key)
        result[key] = value
    return result


def _canonical_json(branches: Tuple[EncodedBranch, ...]) -> bytes:
    document = {
        "schema": TREE_WIRE_SCHEMA,
        "branches": [
            {
                "path": list(branch.path),
                "slots": [None if slot.is_null else base64.b64encode(cast(bytes, slot.payload)).decode("ascii") for slot in branch.slots],
            }
            for branch in branches
        ],
    }
    return json.dumps(document, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")


def _encode_branches(tree: Tree[T], codec: TreeValueCodec[T]) -> Tuple[EncodedBranch, ...]:
    return tuple(
        EncodedBranch.build(
            branch.path.indices,
            tuple(EncodedSlot.null() if item.is_null else EncodedSlot.value(codec.encode(cast(T, item.item))) for item in branch.items),
        )
        for branch in tree.branches
    )


def encode_tree(tree: Tree[T], codec: TreeValueCodec[T]) -> EncodedTree:
    """Encode exact topology and values while retaining the root only for routing."""
    if type(tree) is not Tree:
        raise InvalidEncodedTreeError("Tree encoder requires an exact Tree value.")
    if type(codec) is not TreeValueCodec:
        raise InvalidTreeValueCodecError("Tree encoder requires an exact TreeValueCodec value.")
    validated_tree = _validated_tree_for_encoding(tree)
    branches = _encode_branches(validated_tree, codec)
    return EncodedTree.build(validated_tree.root_id, _canonical_json(branches))


def _parse_document(canonical_json: bytes) -> Dict[str, object]:
    try:
        parsed = json.loads(canonical_json, object_pairs_hook=_object_without_duplicate_keys)
    except (UnicodeDecodeError, json.JSONDecodeError, _DuplicateObjectKeyError) as error:
        raise MalformedTreeJsonError("Tree wire payload must be unique-key UTF-8 JSON.") from error
    if type(parsed) is not dict or set(parsed) != {"branches", "schema"}:
        raise MalformedTreeJsonError("Tree wire document must contain only schema and branches.")
    return cast(Dict[str, object], parsed)


def _parse_slot(raw_slot: object) -> EncodedSlot:
    if raw_slot is None:
        return EncodedSlot.null()
    if type(raw_slot) is not str:
        raise MalformedTreePayloadError("Tree wire slot must be null or canonical base64 text.")
    try:
        ascii_payload = raw_slot.encode("ascii")
        payload = base64.b64decode(ascii_payload, validate=True)
    except (UnicodeEncodeError, binascii.Error, ValueError) as error:
        raise MalformedTreePayloadError("Tree wire slot contains malformed base64.") from error
    if base64.b64encode(payload) != ascii_payload:
        raise MalformedTreePayloadError("Tree wire slot contains non-canonical base64.")
    return EncodedSlot.value(payload)


def _parse_branch(raw_branch: object) -> EncodedBranch:
    if type(raw_branch) is not dict or set(raw_branch) != {"path", "slots"}:
        raise MalformedTreeJsonError("Tree wire branch must contain only path and slots.")
    branch = cast(Dict[str, object], raw_branch)
    raw_path = branch["path"]
    raw_slots = branch["slots"]
    if type(raw_path) is not list or type(raw_slots) is not list:
        raise MalformedTreeJsonError("Tree wire branch path and slots must be arrays.")
    path_values = cast(List[object], raw_path)
    if not path_values or any(type(index) is not int or index < 0 for index in path_values):
        raise InvalidEncodedBranchError("Tree wire path requires exact non-negative integer segments.")
    return EncodedBranch.build(
        tuple(cast(int, index) for index in path_values),
        tuple(_parse_slot(slot) for slot in cast(List[object], raw_slots)),
    )


def _parse_branches(document: Dict[str, object]) -> Tuple[EncodedBranch, ...]:
    if document["schema"] != TREE_WIRE_SCHEMA or type(document["schema"]) is not str:
        raise TreeSchemaError("Tree wire document schema must be compas_fab.gh_tree/v1.")
    raw_branches = document["branches"]
    if type(raw_branches) is not list:
        raise MalformedTreeJsonError("Tree wire branches must be an array.")
    branches = tuple(_parse_branch(branch) for branch in cast(List[object], raw_branches))
    paths = tuple(branch.path for branch in branches)
    if len(set(paths)) != len(paths):
        raise DuplicateTreePathError("Decoded tree paths must be unique.")
    if paths != tuple(sorted(paths)):
        raise NonCanonicalTreeOrderError("Decoded tree paths must retain canonical host order.")
    return branches


def _decode_branches(branches: Tuple[EncodedBranch, ...], codec: TreeValueCodec[T]) -> Tuple[TreeBranch[T], ...]:
    return tuple(
        TreeBranch.build(
            GhPath.build(*branch.path),
            tuple(TreeItem.null() if slot.is_null else TreeItem.value(codec.decode(cast(bytes, slot.payload))) for slot in branch.slots),
        )
        for branch in branches
    )


def decode_tree(encoded: EncodedTree, codec: TreeValueCodec[T]) -> Tree[T]:
    """Decode through exact factories and require canonical byte reproduction."""
    if type(encoded) is not EncodedTree or type(encoded.root_id) is not TreeRootId or type(encoded.canonical_json) is not bytes:
        raise InvalidEncodedTreeError("Tree decoder requires an exact valid EncodedTree envelope.")
    if type(codec) is not TreeValueCodec:
        raise InvalidTreeValueCodecError("Tree decoder requires an exact TreeValueCodec value.")
    root_id = _validated_runtime_root(encoded.root_id)
    branches = _parse_branches(_parse_document(encoded.canonical_json))
    tree = Tree.build(root_id, _decode_branches(branches, codec))
    if encode_tree(tree, codec).canonical_json != encoded.canonical_json:
        raise NonCanonicalTreeJsonError("Decoded tree does not reproduce the exact canonical JSON bytes.")
    return tree
