from __future__ import annotations

import json
from typing import Optional
from typing import Tuple

import pytest
from hypothesis import given
from hypothesis import strategies as st

from compas_fab.ghpython.tree_codec import EncodedBranch
from compas_fab.ghpython.tree_codec import EncodedSlot
from compas_fab.ghpython.tree_codec import EncodedTree
from compas_fab.ghpython.tree_codec import InvalidEncodedBranchError
from compas_fab.ghpython.tree_codec import InvalidEncodedSlotError
from compas_fab.ghpython.tree_codec import InvalidEncodedTreeError
from compas_fab.ghpython.tree_codec import InvalidTreeValueCodecError
from compas_fab.ghpython.tree_codec import MalformedTreePayloadError
from compas_fab.ghpython.tree_codec import NonCanonicalTreeJsonError
from compas_fab.ghpython.tree_codec import TreeSchemaError
from compas_fab.ghpython.tree_codec import TreeValueCodec
from compas_fab.ghpython.tree_codec import decode_tree
from compas_fab.ghpython.tree_codec import encode_tree
from compas_fab.ghpython.tree_coordinates import GhPath
from compas_fab.ghpython.tree_coordinates import TreeRootId
from compas_fab.ghpython.tree_errors import DuplicateTreePathError
from compas_fab.ghpython.tree_errors import NonCanonicalTreeOrderError
from compas_fab.ghpython.tree_values import Tree
from compas_fab.ghpython.tree_values import TreeBranch
from compas_fab.ghpython.tree_values import TreeItem


def _encode_utf8(value: str) -> bytes:
    return value.encode("utf-8")


def _decode_utf8(payload: bytes) -> str:
    return payload.decode("utf-8")


UTF8_CODEC: TreeValueCodec[str] = TreeValueCodec.build(str, _encode_utf8, _decode_utf8)


@st.composite
def canonical_text_trees(draw: st.DrawFn) -> Tree[str]:
    raw_paths = draw(
        st.lists(
            st.lists(st.integers(min_value=0, max_value=8), min_size=1, max_size=4).map(tuple),
            min_size=0,
            max_size=6,
            unique=True,
        )
    )
    paths = sorted(raw_paths)
    item = st.one_of(st.none(), st.text(max_size=24))
    branches = tuple(
        TreeBranch.build(
            GhPath.build(*path),
            tuple(TreeItem.null() if value is None else TreeItem.value(value) for value in draw(st.lists(item, min_size=0, max_size=6))),
        )
        for path in paths
    )
    return Tree.build(TreeRootId.build("emulated-runtime-root"), branches)


def encoded_tree(
    paths: Tuple[Tuple[int, ...], ...],
    *,
    schema: str = "compas_fab.gh_tree/v1",
    slots: Optional[Tuple[Tuple[object, ...], ...]] = None,
) -> EncodedTree:
    branch_slots = slots if slots is not None else tuple(() for _ in paths)
    document = {
        "schema": schema,
        "branches": [{"path": list(path), "slots": list(values)} for path, values in zip(paths, branch_slots)],
    }
    canonical_json = json.dumps(document, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return EncodedTree.build(TreeRootId.build("emulated-runtime-root"), canonical_json)


@given(canonical_text_trees())
def test_tree_codec_round_trips_canonical_topology(tree: Tree[str]) -> None:
    encoded = encode_tree(tree, UTF8_CODEC)
    assert decode_tree(encoded, UTF8_CODEC) == tree
    assert encode_tree(decode_tree(encoded, UTF8_CODEC), UTF8_CODEC).canonical_json == encoded.canonical_json


def test_tree_codec_retains_prefix_nonzero_empty_null_and_empty_payload() -> None:
    tree = Tree.build(
        TreeRootId.build("runtime-only"),
        (
            TreeBranch.build(
                GhPath.build(2),
                (TreeItem.null(), TreeItem.value(""), TreeItem.value("exact")),
            ),
            TreeBranch.build(GhPath.build(2, 0), ()),
            TreeBranch.build(GhPath.build(9, 4), (TreeItem.value("tail"),)),
        ),
    )

    encoded = encode_tree(tree, UTF8_CODEC)

    assert encoded.root_id == tree.root_id
    assert b"runtime-only" not in encoded.canonical_json
    assert b'"slots":[null,"","ZXhhY3Q="]' in encoded.canonical_json
    assert decode_tree(encoded, UTF8_CODEC) == tree


def test_encoded_records_are_frozen_slotted_and_raw_safe() -> None:
    null = EncodedSlot.null()
    empty = EncodedSlot.value(b"")
    branch = EncodedBranch.build((3, 1), (null, empty))
    tree = EncodedTree.build(TreeRootId.build("route"), b"{}")

    assert null.payload is None
    assert empty.payload == b""
    assert branch.path == (3, 1)
    assert tree.canonical_json == b"{}"
    with pytest.raises(InvalidEncodedSlotError):
        EncodedSlot(payload=b"", is_null=True)
    with pytest.raises(InvalidEncodedBranchError):
        EncodedBranch((True,), ())
    with pytest.raises(InvalidEncodedTreeError):
        EncodedTree(TreeRootId.build("route"), "{}")  # type: ignore[arg-type]
    with pytest.raises(AttributeError):
        tree.canonical_json = b"changed"  # type: ignore[misc]


def test_decode_rejects_nested_runtime_root_mutation() -> None:
    encoded = encoded_tree(((0,),))
    object.__setattr__(encoded.root_id, "value", "")

    with pytest.raises(InvalidEncodedTreeError):
        decode_tree(encoded, UTF8_CODEC)


def test_decode_rejects_unsorted_host_paths_instead_of_reordering() -> None:
    encoded = encoded_tree(paths=((2,), (1,)))
    with pytest.raises(NonCanonicalTreeOrderError):
        decode_tree(encoded, UTF8_CODEC)


def test_decode_rejects_duplicate_paths_before_tree_construction() -> None:
    encoded = encoded_tree(paths=((2,), (2,)))
    with pytest.raises(DuplicateTreePathError):
        decode_tree(encoded, UTF8_CODEC)


def test_decode_rejects_wrong_schema_and_malformed_base64_with_named_errors() -> None:
    with pytest.raises(TreeSchemaError):
        decode_tree(encoded_tree(((0,),), schema="compas_fab.gh_tree/v2"), UTF8_CODEC)
    with pytest.raises(MalformedTreePayloadError):
        decode_tree(encoded_tree(((0,),), slots=(("not base64!",),)), UTF8_CODEC)


def test_decode_requires_canonical_json_byte_equality() -> None:
    canonical = encoded_tree(((0,),), slots=(("YQ==",),))
    spaced = EncodedTree.build(canonical.root_id, canonical.canonical_json.replace(b",", b", "))
    reordered = EncodedTree.build(
        canonical.root_id,
        b'{"schema":"compas_fab.gh_tree/v1","branches":[{"path":[0],"slots":["YQ=="]}]}',
    )

    with pytest.raises(NonCanonicalTreeJsonError):
        decode_tree(spaced, UTF8_CODEC)
    with pytest.raises(NonCanonicalTreeJsonError):
        decode_tree(reordered, UTF8_CODEC)


def test_decode_requires_codec_exact_value_round_trip() -> None:
    normalizing = TreeValueCodec.build(str, _encode_utf8, lambda payload: payload.decode("utf-8").upper())
    encoded = encoded_tree(((0,),), slots=(("YQ==",),))
    with pytest.raises(NonCanonicalTreeJsonError):
        decode_tree(encoded, normalizing)


def test_tree_value_codec_rejects_raw_type_and_payload_bypass() -> None:
    with pytest.raises(InvalidTreeValueCodecError):
        TreeValueCodec(str, _encode_utf8, object())  # type: ignore[arg-type]
    with pytest.raises(InvalidTreeValueCodecError):
        TreeValueCodec.build(str, lambda value: value, _decode_utf8).encode("text")  # type: ignore[arg-type,return-value]
    with pytest.raises(InvalidTreeValueCodecError):
        TreeValueCodec.build(str, _encode_utf8, lambda payload: payload).decode(b"text")  # type: ignore[arg-type,return-value]
