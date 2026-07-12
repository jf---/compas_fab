from __future__ import annotations

from math import inf
from typing import Any
from typing import Tuple
from typing import cast

import pytest
from compas.geometry import Frame
from hypothesis import assume
from hypothesis import given
from hypothesis import strategies as st

from compas_fab.ghpython.item_values import ItemShape
from compas_fab.ghpython.item_values import ShapeTag
from compas_fab.ghpython.port_semantics import BranchSemantics
from compas_fab.ghpython.port_semantics import PortSemantics
from compas_fab.ghpython.port_semantics import TopologyRole
from compas_fab.ghpython.tree_coordinates import BranchCoordinate
from compas_fab.ghpython.tree_coordinates import GhPath
from compas_fab.ghpython.tree_coordinates import ItemIndex
from compas_fab.ghpython.tree_coordinates import TreeCoordinate
from compas_fab.ghpython.tree_coordinates import TreeRootId
from compas_fab.ghpython.tree_diagnostics import SourceCoordinateEntry
from compas_fab.ghpython.tree_diagnostics import SourceCoordinateMap
from compas_fab.ghpython.tree_identity import BOOL_CODEC
from compas_fab.ghpython.tree_identity import FLOAT64_CODEC
from compas_fab.ghpython.tree_identity import FRAME_CODEC
from compas_fab.ghpython.tree_identity import TEXT_CODEC
from compas_fab.ghpython.tree_identity import IdentityVerification
from compas_fab.ghpython.tree_identity import InvalidExactItemCodecError
from compas_fab.ghpython.tree_identity import InvalidItemPayloadError
from compas_fab.ghpython.tree_identity import InvalidSourceTreeIdentityError
from compas_fab.ghpython.tree_identity import InvalidStageParameterError
from compas_fab.ghpython.tree_identity import InvalidStageTreeIdentityError
from compas_fab.ghpython.tree_identity import SourceTreeIdentity
from compas_fab.ghpython.tree_identity import StageParameter
from compas_fab.ghpython.tree_identity import StageTreeIdentity
from compas_fab.ghpython.tree_identity import TreeContentDigest
from compas_fab.ghpython.tree_values import Tree
from compas_fab.ghpython.tree_values import TreeBranch
from compas_fab.ghpython.tree_values import TreeItem
from compas_fab.ghpython.tree_values import TreeTopology

TEXT_TREE_SEMANTICS = PortSemantics.build(
    TopologyRole.TREE,
    BranchSemantics.ORDERED_SEQUENCE,
    ItemShape.scalar(),
)
FRAME_TREE_SEMANTICS = PortSemantics.build(
    TopologyRole.TREE,
    BranchSemantics.ORDERED_SEQUENCE,
    ItemShape.domain_atomic(ShapeTag.build("compas.geometry.Frame")),
)


def text_tree(root: TreeRootId, branches: Tuple[Tuple[Tuple[int, ...], Tuple[str | None, ...]], ...]) -> Tree[str]:
    return Tree.build(
        root,
        tuple(
            TreeBranch.build(
                GhPath.build(*path),
                tuple(TreeItem.null() if item is None else TreeItem.value(item) for item in items),
            )
            for path, items in branches
        ),
    )


def frame_tree(frame: Frame | None = None) -> Tree[Frame]:
    value = frame or Frame.worldXY()
    return Tree.build(
        TreeRootId.build("frames"),
        (TreeBranch.build(GhPath.build(0), (TreeItem.value(value),)),),
    )


def coordinate(root: str, path: Tuple[int, ...], index: int) -> TreeCoordinate:
    return TreeCoordinate.build(
        BranchCoordinate.build(TreeRootId.build(root), GhPath.build(*path)),
        ItemIndex.build(index),
    )


@given(root_a=st.text(min_size=1), root_b=st.text(min_size=1))
def test_runtime_root_never_changes_content_digest(root_a: str, root_b: str) -> None:
    assume(root_a.strip() and root_b.strip())
    branches = (((0,), ("a", None)), ((2,), ()))
    left = text_tree(TreeRootId.build(root_a.strip()), branches)
    right = text_tree(TreeRootId.build(root_b.strip()), branches)

    assert SourceTreeIdentity.build(left, TEXT_CODEC, TEXT_TREE_SEMANTICS) == SourceTreeIdentity.build(right, TEXT_CODEC, TEXT_TREE_SEMANTICS)


def test_stage_identity_is_provenance_not_native_object_serialization() -> None:
    source = SourceTreeIdentity.build(frame_tree(), FRAME_CODEC, FRAME_TREE_SEMANTICS)
    built = StageTreeIdentity.build(
        source,
        "compas_fab.tesseract.pose_series/v1",
        (
            StageParameter.f64("metres_per_user_unit", 0.001),
            StageParameter.text("working_frame", "base_link"),
        ),
        source.topology,
    )

    assert built.verification is IdentityVerification.VERIFIED
    assert built.digest != source.digest


@pytest.mark.parametrize(
    "changed",
    (
        text_tree(TreeRootId.build("source"), (((1,), ("a", None)), ((2,), ()))),
        text_tree(TreeRootId.build("source"), (((0,), ("b", None)), ((2,), ()))),
        text_tree(TreeRootId.build("source"), (((0,), (None, "a")), ((2,), ()))),
        text_tree(TreeRootId.build("source"), (((0,), ("a", None)), ((2,), ("x",)))),
    ),
)
def test_path_item_null_and_empty_branch_changes_alter_digest(changed: Tree[str]) -> None:
    baseline = text_tree(TreeRootId.build("source"), (((0,), ("a", None)), ((2,), ())))

    assert SourceTreeIdentity.build(changed, TEXT_CODEC, TEXT_TREE_SEMANTICS).digest != SourceTreeIdentity.build(baseline, TEXT_CODEC, TEXT_TREE_SEMANTICS).digest


def test_semantics_and_item_order_change_digest() -> None:
    baseline = text_tree(TreeRootId.build("source"), (((0,), ("a", "b")),))
    reversed_items = text_tree(TreeRootId.build("source"), (((0,), ("b", "a")),))
    elementwise = PortSemantics.build(TopologyRole.TREE, BranchSemantics.ELEMENTWISE, ItemShape.scalar())

    identity = SourceTreeIdentity.build(baseline, TEXT_CODEC, TEXT_TREE_SEMANTICS)
    assert SourceTreeIdentity.build(reversed_items, TEXT_CODEC, TEXT_TREE_SEMANTICS).digest != identity.digest
    assert SourceTreeIdentity.build(baseline, TEXT_CODEC, elementwise).digest != identity.digest


def test_length_prefixing_prevents_item_boundary_collision() -> None:
    joined_left = text_tree(TreeRootId.build("source"), (((0,), ("ab", "c")),))
    joined_right = text_tree(TreeRootId.build("source"), (((0,), ("a", "bc")),))

    assert SourceTreeIdentity.build(joined_left, TEXT_CODEC, TEXT_TREE_SEMANTICS).digest != SourceTreeIdentity.build(joined_right, TEXT_CODEC, TEXT_TREE_SEMANTICS).digest


@given(solve=st.integers(min_value=0), request=st.integers(min_value=0))
def test_runtime_generations_do_not_enter_content_identity(solve: int, request: int) -> None:
    tree = text_tree(TreeRootId.build("source"), (((0,), ("a",)),))

    def identity_for_runtime(_solve: int, _request: int) -> SourceTreeIdentity:
        return SourceTreeIdentity.build(tree, TEXT_CODEC, TEXT_TREE_SEMANTICS)

    assert identity_for_runtime(solve, request) == identity_for_runtime(solve + 1, request + 1)


def test_exact_primitive_codecs_reject_cross_type_values() -> None:
    bool_tree = Tree.build(
        TreeRootId.build("bool"),
        (TreeBranch.build(GhPath.build(0), (TreeItem.value(True),)),),
    )
    float_tree = Tree.build(
        TreeRootId.build("float"),
        (TreeBranch.build(GhPath.build(0), (TreeItem.value(1.5),)),),
    )

    assert SourceTreeIdentity.build(bool_tree, BOOL_CODEC, TEXT_TREE_SEMANTICS).verification is IdentityVerification.VERIFIED
    assert SourceTreeIdentity.build(float_tree, FLOAT64_CODEC, TEXT_TREE_SEMANTICS).verification is IdentityVerification.VERIFIED
    with pytest.raises(InvalidItemPayloadError):
        SourceTreeIdentity.build(cast(Any, bool_tree), FLOAT64_CODEC, TEXT_TREE_SEMANTICS)
    with pytest.raises(InvalidItemPayloadError):
        SourceTreeIdentity.build(cast(Any, float_tree), BOOL_CODEC, TEXT_TREE_SEMANTICS)


def test_non_finite_float_and_frame_fields_fail_loudly() -> None:
    float_tree = Tree.build(
        TreeRootId.build("float"),
        (TreeBranch.build(GhPath.build(0), (TreeItem.value(inf),)),),
    )
    frame = Frame.worldXY()
    frame.point.x = inf

    with pytest.raises(InvalidItemPayloadError):
        SourceTreeIdentity.build(float_tree, FLOAT64_CODEC, TEXT_TREE_SEMANTICS)
    with pytest.raises(InvalidItemPayloadError):
        SourceTreeIdentity.build(frame_tree(frame), FRAME_CODEC, FRAME_TREE_SEMANTICS)


def test_arbitrary_native_objects_require_an_exact_codec() -> None:
    native_tree = Tree.build(
        TreeRootId.build("native"),
        (TreeBranch.build(GhPath.build(0), (TreeItem.value(object()),)),),
    )

    with pytest.raises(InvalidExactItemCodecError):
        SourceTreeIdentity.build(native_tree, cast(Any, object()), TEXT_TREE_SEMANTICS)
    with pytest.raises(InvalidItemPayloadError):
        SourceTreeIdentity.build(cast(Any, native_tree), TEXT_CODEC, TEXT_TREE_SEMANTICS)


def test_stage_hashes_root_free_topology_source_map_and_ordered_parameters() -> None:
    source = SourceTreeIdentity.build(text_tree(TreeRootId.build("source"), (((3,), ("a",)),)), TEXT_CODEC, TEXT_TREE_SEMANTICS)
    left_map = SourceCoordinateMap.build((SourceCoordinateEntry.build(coordinate("out-a", (3,), 0), (coordinate("source-a", (3,), 0),)),))
    right_map = SourceCoordinateMap.build((SourceCoordinateEntry.build(coordinate("out-b", (3,), 0), (coordinate("source-b", (3,), 0),)),))
    parameters = (StageParameter.text("frame", "base"), StageParameter.bool("normalize", True))
    left = StageTreeIdentity.build(source, "compas_fab.tesseract.pose_series/v1", parameters, source.topology, left_map)
    right = StageTreeIdentity.build(source, "compas_fab.tesseract.pose_series/v1", parameters, source.topology, right_map)
    reordered = StageTreeIdentity.build(source, "compas_fab.tesseract.pose_series/v1", tuple(reversed(parameters)), source.topology, left_map)

    assert left == right
    assert left.digest != reordered.digest


def test_invalid_stage_schema_parameter_and_raw_identities_fail() -> None:
    source = SourceTreeIdentity.build(text_tree(TreeRootId.build("source"), (((0,), ("a",)),)), TEXT_CODEC, TEXT_TREE_SEMANTICS)

    with pytest.raises(InvalidStageTreeIdentityError):
        StageTreeIdentity.build(source, "caller.native.dumps/v1", (), source.topology)
    with pytest.raises(InvalidStageParameterError):
        StageParameter.f64("scale", inf)
    with pytest.raises(InvalidStageParameterError):
        StageParameter("scale", "f64", 1.0)  # type: ignore[arg-type]
    with pytest.raises(InvalidSourceTreeIdentityError):
        SourceTreeIdentity(
            TreeContentDigest.build("0" * 64),
            (),
            TreeTopology.build(()),
            TEXT_TREE_SEMANTICS,
            IdentityVerification.VERIFIED,
            b"",
            (),
        )


def test_external_native_stage_is_explicitly_unverifiable_and_not_cacheable() -> None:
    source = SourceTreeIdentity.build(text_tree(TreeRootId.build("source"), (((0,), ("a",)),)), TEXT_CODEC, TEXT_TREE_SEMANTICS)

    external = StageTreeIdentity.build(
        source,
        "compas_fab.tesseract.pose_series/v1",
        (),
        source.topology,
        verification=IdentityVerification.UNVERIFIABLE,
    )

    assert external.verification is IdentityVerification.UNVERIFIABLE
    assert external.requires_fresh_compute_token
    assert not external.reusable_from_content_cache


def test_stage_branch_digest_isolates_unchanged_sibling_content() -> None:
    left_source = SourceTreeIdentity.build(
        text_tree(TreeRootId.build("source-a"), (((0,), ("left",)), ((1,), ("stable",)))),
        TEXT_CODEC,
        TEXT_TREE_SEMANTICS,
    )
    right_source = SourceTreeIdentity.build(
        text_tree(TreeRootId.build("source-b"), (((0,), ("right",)), ((1,), ("stable",)))),
        TEXT_CODEC,
        TEXT_TREE_SEMANTICS,
    )
    source_map = SourceCoordinateMap.build(
        tuple(
            SourceCoordinateEntry.build(
                coordinate("output", (index,), 0),
                (coordinate("source-a", (index,), 0),),
            )
            for index in range(2)
        )
    )

    left = StageTreeIdentity.build(left_source, "compas_fab.tesseract.pose_series/v1", (), left_source.topology, source_map)
    right = StageTreeIdentity.build(right_source, "compas_fab.tesseract.pose_series/v1", (), right_source.topology, source_map)

    assert left.branch(GhPath.build(0)) != right.branch(GhPath.build(0))
    assert left.branch(GhPath.build(1)) == right.branch(GhPath.build(1))
    assert left.digest != right.digest
