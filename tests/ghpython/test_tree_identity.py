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

import compas_fab.ghpython.tree_identity as tree_identity
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
from compas_fab.ghpython.tesseract_program_series import PROGRAM_SERIES_SCHEMA
from compas_fab.ghpython.tree_diagnostics import SourceCoordinateEntry
from compas_fab.ghpython.tree_diagnostics import SourceCoordinateMap
from compas_fab.ghpython.tree_identity import BOOL_CODEC
from compas_fab.ghpython.tree_identity import FLOAT64_CODEC
from compas_fab.ghpython.tree_identity import FRAME_CODEC
from compas_fab.ghpython.tree_identity import TEXT_CODEC
from compas_fab.ghpython.tree_identity import IdentityVerification
from compas_fab.ghpython.tree_identity import IncompleteStageSourceCoordinatesError
from compas_fab.ghpython.tree_identity import InvalidExactItemCodecError
from compas_fab.ghpython.tree_identity import InvalidItemPayloadError
from compas_fab.ghpython.tree_identity import InvalidSourceTreeIdentityError
from compas_fab.ghpython.tree_identity import InvalidStageBuilderAuthorityError
from compas_fab.ghpython.tree_identity import InvalidStageParameterError
from compas_fab.ghpython.tree_identity import InvalidStageTreeIdentityError
from compas_fab.ghpython.tree_identity import NonEmptyStageSourceBranchError
from compas_fab.ghpython.tree_identity import SourceTreeIdentity
from compas_fab.ghpython.tree_identity import SourceCoordinateCoverage
from compas_fab.ghpython.tree_identity import SourceCoordinateRequirement
from compas_fab.ghpython.tree_identity import StageParameter
from compas_fab.ghpython.tree_identity import StageTreeIdentity
from compas_fab.ghpython.tree_identity import TreeContentDigest
from compas_fab.ghpython.tree_identity import UnknownStageBuilderSchemaError
from compas_fab.ghpython.tree_identity import UnknownStageOutputCoordinateError
from compas_fab.ghpython.tree_identity import UnknownStageSourceBranchError
from compas_fab.ghpython.tree_identity import UnknownStageSourceCoordinateError
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
TEST_STAGE_SCHEMA = "tests.ghpython.verified_stage/v1"


def _test_stage_builder() -> None:
    pass


@pytest.fixture
def audited_stage_schema(monkeypatch: pytest.MonkeyPatch) -> str:
    descriptor = tree_identity._AuditedStageSchema.build(
        TEST_STAGE_SCHEMA,
        _test_stage_builder,
        SourceCoordinateRequirement.COMPLETE_OUTPUTS,
    )
    monkeypatch.setitem(tree_identity._AUDITED_STAGE_SCHEMAS, TEST_STAGE_SCHEMA, descriptor)
    return TEST_STAGE_SCHEMA


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


def test_stage_identity_is_provenance_not_native_object_serialization(audited_stage_schema: str) -> None:
    source = SourceTreeIdentity.build(frame_tree(), FRAME_CODEC, FRAME_TREE_SEMANTICS)
    source_map = SourceCoordinateMap.build((SourceCoordinateEntry.build(coordinate("output", (0,), 0), (coordinate("frames", (0,), 0),)),))
    built = StageTreeIdentity.build(
        source,
        audited_stage_schema,
        (
            StageParameter.f64("metres_per_user_unit", 0.001),
            StageParameter.text("working_frame", "base_link"),
        ),
        source.topology,
        source_map,
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


def test_stage_hashes_root_free_topology_source_map_and_ordered_parameters(audited_stage_schema: str) -> None:
    source = SourceTreeIdentity.build(text_tree(TreeRootId.build("source"), (((3,), ("a",)),)), TEXT_CODEC, TEXT_TREE_SEMANTICS)
    left_map = SourceCoordinateMap.build((SourceCoordinateEntry.build(coordinate("out-a", (3,), 0), (coordinate("source-a", (3,), 0),)),))
    right_map = SourceCoordinateMap.build((SourceCoordinateEntry.build(coordinate("out-b", (3,), 0), (coordinate("source-b", (3,), 0),)),))
    parameters = (StageParameter.text("frame", "base"), StageParameter.bool("normalize", True))
    left = StageTreeIdentity.build(source, audited_stage_schema, parameters, source.topology, left_map)
    right = StageTreeIdentity.build(source, audited_stage_schema, parameters, source.topology, right_map)
    reordered = StageTreeIdentity.build(source, audited_stage_schema, tuple(reversed(parameters)), source.topology, left_map)

    assert left == right
    assert left.digest != reordered.digest


@pytest.mark.parametrize(
    "builder_schema",
    (
        "compas_fab.tesseract.pose_series/v1",
        "compas_fab.tesseract.cartesian_target_series/v1",
        "compas_fab.tesseract.target_series/v1",
        "compas_fab.tesseract.program_series/v1",
        "compas_fab.tesseract.planning/v1",
    ),
)
def test_unimplemented_builder_schema_cannot_claim_verified(builder_schema: str) -> None:
    source = SourceTreeIdentity.build(text_tree(TreeRootId.build("source"), (((0,), ("a",)),)), TEXT_CODEC, TEXT_TREE_SEMANTICS)

    with pytest.raises(UnknownStageBuilderSchemaError):
        StageTreeIdentity.build(source, builder_schema, (), source.topology)


def test_registered_builder_schema_cannot_claim_verified_without_sealed_authority() -> None:
    source = SourceTreeIdentity.build(text_tree(TreeRootId.build("source"), (((0,), ("a",)),)), TEXT_CODEC, TEXT_TREE_SEMANTICS)

    with pytest.raises(InvalidStageBuilderAuthorityError):
        StageTreeIdentity.build(source, PROGRAM_SERIES_SCHEMA, (), source.topology)


def test_invalid_stage_parameter_and_raw_source_identity_fail() -> None:
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
        "external.native.pose_series/v1",
        (),
        source.topology,
        verification=IdentityVerification.UNVERIFIABLE,
    )

    assert external.verification is IdentityVerification.UNVERIFIABLE
    assert external.requires_fresh_compute_token
    assert not external.reusable_from_content_cache
    assert external.source_coverage is SourceCoordinateCoverage.PARTIAL_OUTPUTS


@pytest.mark.parametrize(
    "output",
    (
        coordinate("output", (7,), 0),
        coordinate("output", (0,), 1),
    ),
)
def test_stage_rejects_nonexistent_output_coordinates(output: TreeCoordinate) -> None:
    source = SourceTreeIdentity.build(text_tree(TreeRootId.build("source"), (((0,), ("a",)),)), TEXT_CODEC, TEXT_TREE_SEMANTICS)
    source_map = SourceCoordinateMap.build((SourceCoordinateEntry.build(output, (coordinate("source", (0,), 0),)),))

    with pytest.raises(UnknownStageOutputCoordinateError):
        StageTreeIdentity.build(
            source,
            "external.native.pose_series/v1",
            (),
            source.topology,
            source_map,
            verification=IdentityVerification.UNVERIFIABLE,
        )


@pytest.mark.parametrize(
    "mapped_source",
    (
        coordinate("source", (7,), 0),
        coordinate("source", (0,), 1),
    ),
)
def test_stage_rejects_nonexistent_source_coordinates(mapped_source: TreeCoordinate) -> None:
    source = SourceTreeIdentity.build(text_tree(TreeRootId.build("source"), (((0,), ("a",)),)), TEXT_CODEC, TEXT_TREE_SEMANTICS)
    source_map = SourceCoordinateMap.build((SourceCoordinateEntry.build(coordinate("output", (0,), 0), (mapped_source,)),))

    with pytest.raises(UnknownStageSourceCoordinateError):
        StageTreeIdentity.build(
            source,
            "external.native.pose_series/v1",
            (),
            source.topology,
            source_map,
            verification=IdentityVerification.UNVERIFIABLE,
        )


def test_verified_stage_accepts_explicit_empty_source_branch_reduction(audited_stage_schema: str) -> None:
    source = SourceTreeIdentity.build(
        text_tree(TreeRootId.build("source"), (((0,), ("a",)), ((1,), ()))),
        TEXT_CODEC,
        TEXT_TREE_SEMANTICS,
    )
    output_topology = text_tree(
        TreeRootId.build("output"),
        (((0,), ("reduced",)), ((1,), (None,))),
    ).topology
    source_map = SourceCoordinateMap.build(
        (
            SourceCoordinateEntry.build(
                coordinate("output", (0,), 0),
                (coordinate("source", (0,), 0),),
            ),
            SourceCoordinateEntry.from_empty_branch(
                coordinate("output", (1,), 0),
                BranchCoordinate.build(TreeRootId.build("source"), GhPath.build(1)),
            ),
        )
    )

    built = StageTreeIdentity.build(source, audited_stage_schema, (), output_topology, source_map)

    assert built.verification is IdentityVerification.VERIFIED
    assert built.source_coverage is SourceCoordinateCoverage.COMPLETE_OUTPUTS


@pytest.mark.parametrize(
    ("source_branch", "expected_error"),
    (
        (
            BranchCoordinate.build(TreeRootId.build("source"), GhPath.build(9)),
            UnknownStageSourceBranchError,
        ),
        (
            BranchCoordinate.build(TreeRootId.build("source"), GhPath.build(0)),
            NonEmptyStageSourceBranchError,
        ),
    ),
)
def test_stage_rejects_invalid_explicit_empty_source_branch(
    source_branch: BranchCoordinate,
    expected_error: type[InvalidStageTreeIdentityError],
) -> None:
    source = SourceTreeIdentity.build(
        text_tree(TreeRootId.build("source"), (((0,), ("a",)), ((1,), ()))),
        TEXT_CODEC,
        TEXT_TREE_SEMANTICS,
    )
    source_map = SourceCoordinateMap.build(
        (
            SourceCoordinateEntry.from_empty_branch(
                coordinate("output", (0,), 0),
                source_branch,
            ),
        )
    )

    with pytest.raises(expected_error):
        StageTreeIdentity.build(
            source,
            "external.native.pose_series/v1",
            (),
            text_tree(TreeRootId.build("output"), (((0,), ("reduced",)),)).topology,
            source_map,
            verification=IdentityVerification.UNVERIFIABLE,
        )


def test_verified_stage_requires_complete_output_attribution(audited_stage_schema: str) -> None:
    source = SourceTreeIdentity.build(
        text_tree(TreeRootId.build("source"), (((0,), ("a", "b")), ((1,), ("c",)))),
        TEXT_CODEC,
        TEXT_TREE_SEMANTICS,
    )
    incomplete_map = SourceCoordinateMap.build(
        (
            SourceCoordinateEntry.build(coordinate("output", (0,), 0), (coordinate("source", (0,), 0),)),
            SourceCoordinateEntry.build(coordinate("output", (0,), 1), (coordinate("source", (0,), 1),)),
        )
    )
    complete_map = SourceCoordinateMap.build(incomplete_map.entries + (SourceCoordinateEntry.build(coordinate("output", (1,), 0), (coordinate("source", (1,), 0),)),))

    with pytest.raises(IncompleteStageSourceCoordinatesError):
        StageTreeIdentity.build(source, audited_stage_schema, (), source.topology, incomplete_map)

    built = StageTreeIdentity.build(source, audited_stage_schema, (), source.topology, complete_map)

    assert built.verification is IdentityVerification.VERIFIED
    assert built.source_coverage is SourceCoordinateCoverage.COMPLETE_OUTPUTS


def test_unverifiable_stage_records_partial_output_attribution() -> None:
    source = SourceTreeIdentity.build(
        text_tree(TreeRootId.build("source"), (((0,), ("a", "b")),)),
        TEXT_CODEC,
        TEXT_TREE_SEMANTICS,
    )
    partial_map = SourceCoordinateMap.build((SourceCoordinateEntry.build(coordinate("output", (0,), 0), (coordinate("source", (0,), 0),)),))

    built = StageTreeIdentity.build(
        source,
        "external.native.pose_series/v1",
        (),
        source.topology,
        partial_map,
        verification=IdentityVerification.UNVERIFIABLE,
    )

    assert built.source_coverage is SourceCoordinateCoverage.PARTIAL_OUTPUTS


def test_stage_tree_identity_rejects_raw_construction(audited_stage_schema: str) -> None:
    source = SourceTreeIdentity.build(text_tree(TreeRootId.build("source"), (((0,), ("a",)),)), TEXT_CODEC, TEXT_TREE_SEMANTICS)
    source_map = SourceCoordinateMap.build((SourceCoordinateEntry.build(coordinate("output", (0,), 0), (coordinate("source", (0,), 0),)),))
    built = StageTreeIdentity.build(source, audited_stage_schema, (), source.topology, source_map)

    with pytest.raises(InvalidStageTreeIdentityError):
        StageTreeIdentity(
            built.digest,
            built.branch_digests,
            built.prior_digest,
            built.builder_schema,
            built.parameters,
            built.topology,
            built.source_coordinates,
            built.source_coverage,
            built.verification,
            built._canonical,
            built._branch_canonical,
        )


def test_stage_branch_digest_isolates_unchanged_sibling_content(audited_stage_schema: str) -> None:
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

    left = StageTreeIdentity.build(left_source, audited_stage_schema, (), left_source.topology, source_map)
    right = StageTreeIdentity.build(right_source, audited_stage_schema, (), right_source.topology, source_map)

    assert left.branch(GhPath.build(0)) != right.branch(GhPath.build(0))
    assert left.branch(GhPath.build(1)) == right.branch(GhPath.build(1))
    assert left.digest != right.digest


def test_stage_branch_digest_hashes_branch_local_attribution_coverage(audited_stage_schema: str) -> None:
    source = SourceTreeIdentity.build(
        text_tree(TreeRootId.build("source"), (((0,), ("stable",)), ((1,), ("sibling",)))),
        TEXT_CODEC,
        TEXT_TREE_SEMANTICS,
    )
    stable_entry = SourceCoordinateEntry.build(
        coordinate("output", (0,), 0),
        (coordinate("source", (0,), 0),),
    )
    sibling_entry = SourceCoordinateEntry.build(
        coordinate("output", (1,), 0),
        (coordinate("source", (1,), 0),),
    )
    partial = StageTreeIdentity.build(
        source,
        audited_stage_schema,
        (),
        source.topology,
        SourceCoordinateMap.build((stable_entry,)),
        verification=IdentityVerification.UNVERIFIABLE,
    )
    complete = StageTreeIdentity.build(
        source,
        audited_stage_schema,
        (),
        source.topology,
        SourceCoordinateMap.build((stable_entry, sibling_entry)),
        verification=IdentityVerification.UNVERIFIABLE,
    )

    assert partial.source_coverage is SourceCoordinateCoverage.PARTIAL_OUTPUTS
    assert complete.source_coverage is SourceCoordinateCoverage.COMPLETE_OUTPUTS
    assert partial.branch(GhPath.build(0)) == complete.branch(GhPath.build(0))
    assert partial.digest != complete.digest
