from typing import Tuple

import pytest
from hypothesis import given
from hypothesis import strategies as st

from compas_fab.ghpython.port_semantics import BatchPublicationPolicy
from compas_fab.ghpython.port_semantics import BranchSemantics
from compas_fab.ghpython.port_semantics import ItemValidationPolicy
from compas_fab.ghpython.port_semantics import PortSemantics
from compas_fab.ghpython.port_semantics import SequenceReductionPolicy
from compas_fab.ghpython.port_semantics import TopologyRole
from compas_fab.ghpython.item_values import FixedVector
from compas_fab.ghpython.item_values import ItemShape
from compas_fab.ghpython.item_values import ItemShapeKind
from compas_fab.ghpython.item_values import Scalar
from compas_fab.ghpython.item_values import ShapeTag
from compas_fab.ghpython.tree_coordinates import BranchCoordinate
from compas_fab.ghpython.tree_coordinates import GhPath
from compas_fab.ghpython.tree_coordinates import ItemIndex
from compas_fab.ghpython.tree_coordinates import TreeCoordinate
from compas_fab.ghpython.tree_coordinates import TreeRootId
from compas_fab.ghpython.tree_errors import DuplicateTreePathError
from compas_fab.ghpython.tree_errors import FixedVectorLengthError
from compas_fab.ghpython.tree_errors import InvalidFixedVectorContainerError
from compas_fab.ghpython.tree_errors import InvalidFixedVectorShapeError
from compas_fab.ghpython.tree_errors import InvalidGhPathError
from compas_fab.ghpython.tree_errors import InvalidAtomicTreeItemTypeError
from compas_fab.ghpython.tree_errors import InvalidItemIndexError
from compas_fab.ghpython.tree_errors import InvalidItemShapeError
from compas_fab.ghpython.tree_errors import InvalidPortSemanticsError
from compas_fab.ghpython.tree_errors import InvalidShapeTagError
from compas_fab.ghpython.tree_errors import InvalidTreeBranchError
from compas_fab.ghpython.tree_errors import InvalidTreeCoordinateError
from compas_fab.ghpython.tree_errors import InvalidTreeDecoderInputError
from compas_fab.ghpython.tree_errors import InvalidTreeItemError
from compas_fab.ghpython.tree_errors import InvalidTreeRootIdError
from compas_fab.ghpython.tree_errors import InvalidTreeTopologyError
from compas_fab.ghpython.tree_errors import NonCanonicalTreeOrderError
from compas_fab.ghpython.tree_errors import NullTreeAtomicError
from compas_fab.ghpython.tree_errors import NullTreeScalarError
from compas_fab.ghpython.tree_errors import TreeAtomicBranchCardinalityError
from compas_fab.ghpython.tree_errors import TreeAtomicItemCardinalityError
from compas_fab.ghpython.tree_errors import TreeScalarBranchCardinalityError
from compas_fab.ghpython.tree_errors import TreeScalarItemCardinalityError
from compas_fab.ghpython.tree_decoding import AtomicFromTree
from compas_fab.ghpython.tree_decoding import ScalarFromTree
from compas_fab.ghpython.tree_values import Tree
from compas_fab.ghpython.tree_values import TreeBranch
from compas_fab.ghpython.tree_values import TreeItem
from compas_fab.ghpython.tree_values import TreeTopology


def _tree(*branches: TreeBranch[object]) -> Tree[object]:
    return Tree.build(TreeRootId.build("routing-a"), branches)


@given(st.lists(st.lists(st.integers(min_value=0), min_size=1).map(tuple), min_size=1, unique=True))
def test_tree_accepts_only_canonical_host_path_order(raw_paths: list[Tuple[int, ...]]) -> None:
    paths = [GhPath.build(*parts) for parts in raw_paths]
    canonical = sorted(paths, key=lambda path: path.indices)
    branches: Tuple[TreeBranch[object], ...] = tuple(TreeBranch.build(path, ()) for path in canonical)
    assert Tree.build(TreeRootId.build("routing-a"), branches).branches == branches
    if paths != canonical:
        with pytest.raises(NonCanonicalTreeOrderError):
            Tree.build(TreeRootId.build("routing-a"), tuple(TreeBranch.build(path, ()) for path in paths))


def test_tree_preserves_empty_branch_null_slot_and_fixed_vector_atomicity() -> None:
    shape = ItemShape.fixed_vector(ShapeTag.build("group/joints/6"), 6)
    vector = FixedVector.build(shape, (1.0, 2.0, 3.0, 4.0, 5.0, 6.0))
    tree = Tree.build(
        TreeRootId.build("routing-a"),
        (
            TreeBranch.build(GhPath.build(0, 2), (TreeItem.value(vector), TreeItem.null())),
            TreeBranch.build(GhPath.build(3), ()),
        ),
    )
    assert tree.branches[0].items[0].item is vector
    assert tree.branches[0].items[1].is_null
    assert tree.branches[1].items == ()
    assert tree.topology.paths == (GhPath.build(0, 2), GhPath.build(3))
    assert tree.topology.item_counts == (2, 0)
    assert tree.topology.null_bitmaps == ((False, True), ())


def test_tree_rejects_duplicate_paths_before_order_check() -> None:
    path = GhPath.build(2)
    with pytest.raises(DuplicateTreePathError):
        _tree(TreeBranch.build(path, ()), TreeBranch.build(path, ()))


def test_tree_rejects_unsorted_prefix_paths() -> None:
    with pytest.raises(NonCanonicalTreeOrderError):
        _tree(TreeBranch.build(GhPath.build(1, 0), ()), TreeBranch.build(GhPath.build(1), ()))


@pytest.mark.parametrize("segments", ((), (-1,), (True,), (0, False)))
def test_path_rejects_empty_negative_and_boolean_segments(segments: Tuple[object, ...]) -> None:
    with pytest.raises(InvalidGhPathError):
        GhPath.build(*segments)  # type: ignore[arg-type]


def test_path_canonical_key_is_exact_indices_tuple() -> None:
    path = GhPath.build(0, 4, 9)
    assert path.canonical_key() is path.indices
    assert path.canonical_key() == (0, 4, 9)


def test_coordinates_retain_exact_root_path_and_slot() -> None:
    root = TreeRootId.build("routing-a")
    path = GhPath.build(4, 9)
    branch = BranchCoordinate.build(root, path)
    coordinate = TreeCoordinate.build(branch, ItemIndex.build(3))
    assert coordinate.branch.root_id is root
    assert coordinate.branch.path is path
    assert coordinate.item_index.value == 3


@pytest.mark.parametrize("invalid", ("", 1, None))
def test_tree_root_rejects_empty_and_non_text(invalid: object) -> None:
    with pytest.raises(InvalidTreeRootIdError):
        TreeRootId.build(invalid)  # type: ignore[arg-type]


def test_tree_root_retains_non_empty_routing_text_exactly() -> None:
    root = TreeRootId.build(" routing-a ")
    assert root.value == " routing-a "


@pytest.mark.parametrize("invalid", (-1, True, 1.0))
def test_item_index_requires_exact_non_negative_integer(invalid: object) -> None:
    with pytest.raises(InvalidItemIndexError):
        ItemIndex.build(invalid)  # type: ignore[arg-type]


@pytest.mark.parametrize("invalid", ("", " ", " vector", "vector ", 2, None))
def test_shape_tag_rejects_empty_invalid_or_surrounded_text(invalid: object) -> None:
    with pytest.raises(InvalidShapeTagError):
        ShapeTag.build(invalid)  # type: ignore[arg-type]


def test_item_shapes_keep_orthogonal_shape_metadata() -> None:
    tag = ShapeTag.build("profile/native")
    scalar = ItemShape.scalar()
    atomic = ItemShape.domain_atomic(tag)
    vector = ItemShape.fixed_vector(tag, 4)
    assert (scalar.kind, scalar.tag, scalar.length) == (ItemShapeKind.SCALAR, None, None)
    assert (atomic.kind, atomic.tag, atomic.length) == (ItemShapeKind.DOMAIN_ATOMIC, tag, None)
    assert (vector.kind, vector.tag, vector.length) == (ItemShapeKind.FIXED_VECTOR, tag, 4)


@pytest.mark.parametrize("invalid_length", (0, -1, True, 2.0))
def test_fixed_vector_shape_requires_positive_exact_integer_length(invalid_length: object) -> None:
    with pytest.raises(InvalidItemShapeError):
        ItemShape.fixed_vector(ShapeTag.build("group/joints"), invalid_length)  # type: ignore[arg-type]


def test_fixed_vector_rejects_wrong_length_and_non_vector_shape() -> None:
    shape = ItemShape.fixed_vector(ShapeTag.build("group/joints/2"), 2)
    with pytest.raises(FixedVectorLengthError):
        FixedVector.build(shape, (1.0,))
    with pytest.raises(InvalidFixedVectorShapeError):
        FixedVector.build(ItemShape.scalar(), (1.0,))


def test_fixed_vector_rejects_non_tuple_container_separately_from_length() -> None:
    shape = ItemShape.fixed_vector(ShapeTag.build("group/joints/2"), 2)
    with pytest.raises(InvalidFixedVectorContainerError):
        FixedVector.build(shape, [1.0, 2.0])  # type: ignore[arg-type]


def test_scalar_from_tree_decodes_one_item_without_authorizing_broadcast() -> None:
    source = _tree(TreeBranch.build(GhPath.build(7), (TreeItem.value("speed"),)))
    scalar = ScalarFromTree.build(source)
    assert isinstance(scalar, Scalar)
    assert scalar.value == "speed"
    assert not isinstance(source, Scalar)


def test_scalar_from_tree_rejects_zero_or_multiple_branches() -> None:
    with pytest.raises(TreeScalarBranchCardinalityError):
        ScalarFromTree.build(_tree())
    with pytest.raises(TreeScalarBranchCardinalityError):
        ScalarFromTree.build(_tree(TreeBranch.build(GhPath.build(0), ()), TreeBranch.build(GhPath.build(1), ())))


def test_scalar_from_tree_rejects_non_tree_input_with_decoder_error() -> None:
    with pytest.raises(InvalidTreeDecoderInputError):
        ScalarFromTree.build(object())  # type: ignore[arg-type]


def test_scalar_from_tree_rejects_zero_or_multiple_items_and_null() -> None:
    with pytest.raises(TreeScalarItemCardinalityError):
        ScalarFromTree.build(_tree(TreeBranch.build(GhPath.build(0), ())))
    with pytest.raises(TreeScalarItemCardinalityError):
        ScalarFromTree.build(_tree(TreeBranch.build(GhPath.build(0), (TreeItem.value("a"), TreeItem.value("b")))))
    with pytest.raises(NullTreeScalarError):
        ScalarFromTree.build(_tree(TreeBranch.build(GhPath.build(0), (TreeItem.null(),))))


def test_atomic_from_tree_retains_exact_domain_object_without_scalar_label() -> None:
    domain_object = {"native": "profile"}
    source = _tree(TreeBranch.build(GhPath.build(2), (TreeItem.value(domain_object),)))
    atomic = AtomicFromTree.build(source, dict)
    assert isinstance(atomic, AtomicFromTree)
    assert not isinstance(atomic, Scalar)
    assert atomic.value is domain_object
    with pytest.raises(InvalidAtomicTreeItemTypeError):
        AtomicFromTree.build(source, tuple)


def test_atomic_from_tree_has_atomic_specific_cardinality_and_null_failures() -> None:
    with pytest.raises(TreeAtomicBranchCardinalityError):
        AtomicFromTree.build(_tree(), object)
    with pytest.raises(TreeAtomicItemCardinalityError):
        AtomicFromTree.build(_tree(TreeBranch.build(GhPath.build(0), ())), object)
    with pytest.raises(NullTreeAtomicError):
        AtomicFromTree.build(_tree(TreeBranch.build(GhPath.build(0), (TreeItem.null(),))), object)


def test_atomic_from_tree_rejects_non_tree_input_with_decoder_error() -> None:
    with pytest.raises(InvalidTreeDecoderInputError):
        AtomicFromTree.build(object(), object)  # type: ignore[arg-type]


def test_atomic_raw_constructor_cannot_bypass_expected_type() -> None:
    with pytest.raises(InvalidAtomicTreeItemTypeError):
        AtomicFromTree("native", tuple)


def test_atomic_expected_type_narrows_host_object_tree() -> None:
    class NativeDomain:
        def marker(self) -> str:
            return "native"

    domain_object = NativeDomain()
    host_tree: Tree[object] = _tree(TreeBranch.build(GhPath.build(0), (TreeItem.value(domain_object),)))
    atomic: AtomicFromTree[NativeDomain] = AtomicFromTree.build(host_tree, NativeDomain)
    assert atomic.value.marker() == "native"


def test_port_semantics_declares_three_independent_axes() -> None:
    shape = ItemShape.fixed_vector(ShapeTag.build("group/joints/6"), 6)
    semantics = PortSemantics.build(TopologyRole.TREE, BranchSemantics.ORDERED_SEQUENCE, shape)
    assert semantics.topology_role is TopologyRole.TREE
    assert semantics.branch_semantics is BranchSemantics.ORDERED_SEQUENCE
    assert semantics.item_shape is shape
    assert tuple(BatchPublicationPolicy.__members__) == ("PUBLISH_INDEPENDENT", "FAIL_BATCH")
    assert BatchPublicationPolicy.PUBLISH_INDEPENDENT.value == "publish_independent"
    assert BatchPublicationPolicy.FAIL_BATCH.value == "fail_batch"
    assert ItemValidationPolicy.VALIDATE_INDEPENDENT.value == "validate_independent"
    assert SequenceReductionPolicy.REQUIRE_ALL_VALID.value == "require_all_valid"


def test_tree_topology_raw_and_factory_construction_enforce_canonical_paths() -> None:
    prefix = GhPath.build(1)
    extension = GhPath.build(1, 0)
    with pytest.raises(NonCanonicalTreeOrderError):
        TreeTopology((extension, prefix), (0, 0), ((), ()))
    with pytest.raises(DuplicateTreePathError):
        TreeTopology((prefix, prefix), (0, 0), ((), ()))
    with pytest.raises(InvalidTreeTopologyError):
        TreeTopology.build(("raw",))  # type: ignore[arg-type]


@pytest.mark.parametrize(
    ("paths", "counts", "bitmaps"),
    (
        ([], (), ()),
        ((), [], ()),
        ((), (), []),
        ((GhPath.build(0),), (), ()),
        ((GhPath.build(0),), (True,), ((),)),
        ((GhPath.build(0),), (1,), ((False, False),)),
    ),
)
def test_tree_topology_malformed_fields_raise_topology_error(paths: object, counts: object, bitmaps: object) -> None:
    with pytest.raises(InvalidTreeTopologyError):
        TreeTopology(paths, counts, bitmaps)  # type: ignore[arg-type]


def test_raw_constructors_cannot_bypass_invariants() -> None:
    with pytest.raises(InvalidTreeRootIdError):
        TreeRootId("")
    with pytest.raises(InvalidItemIndexError):
        ItemIndex(True)
    with pytest.raises(InvalidGhPathError):
        GhPath((-1,))
    with pytest.raises(InvalidTreeCoordinateError):
        BranchCoordinate("root", GhPath.build(0))  # type: ignore[arg-type]
    with pytest.raises(InvalidTreeCoordinateError):
        TreeCoordinate(BranchCoordinate.build(TreeRootId.build("root"), GhPath.build(0)), 0)  # type: ignore[arg-type]
    with pytest.raises(InvalidTreeItemError):
        TreeItem(item="value", is_null=True)
    with pytest.raises(InvalidTreeBranchError):
        TreeBranch(path=GhPath.build(0), items=("raw",))  # type: ignore[arg-type]
    with pytest.raises(DuplicateTreePathError):
        path = GhPath.build(0)
        Tree(TreeRootId.build("root"), (TreeBranch.build(path, ()), TreeBranch.build(path, ())))
    with pytest.raises(InvalidShapeTagError):
        ShapeTag(" tag")
    with pytest.raises(InvalidItemShapeError):
        ItemShape(ItemShapeKind.SCALAR, ShapeTag.build("forbidden"), None)
    with pytest.raises(InvalidFixedVectorShapeError):
        FixedVector(ItemShape.scalar(), (1.0,))
    with pytest.raises(InvalidPortSemanticsError):
        PortSemantics("tree", BranchSemantics.ELEMENTWISE, ItemShape.scalar())  # type: ignore[arg-type]
