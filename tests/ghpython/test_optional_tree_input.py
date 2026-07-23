from attrs import evolve
import pytest

from compas_fab.ghpython.optional_tree_input import InvalidOptionalTreeInputError
from compas_fab.ghpython.optional_tree_input import OptionalTreeInput
from compas_fab.ghpython.tree_coordinates import GhPath
from compas_fab.ghpython.tree_coordinates import TreeRootId
from compas_fab.ghpython.tree_values import Tree
from compas_fab.ghpython.tree_values import TreeBranch
from compas_fab.ghpython.tree_values import TreeItem


def _tree() -> Tree[str]:
    return Tree.build(
        TreeRootId.build("input"),
        (
            TreeBranch.build(GhPath.build(0), ()),
            TreeBranch.build(GhPath.build(2), (TreeItem.null(), TreeItem.value("joint_1"))),
        ),
    )


def test_absent_has_no_topology_and_present_retains_exact_tree() -> None:
    tree = _tree()
    absent = OptionalTreeInput[str].absent()
    present = OptionalTreeInput.present(tree)

    assert absent.is_absent
    assert absent.tree is None
    assert present.is_present
    assert present.tree is tree
    assert present.require_present() is tree


def test_raw_constructor_cannot_forge_presence() -> None:
    present = OptionalTreeInput.present(_tree())

    with pytest.raises(InvalidOptionalTreeInputError):
        evolve(present, tree=None)
