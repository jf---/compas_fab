from __future__ import annotations

from typing import Tuple

import pytest

from compas_fab.ghpython.item_values import FixedVector
from compas_fab.ghpython.item_values import ItemShape
from compas_fab.ghpython.item_values import ShapeTag
from compas_fab.ghpython.tree_coordinates import GhPath
from compas_fab.ghpython.tree_coordinates import TreeRootId
from compas_fab.ghpython.tree_matching import MatchInput
from compas_fab.ghpython.tree_matching import MatchPolicy
from compas_fab.ghpython.tree_matching import MatchRole
from compas_fab.ghpython.tree_matching import match_inputs
from compas_fab.ghpython.tree_values import Tree
from compas_fab.ghpython.tree_values import TreeBranch
from compas_fab.ghpython.tree_values import TreeItem
from tests.ghpython.tree_host_emulation import EmulatedAccess
from tests.ghpython.tree_host_emulation import EmulatedInvocationCountError
from tests.ghpython.tree_host_emulation import EmulatedParameter
from tests.ghpython.tree_host_emulation import InvalidEmulatedParameterError
from tests.ghpython.tree_host_emulation import PendingRhinoSolveModel


def _pending_rhino_tree() -> Tree[object]:
    return Tree.build(
        TreeRootId.build("pending-rhino-route"),
        (
            TreeBranch.build(GhPath.build(0), (TreeItem.value("a"), TreeItem.null())),
            TreeBranch.build(GhPath.build(3, 1), ()),
        ),
    )


def test_pending_rhino_tree_access_emulates_one_full_tree_invocation() -> None:
    """Pending Rhino tree access emulates one invocation containing the full tree."""
    tree = _pending_rhino_tree()
    parameter = EmulatedParameter.tree("series", tree)
    invocations = PendingRhinoSolveModel.build((parameter,)).invocations()

    assert parameter.access is EmulatedAccess.TREE, "emulated access remains tree"
    assert len(invocations) == 1, "pending Rhino tree access has one emulated invocation"
    assert invocations[0].value("series") is tree, "emulated invocation retains the full tree"


def test_pending_rhino_item_and_list_access_emulate_multiple_invocations() -> None:
    """Pending Rhino item/list hypotheses emulate pre-scheduled multiple invocations."""
    items = EmulatedParameter.item("item", ("a", "b"))
    lists = EmulatedParameter.list("list", ((1, 2), (3, 4)))
    invocations = PendingRhinoSolveModel.build((items, lists)).invocations()

    assert len(invocations) == 2, "pending Rhino item/list access may have multiple emulated invocations"
    assert invocations[0].value("item") == "a", "first emulated item transport is exact"
    assert invocations[1].value("list") == (3, 4), "second emulated list transport remains a list tuple"


def test_pending_rhino_fixed_vector_is_one_atomic_emulated_item() -> None:
    """Pending Rhino item access treats a fixed vector as one emulated domain item."""
    vector = FixedVector.build(
        ItemShape.fixed_vector(ShapeTag.build("group/joints/3"), 3),
        (1.0, 2.0, 3.0),
    )
    invocations = PendingRhinoSolveModel.build((EmulatedParameter.item("joints", (vector,)),)).invocations()

    assert len(invocations) == 1, "pending Rhino fixed vector has one emulated invocation"
    assert invocations[0].value("joints") is vector, "emulated fixed vector remains one atomic domain item"


def test_pending_rhino_model_has_no_flattening_or_matching_helper() -> None:
    """Pending Rhino transport exposes neither emulated flattening nor domain matching."""
    model = PendingRhinoSolveModel.build((EmulatedParameter.tree("series", _pending_rhino_tree()),))

    assert not hasattr(model, "flatten"), "pending Rhino model has no emulated flatten helper"
    assert not hasattr(model, "match"), "pending Rhino model has no emulated domain matcher"


def test_pending_rhino_domain_matching_uses_production_match_inputs_only() -> None:
    """Pending Rhino transport receives the production-matched tree as an atomic value."""
    tree = _pending_rhino_tree()
    policy = MatchPolicy.build(("series",), (MatchRole.EXACT_TREE,))
    matched = match_inputs((MatchInput.tree("series", tree),), policy)
    invocation = PendingRhinoSolveModel.build((EmulatedParameter.tree("matched", matched),)).invocations()[0]

    assert invocation.value("matched") is matched, "emulated transport retains production matching output exactly"


def test_pending_rhino_transport_rejects_implicit_invocation_matching() -> None:
    """Pending Rhino transport rejects unequal schedules instead of emulating matching."""
    parameters: Tuple[EmulatedParameter, ...] = (
        EmulatedParameter.item("item", (1, 2)),
        EmulatedParameter.list("list", ((3, 4),)),
    )

    with pytest.raises(EmulatedInvocationCountError, match="without matching"):
        PendingRhinoSolveModel.build(parameters)


def test_pending_rhino_model_rejects_nested_emulated_parameter_mutation() -> None:
    """Pending Rhino solve construction revalidates nested emulated parameters."""
    parameter = EmulatedParameter.item("item", (1,))
    object.__setattr__(parameter, "name", "")

    with pytest.raises(InvalidEmulatedParameterError, match="exact emulated parameters"):
        PendingRhinoSolveModel.build((parameter,))
