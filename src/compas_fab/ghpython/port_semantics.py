"""Orthogonal Grasshopper port declarations."""

from __future__ import annotations

from enum import Enum

from attrs import define

import compas_fab.ghpython.item_values as item_values
from compas_fab.ghpython.tree_errors import InvalidPortSemanticsError


class TopologyRole(Enum):
    ATOMIC = "atomic"
    TREE = "tree"


class BranchSemantics(Enum):
    ELEMENTWISE = "elementwise"
    ORDERED_SEQUENCE = "ordered_sequence"
    BATCH = "batch"


class ItemValidationPolicy(Enum):
    VALIDATE_INDEPENDENT = "validate_independent"


class SequenceReductionPolicy(Enum):
    REQUIRE_ALL_VALID = "require_all_valid"


class BatchPublicationPolicy(Enum):
    PUBLISH_INDEPENDENT = "publish_independent"
    FAIL_BATCH = "fail_batch"


@define(frozen=True, slots=True)
class PortSemantics:
    """Three independent axes describing a Grasshopper port contract."""

    topology_role: TopologyRole
    branch_semantics: BranchSemantics
    item_shape: item_values.ItemShape

    @classmethod
    def build(
        cls,
        topology_role: TopologyRole,
        branch_semantics: BranchSemantics,
        item_shape: item_values.ItemShape,
    ) -> "PortSemantics":
        return cls(topology_role, branch_semantics, item_shape)

    def __attrs_post_init__(self) -> None:
        if type(self.topology_role) is not TopologyRole or type(self.branch_semantics) is not BranchSemantics or type(self.item_shape) is not item_values.ItemShape:
            raise InvalidPortSemanticsError("Port semantics require exact topology, branch, and item-shape values.")
