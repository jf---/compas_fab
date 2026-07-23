"""Sticky-cached controller-owner lifecycle, keyed per Grasshopper component.

A controller connection must survive Grasshopper's re-solves rather than being
rebuilt on every recompute (a reconnect storm), yet must be rebuilt when the
target controller changes. This module holds that contract in one place:
``acquire_owner`` reuses a cached owner for the same controller, and replaces
(closing the old, so no session leaks) when the controller id changes.

The cache is a plain ``MutableMapping`` so tests pass a ``dict`` and Grasshopper
passes ``scriptcontext.sticky``; neither Grasshopper nor scriptcontext is
imported here. Sticky is runtime-only, so nothing here is ever serialized.
"""

from __future__ import annotations

from typing import Callable
from typing import MutableMapping

from .arming import ControllerId
from .controller_owner import ControllerOwner
from .session import ControllerSession


def acquire_owner(
    sticky: MutableMapping[str, object],
    slot_key: str,
    controller_id: ControllerId,
    session_factory: Callable[[], ControllerSession],
) -> ControllerOwner:
    """Return the owner for ``slot_key``, reconnecting only when the id changes.

    Reconnect-only-on-id-change, keyed per component slot: if the slot already
    holds an owner for ``controller_id``, it is returned unchanged (connections
    persist across solves); if it holds an owner for a *different* controller,
    that owner is closed (logging out its session) and replaced; if the slot is
    empty, a new owner is created and stored.

    Args:
        sticky: The runtime-only cache (a ``dict`` in tests,
            ``scriptcontext.sticky`` in Grasshopper).
        slot_key: Identifies the owning component's slot, e.g.
            ``create_id(component, "abb_controller")`` in Grasshopper.
        controller_id: Identity of the controller to own.
        session_factory: Zero-argument builder of a fresh
            :class:`ControllerSession`, handed to a newly created owner.

    Returns:
        The cached-or-created owner for ``slot_key`` and ``controller_id``.
    """
    existing = sticky.get(slot_key)
    if isinstance(existing, ControllerOwner):
        if existing.controller_id == controller_id:
            return existing
        # Different controller in this slot: closing releases the old session so
        # replacing an owner never leaks a connection.
        existing.close()
    owner = ControllerOwner(controller_id, session_factory)
    sticky[slot_key] = owner
    return owner
