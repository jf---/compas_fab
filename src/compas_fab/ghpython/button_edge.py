"""Fire-once button-edge detection for Grasshopper mutation components.

A Grasshopper solve re-runs on every recompute, so a component that commands a
robot from a boolean input would re-issue that command on each recompute while
the button stays True. :func:`rising_edge` collapses a held or recomputed True to
a single ``False -> True`` transition: the mutation fires once per press, never on
the steady-True recomputes that follow. The previous state lives in the
runtime-only ``sticky`` cache, keyed to the owning component and a per-input slot,
so two buttons on one component never share an edge.

This is the ONLY re-fire guard the ABB mutation components rely on; there is no
arming, ledger, or nonce ceremony around it.
"""

from __future__ import annotations

from typing import MutableMapping

from compas_ghpython import create_id


def rising_edge(component, value, sticky: MutableMapping[str, object], slot: str) -> bool:
    """Return True only on a ``False -> True`` transition of ``value``.

    Args:
        component: The owning Grasshopper component (pass ``ghenv.Component``);
            its identity keys the stored state via ``create_id``.
        value: The current boolean input, e.g. a Button or Boolean Toggle.
        sticky: The runtime-only cache (``scriptcontext.sticky``).
        slot: A per-input name disambiguating multiple buttons on one component.

    Returns:
        True on the solve where ``value`` first becomes True; False while it stays
        True (a held button or a recompute), while it is False, and on the falling
        edge. A steady-True recompute therefore never re-fires the mutation.
    """
    key = create_id(component, slot)
    last = bool(sticky.get(key, False))
    current = bool(value)
    sticky[key] = current
    return current and not last
