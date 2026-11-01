"""Scenario builder for B1 (short layout).

Layout (m): store at the centre of the corridor, the four stages on an arc 1.3-1.5 m from the store, the shipping
area / NG drop ("dock") 0.7 m beside the store. Straight-line paths, no obstacles. CT, batch sizes, buffer capacities
and standard stock are fixed here.
"""
from __future__ import annotations

from .engine import Scenario, Transport

POSITIONS = {
    "store": (0.0, 0.0),
    "M1": (-1.2, 0.8),
    "M2": (-0.45, 1.2),
    "M3": (0.45, 1.2),
    "M4": (1.2, 0.8),
    "dock": (0.7, 0.0),
}
IDS = ("M1", "M2", "M3", "M4")
CT = (64.0, 72.0, 86.0, 76.0)
BS1 = (1, 3, 2, 4)
CAP1 = (3, 9, 6, 12)
INIT1 = (2, 6, 4, 8)
SPEED = 0.6
OVERHEAD = 2.0  # accel/decel + confirmation per leg [s]

# handling (load / unload) time per stop [s]; setting "ii" is the one used
HANDLING = {("B1", "ii"): 2.0}


def transport(handling: float) -> Transport:
    return Transport(positions=POSITIONS, speed=SPEED, overhead=OVERHEAD, load=handling, unload=handling)


def make_scenario(case: str = "B1", setting: str = "ii", handling: float | None = None, ct_scale: float = 1.0) -> Scenario:
    """The B1 scenario. ct_scale scales all cycle times (parts per cycle, capacities, stock unchanged)."""
    if case != "B1":
        raise ValueError("only B1 is included in this repository")
    h = HANDLING[(case, setting)] if handling is None else handling
    bs = tuple((b,) for b in BS1)
    cap = tuple((c,) for c in CAP1)
    init = tuple((x,) for x in INIT1)
    return Scenario(name=case, ids=IDS, ct=tuple(c * ct_scale for c in CT), bs=bs, cap=cap, init_std=init,
                    transport=transport(h), out_cap=2, defect_bins=True)
