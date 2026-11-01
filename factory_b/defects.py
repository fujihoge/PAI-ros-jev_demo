"""Common defect series. Pure function of (seed, machine id, cycle number): no shared random state.

Whether the n-th processing cycle of a machine yields a defective workpiece does not depend on the policy, on
timing, or on the order in which other things happened -- only on `(machine_id, cycle_no, seed)`.
"""
from __future__ import annotations

import hashlib


def defect_uniform(seed: int, machine_id: str, cycle_no: int) -> float:
    """Uniform number in [0, 1) derived from sha256 of 'seed|machine_id|cycle_no' (first 8 bytes / 2**64)."""
    h = hashlib.sha256(f"{int(seed)}|{machine_id}|{int(cycle_no)}".encode("utf-8")).digest()
    return int.from_bytes(h[:8], "big") / 2.0**64


def is_defective(seed: int, machine_id: str, cycle_no: int, rate_pct: float) -> bool:
    """True if cycle number `cycle_no` (1-based: the n-th cycle started on this machine) is defective.
    rate_pct = 0 is never defective, 100 always."""
    if rate_pct <= 0.0:
        return False
    return defect_uniform(seed, machine_id, cycle_no) * 100.0 < rate_pct
