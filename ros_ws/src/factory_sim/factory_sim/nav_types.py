"""Navigation goal / observation records exchanged between B1Runner and the node (no ROS)."""
from __future__ import annotations

from dataclasses import dataclass


@dataclass
class NavGoal:
    goal_id: int
    name: str                          # "M1".."M4", "dock" or "store"
    xy: tuple
    kind: str                          # "to_station" | "returning"
    task_id: str
    t_start: float
    timeout_t: float
    planned_s: float
    from_name: str


@dataclass
class NavObs:
    """What the node reports about the driver. `goal_id` is the goal the driver is working on."""
    goal_id: int = 0
    status: str = "idle"               # idle | running | arrived | failed
    result: dict | None = None
    pose: tuple = (0.0, 0.0, 0.0)
