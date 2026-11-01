"""B1 workpiece flow on the conveyors: a VISUAL-ONLY animation (no ROS, pure Python).

The calculation model (factory_b.engine.Sim) hands a finished part from machine i to the slot of i+1 in zero time. This module
only decides what to DRAW: when machine i finishes a good part, a small yellow box travels along the belt polyline of that leg
(M1>M2, M2>M3, M3>M4, M4>FIN) in FLOW_SECONDS (fixed). Nothing here feeds back into the model, the trace or any timing.

Visual-only delays (all <= FLOW_SECONDS, the model is not touched):
  * the yellow box inside the downstream machine appears when the moving piece arrives (`work_hidden`);
  * the green FIN cube appears when the piece arrives at the FIN box (`adjust_view`).
Lamps and every other state follow the model without delay.

Wire format of /m5/flow_cmd (std_msgs/String):  "<seconds>|x0,y0;x1,y1;..."   (world coordinates, metres, belt centre line;
the plugin keeps the height). The plugin starts the piece when it receives the message and moves it at constant speed.
"""
from __future__ import annotations

import math
from collections import deque

FLOW_SECONDS = 2.0                  # fixed travel time of every leg (user decision 2026-10-28: one fixed value <= 3 s)
MAX_PIECES = 8                      # pre-spawned visual-only models in the world (plugin <flow_pieces>); > 5 concurrent
TOPIC = "/m5/flow_cmd"


def leg_name(i: int, n: int) -> str:
    """Leg of machine index i (0-based): 'M1>M2' ... and the last one 'M4>FIN'."""
    return f"M{i + 1}>M{i + 2}" if i < n - 1 else f"M{i + 1}>FIN"


def path_length(path) -> float:
    return sum(math.dist(a, b) for a, b in zip(path, path[1:]))


def point_at(path, frac: float) -> tuple[float, float]:
    """Point at fraction `frac` (0..1, clamped) of the arc length of the polyline (constant speed)."""
    frac = min(1.0, max(0.0, frac))
    total = path_length(path)
    if total <= 0.0:
        return tuple(path[0])
    want, acc = frac * total, 0.0
    for a, b in zip(path, path[1:]):
        seg = math.dist(a, b)
        if seg > 0 and acc + seg >= want:
            u = (want - acc) / seg
            return (a[0] + (b[0] - a[0]) * u, a[1] + (b[1] - a[1]) * u)
        acc += seg
    return tuple(path[-1])


def encode_cmd(path, seconds: float = FLOW_SECONDS) -> str:
    return f"{seconds:g}|" + ";".join(f"{x:.4f},{y:.4f}" for x, y in path)


def decode_cmd(text: str) -> tuple[float, list[tuple[float, float]]]:
    head, _, body = text.partition("|")
    pts = [tuple(float(v) for v in p.split(",")) for p in body.split(";") if p]
    if len(pts) < 2 or float(head) <= 0:
        raise ValueError(f"bad flow command: {text!r}")
    return float(head), pts


def concurrent_peak(starts, seconds: float = FLOW_SECONDS) -> int:
    """Largest number of pieces on belts at the same time, for transfers starting at `starts`."""
    ev = sorted([(s, 1) for s in starts] + [(s + seconds, -1) for s in starts], key=lambda e: (e[0], e[1]))
    cur = peak = 0
    for _, d in ev:
        cur += d
        peak = max(peak, cur)
    return peak


class FlowTracker:
    """Turns model events into flow commands and visual-only delays.

    Events come from B1Runner.drain_flow(): ('h', i) = machine i handed a good part over (to i+1, or to FIN for the last one);
    ('c', j) = machine j started a cycle (took the part from the slot between j-1 and j). Both in model order."""

    def __init__(self, paths: dict[str, list], n: int, wip0: int = 0, seconds: float = FLOW_SECONDS) -> None:
        self.paths, self.n, self.seconds = paths, n, seconds
        self.fill = [deque([-math.inf] * int(wip0)) for _ in range(n - 1)]    # times parts were put in slot i (waiting to be taken)
        self.gate_until = [-math.inf] * n                                      # machine j: box visible only after this time
        self.fin_arrivals: list[float] = []

    def feed(self, events, t: float) -> list[str]:
        """Process events that happened at episode time t. Returns the /m5/flow_cmd texts to publish now."""
        cmds = []
        for kind, i in events:
            if kind == "h":
                if i < self.n - 1:
                    self.fill[i].append(t)
                else:
                    self.fin_arrivals.append(t + self.seconds)
                cmds.append(encode_cmd(self.paths[leg_name(i, self.n)], self.seconds))
            elif kind == "c" and i > 0:
                src = self.fill[i - 1].popleft() if self.fill[i - 1] else -math.inf
                self.gate_until[i] = src + self.seconds
        return cmds

    def work_hidden(self, t: float) -> set[int]:
        """Machines whose inner box must still be hidden because the piece for the running cycle has not arrived yet."""
        return {j for j in range(1, self.n) if t < self.gate_until[j]}

    def fin_shown(self, out_bin: int, t: float) -> int:
        """Cubes shown in FIN = model count minus pieces still travelling (those taken by the robot meanwhile are dropped)."""
        self.fin_arrivals = [a for a in self.fin_arrivals if a > t]
        if len(self.fin_arrivals) > out_bin:
            self.fin_arrivals = self.fin_arrivals[len(self.fin_arrivals) - out_bin:]
        return max(0, out_bin - len(self.fin_arrivals))

    def adjust_view(self, view: dict, ids: list[str], t: float) -> dict:
        """Apply the visual-only delays to B1Runner.visual_view() (in place): work boxes and the FIN cubes."""
        view["work_hidden"] = {ids[j] for j in self.work_hidden(t)}
        n_fin, cap = view["stock"]["FIN"]
        view["stock"]["FIN"] = (self.fin_shown(n_fin, t), cap)
        return view
