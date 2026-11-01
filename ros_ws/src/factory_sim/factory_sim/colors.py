"""State -> visual command mapping for the M5 plugin (/m5/lamp_cmd, /m5/cargo_cmd, /m5/wip_cmd, /m5/work_cmd,
/m5/banner_cmd, /m5/stock_cmd). No ROS."""
from __future__ import annotations

import json

# User decision 2026-10-24: PROCESSING green; STARVED / WAITING_UPSTREAM / BLOCKED (any wait) orange; NG_EJECTING red.
DEFAULT_LAMP_COLORS = {
    "PROCESSING": "green",
    "STARVED": "orange",
    "NG_EJECTING": "red",
    "WAITING_UPSTREAM": "orange",
    "BLOCKED": "orange",
}
# The workpiece box inside a machine is shown while the machine holds a workpiece. RUN (-> PROCESSING) is
# cutting it; DONE (-> BLOCKED) holds the finished one until the downstream slot is free. IDLE (STARVED /
# WAITING_UPSTREAM) holds none; EJECT (NG_EJECTING) already scrapped the defective one.
WORK_VISIBLE_STATES = frozenset({"PROCESSING", "BLOCKED"})
BANNER_SECONDS = 3.0                # "Completed!" board stays this long (sim time) after the line count goes up
WIP_FULL_COLOR = "brown"            # a workpiece is in the WIP slot
WIP_EMPTY_COLOR = "0.7,0.7,0.72"    # original plate colour in the world file
CARGO_COLOR = None                  # None = keep the colour defined in the URDF


def merge_lamp_colors(override: dict | str | None) -> dict:
    """Defaults + override (dict or JSON text). Unknown state names are an error."""
    colors = dict(DEFAULT_LAMP_COLORS)
    if override:
        if isinstance(override, str):
            override = json.loads(override)
        unknown = set(override) - set(DEFAULT_LAMP_COLORS)
        if unknown:
            raise ValueError(f"unknown machine states in lamp colours: {sorted(unknown)}")
        colors.update(override)
    return colors


def lamp_command(machine_id: str, state: str, colors: dict) -> str:
    return f"{machine_id}:{colors[state]}"


def wip_command(upstream: str, downstream: str, occupied: bool) -> str:
    return f"{upstream}_{downstream}:{WIP_FULL_COLOR if occupied else WIP_EMPTY_COLOR}"


def cargo_command(visible: bool, color: str | None = CARGO_COLOR) -> str:
    mode = "show" if visible else "hide"
    return f"{mode}:{color}" if color else mode


def work_visible(state: str) -> bool:
    """Is the workpiece box shown inside a machine in this state?"""
    return state in WORK_VISIBLE_STATES


def work_command(machine_id: str, visible: bool) -> str:
    return f"{machine_id}:{'show' if visible else 'hide'}"


def stock_command(machine_id: str, n: int, cap: int) -> str:
    """"M1:3" = show the first 3 stock cubes of M1 (n is clamped to 0..cap). n = Machine.buffer."""
    return f"{machine_id}:{max(0, min(int(n), int(cap)))}"


def banner_command(visible: bool) -> str:
    return "show" if visible else "hide"


class CompletionBanner:
    """Shows the "Completed!" board for `seconds` (sim time) from the moment the line's completed count goes up.
    The first call only records the baseline count. Pure: time and count are passed in."""

    def __init__(self, seconds: float = BANNER_SECONDS) -> None:
        self.seconds = seconds
        self._count: int | None = None
        self._until = -1e18

    def update(self, t: float, completed: int) -> bool:
        if self._count is not None and completed > self._count:
            self._until = t + self.seconds
        self._count = completed
        return t < self._until


def visual_commands(view: dict, colors: dict) -> dict[str, str]:
    """All commands for one view {"machines": {id: state}, "wip": [0/1 ...], "cargo": bool,
    optional "work": {id: bool}, "banner": bool, "stock": {id: (buffer, capacity)}}.
    Key = a stable name of the visual (used to send only what changed)."""
    ids = list(view["machines"])
    out = {}
    for mid, st in view["machines"].items():
        out[f"lamp:{mid}"] = lamp_command(mid, st, colors)
    for i, occ in enumerate(view["wip"]):
        out[f"wip:{ids[i]}_{ids[i + 1]}"] = wip_command(ids[i], ids[i + 1], bool(occ))
    out["cargo"] = cargo_command(bool(view["cargo"]))
    for mid in ids:
        st = view["machines"][mid]
        out[f"work:{mid}"] = work_command(mid, view["work"][mid] if "work" in view else work_visible(st))
    for mid, (n, cap) in view.get("stock", {}).items():
        out[f"stock:{mid}"] = stock_command(mid, n, cap)
    if "banner" in view:
        out["banner"] = banner_command(bool(view["banner"]))
    return out


TOPIC_OF = {"lamp": "/m5/lamp_cmd", "wip": "/m5/wip_cmd", "cargo": "/m5/cargo_cmd",
            "work": "/m5/work_cmd", "stock": "/m5/stock_cmd", "banner": "/m5/banner_cmd"}


class VisualTracker:
    """Remembers what was sent; `changed()` returns only new/different commands, `everything()` all of them."""

    def __init__(self, colors: dict) -> None:
        self.colors = colors
        self._sent: dict[str, str] = {}

    def changed(self, view: dict) -> list[tuple[str, str]]:
        cmds = visual_commands(view, self.colors)
        out = [(TOPIC_OF[k.split(":")[0]], v) for k, v in cmds.items() if self._sent.get(k) != v]
        self._sent = cmds
        return out

    def everything(self) -> list[tuple[str, str]]:
        return [(TOPIC_OF[k.split(":")[0]], v) for k, v in self._sent.items()]
