"""B1 visual commands for the m5_visual_state plugin (no ROS). Extends colors.py with the stock models of the B1 world:
NG1..NG4 (scrap bins, capacity 1), FIN (finished buffer, 2), DOCKS (pieces just dropped at the dock), DOCKX (scrap just dropped)."""
from __future__ import annotations

from . import colors

TOPIC = {"lamp": "/m5/lamp_cmd", "work": "/m5/work_cmd", "stock": "/m5/stock_cmd", "cargo": "/m5/cargo_cmd",
         "banner": "/m5/banner_cmd", "flow": "/m5/flow_cmd"}


def commands(view: dict, lamp_colors: dict) -> dict[str, tuple[str, str]]:
    """view = B1Runner.visual_view() (+ optional "banner"). Key = stable name of the visual -> (topic, text)."""
    out = {}
    hidden = view.get("work_hidden", ())   # visual-only delay (b1_flow): the piece is still on the belt
    for mid, st in view["machines"].items():
        out[f"lamp:{mid}"] = (TOPIC["lamp"], colors.lamp_command(mid, st, lamp_colors))
        out[f"work:{mid}"] = (TOPIC["work"], colors.work_command(mid, colors.work_visible(st) and mid not in hidden))
    for sid, (n, cap) in view["stock"].items():
        out[f"stock:{sid}"] = (TOPIC["stock"], colors.stock_command(sid, n, cap))
    color = view.get("cargo_color") or None
    if view["cargo"]:
        out["cargo"] = (TOPIC["cargo"], f"show:{color}" if color else "show")
    else:
        out["cargo"] = (TOPIC["cargo"], "hide")
    if "banner" in view:
        out["banner"] = (TOPIC["banner"], colors.banner_command(bool(view["banner"])))
    return out


class Tracker:
    """Sends only what changed; `everything()` re-sends the last state (the plugin also re-publishes by itself)."""

    def __init__(self, lamp_colors: dict) -> None:
        self.colors, self._sent = lamp_colors, {}

    def changed(self, view: dict) -> list[tuple[str, str]]:
        cmds = commands(view, self.colors)
        out = [v for k, v in cmds.items() if self._sent.get(k) != v]
        self._sent = cmds
        return out

    def everything(self) -> list[tuple[str, str]]:
        return list(self._sent.values())
