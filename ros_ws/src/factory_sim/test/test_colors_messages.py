import json
import math

import pytest

from factory_sim import colors, messages


def view(states, wip, cargo=False):
    return {"machines": dict(zip(["M1", "M2", "M3", "M4"], states)), "wip": wip, "cargo": cargo}


def test_default_lamp_colors_follow_the_user_decision():
    # user decision 2026-10-24: processing green, every kind of waiting orange, NG ejection red
    c = colors.merge_lamp_colors(None)
    assert c["PROCESSING"] == "green" and c["NG_EJECTING"] == "red"
    assert c["STARVED"] == c["WAITING_UPSTREAM"] == c["BLOCKED"] == "orange"
    assert set(c) == {"PROCESSING", "STARVED", "NG_EJECTING", "WAITING_UPSTREAM", "BLOCKED"}


def test_lamp_colors_can_be_overridden_by_json_and_unknown_states_are_rejected():
    c = colors.merge_lamp_colors('{"BLOCKED": "blue"}')
    assert c["BLOCKED"] == "blue" and c["PROCESSING"] == "green"
    with pytest.raises(ValueError):
        colors.merge_lamp_colors({"IDLE": "red"})


def test_visual_commands_cover_lamps_wip_plates_and_cargo():
    v = view(["PROCESSING", "STARVED", "BLOCKED", "WAITING_UPSTREAM"], [1, 0, 1], cargo=True)
    cmds = colors.visual_commands(v, colors.merge_lamp_colors(None))
    assert cmds["lamp:M1"] == "M1:green" and cmds["lamp:M2"] == "M2:orange"
    assert cmds["lamp:M3"] == "M3:orange" and cmds["lamp:M4"] == "M4:orange"
    assert cmds["wip:M1_M2"] == "M1_M2:brown"
    assert cmds["wip:M2_M3"] == "M2_M3:" + colors.WIP_EMPTY_COLOR
    assert cmds["wip:M3_M4"] == "M3_M4:brown"
    assert cmds["cargo"] == "show"
    assert colors.cargo_command(False) == "hide"
    # work boxes (from the state when the view has no explicit "work"): PROCESSING and BLOCKED only
    assert [cmds[f"work:M{i}"] for i in range(1, 5)] == ["M1:show", "M2:hide", "M3:show", "M4:hide"]
    assert "banner" not in cmds


def test_tracker_sends_only_changes_and_can_resend_everything():
    tr = colors.VisualTracker(colors.merge_lamp_colors(None))
    v = view(["PROCESSING"] * 4, [1, 1, 1])
    first = tr.changed(v)
    assert len(first) == 4 + 3 + 1 + 4
    assert tr.changed(v) == []
    v2 = view(["PROCESSING", "STARVED", "PROCESSING", "PROCESSING"], [1, 1, 0], cargo=True)
    out = tr.changed(v2)
    assert sorted(out) == sorted([("/m5/lamp_cmd", "M2:orange"), ("/m5/wip_cmd", "M3_M4:" + colors.WIP_EMPTY_COLOR),
                                  ("/m5/cargo_cmd", "show"), ("/m5/work_cmd", "M2:hide")])
    assert len(tr.everything()) == 12


def test_work_box_visible_only_while_the_machine_holds_a_workpiece():
    # factory_core.machine: RUN->PROCESSING and DONE->BLOCKED hold one; IDLE (STARVED/WAITING_UPSTREAM) none;
    # EJECT (NG_EJECTING) has already scrapped the defective one
    shown = {s: colors.work_visible(s) for s in
             ["PROCESSING", "BLOCKED", "STARVED", "WAITING_UPSTREAM", "NG_EJECTING"]}
    assert shown == {"PROCESSING": True, "BLOCKED": True, "STARVED": False, "WAITING_UPSTREAM": False,
                     "NG_EJECTING": False}
    assert colors.work_command("M3", True) == "M3:show" and colors.work_command("M3", False) == "M3:hide"


def test_explicit_work_and_banner_in_the_view_go_to_their_topics():
    v = view(["PROCESSING"] * 4, [1, 1, 1])
    v["work"] = {"M1": True, "M2": False, "M3": True, "M4": True}
    v["banner"] = False
    tr = colors.VisualTracker(colors.merge_lamp_colors(None))
    assert ("/m5/banner_cmd", "hide") in tr.changed(v)
    v["banner"] = True
    assert tr.changed(v) == [("/m5/banner_cmd", "show")]
    assert tr.changed(v) == []


def test_completed_banner_shows_for_three_sim_seconds_from_the_count_increase():
    b = colors.CompletionBanner()
    assert colors.BANNER_SECONDS == 3.0
    assert b.update(0.0, 0) is False
    assert b.update(100.0, 0) is False
    assert b.update(100.05, 1) is True            # shown at the very tick the count went up
    assert b.update(102.0, 1) is True
    assert b.update(103.04, 1) is True            # 100.05 + 3 = 103.05
    assert b.update(103.05, 1) is False
    assert b.update(110.0, 1) is False
    # a second completion during the display restarts the 3 s
    assert b.update(120.0, 2) is True
    assert b.update(121.0, 3) is True
    assert b.update(123.9, 3) is True
    assert b.update(124.0, 3) is False


def test_completed_banner_ignores_the_starting_count():
    b = colors.CompletionBanner()
    assert b.update(0.0, 5) is False              # first call = baseline, no display
    assert b.update(0.05, 5) is False


def test_envelope_has_schema_and_monotonic_version_and_is_valid_json():
    ver = messages.Versioner()
    a = messages.envelope(ver.next(), "factory_state", {"time_s": 1.23456, "x": float("inf"), "y": [float("nan"), 2.0]})
    b = messages.envelope(ver.next(), "metrics", {"v": 1})
    assert a["schema_version"] == messages.SCHEMA_VERSION == 1
    assert b["state_version"] == a["state_version"] + 1
    d = json.loads(messages.dumps(a))   # allow_nan=False: inf/nan were replaced by null
    assert d["time_s"] == 1.235 and d["x"] is None and d["y"][0] is None
    assert not any(isinstance(v, float) and math.isnan(v) for v in a.values())


def test_stock_command_is_the_buffer_count_clamped_to_capacity():
    assert colors.stock_command("M1", 3, 3) == "M1:3"
    assert colors.stock_command("M4", 0, 12) == "M4:0"
    assert colors.stock_command("M2", 99, 9) == "M2:9" and colors.stock_command("M2", -1, 9) == "M2:0"


def test_stock_commands_are_sent_only_when_the_count_changes():
    v = view(["PROCESSING"] * 4, [1, 1, 1])
    assert not any(k.startswith("stock") for k in colors.visual_commands(v, colors.merge_lamp_colors(None)))
    v["stock"] = {"M1": (2, 3), "M2": (6, 9), "M3": (4, 6), "M4": (8, 12)}
    tr = colors.VisualTracker(colors.merge_lamp_colors(None))
    first = tr.changed(v)
    assert [c for t, c in first if t == "/m5/stock_cmd"] == ["M1:2", "M2:6", "M3:4", "M4:8"]
    assert tr.changed(v) == []
    v["stock"]["M1"] = (3, 3)
    assert tr.changed(v) == [("/m5/stock_cmd", "M1:3")]
    assert ("/m5/stock_cmd", "M4:8") in tr.everything()
