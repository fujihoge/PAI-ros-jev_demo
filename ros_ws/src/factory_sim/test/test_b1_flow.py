"""b1_flow: visual-only workpiece animation (path / timing / delay logic). The calculation model must stay unchanged."""
import json
import math
from pathlib import Path

import pytest

from factory_b import make_scenario
from factory_sim import b1_flow, b1_visuals
from factory_sim.b1_runner import B1Runner, ReplayPolicy
from factory_sim.nav_types import NavObs

PKG = Path(__file__).resolve().parents[1]
ROOT = PKG.parents[2]
TRACE = ROOT / "recorded" / "b1x_seed5_empty.json"
PATHS = {"M1>M2": [(0, 0), (0, 1), (2, 1)], "M2>M3": [(0, 0), (1, 0)], "M3>M4": [(0, 0), (1, 0.5)], "M4>FIN": [(0, 0), (0, 2)]}
LAMPS = {"PROCESSING": "green", "STARVED": "orange", "WAITING_UPSTREAM": "orange", "BLOCKED": "orange", "NG_EJECTING": "red"}


def scn():
    return make_scenario("B1", "ii", ct_scale=0.6).with_condition(0.10, 60.0, True).with_start("empty")


def test_leg_names():
    assert [b1_flow.leg_name(i, 4) for i in range(4)] == ["M1>M2", "M2>M3", "M3>M4", "M4>FIN"]


def test_duration_is_fixed_and_at_most_three_seconds():
    assert b1_flow.FLOW_SECONDS == 2.0 <= 3.0
    assert b1_flow.MAX_PIECES >= 5


def test_point_at_follows_the_polyline_at_constant_speed():
    p = PATHS["M1>M2"]                                  # length 1 + 2 = 3
    assert b1_flow.path_length(p) == 3
    assert b1_flow.point_at(p, 0.0) == (0, 0)
    assert b1_flow.point_at(p, 1 / 3) == pytest.approx((0, 1))
    assert b1_flow.point_at(p, 0.5) == pytest.approx((0.5, 1))
    assert b1_flow.point_at(p, 1.0) == (2, 1)
    assert b1_flow.point_at(p, 7) == (2, 1) and b1_flow.point_at(p, -1) == (0, 0)
    a, b = b1_flow.point_at(p, 0.1), b1_flow.point_at(p, 0.2)
    c, d = b1_flow.point_at(p, 0.2), b1_flow.point_at(p, 0.3)
    assert math.dist(a, b) == pytest.approx(math.dist(c, d))


def test_cmd_roundtrip_and_errors():
    dur, pts = b1_flow.decode_cmd(b1_flow.encode_cmd(PATHS["M1>M2"]))
    assert dur == 2.0 and pts == [(0, 0), (0, 1), (2, 1)]
    for bad in ("", "2|1,1", "0|0,0;1,1", "x"):
        with pytest.raises(ValueError):
            b1_flow.decode_cmd(bad)


def test_five_concurrent_pieces_are_possible():
    starts = [0.0, 0.3, 0.6, 0.9, 1.2, 1.5]
    assert b1_flow.concurrent_peak(starts) == 6 <= b1_flow.MAX_PIECES
    assert b1_flow.concurrent_peak([0.0, 2.0, 4.0]) == 1          # back to back: one at a time


def test_handoff_sends_a_command_with_the_leg_path_and_delays_the_downstream_box():
    tr = b1_flow.FlowTracker(PATHS, 4)
    cmds = tr.feed([("h", 0)], 10.0)
    assert cmds == [b1_flow.encode_cmd(PATHS["M1>M2"])]
    tr.feed([("c", 1)], 10.0)                              # M2 takes the part at once: its box must wait for the arrival
    assert tr.work_hidden(10.0) == {1} and tr.work_hidden(11.9) == {1}
    assert tr.work_hidden(12.0) == set()


def test_downstream_started_long_after_the_handoff_is_not_delayed():
    tr = b1_flow.FlowTracker(PATHS, 4)
    tr.feed([("h", 1)], 10.0)
    tr.feed([("c", 2)], 40.0)                              # slot waited 30 s, the piece has long arrived
    assert tr.work_hidden(40.0) == set()


def test_same_tick_consume_then_refill_uses_the_old_fill_time():
    tr = b1_flow.FlowTracker(PATHS, 4)
    tr.feed([("h", 0)], 5.0)
    # one tick: M2 consumed the old part, M1 handed over a new one; log order is h then c
    tr.feed([("h", 0), ("c", 1)], 30.0)
    assert tr.work_hidden(30.0) == set()                   # gate = 5 + 2, not 30 + 2


def test_initial_wip_slot_has_no_delay_and_machine_one_is_never_gated():
    tr = b1_flow.FlowTracker(PATHS, 4, wip0=1)
    tr.feed([("c", 0), ("c", 1)], 0.0)
    assert tr.work_hidden(0.0) == set()


def test_defect_events_do_not_move_anything():
    tr = b1_flow.FlowTracker(PATHS, 4)
    assert tr.feed([("c", 0)], 3.0) == []                  # no 'h' for an NG part (the engine does not hand it over)


def test_last_machine_goes_to_fin_and_the_cube_appears_on_arrival():
    tr = b1_flow.FlowTracker(PATHS, 4)
    assert tr.feed([("h", 3)], 100.0) == [b1_flow.encode_cmd(PATHS["M4>FIN"])]
    assert tr.fin_shown(1, 100.5) == 0 and tr.fin_shown(1, 102.0) == 1
    tr.feed([("h", 3)], 110.0)
    assert tr.fin_shown(2, 111.0) == 1 and tr.fin_shown(2, 112.5) == 2


def test_robot_taking_a_travelling_piece_drops_it():
    tr = b1_flow.FlowTracker(PATHS, 4)
    tr.feed([("h", 3)], 100.0)
    assert tr.fin_shown(0, 100.5) == 0 and tr.fin_shown(1, 105.0) == 1   # a later part is not held back by the old record


def test_adjust_view_changes_only_work_box_and_fin_count():
    r = B1Runner(scn(), 5, ReplayPolicy([]), duration_s=10.0)
    tr = b1_flow.FlowTracker(PATHS, 4)
    tr.feed([("h", 2), ("c", 3)], 10.0)
    v = tr.adjust_view(r.visual_view(), r.ids, 10.5)
    assert v["work_hidden"] == {"M4"}
    v["machines"]["M4"] = "PROCESSING"
    cmds = b1_visuals.commands(v, LAMPS)
    assert cmds["work:M4"][1] == "M4:hide"            # model says PROCESSING, the piece is still on the belt
    assert cmds["lamp:M4"][1] == "M4:green"           # the lamp follows the model


def test_committed_flow_paths_cover_the_four_legs():
    committed = json.loads((PKG / "worlds" / "factory_b1.flow.json").read_text(encoding="utf-8"))
    assert set(committed) == set(PATHS)
    for leg, pts in committed.items():
        assert len(pts) >= 2 and all(len(p) == 2 for p in pts), leg
        assert b1_flow.path_length([tuple(p) for p in pts]) > 0.05, leg


@pytest.mark.skipif(not TRACE.exists(), reason="recorded episode not present")
def test_animation_events_do_not_change_the_model():
    plan = json.loads(TRACE.read_text(encoding="utf-8"))["decisions"]
    r = B1Runner(scn(), 5, ReplayPolicy(plan), pace=True)
    tr = b1_flow.FlowTracker(PATHS, 4, scn().wip0)
    t, sent = 0.0, []
    while not r.ended:
        g = r.nav_goal
        r.update(t, NavObs(goal_id=g.goal_id, status="arrived", result={"duration_s": 0.0}) if g else NavObs())
        for c in tr.feed(r.drain_flow(), t):
            sent.append((t, c))
        t += 0.05
    m = r.metrics()
    assert m["completed"] == 20 and m["decisions"] == 133 and m["defects"] == 7      # same numbers as test_b1_runner
    by_leg = {k: sum(1 for _, c in sent if c == b1_flow.encode_cmd(v)) for k, v in PATHS.items()}
    assert by_leg["M4>FIN"] == r.sim.produced                                       # one animation per good part into FIN
    assert by_leg["M1>M2"] >= by_leg["M2>M3"] >= by_leg["M3>M4"] >= by_leg["M4>FIN"] >= 20
    assert b1_flow.concurrent_peak([t for t, _ in sent]) <= b1_flow.MAX_PIECES
