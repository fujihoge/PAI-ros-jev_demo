"""B1 runner: replay of the recorded b1x episode with an ideal navigation stub (arrives at once; pace=True keeps the
planned leg times) must reproduce the calculation model: 20 shipped, 133 decisions, 7 defects."""
import json
from pathlib import Path

import pytest

from factory_b import make_scenario
from factory_sim import b1_visuals
from factory_sim.b1_runner import B1Runner, ReplayPolicy, cand_name, machine_lamp_state
from factory_sim.nav_types import NavObs

ROOT = Path(__file__).resolve().parents[4]
TRACE = ROOT / "recorded" / "b1x_seed5_empty.json"
pytestmark = pytest.mark.skipif(not TRACE.exists(), reason="recorded episode not present")


def scn():
    return make_scenario("B1", "ii", ct_scale=0.6).with_condition(0.10, 60.0, True).with_start("empty")


def drive(r, dt=0.02, until=None):
    t = 0.0
    while not r.ended and (until is None or t < until):
        g = r.nav_goal
        nav = NavObs(goal_id=g.goal_id, status="arrived", result={"duration_s": 0.0}) if g else NavObs()
        r.update(t, nav)
        t += dt
    return t


@pytest.fixture(scope="module")
def plan():
    return json.loads(TRACE.read_text(encoding="utf-8"))["decisions"]


def test_replay_reproduces_the_calculation_model(plan):
    r = B1Runner(scn(), 5, ReplayPolicy(plan), pace=True)
    drive(r)
    m = r.metrics()
    assert m["completed"] == 20
    assert m["decisions"] == 133 and m["replay_used"] == 133 and m["replay_skipped"] == []
    assert m["defects"] == 7 and m["deliveries"] == 116 and m["lock_wait_count"] == 5


def test_replay_order_is_kept_and_infeasible_decisions_are_skipped_after_the_wait():
    plan = [{"idx": 1, "t": 0.0, "chosen": "ship", "source": "jev"},      # nothing to ship at the start
            {"idx": 2, "t": 1.0, "chosen": "supply_M1", "source": "jev"}]
    p = ReplayPolicy(plan, max_wait_s=3.0)
    r = B1Runner(scn(), 5, p, duration_s=40.0, pace=True)
    drive(r)
    assert [s["wanted"] for s in p.skipped] == ["ship"]
    assert r.decisions[0]["choice"] == "supply_M1" and r.decisions[0]["time_s"] >= 3.0


def test_slow_robot_drifts_but_does_not_break():
    r = B1Runner(scn(), 5, ReplayPolicy(json.loads(TRACE.read_text(encoding="utf-8"))["decisions"]), duration_s=300.0)
    t = 0.0
    arrive_at = None
    while not r.ended:
        g = r.nav_goal
        if g is None:
            arrive_at = None
            nav = NavObs()
        else:
            arrive_at = arrive_at if arrive_at is not None else (g.goal_id, t + g.planned_s + 3.0)
            ok = arrive_at[0] == g.goal_id and t >= arrive_at[1]
            nav = NavObs(goal_id=g.goal_id, status="arrived" if ok else "running", result={"duration_s": 1.0})
            if arrive_at[0] != g.goal_id:
                arrive_at = (g.goal_id, t + g.planned_s + 3.0)
        r.update(t, nav)
        t += 0.05
    m = r.metrics()
    assert m["leg_over_plan_mean_s"] > 2.5 and m["aborted"] is None and m["completed"] >= 0


def test_lamp_states_and_visual_view():
    r = B1Runner(scn(), 5, ReplayPolicy([]), duration_s=10.0)
    assert machine_lamp_state(r.sim, 0) == "STARVED"
    v = r.visual_view()
    assert v["machines"]["M1"] == "STARVED" and v["stock"]["FIN"] == (0, 2) and v["stock"]["NG3"] == (0, 1)
    cmds = b1_visuals.commands(v, {"PROCESSING": "green", "STARVED": "orange", "NG_EJECTING": "red",
                                   "WAITING_UPSTREAM": "orange", "BLOCKED": "orange"})
    assert cmds["lamp:M1"] == ("/m5/lamp_cmd", "M1:orange")
    assert cmds["stock:FIN"] == ("/m5/stock_cmd", "FIN:0")
    assert cmds["stock:NG2"] == ("/m5/stock_cmd", "NG2:0")
    assert cmds["cargo"][1] == "hide"


def test_cargo_color_follows_the_load():
    v = {"machines": {"M1": "PROCESSING"}, "stock": {}, "cargo": True, "cargo_color": "green", "completed": 0}
    assert b1_visuals.commands(v, {"PROCESSING": "green"})["cargo"] == ("/m5/cargo_cmd", "show:green")


def test_cand_name_matches_policy_ids(plan):
    from policies.jev_b1 import cand_id
    s = scn()
    from factory_b import Sim
    sim = Sim(s, 5)
    for c in sim.candidates():
        assert cand_name(c) == cand_id(c)
