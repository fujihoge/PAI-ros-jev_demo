"""Decision split: message encoding, id matching, timeout / fallback, and the loopback equivalence with the in-process replay
(no ROS, no network; the Jev path runs on a MockTransport)."""
import json
from pathlib import Path

import pytest

from adapters.jev import JevError, MockTransport
from factory_b import make_scenario
from factory_sim import decision_core as dc
from factory_sim.b1_runner import B1Runner, ReplayPolicy
from factory_sim.nav_types import NavObs
from policies.jev_b1 import JevB1Policy, fallback_choice

ROOT = Path(__file__).resolve().parents[4]
TRACE = ROOT / "recorded" / "b1x_seed5_empty.json"
needs_trace = pytest.mark.skipif(not TRACE.exists(), reason="recorded episode not present")


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


def loopback(decider, **kw):
    """RemotePolicy whose requests are answered at once through JSON (what the two nodes do, without the network)."""
    def send(req):
        req = json.loads(json.dumps(req, allow_nan=False))
        ans = json.loads(json.dumps(decider.decide(req), allow_nan=False))
        rp.on_message(ans)
    rp = dc.RemotePolicy(send, **kw)
    return rp


# ---------------------------------------------------------------- encoding
def test_finite_maps_non_finite_numbers_and_keeps_floats():
    x = {"a": float("inf"), "b": [float("-inf"), 1.23456789, float("nan")], "c": ("x", 2)}
    f = dc.finite(x)
    assert f == {"a": dc.BIG, "b": [-dc.BIG, 1.23456789, None], "c": ["x", 2]}
    json.dumps(f, allow_nan=False)


def test_request_and_answer_round_trip_and_validation():
    snap = {"state": {"x": 1.0}, "criteria": {"ship": {}, "supply_M1": {}}, "meta": {"ship": {"slack": float("inf")}},
            "order": ["supply_M1", "ship"]}
    req = json.loads(json.dumps(dc.make_request("r000001", 12.34567, 1800.0, snap), allow_nan=False))
    assert dc.valid_request(req) and req["candidates"] == ["supply_M1", "ship"] and req["time_s"] == 12.34567
    assert req["snapshot"]["meta"]["ship"]["slack"] == dc.BIG
    ans = dc.make_answer(req, "replay", "ship", "jev", "r", {"p": 0.5})
    assert dc.valid_answer(json.loads(json.dumps(ans)))
    assert not dc.valid_request({"request_id": 1}) and not dc.valid_request(None)
    assert not dc.valid_answer({"kind": "x", "request_id": "r", "wait": False}) and not dc.valid_answer([])
    assert not dc.valid_answer({**ans, "wait": "no"})


# ---------------------------------------------------------------- id matching, timeout, fallback
class Clock:
    t = 100.0

    def __call__(self):
        return self.t


def runner_at_first_decision():
    r = B1Runner(scn(), 5, ReplayPolicy([]), 300.0)
    r.sim.now = 0.0
    return r.sim, r.sim.candidates()


def answer(req_id, choice, wait=False, **kw):
    return {"kind": dc.KIND_DECISION, "request_id": req_id, "time_s": 0.0, "choice": choice, "wait": wait, "policy": "replay",
            "source": "jev", "rule": "", "info": kw.get("info", {"source": "jev"}), "stats": kw.get("stats", {})}


def test_pending_then_matching_answer_is_used_and_mismatching_ids_are_ignored():
    sim, cands = runner_at_first_decision()
    sent, clock = [], Clock()
    rp = dc.RemotePolicy(sent.append, timeout_s=5.0, mono=clock)
    c, info = rp.choose(sim, cands, 0.0, 300.0)
    assert c is None and info == {"pending": True} and len(sent) == 1
    rid = sent[0]["request_id"]
    assert rp.pending_request["request_id"] == rid
    # asked again while pending: no second request
    assert rp.choose(sim, cands, 0.05, 300.0)[1] == {"pending": True} and len(sent) == 1
    assert not rp.on_message(answer("r999999", "supply_M1"))            # other id
    assert not rp.on_message({"kind": "something_else"})               # garbage
    clock.t += 0.02
    assert rp.on_message(answer(rid, "supply_M1"))
    assert not rp.on_message(answer(rid, "supply_M2"))                  # duplicate answer
    c, info = rp.choose(sim, cands, 0.1, 300.0)
    assert c is not None and c.job == ("S", ((0, 0),)) and info["request_id"] == rid and info["rtt_ms"] == pytest.approx(20.0)
    assert rp.pending_request is None
    assert not rp.on_message(answer(rid, "supply_M1"))                  # late answer after use
    st = rp.stats()
    assert st["stale_ignored"] == 3 and st["invalid_ignored"] == 1 and st["timeouts"] == 0 and st["answers"] == 1


def test_wait_answer_returns_none_with_info_and_updates_replay_stats():
    sim, cands = runner_at_first_decision()
    sent = []
    rp = dc.RemotePolicy(sent.append, mono=Clock())
    rp.choose(sim, cands, 0.0, 300.0)
    rp.on_message(answer(sent[0]["request_id"], None, wait=True, info={"replay": "waiting"},
                         stats={"planned": 133, "used": 4, "skipped": [{"plan_idx": 5}]}))
    c, info = rp.choose(sim, cands, 0.05, 300.0)
    assert c is None and info["replay"] == "waiting" and rp.k == 4 and rp.plan_len == 133 and len(rp.skipped) == 1
    assert rp.stats()["waits"] == 1 and rp.pending_request is None


def test_timeout_uses_the_smallest_slack_fallback_once_and_a_late_answer_is_ignored():
    sim, cands = runner_at_first_decision()
    sent, clock = [], Clock()
    rp = dc.RemotePolicy(sent.append, timeout_s=5.0, mono=clock)
    rp.choose(sim, cands, 0.0, 300.0)
    clock.t += 4.9
    assert rp.choose(sim, cands, 0.1, 300.0)[1] == {"pending": True}
    clock.t += 0.2
    c, info = rp.choose(sim, cands, 0.2, 300.0)
    assert c is not None and info["source"] == "fallback_timeout" and info["rule"] == "smallest_slack_fallback"
    snap = sent[0]["snapshot"]
    expect = fallback_choice(snap, sent[0]["candidates"])
    assert info["request_id"] == sent[0]["request_id"] and dc.cand_name(c) == expect
    assert rp.stats()["timeouts"] == 1 and rp.timeouts[0]["waited_s"] == pytest.approx(5.1)
    assert not rp.on_message(answer(sent[0]["request_id"], "supply_M1"))   # too late: ignored
    assert rp.choose(sim, cands, 0.3, 300.0)[1] == {"pending": True} and len(sent) == 2   # the next decision is a new request


def test_choice_that_is_no_longer_a_candidate_is_dropped_and_asked_again():
    sim, cands = runner_at_first_decision()
    sent = []
    rp = dc.RemotePolicy(sent.append, mono=Clock())
    rp.choose(sim, cands, 0.0, 300.0)
    rp.on_message(answer(sent[0]["request_id"], "ship"))                 # nothing to ship at the start
    c, info = rp.choose(sim, cands, 0.05, 300.0)
    assert c is None and info == {"pending": True} and len(sent) == 2 and rp.stats()["choice_not_candidate"] == 1


# ---------------------------------------------------------------- loopback equals in-process
@needs_trace
def test_replay_through_the_decision_protocol_reproduces_the_in_process_episode():
    plan = json.loads(TRACE.read_text(encoding="utf-8"))["decisions"]
    a = B1Runner(scn(), 5, ReplayPolicy(plan), pace=True)
    drive(a)
    rp = loopback(dc.ReplayDecider(plan))
    b = B1Runner(scn(), 5, rp, pace=True)
    drive(b)
    m = b.metrics()
    assert m["completed"] == 20 and m["decisions"] == 133 and m["replay_used"] == 133 and m["replay_skipped"] == []
    assert m["defects"] == 7 and m["deliveries"] == 116
    assert [(d["time_s"], d["choice"]) for d in b.decisions] == [(d["time_s"], d["choice"]) for d in a.decisions]
    assert all(d["policy"] == "replay" and d["decider"] == "node" for d in b.decisions)
    st = rp.stats()
    assert st["timeouts"] == 0 and st["answers"] >= 133 and st["stale_ignored"] == 0


@needs_trace
def test_jev_b1x_path_works_with_a_mock_transport_and_no_network():
    mock = MockTransport()
    pol = JevB1Policy(mock, "b1x")
    rp = loopback(dc.JevDecider(pol, "b1x"))
    r = B1Runner(scn(), 5, rp, duration_s=900.0, pace=True)
    drive(r)
    assert r.nav_failures == 0 and len(r.decisions) > 20 and rp.stats()["timeouts"] == 0
    sources = {d["source"] for d in r.decisions}
    assert sources <= {"rule", "single_candidate", "mock"} and "mock" in sources
    asked = [d for d in r.decisions if d["source"] == "mock"]
    assert all(d["probabilities"] and d["choice"] in d["probabilities"] and d["real_api"] is False for d in asked)
    assert mock.calls == len(asked)                                      # only real asks reach the transport


@needs_trace
def test_jev_error_falls_back_inside_the_decider_and_is_reported():
    mock = MockTransport(failures=[JevError("bad_request", retryable=False)] * 3)
    pol = JevB1Policy(mock, "b1x", max_retries=0)
    rp = loopback(dc.JevDecider(pol, "b1x"))
    r = B1Runner(scn(), 5, rp, duration_s=600.0, pace=True)
    drive(r)
    assert "fallback" in {d["source"] for d in r.decisions}


def test_a_quick_answer_starts_the_task_at_the_request_time_a_slow_one_when_it_arrives():
    sim, cands = runner_at_first_decision()
    for later, expect_backdate in ((0.3, True), (0.9, False)):
        sent = []
        rp = dc.RemotePolicy(sent.append, mono=Clock())
        rp.choose(sim, cands, 5.0, 300.0)
        rp.on_message(answer(sent[0]["request_id"], "supply_M1"))
        c, info = rp.choose(sim, cands, 5.0 + later, 300.0)
        assert c is not None and info["answered_t"] == pytest.approx(5.0 + later)
        assert ("decision_t" in info) == expect_backdate and (not expect_backdate or info["decision_t"] == 5.0)
