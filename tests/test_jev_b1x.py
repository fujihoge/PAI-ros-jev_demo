"""b1x policy (code decides the shipping timing, Jev decides the rest). Mock transport only: no API, no network."""
import json
from pathlib import Path

import pytest

from adapters.jev import JevError, MockTransport
from policies import jev_b1 as J

ROOT = Path(__file__).resolve().parents[1]
RECORDED = ROOT / "recorded" / "b1x_seed5_empty.json"


def _snap_syn(**kw):
    """Synthetic snapshot: id -> (kind, slack, robot_free[, m4_block, free_slots])."""
    meta = {k: {"kind": v[0], "slack": v[1], "robot_free": v[2],
                "m4_block": v[3] if len(v) > 3 else 999.0, "free_slots": v[4] if len(v) > 4 else 3}
            for k, v in kw.items()}
    return {"state": {"t": 0}, "criteria": {k: {"job": k} for k in kw}, "meta": meta, "order": list(kw)}


def _sent(snap):
    sent = {}

    class T(MockTransport):
        def ask(self, state, instructions, criteria, timeout_s):
            sent.update(instructions=instructions, criteria=criteria)
            return super().ask(state, instructions, criteria, timeout_s)

    return J.JevB1Policy(T(), "b1x").decide(snap), sent


def _nulls(x):
    if x is None:
        return True
    if isinstance(x, dict):
        return any(_nulls(v) for v in x.values())
    if isinstance(x, (list, tuple)):
        return any(_nulls(v) for v in x)
    return False


def test_only_b1x_exists_and_the_question_is_one_neutral_sentence():
    assert J.VERSIONS == ("b1x",)
    q = J.INSTRUCTIONS["b1x"]
    assert q == "Which job should the robot do next to keep every machine running for the rest of the shift?"
    assert q.count("?") == 1 and "ship" not in q.lower()
    with pytest.raises(ValueError):
        J.JevB1Policy(MockTransport(), "b1s")


def test_ship_decided_by_code_when_buffer_full_or_m4_blocks_soon():
    d, sent = _sent(_snap_syn(a=("supply", 5, 20), s=("ship", 90, 20, 100.0, 0)))          # buffer full
    assert d["choice"] == "s" and d["source"] == "rule" and d["info"]["rule"] == "X1_buffer_full" and not d["called"] and not sent
    d, sent = _sent(_snap_syn(a=("supply", 5, 30), s=("ship", 90, 20, 35.0, 2)))           # 35 <= 30 + 5
    assert d["choice"] == "s" and d["info"]["rule"] == "X1_m4_block_soon" and not sent
    d, sent = _sent(_snap_syn(a=("supply", 5, 30), s=("ship", 90, 20, 35.1, 2)))           # just above
    assert d["choice"] == "a" and not d["called"] and d["source"] == "single_candidate"


def test_ship_is_removed_and_jev_is_asked_over_the_rest():
    d, sent = _sent(_snap_syn(a=("supply", 5, 30), b=("supply", 9, 30), s=("ship", 90, 20, 200.0, 2)))
    assert d["called"] and set(sent["criteria"]) == {"a", "b"} and sent["instructions"] == J.QUESTION_NEUTRAL
    assert d["choice"] in ("a", "b")
    d, sent = _sent(_snap_syn(a=("supply", 500, 30), s=("ship", 90, 20, 1.0, 2)))           # no other useful candidate
    assert d["choice"] == "a" and not d["called"]
    d, sent = _sent(_snap_syn(s=("ship", 90, 20, 1000.0, 2)))                               # ship only
    assert d["choice"] == "s" and not d["called"] and d["info"]["rule"] == "X2_ship_only"
    d, sent = _sent(_snap_syn(a=("supply", 5, 30), b=("supply", 9, 30)))                    # no ship at all
    assert d["called"] and set(sent["criteria"]) == {"a", "b"}


def test_retry_then_timeout_fallback_to_smallest_slack():
    snap = _snap_syn(a=("supply", 50, 20), b=("supply", 6, 20))
    t = MockTransport(failures=[JevError("timeout", True), JevError("timeout", True)])
    pol = J.JevB1Policy(t, "b1x", max_retries=1, backoff_s=0.0)
    d = pol.decide(snap)
    assert d["source"] == "fallback" and d["choice"] == "b" and pol.calls == 2
    t = MockTransport(failures=[JevError("timeout", True)])
    pol = J.JevB1Policy(t, "b1x", max_retries=1, backoff_s=0.0)
    d = pol.decide(snap)
    assert d["source"] == "mock" and d["info"]["attempts"] == 2 and pol.calls == 2
    assert J.JevB1Policy(MockTransport(picker=lambda c: "zzz"), "b1x").decide(snap)["source"] == "fallback"


def test_snapshot_facts_are_numbers_or_strings_never_null():
    from factory_b import Sim, make_scenario
    sc = make_scenario("B1", "ii", ct_scale=0.6).with_condition(0.10, 60.0, True).with_start("empty")
    sim = Sim(sc, 5)
    snap = J.build_snapshot(sim, sim.candidates(), 1800.0)
    assert not _nulls(snap["state"]) and not _nulls(snap["criteria"])
    assert set(snap["criteria"]) == set(snap["meta"]) == set(snap["order"])
    assert all(i.startswith(("supply_M", "ship", "collect_scrap_M")) for i in snap["criteria"])


def test_recorded_decisions_have_the_expected_shape():
    rec = json.loads(RECORDED.read_text(encoding="utf-8"))
    ds = rec["decisions"]
    assert len(ds) == 133
    for d in ds:
        assert not _nulls(d["candidates"])
        if d["source"] == "jev":
            assert d["chosen"] in d["probabilities"] and d["model"] == "jev-1.14.0"
            assert "ship" not in d["probabilities"]                      # the code never asks Jev about shipping
        if d["source"] == "rule":
            assert d["rule"] == "X1_buffer_full" and d["chosen"] == "ship"


def test_choose_drives_the_factory_loop_with_the_mock():
    from factory_b import Sim, make_scenario, run
    sim = Sim(make_scenario("B1", "ii", ct_scale=0.6).with_condition(0.10, 60.0, True).with_start("empty"), 5)
    pol = J.JevB1Policy(MockTransport(), "b1x")
    run(sim, pol, 600.0)
    assert sim.done > 0 and pol.calls > 0
