"""Replay of the recorded decisions on the calculation model (no API) and the entry script."""
import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import run_b1x_episode as R  # noqa: E402
from policies import jev_b1 as J  # noqa: E402

RECORDED = ROOT / "recorded" / "b1x_seed5_empty.json"


@pytest.fixture(scope="module")
def plan():
    return json.loads(RECORDED.read_text(encoding="utf-8"))["decisions"]


def test_replay_gives_20_finished(plan):
    ep = R.Episode(R.Sim(R.make_sc("empty"), 5), plan=plan)
    ep.run()
    assert ep.stopped is None and ep.sim.done == 20 and len(ep.trace) == 133
    assert ep.sim.defects == 7 and ep.sim.deliveries == 116
    assert [t["chosen"] for t in ep.trace] == [d["chosen"] for d in plan]
    assert [t["t"] for t in ep.trace] == [d["t"] for d in plan]
    assert sum(n for _, n in ep.ship_events) == 20


def test_facts_and_code_rules_reproduce_the_recorded_decisions(plan, monkeypatch):
    """Rebuild every snapshot: the facts equal the recorded ones; the ship rule gives the recorded rule / candidate set."""
    snaps = []
    orig = R.build_snapshot

    def spy(sim, cands, h):
        s = orig(sim, cands, h)
        snaps.append(s)
        return s
    monkeypatch.setattr(R, "build_snapshot", spy)
    ep = R.Episode(R.Sim(R.make_sc("empty"), 5), plan=plan)
    ep.run()
    assert len(snaps) == len(plan)
    for s, d in zip(snaps, plan):
        assert json.loads(json.dumps(s["criteria"])) == d["candidates"]
        rd = J.ship_rule(s)
        if d["source"] == "rule":
            assert rd["choice"] == "ship" and rd["rule"] == d["rule"]
        elif d["source"] == "jev":
            assert rd["choice"] is None and set(rd["ask"]) == set(d["probabilities"])


def test_mock_run_stays_within_the_cap():
    capped = R.CappedTransport(R.MockTransport(picker=R.mock_picker("hash")), 117)
    ep = R.Episode(R.Sim(R.make_sc("empty"), 5), J.JevB1Policy(capped, "b1x", max_retries=1))
    ep.run()
    assert ep.stopped is None and len(ep.trace) > 20 and capped.n <= 117


def test_script_replay_prints_finished_20():
    out = subprocess.run([sys.executable, str(ROOT / "scripts" / "run_b1x_episode.py"), "replay"],
                         capture_output=True, text=True, check=True).stdout
    assert "finished = 20" in out


def test_real_mode_refuses_without_a_key():
    env = {k: v for k, v in os.environ.items() if k != "TYPESAFE_API_KEY"}
    r = subprocess.run([sys.executable, str(ROOT / "scripts" / "run_b1x_episode.py"), "real", "--cap", "5"],
                       capture_output=True, text=True, env=env)
    assert r.returncode != 0 and "not set" in (r.stderr + r.stdout)
