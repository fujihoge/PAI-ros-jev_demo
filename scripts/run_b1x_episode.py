"""One 1800 s episode of the B1 calculation model, with the decisions taken from one of three sources.

  replay : the decisions recorded in recorded/b1x_seed5_empty.json, in order. No API, no key. Prints finished = 20.
  mock   : the b1x policy with a deterministic stand-in for Jev (NOT the Jev API; the result is only a smoke test).
  real   : the b1x policy with the Jev API through typesafe-sdk. Needs the environment variable TYPESAFE_API_KEY
           (only its presence is checked, it is never printed) and a hard limit --cap on all API attempts.

Usage (from the repository root):
  python scripts/run_b1x_episode.py replay
  python scripts/run_b1x_episode.py mock --cap 150
  python scripts/run_b1x_episode.py real --cap 117 --out out/my_run

Setting (fixed): seed 5, empty stock, cycle times x0.6, defect rate 10 %, NG stop 60 s, input lock on,
handling 2 s, overhead 2 s, short layout. Output (with --out): metrics.json and trace.json.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from adapters import SdkTransport  # noqa: E402
from adapters.jev import JevError, MockTransport  # noqa: E402
from factory_b import Sim, make_scenario  # noqa: E402
from policies.jev_b1 import JevB1Policy, build_snapshot, cand_id  # noqa: E402

H = 1800.0
MAX_CAP = 117
CT_SCALE, RATE, STOP_S, LOCK = 0.6, 0.10, 60.0, True
VERSION = "b1x"
RECORDED = ROOT / "recorded" / "b1x_seed5_empty.json"
FATAL = ("http_429", "auth_error", "api_error_", "api_key_not_set", "unexpected_answer_type")


class CapExceeded(Exception):
    pass


class AbortRun(Exception):
    pass


class CappedTransport:
    """Counts every attempt (retries included), enforces a hard cap, logs each outcome, aborts on fatal errors."""

    def __init__(self, inner, cap: int) -> None:
        self.inner, self.cap = inner, cap
        self.real_api = inner.real_api
        self.attempts: list[dict] = []
        self.aborted: str | None = None

    @property
    def n(self) -> int:
        return len(self.attempts)

    def ask(self, state, instructions, criteria, timeout_s):
        if self.aborted:
            raise AbortRun(self.aborted)
        if self.n >= self.cap:
            raise CapExceeded(f"cap {self.cap} reached")
        t0 = time.perf_counter()
        try:
            ans = self.inner.ask(state, instructions, criteria, timeout_s)
        except JevError as e:
            self.attempts.append({"n": self.n + 1, "ok": False, "error": e.reason,
                                  "wall_s": round(time.perf_counter() - t0, 4)})
            if e.reason.startswith(FATAL):
                self.aborted = e.reason
            raise
        self.attempts.append({"n": self.n + 1, "ok": True, "latency_s": round(ans.latency_s, 4),
                              "wall_s": round(time.perf_counter() - t0, 4), "model": ans.model})
        return ans


def make_sc(stock: str = "empty"):
    return make_scenario("B1", "ii", ct_scale=CT_SCALE).with_condition(RATE, STOP_S, LOCK).with_start(stock)


class Episode:
    """Runs a decision source on Sim and records every decision (also forced ones) and the shipping events.
    `plan` (list of recorded decisions) replaces the policy: decision k takes plan[k]["chosen"]."""

    def __init__(self, sim: Sim, policy=None, plan: list | None = None, h: float = H) -> None:
        self.sim, self.policy, self.plan, self.h = sim, policy, plan, h
        self.ship_events: list[tuple[float, int]] = []
        self.trace: list[dict] = []
        self.stopped: str | None = None

    def run(self) -> None:
        sim, h, tr = self.sim, self.h, self.sim.scn.transport
        try:
            while sim.now < h:
                sim.advance_to(sim.now)
                cands = sim.candidates()
                if not cands:
                    nxt = sim.next_event()
                    if nxt is None or nxt <= sim.now:
                        break
                    sim.now = nxt
                    continue
                t0 = sim.now
                snap = build_snapshot(sim, cands, h)
                by = {cand_id(x): x for x in cands}
                k = len(self.trace)
                if self.plan is not None:
                    want = self.plan[k]["chosen"] if k < len(self.plan) else None
                    if want not in by:
                        raise AbortRun(f"recorded decision {k + 1} ({want!r}) is not a candidate at t={t0:.1f}")
                    c, rec = by[want], {"source": self.plan[k].get("source"), "rule": self.plan[k].get("rule")}
                elif len(cands) == 1:
                    c, rec = cands[0], {"source": "forced_single_candidate"}
                else:
                    d = self.policy.decide(snap)
                    c = by[d["choice"]]
                    rec = {"source": d["source"], "rule": d["info"].get("rule"), "called": d["called"],
                           **{x: d["info"].get(x) for x in ("model", "probabilities", "confidence", "attempts", "errors",
                                                            "fallback_reason", "latency_s") if x in d["info"]}}
                self.trace.append({"idx": k + 1, "t": round(t0, 1), "chosen": cand_id(c), **rec,
                                   "n_candidates": len(cands), "shipped_so_far": sim.shipped,
                                   "finished_buffer": snap["state"]["finished_buffer"], "candidates": snap["criteria"]})
                s0 = sim.shipped
                sim.execute(c.job, h)
                if sim.shipped > s0:
                    last = sim.scn.n - 1
                    ids = sim.scn.ids
                    te = t0 + tr.travel("store", ids[last]) + tr.load + tr.travel(ids[last], "dock") + tr.unload
                    self.ship_events.append((round(te, 1), sim.shipped - s0))
            sim.advance_to(h)
        except (CapExceeded, AbortRun) as e:
            self.stopped = f"{type(e).__name__}: {e} (at t={sim.now:.1f} s, decision {len(self.trace) + 1})"


def mock_picker(kind: str):
    def first(crit):
        return sorted(crit)[0]

    def hashp(crit):
        ks = sorted(crit)
        return ks[len(json.dumps(crit, sort_keys=True)) % len(ks)]
    return {"first": first, "hash": hashp}[kind]


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mode", choices=["replay", "mock", "real"])
    ap.add_argument("--cap", type=int, default=MAX_CAP, help=f"hard limit on API attempts, retries included (mock/real, 1..{MAX_CAP})")
    ap.add_argument("--seed", type=int, default=5)
    ap.add_argument("--stock", choices=["empty", "std"], default="empty")
    ap.add_argument("--recorded", default=str(RECORDED), help="replay: recorded decisions (json with a 'decisions' list)")
    ap.add_argument("--mock-picker", default="hash", choices=["first", "hash"])
    ap.add_argument("--out", default=None, help="folder for metrics.json and trace.json (not written if omitted)")
    a = ap.parse_args()
    if not 1 <= a.cap <= MAX_CAP:
        sys.exit(f"--cap must be 1..{MAX_CAP}")

    sim = Sim(make_sc(a.stock), a.seed)
    capped = None
    if a.mode == "replay":
        plan = json.loads(Path(a.recorded).read_text(encoding="utf-8"))["decisions"]
        ep = Episode(sim, plan=plan)
    else:
        if a.mode == "real":
            if not os.environ.get("TYPESAFE_API_KEY"):
                sys.exit("TYPESAFE_API_KEY is not set: stop")
            inner = SdkTransport(model="jev-latest")
        else:
            inner = MockTransport(picker=mock_picker(a.mock_picker))
        capped = CappedTransport(inner, a.cap)
        ep = Episode(sim, JevB1Policy(capped, VERSION, max_retries=1))
    ep.run()

    by_source: dict = {}
    for t in ep.trace:
        k = t["source"] if t["source"] != "rule" else "rule:" + str(t.get("rule"))
        by_source[k] = by_source.get(k, 0) + 1
    metrics = {"mode": a.mode, "version": VERSION, "seed": a.seed, "stock": a.stock,
               "setting": {"ct_scale": CT_SCALE, "defect_rate": RATE, "ng_stop_s": STOP_S, "input_lock": LOCK,
                           "handling_s": 2.0, "overhead_s": 2.0, "layout": "short"},
               "finished": sim.done, "defects": sim.defects, "deliveries": sim.deliveries,
               "decisions": len(ep.trace), "by_source": by_source,
               "api_attempts": capped.n if capped else 0, "stopped": ep.stopped or (capped.aborted if capped else None)}
    print(json.dumps(metrics, ensure_ascii=False))
    print(f"finished = {sim.done}  (mode {a.mode}; {'recorded decisions, no API' if a.mode == 'replay' else 'NOT a replay'})")
    if a.out:
        out = Path(a.out)
        out.mkdir(parents=True, exist_ok=True)
        (out / "metrics.json").write_text(json.dumps(metrics, ensure_ascii=False, indent=1), encoding="utf-8")
        (out / "trace.json").write_text(json.dumps({"ship_events": ep.ship_events,
                                                    "attempts": capped.attempts if capped else [],
                                                    "decisions": ep.trace}, ensure_ascii=False, indent=1), encoding="utf-8")
        print("out:", out)


if __name__ == "__main__":
    main()
