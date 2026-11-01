"""Decision split of B1 (no ROS in this file): message encoding, the policies of the decision node, and the client side
(`RemotePolicy`) that b1_node uses instead of an in-process policy.

Protocol (std_msgs/String JSON, schema_version 1)
  request  : a key `decision_request` inside the /factory/state message, present while a decision is wanted
             {"request_id": "r000012", "time_s": sim time, "horizon_s": episode length, "candidates": [ids],
              "snapshot": {"state", "criteria", "meta", "order"}}      # policies.jev_b1.build_snapshot (facts computed in code)
             The same request stays in every /factory/state message (1 Hz) until it is answered, so a lost message is repeated.
  answer   : /dispatch/decision
             {"kind": "dispatch_decision", "request_id", "time_s", "choice": id | null, "wait": bool, "policy", "source",
              "rule", "info": {...probabilities, model, ... for Jev}, "stats": {replay counters}, "decider_wall_s"}
             wait=true (choice null): the replay's recorded decision is not possible yet (b1_node looks again 0.5 s later).
  ids      : supply_M1..M4 / ship / collect_scrap_M1..M4 (same as policies.jev_b1.cand_id)
Non-finite numbers are sent as +-1e12 (the facts are finite in practice). Numbers are NOT rounded (the b1x rules compare them).

Timing: the task starts at the request time when the answer came within `backdate_s` (0.5) of simulation time (normally a few
ms of wall time); a slower answer starts the task when it arrives (the robot really waited). `answered_t` is always recorded.
Client rules (RemotePolicy): answers whose request_id is not the pending one are ignored (stale / duplicate / mismatch);
an answer whose choice is no longer a candidate is dropped and the request is repeated; no valid answer within
`timeout_s` of wall time -> the timeout fallback (smallest slack, then shortest trip: policies.jev_b1.fallback_choice) is used, logged
(source "fallback_timeout") and counted.
"""
from __future__ import annotations

import math
import time

from policies.jev_b1 import build_snapshot, fallback_choice

from .b1_runner import ReplayPolicy, cand_name

BIG = 1e12
KIND_DECISION = "dispatch_decision"


# ---------------------------------------------------------------------------------------------------------- encoding
def finite(x):
    """Recursively replace inf/nan by +-BIG (JSON without NaN/Infinity tokens). Floats are otherwise untouched."""
    if isinstance(x, float):
        if math.isnan(x):
            return None
        if math.isinf(x):
            return BIG if x > 0 else -BIG
        return x
    if isinstance(x, dict):
        return {str(k): finite(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [finite(v) for v in x]
    return x


def make_request(req_id: str, t: float, horizon_s: float, snapshot: dict) -> dict:
    return {"request_id": req_id, "time_s": t, "horizon_s": horizon_s, "candidates": list(snapshot["order"]),
            "snapshot": finite(snapshot)}


def valid_request(req) -> bool:
    return (isinstance(req, dict) and isinstance(req.get("request_id"), str) and isinstance(req.get("snapshot"), dict)
            and isinstance(req.get("candidates"), list) and isinstance(req.get("time_s"), (int, float)))


def make_answer(req: dict, policy: str, choice, source: str, rule: str, info: dict, *, wait: bool = False,
                stats: dict | None = None, wall_s: float = 0.0) -> dict:
    return {"kind": KIND_DECISION, "request_id": req["request_id"], "time_s": req["time_s"], "choice": choice,
            "wait": wait, "policy": policy, "source": source, "rule": rule, "info": finite(info),
            "stats": stats or {}, "decider_wall_s": round(wall_s, 4)}


def valid_answer(ans) -> bool:
    return (isinstance(ans, dict) and ans.get("kind") == KIND_DECISION and isinstance(ans.get("request_id"), str)
            and isinstance(ans.get("wait"), bool) and (ans.get("choice") is None or isinstance(ans.get("choice"), str)))


# ---------------------------------------------------------------------------------------------------------- deciders
class ReplayDecider:
    """Recorded decisions in order (the same ReplayPolicy as in-process)."""
    name = "replay"

    def __init__(self, plan: list[dict], max_wait_s: float = 20.0) -> None:
        self.rp = ReplayPolicy(plan, max_wait_s)

    def decide(self, req: dict) -> dict:
        t0 = time.perf_counter()
        names = req["candidates"]
        name, info = self.rp.choose_name(names, req["time_s"])
        stats = {"planned": self.rp.plan_len, "used": self.rp.k, "skipped": list(self.rp.skipped)}
        w = time.perf_counter() - t0
        if name is not None:
            return make_answer(req, self.name, name, info.get("source") or "replay", info.get("rule") or "", info, stats=stats, wall_s=w)
        if info["replay"] == "waiting":
            return make_answer(req, self.name, None, "replay", "waiting", info, wait=True, stats=stats, wall_s=w)
        fb = fallback_choice(req["snapshot"], names)             # recorded plan used up
        return make_answer(req, self.name, fb, "fallback_smallest_slack", "plan_exhausted", info, stats=stats, wall_s=w)


class JevDecider:
    """policies.jev_b1.JevB1Policy on the snapshot. The transport is injected (MockTransport in tests / dry runs; the SDK
    transport only when the user starts it on purpose). `decide` may block for the API call: run it in a worker thread."""

    def __init__(self, policy, version: str = "b1x") -> None:
        self.policy, self.version = policy, version
        self.name = policy.name

    def decide(self, req: dict) -> dict:
        t0 = time.perf_counter()
        d = self.policy.decide(req["snapshot"])
        info = dict(d["info"])
        info["called"] = d["called"]
        return make_answer(req, self.name, d["choice"], d["source"], str(info.get("rule", "")), info, wall_s=time.perf_counter() - t0)


# ---------------------------------------------------------------------------------------------------------- client
def _pct(xs: list[float], q: float) -> float:
    s = sorted(xs)
    return s[min(len(s) - 1, int(math.ceil(q * len(s))) - 1)]


class RemotePolicy:
    """Drop-in for B1Runner's policy: `choose(sim, cands, t, h)` -> (cand, info) | (None, info).
    info {"pending": True} = no answer yet (the runner stays in DECIDE and asks again at its next tick);
    info {"replay": "waiting"} = the decider says wait (the runner looks again 0.5 s later), as with the in-process replay.
    `send(request)` publishes the request; `on_message(answer)` is called for every /dispatch/decision message."""

    def __init__(self, send, timeout_s: float = 8.0, mono=time.monotonic, fallback=fallback_choice,
                 backdate_s: float = 0.5) -> None:
        self.send, self.timeout_s, self.mono, self.fallback = send, timeout_s, mono, fallback
        self.backdate_s = backdate_s       # an answer that took <= this many SIM seconds counts as made at the request time
        self.policy_name = "replay"
        self.plan_len, self.k, self.skipped = 0, 0, []
        self.rtts: list[float] = []
        self.timeouts: list[dict] = []
        self.n_requests = self.n_wait = self.n_stale = self.n_bad = self.n_dropped_choice = 0
        self._seq = 0
        self._pending: dict | None = None
        self._answer: dict | None = None
        self._recv_t = 0.0

    # -- bookkeeping seen from outside
    @property
    def pending_request(self) -> dict | None:
        return self._pending["req"] if self._pending else None

    def on_message(self, ans: dict) -> bool:
        """True if the answer was accepted for the pending request; stale / duplicate / mismatching ids are ignored."""
        if not valid_answer(ans):
            self.n_bad += 1
            return False
        p = self._pending
        if p is None or ans["request_id"] != p["req"]["request_id"] or self._answer is not None:
            self.n_stale += 1
            return False
        self._answer, self._recv_t = ans, self.mono()
        return True

    def _new_request(self, sim, cands, t: float, h: float) -> None:
        self._seq += 1
        snap = build_snapshot(sim, cands, h)
        req = make_request(f"r{self._seq:06d}", t, h, snap)
        self._pending = {"req": req, "sent": self.mono(), "snap": req["snapshot"]}
        self._answer = None
        self.n_requests += 1
        self.send(req)

    def choose(self, sim, cands, t: float, h: float):
        by = {cand_name(c): c for c in cands}
        for _ in range(2):
            if self._pending is None:
                self._new_request(sim, cands, t, h)
            p = self._pending
            ans = self._answer
            if ans is None:
                if self.mono() - p["sent"] > self.timeout_s:
                    return self._timeout(by)
                return None, {"pending": True}
            self._pending, self._answer = None, None
            rtt = self._recv_t - p["sent"]
            self.rtts.append(rtt)
            st = ans.get("stats") or {}
            if st:
                self.plan_len, self.k, self.skipped = st.get("planned", 0), st.get("used", 0), list(st.get("skipped", []))
            self.policy_name = ans.get("policy") or self.policy_name
            base = dict(ans.get("info") or {})
            base.setdefault("source", ans.get("source"))
            base.setdefault("rule", ans.get("rule"))
            if ans["wait"]:
                self.n_wait += 1
                return None, base
            if ans["choice"] in by:
                base.update({"request_id": ans["request_id"], "request_t": p["req"]["time_s"], "answered_t": round(t, 3), "rtt_ms": round(rtt * 1000, 1), "decider": "node"})
                if t - p["req"]["time_s"] <= self.backdate_s:
                    base["decision_t"] = p["req"]["time_s"]       # the runner starts the task at the request time
                return by[ans["choice"]], base
            self.n_dropped_choice += 1        # not a candidate (any more): ask again with the current state
        return None, {"pending": True}

    def _timeout(self, by: dict):
        p, self._pending, self._answer = self._pending, None, None
        ids = list(p["req"]["candidates"])
        ids = [i for i in ids if i in by] or list(by)
        choice = self.fallback(p["snap"], ids) if all(i in p["snap"]["meta"] for i in ids) else ids[0]
        wait = self.mono() - p["sent"]
        self.timeouts.append({"request_id": p["req"]["request_id"], "time_s": p["req"]["time_s"], "waited_s": round(wait, 2), "fallback": choice})
        return by[choice], {"source": "fallback_timeout", "rule": "smallest_slack_fallback", "request_id": p["req"]["request_id"],
                            "fallback_reason": f"no_valid_answer_in_{self.timeout_s:g}s", "decider": "node"}

    def stats(self) -> dict:
        r = self.rtts
        d = {"requests": self.n_requests, "answers": len(r), "waits": self.n_wait, "timeouts": len(self.timeouts),
             "timeout_list": self.timeouts, "stale_ignored": self.n_stale, "invalid_ignored": self.n_bad,
             "choice_not_candidate": self.n_dropped_choice, "timeout_s": self.timeout_s}
        if r:
            ms = [x * 1000 for x in r]
            d.update({"rtt_ms_mean": round(sum(ms) / len(ms), 1), "rtt_ms_median": round(_pct(ms, 0.5), 1),
                      "rtt_ms_p95": round(_pct(ms, 0.95), 1), "rtt_ms_max": round(max(ms), 1)})
        return d
