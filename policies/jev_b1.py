"""Jev as the dispatch policy of B1 (one AMR: supply M1..M4, ship finished goods, collect NG scrap). Version "b1x".

Style (typesafe-sdk 0.7.2): `instructions` is one short question for one judgement; what each option is goes into
`criteria` (one object per candidate); arithmetic belongs in code. So every timing fact is computed here, every field
is a number or a short string, and nothing is null.

What is sent
  question  QUESTION_NEUTRAL, one sentence. It names no priority and does not mention shipping as the goal.
  state     machines, finished buffer, time left (see build_snapshot).
  criteria  one object of facts per candidate (arrival time, lock wait, parts in stock, spare time, ...).

b1x: the code decides the shipping timing, Jev decides the rest. If "ship" is a candidate:
  X1 choose ship (no ask) if finished_buffer_free_slots == 0
     or m4_blocked_in_s <= (shortest robot_free_in_s among the OTHER useful candidates) + SHIP_MARGIN_S.
     "Useful" = slack < USELESS_SLACK_S. With no other useful candidate the second clause is false.
  X2 if ship is the only candidate, choose it.
  X3 otherwise remove ship from the candidates and ask Jev over the rest.
If there is no ship candidate, Jev is asked over all candidates. A single remaining candidate is chosen without asking.
Only real asks (>= 2 candidates sent) are API calls. The thresholds below were fixed before the real run.

slack = (time until the target would stop without this job) - (time until the job takes effect); both from code.
fallback_choice (smallest slack) is used only when a Jev call fails or times out.
"""
from __future__ import annotations

import time

from adapters.jev import JevError, JevTransport
from factory_b.engine import DONE, EJECT, NGWAIT, RUN, Cand, Sim

VERSIONS = ("b1x",)
QUESTION_NEUTRAL = "Which job should the robot do next to keep every machine running for the rest of the shift?"
INSTRUCTIONS = {"b1x": QUESTION_NEUTRAL}

USELESS_SLACK_S = 150.0  # a non-shipping job with at least this much spare time is not "useful" now
SHIP_MARGIN_S = 5.0      # X1: margin added to the shortest other useful trip

SCRAP_ID = "collect_scrap_M{}"


def cand_id(c: Cand) -> str:
    j = c.job
    if j[0] == "S":
        return f"supply_M{j[1][0][0] + 1}"
    if j[0] == "SHIP":
        return "ship"
    return SCRAP_ID.format(j[1] + 1)


def _r(x: float) -> float:
    return round(float(x), 1)


def _status(sim: Sim, i: int) -> tuple[str, str]:
    """(short code, sentence) of machine i at sim.now."""
    sc, ph = sim.scn, sim.ph[i]
    if ph == RUN:
        return "RUNNING", f"running, cycle ends in {sim.busy[i] - sim.now:.0f} s"
    if ph == EJECT:
        rem = sim.busy[i] - sim.now
        if sc.lock:
            return "NG_HANDLING", f"NG handling, input locked for another {rem:.0f} s"
        return "NG_HANDLING", f"NG handling, {rem:.0f} s left"
    if ph == NGWAIT:
        return "STOPPED_SCRAP_FULL", "stopped: scrap bin full, cannot restart until the scrap is collected"
    if ph == DONE:
        return "BLOCKED", ("blocked: finished piece cannot be put down, finished buffer full" if i == sc.n - 1
                           else "blocked: finished piece cannot move on, next slot full")
    if not sim.has_parts(i):
        return "NO_PARTS", "idle: not enough parts for one cycle"
    return "WAITING_WORKPIECE", f"idle: waiting for a workpiece from M{i}"


def _lock_left(sim: Sim, i: int) -> float:
    return max(0.0, sim.busy[i] - sim.now) if (sim.scn.lock and sim.ph[i] == EJECT) else 0.0


def _m4_block_in(sim: Sim) -> float:
    """Seconds until the last machine is blocked by a full finished buffer (0 if it is blocked now)."""
    sc = sim.scn
    last = sc.n - 1
    room = sc.out_cap - sim.out_bin
    ct = sc.ct[last]
    ph = sim.ph[last]
    if ph == DONE:
        return 0.0
    if ph == RUN:
        return sim.busy[last] - sim.now + room * ct
    if ph == EJECT:
        return sim.busy[last] - sim.now + ct + room * ct
    return ct + room * ct


def build_snapshot(sim: Sim, cands: list, h: float) -> dict:
    """Everything that is sent (state, criteria) plus `meta` (unrounded numbers for the ship rule; never sent)."""
    sc, tr = sim.scn, sim.scn.transport
    ids = sc.ids
    last = sc.n - 1
    machines = {}
    for i, mid in enumerate(ids):
        code, _ = _status(sim, i)
        machines[mid] = {
            "status": code,
            "cycle_ends_in_s": _r(sim.busy[i] - sim.now) if sim.ph[i] in (RUN, EJECT) else 0.0,
            "input_locked_for_s": _r(_lock_left(sim, i)),
            "parts_in_stock": sim.buf[i][0],
            "parts_per_cycle": sc.bs[i][0],
            "scrap_bin": "full" if sim.ng_bin[i] else "empty",
        }
    state = {
        "sim_time_s": _r(sim.now),
        "time_left_s": _r(h - sim.now),
        "shipped_so_far": sim.shipped,
        "robot": "at the store, free",
        "line_order": list(ids),
        "finished_buffer": {"pieces": sim.out_bin, "capacity": sc.out_cap,
                            "free_slots": sc.out_cap - sim.out_bin,
                            "m4_blocked_in_s": _r(_m4_block_in(sim))},
        "machines": machines,
    }
    criteria, meta = {}, {}
    for c in cands:
        cid, j = cand_id(c), c.job
        if j[0] == "S":
            i = j[1][0][0]
            arrive = tr.load + tr.travel("store", ids[i])
            lock = _lock_left(sim, i)
            wait = max(0.0, lock - arrive)
            effect = arrive + wait + tr.unload
            stops = sim._starve_off(i, 0)
            code, sentence = _status(sim, i)
            d = {
                "job": f"bring {sc.bs[i][0]} part(s) to {ids[i]}",
                "target": ids[i],
                "target_state": sentence,
                "arrives_in_s": _r(arrive),
                "lock_remaining_now_s": _r(lock),
                "lock_wait_on_arrival_s": _r(wait),
                "parts_delivered_in_s": _r(effect),
                "robot_free_in_s": _r(c.total),
                "parts_in_stock": sim.buf[i][0],
                "parts_per_cycle": sc.bs[i][0],
                "cycles_of_stock": sim.buf[i][0] // sc.bs[i][0],
                "target_stops_without_this_in_s": _r(stops),
                "spare_time_s": _r(max(0.0, c.slack)),
                "target_idle_if_chosen_s": _r(max(0.0, -c.slack)),
            }
            kind = "supply"
        elif j[0] == "SHIP":
            i = last
            arrive = tr.travel("store", ids[i])
            pick = arrive + tr.load
            d = {
                "job": f"carry {sim.out_bin} finished piece(s) from {ids[i]} to the shipping area",
                "target": ids[i],
                "target_state": _status(sim, i)[1],
                "arrives_in_s": _r(arrive),
                "lock_remaining_now_s": 0.0,
                "lock_wait_on_arrival_s": 0.0,
                "pieces_shipped_by_this_job": sim.out_bin,
                "buffer_emptied_in_s": _r(pick),
                "robot_free_in_s": _r(c.total),
                "finished_buffer_free_slots": sc.out_cap - sim.out_bin,
                "target_stops_without_this_in_s": _r(_m4_block_in(sim)),
                "spare_time_s": _r(max(0.0, c.slack)),
                "target_idle_if_chosen_s": _r(max(0.0, -c.slack)),
            }
            kind = "ship"
        else:
            i = j[1]
            arrive = tr.travel("store", ids[i])
            pick = arrive + tr.load
            cannot = sim.ph[i] == NGWAIT
            d = {
                "job": f"carry the scrap piece from {ids[i]} to the drop",
                "target": ids[i],
                "target_state": _status(sim, i)[1],
                "arrives_in_s": _r(arrive),
                "lock_remaining_now_s": _r(_lock_left(sim, i)),
                "lock_wait_on_arrival_s": 0.0,
                "scrap_collected_in_s": _r(pick),
                "robot_free_in_s": _r(c.total),
                "scrap_bin": "full (capacity 1)",
                "target_cannot_restart_until_collected": "yes" if cannot else "no",
                "target_stops_if_next_piece_is_also_defective_in_s": _r(max(0.0, c.slack + pick)),
                "spare_time_s": _r(max(0.0, c.slack)),
                "target_idle_if_chosen_s": _r(max(0.0, -c.slack)),
            }
            kind = "collect"
        criteria[cid] = d
        meta[cid] = {"kind": kind, "slack": c.slack, "robot_free": c.total}
    return {"state": state, "criteria": criteria, "meta": meta, "order": list(criteria)}


# ---- b1x ship rule ---------------------------------------------------------------------------------------------
def _ship_facts(snap: dict, sid: str) -> tuple:
    """(finished_buffer_free_slots, m4_blocked_in_s) exactly as sent to Jev (rounded to 0.1 s); synthetic test
    snapshots may carry them in meta instead."""
    m = snap["meta"][sid]
    if "free_slots" in m:
        return m["free_slots"], m["m4_block"]
    fb = snap["state"]["finished_buffer"]
    return fb["free_slots"], fb["m4_blocked_in_s"]


def ship_rule(snap: dict) -> dict:
    """b1x: {"choice": id|None, "rule": str, "ask": [ids]}. Code decides the ship timing."""
    meta, order = snap["meta"], list(snap["order"])
    ship = [i for i in order if meta[i]["kind"] == "ship"]
    if not ship:
        return {"choice": None, "rule": "X_no_ship", "ask": order}
    sid = ship[0]
    others = [i for i in order if i != sid]
    if not others:
        return {"choice": sid, "rule": "X2_ship_only", "ask": [sid]}
    useful = [meta[i]["robot_free"] for i in others if meta[i]["slack"] < USELESS_SLACK_S]
    free_slots, m4_block = _ship_facts(snap, sid)
    if free_slots == 0:
        return {"choice": sid, "rule": "X1_buffer_full", "ask": others}
    if useful and m4_block <= min(useful) + SHIP_MARGIN_S:
        return {"choice": sid, "rule": "X1_m4_block_soon", "ask": others}
    return {"choice": None, "rule": "X3_ship_removed", "ask": others}


def build_request(version: str, snap: dict) -> dict:
    """Exactly what is sent to Jev for this snapshot, or the code decision if the ship rule decides by itself."""
    if version not in VERSIONS:
        raise ValueError(f"unknown version {version!r}")
    rd = ship_rule(snap)
    if rd["choice"] is not None:
        return {"version": version, "delegated": False, "rule": rd["rule"], "choice": rd["choice"],
                "instructions": None, "criteria": None, "state": None}
    ask = rd["ask"]
    return {"version": version, "delegated": True, "rule": rd["rule"], "choice": None,
            "instructions": INSTRUCTIONS[version], "criteria": {i: snap["criteria"][i] for i in ask},
            "state": snap["state"]}


def fallback_choice(snap: dict, ask: list) -> str:
    """Timeout/error fallback (not a compared method): smallest slack, then shortest trip, then candidate order."""
    m = snap["meta"]
    return min(ask, key=lambda i: (m[i]["slack"], m[i]["robot_free"], snap["order"].index(i)))


class JevB1Policy:
    """decide(snapshot) for the scene runner; choose(sim, cands, h) for the factory_b run loop."""

    def __init__(self, transport: JevTransport, version: str = "b1x", *, timeout_s: float = 10.0,
                 max_retries: int = 2, backoff_s: float = 0.5) -> None:
        if version not in VERSIONS:
            raise ValueError(f"unknown version {version!r}")
        self.transport, self.version = transport, version
        self.timeout_s, self.max_retries, self.backoff_s = timeout_s, max_retries, backoff_s
        self.calls = 0  # transport attempts incl. retries
        self.name = ("jev_" if transport.real_api else "mock_jev_") + version

    def decide(self, snap: dict) -> dict:
        req = build_request(self.version, snap)
        base = {"question_version": self.version, "real_api": self.transport.real_api, "rule": req["rule"]}
        if not req["delegated"]:
            return {"choice": req["choice"], "source": "rule", "called": False, "info": base}
        criteria = req["criteria"]
        if len(criteria) == 1:
            return {"choice": next(iter(criteria)), "source": "single_candidate", "called": False, "info": base}
        errors: list[str] = []
        t_start = time.perf_counter()
        for attempt in range(1, self.max_retries + 2):
            self.calls += 1
            try:
                ans = self.transport.ask(req["state"], req["instructions"], criteria, self.timeout_s)
            except JevError as e:
                errors.append(e.reason)
                if e.retryable and attempt <= self.max_retries:
                    time.sleep(self.backoff_s * attempt)
                    continue
                break
            if ans.choice not in criteria:
                errors.append("choice_not_in_candidates")
                break
            return {"choice": ans.choice, "source": "jev" if ans.real_api else "mock", "called": True,
                    "info": {**base, "model": ans.model, "confidence": ans.confidence,
                             "probabilities": ans.probabilities, "latency_s": round(ans.latency_s, 4),
                             "attempts": attempt, "errors": errors, "input_tokens": ans.input_tokens,
                             "output_tokens": ans.output_tokens,
                             "total_wall_s": round(time.perf_counter() - t_start, 4)}}
        return {"choice": fallback_choice(snap, list(criteria)), "source": "fallback", "called": True,
                "info": {**base, "fallback_reason": errors[-1] if errors else "unknown", "errors": errors,
                         "attempts": len(errors), "total_wall_s": round(time.perf_counter() - t_start, 4)}}

    def choose(self, sim: Sim, cands: list, h: float) -> Cand:
        snap = build_snapshot(sim, cands, h)
        d = self.decide(snap)
        by = {cand_id(c): c for c in cands}
        return by[d["choice"]]
