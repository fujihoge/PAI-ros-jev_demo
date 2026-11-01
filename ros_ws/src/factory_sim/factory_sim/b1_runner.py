"""B1 episode runner (task B1: one AMR, supply M1..M4 / ship / collect scrap), driven by an external clock. No ROS.

The line is the factory_b engine (`factory_b.engine.Sim`, the same model as the calculation experiments). The runner keeps
the engine clock equal to the simulation time `t` the node gives it, and moves the robot with *real* navigation legs:
  - supply Mi : load (handling s) at the store -> drive to Mi -> (input lock: wait on the pad) -> unload -> drive back
  - ship      : drive to M4 -> take the finished pieces (handling s) -> drive to the dock -> unload -> drive back
  - scrap Mi  : drive to Mi -> take the scrap (handling s) -> drive to the dock -> unload -> drive back
Engine events (parts into the stock, pieces out of the finished buffer, shipped count) are applied at the moment the
robot really finishes the step, so Gazebo timing differences show up as differences in the result.

Policies: `ReplayPolicy` takes the decisions of a recorded episode (trace.json "decisions") in order. A recorded
decision that cannot be done yet is waited for (at most `max_wait_s`); after that it is skipped and counted.
With `pace=True` a drive is not reported as finished before its planned duration (engine Transport.travel); this keeps
a fast robot on the calculation model's timetable and only lets a slow robot drift.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from factory_b.engine import DONE, EJECT, IDLE, NGWAIT, RUN, Cand, Sim

from .nav_types import NavGoal, NavObs

EPS = 1e-9
STAGE_PUBLIC = {"DECIDE": "deciding", "WAIT": "deciding", "HANDLE_LOAD": "loading", "HANDLE_UNLOAD": "unloading",
                "PAD_WAIT": "waiting_pad", "GO_OUT": "to_station", "GO_DOCK": "to_station", "RETURN": "returning"}


class ObservedSim(Sim):
    """The engine, unchanged, plus a log of two kinds of events for the visual-only workpiece animation (b1_flow):
    ('h', i) = machine i handed a good part over; ('c', j) = machine j started a cycle. Behaviour and numbers are those of
    Sim (every override calls super()). Clones are plain Sim objects, so candidate look-aheads log nothing."""

    def __init__(self, *a, **kw) -> None:
        self.flow_log: list[tuple[str, int]] = []
        super().__init__(*a, **kw)

    def _handoff(self, i: int) -> bool:
        ok = super()._handoff(i)
        if ok:
            self.flow_log.append(("h", i))
        return ok

    def _settle(self, t: float) -> None:
        before = self.cycles[:]
        super()._settle(t)
        for j, (b, a) in enumerate(zip(before, self.cycles)):
            if a > b:
                self.flow_log.append(("c", j))


def cand_name(c: Cand) -> str:
    """Same ids as policies.jev_b1.cand_id: supply_M1 / ship / collect_scrap_M3."""
    j = c.job
    if j[0] == "S":
        return f"supply_M{j[1][0][0] + 1}"
    if j[0] == "SHIP":
        return "ship"
    return f"collect_scrap_M{j[1] + 1}"


def machine_lamp_state(sim: Sim, i: int) -> str:
    """factory_sim.colors state name of stage i (PROCESSING / STARVED / WAITING_UPSTREAM / BLOCKED / NG_EJECTING)."""
    ph = sim.ph[i]
    if ph == RUN:
        return "PROCESSING"
    if ph == EJECT:
        return "NG_EJECTING"
    if ph in (DONE, NGWAIT):
        return "BLOCKED"
    return "STARVED" if not sim.has_parts(i) else "WAITING_UPSTREAM"


class ReplayPolicy:
    """Recorded decisions in order. `plan` = trace.json "decisions" (idx, t, chosen, source, rule ...)."""

    def __init__(self, plan: list[dict], max_wait_s: float = 20.0) -> None:
        self.plan, self.k, self.max_wait_s = plan, 0, max_wait_s
        self.skipped: list[dict] = []
        self._since: float | None = None

    @property
    def plan_len(self) -> int:
        return len(self.plan)

    policy_name = "replay"

    def choose_name(self, names: list[str], t: float):
        """Pure form of `choose` (works on candidate names; used by the decision node).
        Returns (name, info) | (None, info) with info["replay"] == "waiting" (recorded decision not possible yet)
        or "plan_exhausted" (the caller uses its fallback rule)."""
        while self.k < len(self.plan):
            rec = self.plan[self.k]
            if rec["chosen"] in names:
                waited = 0.0 if self._since is None else t - self._since
                self._since = None
                self.k += 1
                return rec["chosen"], {"source": rec.get("source"), "rule": rec.get("rule"), "plan_idx": rec["idx"],
                                       "plan_t": rec["t"], "replay": "recorded", "waited_s": round(waited, 2)}
            if self._since is None:
                self._since = t
            if t - self._since < self.max_wait_s:
                return None, {"replay": "waiting", "plan_idx": rec["idx"], "wanted": rec["chosen"]}
            self.skipped.append({"plan_idx": rec["idx"], "wanted": rec["chosen"], "t": round(t, 1)})
            self._since = None
            self.k += 1
        return None, {"source": "fallback_smallest_slack", "replay": "plan_exhausted"}

    def choose(self, sim: Sim, cands: list[Cand], t: float, h: float):
        """Returns (cand, info). None = wait (the recorded decision is not possible yet)."""
        by = {cand_name(c): c for c in cands}
        name, info = self.choose_name(list(by), t)
        if name is not None:
            return by[name], info
        if info["replay"] == "waiting":
            return None, info
        c = min(cands, key=lambda x: (x.slack, x.total, x.job))      # plan used up: smallest slack, then shortest trip
        return c, info


@dataclass
class Step:
    kind: str                  # "go" | "handle"
    to: str = ""               # go: target name ("M1".."M4" | "dock" | "store")
    secs: float = 0.0          # handle: duration
    effect: str = ""           # handle: "supply" | "pick_ship" | "pick_scrap" | "drop_ship" | "drop_scrap"
    stage: str = ""            # public stage name key of STAGE_PUBLIC


class B1Runner:
    def __init__(self, scn, seed: int, policy: ReplayPolicy, duration_s: float = 1800.0, pace: bool = True,
                 timeout_factor: float = 3.0, timeout_floor_s: float = 10.0) -> None:
        self.scn, self.seed, self.policy, self.duration, self.pace = scn, seed, policy, duration_s, pace
        self.timeout_factor, self.timeout_floor_s = timeout_factor, timeout_floor_s
        self.sim = ObservedSim(scn, seed)
        self.tr = scn.transport
        self.ids = list(scn.ids)
        self.events: list[tuple[str, dict]] = []
        self.legs: list[dict] = []
        self.tasks: list[dict] = []
        self.decisions: list[dict] = []
        self.stage = "DECIDE"
        self.t = 0.0
        self.nav_goal: NavGoal | None = None
        self._goal_seq = 0
        self.task: dict | None = None
        self.steps: list[Step] = []
        self.cur: Step | None = None
        self.step_t0 = 0.0
        self.wait_until = math.inf
        self.pad_since: float | None = None
        self.cur_station, self.cur_k, self.at = 0, 0, "store"
        self.carry = ("", 0)            # (kind, count): "" | "part" | "ship" | "scrap"
        self.dock_ship = 0              # pieces last dropped at the dock (display)
        self.dock_scrap = 0
        self.lock_wait_s = 0.0
        self.lock_wait_count = 0
        self.travel_s = self.handle_s = 0.0
        self.nav_failures = 0
        self.n_task = 0
        self.ended = False
        self.aborted: str | None = None
        self._retries = 0
        self.final_metrics: dict | None = None

    # ------------------------------------------------------------------ helpers
    def xy(self, name: str) -> tuple:
        return tuple(self.tr.positions[name])

    def _planned(self, a: str, b: str) -> float:
        return self.tr.travel(a, b)

    def _emit(self, kind: str, body: dict) -> None:
        self.events.append((kind, body))

    def drain(self) -> list[tuple[str, dict]]:
        ev, self.events = self.events, []
        return ev

    def drain_flow(self) -> list[tuple[str, int]]:
        """Handover / cycle-start events since the last call (for b1_flow.FlowTracker; visual only)."""
        ev, self.sim.flow_log = self.sim.flow_log, []
        return ev

    def _stage_public(self) -> str:
        return STAGE_PUBLIC[self.stage]

    # ------------------------------------------------------------------ clock
    def update(self, t: float, nav: NavObs) -> None:
        if self.ended:
            return
        t = max(min(t, self.duration), self.sim.t)
        self.t = t
        self.sim.advance_to(t)
        if t >= self.duration - EPS:
            self._finish()
            return
        for _ in range(8):
            if not self._step(t, nav) or self.ended:
                break

    def _step(self, t: float, nav: NavObs) -> bool:
        s = self.stage
        if s == "DECIDE":
            return self._decide(t)
        if s == "WAIT":
            if t + EPS >= self.wait_until:
                self.stage = "DECIDE"
                return True
            return False
        if s in ("GO_OUT", "GO_DOCK", "RETURN"):
            return self._monitor_leg(t, nav)
        if s == "PAD_WAIT":
            i = self.cur_station
            if self.sim.ph[i] != EJECT:
                waited = t - self.pad_since
                self.lock_wait_s += waited
                self.lock_wait_count += 1
                self.task["lock_wait_s"] = round(waited, 2)
                self.pad_since = None
                self._start_handle(t)
                return True
            return False
        if s in ("HANDLE_LOAD", "HANDLE_UNLOAD"):
            if t + EPS >= self.step_t0 + self.cur.secs:
                self._end_handle(t)
                return True
            return False
        return False

    # ------------------------------------------------------------------ decide
    def _decide(self, t: float) -> bool:
        sim = self.sim
        sim.now = t
        cands = sim.candidates()
        if not cands:
            nxt = sim.next_event()
            if nxt is None or nxt <= t + EPS:
                self.wait_until = t + 1.0
            else:
                self.wait_until = nxt
            self.stage = "WAIT"
            return False
        c, info = self.policy.choose(sim, cands, t, self.duration)
        if c is None and info.get("pending"):   # answer of the decision node not here yet: ask again at the next tick
            return False
        if c is None:                      # recorded decision not possible yet: look again soon
            self.wait_until = t + 0.5
            self.stage = "WAIT"
            return False
        self._start_task(info.pop("decision_t", t), c, info, cands)
        return True

    def live_facts(self, cands: list[Cand]) -> dict:
        sim = self.sim
        return {"machines": {mid: {"state": machine_lamp_state(sim, i), "lock_left_s": round(max(0.0, (sim.busy[i] - sim.now)) if sim.ph[i] == EJECT else 0.0, 1),
                                   "stock": sim.buf[i][0], "ng_bin": sim.ng_bin[i]} for i, mid in enumerate(self.ids)},
                "finished_buffer": sim.out_bin, "shipped": sim.shipped,
                "candidates": [cand_name(c) for c in cands]}

    def _start_task(self, t: float, c: Cand, info: dict, cands: list[Cand]) -> None:
        job, ids, tr = c.job, self.ids, self.tr
        name = cand_name(c)
        self.n_task += 1
        if job[0] == "S":
            i = job[1][0][0]
            tgt = ids[i]
            self.steps = [Step("handle", secs=tr.load, effect="supply", stage="HANDLE_LOAD"),
                          Step("go", to=tgt, stage="GO_OUT"),
                          Step("handle", secs=tr.unload, effect="deliver", stage="HANDLE_UNLOAD"),
                          Step("go", to="store", stage="RETURN")]
            self.cur_station, self.cur_k = i, job[1][0][1]
        else:
            src = len(ids) - 1 if job[0] == "SHIP" else job[1]
            self.steps = [Step("go", to=ids[src], stage="GO_OUT"),
                          Step("handle", secs=tr.load, effect="pick_ship" if job[0] == "SHIP" else "pick_scrap", stage="HANDLE_LOAD"),
                          Step("go", to="dock", stage="GO_DOCK"),
                          Step("handle", secs=tr.unload, effect="drop_ship" if job[0] == "SHIP" else "drop_scrap", stage="HANDLE_UNLOAD"),
                          Step("go", to="store", stage="RETURN")]
            self.cur_station, self.cur_k = src, 0
        self.task = {"task_id": f"T{self.n_task:03d}", "choice": name, "t_start": round(t, 2), "target": ids[self.cur_station],
                     "lock_wait_s": 0.0}
        dec = {"time_s": round(t, 2), "task_id": self.task["task_id"], "choice": name, "policy": getattr(self.policy, "policy_name", "replay"),
               "n_candidates": len(cands), **info, "live": self.live_facts(cands)}
        self.decisions.append(dec)
        self._emit("decision", dec)
        self.at = "store"
        self._begin_step(t)

    # ------------------------------------------------------------------ steps
    def _begin_step(self, t: float) -> None:
        if not self.steps:
            self._close_task(t)
            return
        self.cur = self.steps.pop(0)
        self.stage = self.cur.stage
        self.step_t0 = t
        if self.cur.kind == "go":
            self._goal_seq += 1
            planned = self._planned(self.at, self.cur.to)
            kind = "returning" if self.cur.to == "store" else "to_station"
            self.nav_goal = NavGoal(goal_id=self._goal_seq, name=self.cur.to, xy=self.xy(self.cur.to), kind=kind,
                                    task_id=self.task["task_id"], t_start=t,
                                    timeout_t=t + max(self.timeout_floor_s, planned * self.timeout_factor),
                                    planned_s=planned, from_name=self.at)
            self._emit("transport", {"time_s": round(t, 2), "stage": self._stage_public(), "task_id": self.task["task_id"],
                                     "from": self.at, "to": self.cur.to})
        else:
            self.nav_goal = None
            self._emit("transport", {"time_s": round(t, 2), "stage": self._stage_public(), "task_id": self.task["task_id"]})
            if self.cur.effect == "supply":
                self.carry = ("part", self.scn.bs[self.cur_station][0])

    def _monitor_leg(self, t: float, nav: NavObs) -> bool:
        g = self.nav_goal
        if g is None:
            return False
        if nav.goal_id == g.goal_id and nav.status == "arrived":
            if self.pace and t + EPS < g.t_start + g.planned_s:
                return False                         # arrived early: keep the calculation model's timetable
            self._leg_done(t, g, nav, True)
            return True
        failed = nav.goal_id == g.goal_id and nav.status == "failed"
        if failed or t > g.timeout_t:
            self.nav_failures += 1
            self._retries += 1
            self._record_leg(g, t, False, (nav.result or {}).get("reason", "timeout" if not failed else "failed"), nav.result)
            if self._retries > 3:
                self.abort("navigation failed 3 times in a row")
                return False
            self._goal_seq += 1                      # try the same leg again from where the robot stands
            self.nav_goal = NavGoal(goal_id=self._goal_seq, name=g.name, xy=g.xy, kind=g.kind, task_id=g.task_id,
                                    t_start=t, timeout_t=t + max(self.timeout_floor_s, g.planned_s * self.timeout_factor),
                                    planned_s=g.planned_s, from_name="here")
            return False
        return False

    def _record_leg(self, g: NavGoal, t: float, ok: bool, reason: str, res: dict | None) -> None:
        rec = {"task_id": g.task_id, "from": g.from_name, "to": g.name, "t_start": round(g.t_start, 2), "t_end": round(t, 2),
               "duration_s": round(t - g.t_start, 2), "planned_s": round(g.planned_s, 2), "ok": ok, "reason": reason}
        if res:
            rec["nav_duration_s"] = round(res.get("duration_s", 0.0), 2)
            rec["path_m"] = round(res.get("path_m", 0.0), 3)
        self.legs.append(rec)
        self._emit("leg", rec)

    def _leg_done(self, t: float, g: NavGoal, nav: NavObs, ok: bool) -> None:
        self._retries = 0
        self._record_leg(g, t, True, "ok", nav.result)
        self.travel_s += t - g.t_start
        self.at = g.name
        self.nav_goal = None
        # the effect of arriving: the engine's lock rule for supply (wait on the pad while the stage ejects an NG piece)
        nxt = self.steps[0] if self.steps else None
        if (nxt is not None and nxt.effect == "deliver" and self.scn.lock and self.sim.ph[self.cur_station] == EJECT):
            self.stage = "PAD_WAIT"
            self.pad_since = t
            self.cur = nxt
            self.steps.pop(0)
            return
        self._begin_step(t)

    def _start_handle(self, t: float) -> None:
        self.stage = self.cur.stage
        self.step_t0 = t
        self._emit("transport", {"time_s": round(t, 2), "stage": self._stage_public(), "task_id": self.task["task_id"]})

    def _end_handle(self, t: float) -> None:
        sim, e, i = self.sim, self.cur.effect, self.cur_station
        self.handle_s += self.cur.secs
        if e == "deliver":
            sim.buf[i][self.cur_k] += self.scn.bs[i][self.cur_k]
            sim.deliveries += 1
            sim._settle(t)
            self.carry = ("", 0)
        elif e == "pick_ship":
            n, sim.out_bin = sim.out_bin, 0
            self.carry = ("ship", n)
            sim._settle(t)
            self.dock_ship = 0
        elif e == "pick_scrap":
            n = 0
            if sim.ng_bin[i] > 0:
                n = 1
                sim.ng_bin[i] -= 1
                if sim.ph[i] == NGWAIT:           # the held piece moves into the freed bin, the stage restarts
                    sim.ng_bin[i] += 1
                    sim.ph[i] = IDLE
            self.carry = ("scrap", n)
            sim._settle(t)
            self.dock_scrap = 0
        elif e == "drop_ship":
            sim.shipped += self.carry[1]
            self.dock_ship = self.carry[1]
            self.carry = ("", 0)
        elif e == "drop_scrap":
            self.dock_scrap = self.carry[1]
            self.carry = ("", 0)
        self._begin_step(t)

    def _close_task(self, t: float) -> None:
        self.task.update({"t_end": round(t, 2), "duration_s": round(t - self.task["t_start"], 2)})
        self.tasks.append(self.task)
        self._emit("task", self.task)
        self.task, self.cur, self.nav_goal = None, None, None
        self.stage = "DECIDE"

    def abort(self, reason: str) -> None:
        self.aborted = reason
        self._finish()

    def _finish(self) -> None:
        if self.ended:
            return
        self.ended = True
        self.nav_goal = None

    # ------------------------------------------------------------------ outputs
    @property
    def completed(self) -> int:
        return self.sim.shipped

    def visual_view(self) -> dict:
        sim, sc = self.sim, self.scn
        states = {mid: machine_lamp_state(sim, i) for i, mid in enumerate(self.ids)}
        stock = {mid: (sim.buf[i][0], sc.cap[i][0]) for i, mid in enumerate(self.ids)}
        for i in range(len(self.ids)):
            stock[f"NG{i + 1}"] = (sim.ng_bin[i], 1)
        stock["FIN"] = (sim.out_bin, sc.out_cap)
        stock["DOCKS"] = (self.dock_ship, sc.out_cap)
        stock["DOCKX"] = (self.dock_scrap, 1)
        kind = self.carry[0]
        cargo_visible = kind in ("part", "ship", "scrap") and (kind == "part" or self.carry[1] > 0)
        cargo_color = {"part": "orange", "ship": "green", "scrap": "red"}.get(kind, "orange")
        return {"machines": states, "stock": stock, "cargo": cargo_visible, "cargo_color": cargo_color, "completed": sim.shipped}

    def state_body(self, t: float) -> dict:
        sim = self.sim
        return {"time_s": round(t, 2), "policy": "b1_replay",
                "line": {"completed": sim.shipped, "bottleneck": self.ids[sim.scn.bott], "bottleneck_idle_s": round(sim.m3_idle(max(t, 1e-9)), 1)},
                "machines": {mid: {"state": machine_lamp_state(sim, i), "buffer": sim.buf[i][0], "ng_bin": sim.ng_bin[i]}
                             for i, mid in enumerate(self.ids)},
                "finished_buffer": sim.out_bin, "dock": {"shipped_lot": self.dock_ship, "scrap": self.dock_scrap},
                "robot": {"stage": self._stage_public(), "task": self.task["choice"] if self.task else None, "at": getattr(self, "at", "store")}}

    def state_signature(self) -> tuple:
        sim = self.sim
        return (tuple(machine_lamp_state(sim, i) for i in range(len(self.ids))), sim.shipped, sim.out_bin,
                tuple(sim.ng_bin), self._stage_public(), self.task["task_id"] if self.task else None)

    def metrics(self, mean_rtf: float | None = None, extra: dict | None = None) -> dict:
        sim, h = self.sim, self.duration
        lw = self.lock_wait_s
        legs_ok = [l for l in self.legs if l["ok"]]
        over = [l["duration_s"] - l["planned_s"] for l in legs_ok]
        m = {"scenario": "B1", "mode": "gazebo_replay", "seed": self.seed, "duration_s": h,
             "completed": sim.shipped, "defects": sim.defects, "deliveries": sim.deliveries, "m3_idle_s": round(sim.m3_idle(h), 1),
             "decisions": len(self.decisions), "replay_planned": self.policy.plan_len, "replay_used": self.policy.k,
             "replay_skipped": self.policy.skipped, "nav_failures": self.nav_failures,
             "robot_s": {"travel": round(self.travel_s, 1), "handling": round(self.handle_s, 1), "lock_wait": round(lw, 1)},
             "lock_wait_count": self.lock_wait_count, "legs": len(self.legs),
             "leg_over_plan_mean_s": round(sum(over) / len(over), 2) if over else None,
             "leg_over_plan_max_s": round(max(over), 2) if over else None,
             "pace": self.pace, "aborted": self.aborted, "mean_rtf": mean_rtf}
        if extra:
            m.update(extra)
        self.final_metrics = m
        return m
