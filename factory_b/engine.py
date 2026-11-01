"""Compact pure-Python discrete-event model of task B1 (serial line M1->M4, one AMR, one store).

Line rules: one-workpiece WIP slots, completion-time defect decision from a common series (factory_b.defects),
NG ejection, optional input lock during ejection, a finished-goods bin at the last stage and per-stage NG bins.
The state is cheap to clone. The robot always starts and ends every task at the store.
(The generic fields for several part types per stage and two-stop trips are unused in B1.)
"""
from __future__ import annotations

import math
from dataclasses import dataclass, replace

from .defects import is_defective

EPS = 1e-9
BIG = 1e9
STORE = "store"
IDLE, RUN, DONE, EJECT, NGWAIT = range(5)  # NGWAIT: ejection finished but the stage's NG bin is full (B1)


@dataclass(frozen=True)
class Transport:
    positions: dict
    speed: float = 0.6
    overhead: float = 4.1
    load: float = 2.0
    unload: float = 2.0

    def travel(self, a: str, b: str) -> float:
        (x1, y1), (x2, y2) = self.positions[a], self.positions[b]
        d = math.hypot(x2 - x1, y2 - y1)
        return 0.0 if d == 0 else d / self.speed + self.overhead


@dataclass(frozen=True)
class Scenario:
    name: str
    ids: tuple
    ct: tuple
    bs: tuple  # bs[i][k]: parts of type k consumed per cycle of stage i
    cap: tuple  # cap[i][k]
    init_std: tuple  # init[i][k] standard stock (P0 equivalent)
    transport: Transport
    out_cap: int | None = None  # B1: finished-goods bin capacity at the last stage (None = unlimited, counts at once)
    defect_bins: bool = False  # B1: NG bin (capacity 1) next to every stage
    max_stops: int = 1  # B3: 2
    rate: float = 0.0  # defect probability per finished cycle
    ng_eject: float = 0.0
    lock: bool = False
    start: str = "std"  # "std" (P0) or "empty" (P5: no parts, WIP slots empty)
    bott: int = 2  # index of the bottleneck (M3)

    @property
    def n(self) -> int:
        return len(self.ids)

    @property
    def ntypes(self) -> int:
        return len(self.bs[0])

    def init(self) -> tuple:
        if self.start == "std":
            return self.init_std
        return tuple(tuple(0 for _ in r) for r in self.init_std)

    @property
    def wip0(self) -> int:
        return 1 if self.start == "std" else 0

    def with_start(self, start: str) -> "Scenario":
        return replace(self, start=start)

    def with_condition(self, rate: float, ng_eject: float, lock: bool) -> "Scenario":
        return replace(self, rate=rate, ng_eject=ng_eject, lock=lock)


class Cand:
    __slots__ = ("job", "slack", "slacks", "total")

    def __init__(self, job, slack, slacks, total):
        self.job, self.slack, self.slacks, self.total = job, slack, slacks, total

    @property
    def stops(self) -> tuple:
        return self.job[1] if self.job[0] == "S" else ()

    @property
    def name(self) -> str:
        j = self.job
        if j[0] == "S":
            return "+".join(f"M{i + 1}{'AB'[k]}" for i, k in j[1])
        if j[0] == "SHIP":
            return "SHIP"
        return f"COL_M{j[1] + 1}"


class Sim:
    def __init__(self, scn: Scenario, seed: int = 0) -> None:
        self.scn, self.seed = scn, seed
        n = scn.n
        self.buf = [list(r) for r in scn.init()]
        self.ph = [IDLE] * n
        self.busy: list = [None] * n
        self.cycles = [0] * n
        self.slots = [scn.wip0] * (n - 1)
        self.ng_bin = [0] * n
        self.out_bin = 0
        self.produced = 0
        self.shipped = 0
        self.defects = 0
        self.proc = 0.0  # bottleneck processing time of started cycles (overshoot removed in m3_idle)
        self.t = 0.0  # DES clock
        self.now = 0.0  # decision time of the robot
        self.lock_wait = 0.0
        self.busy_s = 0.0
        self.travel_s = 0.0
        self.handle_s = 0.0
        self.deliveries = 0
        self._settle(0.0)

    # ---- cloning ----
    def clone(self) -> "Sim":
        o = Sim.__new__(Sim)
        o.scn, o.seed = self.scn, self.seed
        o.buf = [r[:] for r in self.buf]
        o.ph, o.busy, o.cycles, o.slots, o.ng_bin = self.ph[:], self.busy[:], self.cycles[:], self.slots[:], self.ng_bin[:]
        o.out_bin, o.produced, o.shipped, o.defects, o.proc = self.out_bin, self.produced, self.shipped, self.defects, self.proc
        o.t, o.now, o.lock_wait, o.busy_s, o.deliveries = self.t, self.now, self.lock_wait, self.busy_s, self.deliveries
        o.travel_s, o.handle_s = self.travel_s, self.handle_s
        return o

    # ---- line logic ----
    def has_parts(self, i: int) -> bool:
        b, s = self.buf[i], self.scn.bs[i]
        for k in range(len(b)):
            if b[k] < s[k]:
                return False
        return True

    def _handoff(self, i: int) -> bool:
        n, sc = self.scn.n, self.scn
        if self.ph[i] != DONE:
            return False
        if i == n - 1:
            if sc.out_cap is not None:
                if self.out_bin >= sc.out_cap:
                    return False
                self.out_bin += 1
            self.produced += 1
        else:
            if self.slots[i] != 0:
                return False
            self.slots[i] = 1
        self.ph[i] = IDLE
        return True

    def _settle(self, t: float) -> None:
        sc = self.scn
        n = sc.n
        changed = True
        while changed:
            changed = False
            for i in range(n - 1, -1, -1):
                if self.ph[i] == DONE and self._handoff(i):
                    changed = True
                if self.ph[i] == IDLE and self.has_parts(i) and (i == 0 or self.slots[i - 1] == 1):
                    if i > 0:
                        self.slots[i - 1] = 0
                    b, s = self.buf[i], sc.bs[i]
                    for k in range(len(b)):
                        b[k] -= s[k]
                    self.ph[i] = RUN
                    self.busy[i] = t + sc.ct[i]
                    self.cycles[i] += 1
                    if i == sc.bott:
                        self.proc += sc.ct[i]
                    changed = True

    def _after_eject(self, i: int) -> None:
        if self.scn.defect_bins:
            if self.ng_bin[i] == 0:
                self.ng_bin[i] = 1
                self.ph[i] = IDLE
            else:
                self.ph[i] = NGWAIT
        else:
            self.ph[i] = IDLE
        self.busy[i] = None

    def next_event(self):
        ts = [b for b in self.busy if b is not None]
        return min(ts) if ts else None

    def advance_to(self, t: float) -> None:
        sc = self.scn
        n = sc.n
        busy, ph = self.busy, self.ph
        while True:
            bt = None
            for b in busy:
                if b is not None and b <= t + EPS and (bt is None or b < bt):
                    bt = b
            if bt is None:
                break
            for i in range(n - 1, -1, -1):
                if ph[i] == RUN and busy[i] <= bt + EPS:
                    busy[i] = None
                    ph[i] = DONE
                    if sc.rate > 0 and is_defective(self.seed, sc.ids[i], self.cycles[i], sc.rate * 100.0):
                        self.defects += 1
                        if sc.ng_eject > 0:
                            ph[i] = EJECT
                            busy[i] = bt + sc.ng_eject
                        else:
                            self._after_eject(i)
                    else:
                        self._handoff(i)
            for i in range(n):
                if ph[i] == EJECT and busy[i] <= bt + EPS:
                    self._after_eject(i)
            self._settle(bt)
        if t > self.t:
            self.t = t

    # ---- metrics ----
    @property
    def done(self) -> int:
        """Completed workpieces: shipped (B1: delivered to the shipping area) or finished (others)."""
        return self.shipped if self.scn.out_cap is not None else self.produced

    def m3_idle(self, h: float) -> float:
        b = self.scn.bott
        over = max(0.0, self.busy[b] - h) if self.ph[b] == RUN else 0.0
        return h - (self.proc - over)

    # ---- candidates ----
    def _starve_off(self, i: int, k: int) -> float:
        """Time from now until stage i cannot start a cycle for lack of type-k parts (no further supply)."""
        sc = self.scn
        origin = (self.busy[i] - self.now) if self.ph[i] in (RUN, EJECT) else 0.0
        return origin + (self.buf[i][k] // sc.bs[i][k]) * sc.ct[i]

    def feasible_supply(self, i: int, k: int) -> bool:
        return self.buf[i][k] + self.scn.bs[i][k] <= self.scn.cap[i][k]

    def _route_supply(self, stops):
        sc, tr = self.scn, self.scn.transport
        ids = sc.ids
        cur, prev = tr.load * len(stops), STORE
        slacks = []
        for (i, k) in stops:
            cur += tr.travel(prev, ids[i])
            if sc.lock and self.ph[i] == EJECT:
                cur += max(0.0, (self.busy[i] - self.now) - cur)
            cur += tr.unload
            slacks.append(self._starve_off(i, k) - cur)
            prev = ids[i]
        cur += tr.travel(prev, STORE)
        return slacks, cur

    def candidates(self) -> list:
        sc, tr = self.scn, self.scn.transport
        n, nt = sc.n, sc.ntypes
        singles = [(i, k) for i in range(n) for k in range(nt) if self.feasible_supply(i, k)]
        out: list = []
        for s in singles:
            sl, tot = self._route_supply((s,))
            out.append(Cand(("S", (s,)), sl[0], sl, tot))
        if sc.max_stops >= 2:
            for a in singles:
                for b in singles:
                    if a[0] != b[0]:
                        sl, tot = self._route_supply((a, b))
                        out.append(Cand(("S", (a, b)), min(sl), sl, tot))
        last = n - 1
        if sc.out_cap is not None and self.out_bin > 0:
            ids = sc.ids
            pick = tr.travel(STORE, ids[last]) + tr.load
            tot = pick + tr.travel(ids[last], "dock") + tr.unload + tr.travel("dock", STORE)
            ct = sc.ct[last]
            room = sc.out_cap - self.out_bin
            if self.ph[last] == DONE:
                block_in = 0.0
            elif self.ph[last] == RUN:
                block_in = (self.busy[last] - self.now) + room * ct
            elif self.ph[last] == EJECT:
                block_in = (self.busy[last] - self.now) + ct + room * ct
            else:
                block_in = ct + room * ct
            sl = block_in - pick
            out.append(Cand(("SHIP",), sl, [sl], tot))
        if sc.defect_bins:
            for i in range(n):
                if self.ng_bin[i] > 0:
                    pick = tr.travel(STORE, sc.ids[i]) + tr.load
                    tot = pick + tr.travel(sc.ids[i], "dock") + tr.unload + tr.travel("dock", STORE)
                    if self.ph[i] in (RUN, EJECT):
                        block_in = self.busy[i] - self.now  # worst case: the next piece is defective too
                    else:
                        block_in = 0.0
                    sl = block_in - pick
                    out.append(Cand(("COL", i), sl, [sl], tot))
        return out

    # ---- execution ----
    def execute(self, job, h: float) -> None:
        """Run the task starting at self.now. Effects after the horizon h are not applied. Sets self.now = end."""
        sc, tr = self.scn, self.scn.transport
        ids = sc.ids
        t0 = self.now
        last = sc.n - 1
        trav = hand = 0.0
        if job[0] == "S":
            stops = job[1]
            cur, prev = t0 + tr.load * len(stops), STORE
            hand = tr.load * len(stops) + tr.unload * len(stops)
            for (i, k) in stops:
                trav += tr.travel(prev, ids[i])
                cur += tr.travel(prev, ids[i])
                if cur <= h:
                    self.advance_to(cur)
                    if sc.lock and self.ph[i] == EJECT:
                        end = self.busy[i]
                        self.lock_wait += min(end, h) - cur
                        cur = end
                        if cur <= h:
                            self.advance_to(cur)
                cur += tr.unload
                if cur <= h:
                    self.advance_to(cur)
                    self.buf[i][k] += sc.bs[i][k]
                    self.deliveries += 1
                    self._settle(cur)
                prev = ids[i]
            trav += tr.travel(prev, STORE)
            cur += tr.travel(prev, STORE)
        else:
            src = last if job[0] == "SHIP" else job[1]
            trav = tr.travel(STORE, ids[src]) + tr.travel(ids[src], "dock") + tr.travel("dock", STORE)
            hand = tr.load + tr.unload
            cur = t0 + tr.travel(STORE, ids[src]) + tr.load
            carry = 0
            if cur <= h:
                self.advance_to(cur)
                if job[0] == "SHIP":
                    carry, self.out_bin = self.out_bin, 0
                elif self.ng_bin[src] > 0:
                    carry = 1
                    self.ng_bin[src] -= 1
                    if self.ph[src] == NGWAIT:  # the held piece moves into the freed bin, the stage restarts
                        self.ng_bin[src] += 1
                        self.ph[src] = IDLE
                self._settle(cur)
            cur += tr.travel(ids[src], "dock") + tr.unload
            if cur <= h:
                self.advance_to(cur)
                if job[0] == "SHIP":
                    self.shipped += carry
            cur += tr.travel("dock", STORE)
        if cur <= h:
            self.busy_s += cur - t0
            self.travel_s += trav
            self.handle_s += hand
        self.now = cur


def run(sim: Sim, policy, h: float):
    """Drive `sim` with `policy` until the horizon h. Returns (decisions, candidate_count_sum)."""
    nd = nc = 0
    while sim.now < h:
        sim.advance_to(sim.now)
        cands = sim.candidates()
        if not cands:
            nxt = sim.next_event()
            if nxt is None or nxt <= sim.now:
                break
            sim.now = nxt
            continue
        nd += 1
        nc += len(cands)
        c = cands[0] if len(cands) == 1 else policy.choose(sim, cands, h)
        sim.execute(c.job, h)
    sim.advance_to(h)
    return nd, nc
