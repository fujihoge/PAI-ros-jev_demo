"""Waypoint driver: turn in place -> drive straight -> stop, then check the arrival. No ROS, no decisions.

The control law (goto / turn_to) is a non-blocking state machine
so that b1_node can call `step()` from a timer. Times are the caller's clock (simulation seconds).
Position/yaw are the ground truth (/model_states); (vx, wz) come from /odom.
"""
from __future__ import annotations

import math
from dataclasses import dataclass


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


@dataclass
class NavParams:
    vmax: float = 0.6            # same as the calibration runs (travel_overhead_s = 3.6 was measured with 0.6)
    wmax: float = 1.0
    vmin: float = 0.05
    drive_dec: float = 0.8       # approach deceleration [m/s^2]
    turn_dec: float = 1.5        # [rad/s^2]
    tol_pos: float = 0.03        # stop condition of the drive [m]
    tol_yaw: float = 0.03        # stop condition of the turn [rad]
    abort_after_s: float = 3.0   # safety stop lasting this long during a drive -> the leg fails
    confirm_s: float = 0.2       # "standing still" must last this long (user decision 2026-10-12: 0.5 -> 0.2)
    settle_max_s: float = 15.0
    turn_max_s: float = 60.0
    drive_max_s: float = 120.0
    arrive_pos_tol: float = 0.10      # arrival check: distance to the pad centre [m]
    arrive_yaw_tol: float = 0.35      # arrival check: |yaw - approach bearing| [rad] (about 20 deg)
    still_speed: float = 0.02         # arrival check: |vx| [m/s]
    max_attempts: int = 2             # a failed arrival check re-runs the leg this many times in total


def arrival_check(pos_err: float, heading_err: float, speed: float, safety_stop: bool,
                  p: NavParams) -> tuple[bool, str]:
    """Parts are loaded/unloaded only after the robot is really at the pad. Returns (ok, reason)."""
    if safety_stop:
        return False, "safety_stop_active"
    if pos_err > p.arrive_pos_tol:
        return False, "position_error"
    if abs(heading_err) > p.arrive_yaw_tol:
        return False, "heading_error"
    if abs(speed) > p.still_speed:
        return False, "not_still"
    return True, "ok"


class NavDriver:
    """status: idle | running | arrived | failed. `result` is set when the leg ends."""

    def __init__(self, params: NavParams | None = None) -> None:
        self.p = params or NavParams()
        self.status = "idle"
        self.phase = ""
        self.result: dict | None = None
        self.goal_id = 0

    # ---- start / cancel ----
    def start(self, goal_id: int, now: float, pose: tuple, tx: float, ty: float, label: str = "") -> None:
        self.goal_id, self.label = goal_id, label
        self.tx, self.ty = tx, ty
        self.t_start = now
        self.status, self.result = "running", None
        self.attempt = 1
        self.path_len = 0.0
        self._last_xy = (pose[0], pose[1])
        self.straight = math.hypot(tx - pose[0], ty - pose[1])
        self.bearing0 = math.atan2(ty - pose[1], tx - pose[0])
        self.safety_since: float | None = None
        self.t_still = now
        self._begin_attempt(now, pose)

    def cancel(self) -> None:
        self.status, self.phase = "idle", ""

    def _begin_attempt(self, now: float, pose: tuple) -> None:
        self.bearing = math.atan2(self.ty - pose[1], self.tx - pose[0])
        self.turn_tries = 0
        self._enter("turn", now)
        self._first_turn = True

    def _enter(self, phase: str, now: float) -> None:
        self.phase, self.phase_t, self.quiet_since = phase, now, None

    def _fail(self, now: float, pose: tuple, reason: str, stopped_at: float | None = None) -> None:
        self.status = "failed"
        self.result = self._summary(now, pose, False, reason, stopped_at if stopped_at is not None else now)

    def _summary(self, now: float, pose: tuple, reached: bool, reason: str, t_stop: float) -> dict:
        x, y, yaw = pose
        return {
            "leg": self.label, "goal_id": self.goal_id, "reached": reached, "reason": reason,
            "attempts": self.attempt, "t_start": self.t_start, "t_stop": t_stop, "t_confirm": now,
            "duration_s": t_stop - self.t_start,       # until the robot stood still (the confirm window excluded)
            "straight_m": self.straight, "path_m": self.path_len,
            "path_factor": self.path_len / self.straight if self.straight > 0 else None,
            "final_err_m": math.hypot(self.tx - x, self.ty - y),
            "final_heading_err_deg": math.degrees(wrap(yaw - self.bearing0)),
        }

    # ---- helpers ----
    def _still(self, now: float, vx: float, wz: float) -> bool:
        """True once vx/wz have stayed small for confirm_s (or settle_max_s elapsed). Sets self.t_still."""
        if abs(vx) < 0.005 and abs(wz) < 0.01:
            if self.quiet_since is None:
                self.quiet_since = now
            if now - self.quiet_since >= self.p.confirm_s:
                self.t_still = self.quiet_since
                return True
        else:
            self.quiet_since = None
        if now - self.phase_t > self.p.settle_max_s:
            self.t_still = now
            return True
        return False

    # ---- one control step: returns (v, w) to command ----
    def step(self, now: float, pose: tuple, vx: float, wz: float, safety_stop: bool) -> tuple[float, float]:
        p = self.p
        if self.status != "running":
            return 0.0, 0.0
        x, y, yaw = pose
        self.path_len += math.hypot(x - self._last_xy[0], y - self._last_xy[1])
        self._last_xy = (x, y)

        if self.phase == "turn":
            err = wrap(self.bearing - yaw)
            if self._first_turn:
                self._first_turn = False
                if abs(err) < p.tol_yaw and abs(wz) < 0.01:   # already aligned and still: no turn at all
                    self._enter("drive", now)
                    return 0.0, 0.0
            if abs(err) < p.tol_yaw or now - self.phase_t > p.turn_max_s:
                self._enter("turn_settle", now)
                return 0.0, 0.0
            w = min(max(math.sqrt(2 * p.turn_dec * abs(err)), 0.12), p.wmax) * (1 if err > 0 else -1)
            return 0.0, w

        if self.phase == "turn_settle":
            if self._still(now, vx, wz):
                self.turn_tries += 1
                err = wrap(self.bearing - yaw)
                if abs(err) < 2 * p.tol_yaw or self.turn_tries >= 4:
                    self._enter("drive", now)
                else:
                    self._enter("turn", now)
            return 0.0, 0.0

        if self.phase == "drive":
            d = math.hypot(self.tx - x, self.ty - y)
            eh = wrap(math.atan2(self.ty - y, self.tx - x) - yaw)
            if d < p.tol_pos or (d < 0.15 and abs(eh) > math.pi / 2) or now - self.phase_t > p.drive_max_s:
                self._enter("settle", now)
                return 0.0, 0.0
            if safety_stop:
                self.safety_since = now if self.safety_since is None else self.safety_since
                if now - self.safety_since >= p.abort_after_s:
                    self._fail(now, pose, "safety_stop")
                    return 0.0, 0.0
            else:
                self.safety_since = None
            v = max(min(p.vmax, math.sqrt(2 * p.drive_dec * d)), p.vmin)
            v = v * max(0.0, math.cos(eh)) if abs(eh) < 0.5 else 0.0
            w = max(-p.wmax, min(p.wmax, 1.5 * eh))
            return v, w

        if self.phase == "settle":
            if self._still(now, vx, wz):
                pos_err = math.hypot(self.tx - x, self.ty - y)
                hd = wrap(yaw - self.bearing0)
                ok, why = arrival_check(pos_err, hd, vx, safety_stop, p)
                if ok:
                    self.status = "arrived"
                    self.result = self._summary(now, pose, True, "ok", self.t_still)
                elif self.attempt < p.max_attempts and why in ("position_error", "heading_error", "not_still"):
                    self.attempt += 1
                    self._begin_attempt(now, pose)
                else:
                    self._fail(now, pose, why, self.t_still)
            return 0.0, 0.0
        return 0.0, 0.0
