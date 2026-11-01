import math

from factory_sim.nav_driver import NavDriver, NavParams, arrival_check, wrap

DT = 0.05


def simulate(drv, pose, goal, *, safety=lambda t: False, tmax=120.0, lag=0.0):
    """Ideal unicycle: the commanded (v, w) is applied at once (lag = first-order smoothing time constant)."""
    x, y, yaw = pose
    t, vx, wz = 0.0, 0.0, 0.0
    drv.start(1, t, (x, y, yaw), goal[0], goal[1], "test")
    while drv.status == "running" and t < tmax:
        v, w = drv.step(t, (x, y, yaw), vx, wz, safety(t))
        a = 1.0 if lag == 0 else DT / (lag + DT)
        vx += a * (v - vx)
        wz += a * (w - wz)
        x += vx * math.cos(yaw) * DT
        y += vx * math.sin(yaw) * DT
        yaw = wrap(yaw + wz * DT)
        t += DT
    return t, (x, y, yaw)


def test_drives_to_the_goal_and_confirms_the_arrival():
    drv = NavDriver(NavParams())
    t, pose = simulate(drv, (0.0, -1.5, 0.0), (-3.0, 1.5), lag=0.1)
    assert drv.status == "arrived", drv.result
    r = drv.result
    assert r["reached"] and r["final_err_m"] < 0.10 and abs(r["final_heading_err_deg"]) < 20
    assert 4.0 < r["straight_m"] < 4.5 and r["path_factor"] < 1.1
    # about distance / 0.6 + turn: sanity band, not a calibration
    assert 8.0 < r["duration_s"] < 16.0
    assert r["t_confirm"] - r["t_stop"] >= 0.2 - 1e-9     # the standing-still confirmation window


def test_aligned_and_still_start_skips_the_turn():
    drv = NavDriver(NavParams())
    simulate(drv, (0.0, 0.0, 0.0), (2.0, 0.0))
    assert drv.status == "arrived"
    assert abs(drv.result["duration_s"] - (2.0 / 0.6 + 0.0)) < 1.5


def test_a_long_safety_stop_while_driving_fails_the_leg():
    drv = NavDriver(NavParams(abort_after_s=3.0))
    simulate(drv, (0.0, -1.5, 0.0), (3.0, 1.5), safety=lambda t: t > 4.0)
    assert drv.status == "failed" and drv.result["reason"] == "safety_stop"
    assert not drv.result["reached"]


def test_a_short_safety_stop_is_tolerated():
    drv = NavDriver(NavParams(abort_after_s=3.0))
    simulate(drv, (0.0, 0.0, 0.0), (3.0, 0.0), safety=lambda t: 4.0 < t < 5.0)
    assert drv.status == "arrived"


def test_arrival_check_rules():
    p = NavParams()
    assert arrival_check(0.02, 0.05, 0.0, False, p) == (True, "ok")
    assert arrival_check(0.02, 0.05, 0.0, True, p) == (False, "safety_stop_active")
    assert arrival_check(0.30, 0.05, 0.0, False, p) == (False, "position_error")
    assert arrival_check(0.02, 1.0, 0.0, False, p) == (False, "heading_error")
    assert arrival_check(0.02, 0.05, 0.2, False, p) == (False, "not_still")


def test_step_after_the_end_commands_zero():
    drv = NavDriver(NavParams())
    simulate(drv, (0.0, 0.0, 0.0), (1.0, 0.0))
    assert drv.step(99.0, (1.0, 0.0, 0.0), 0.0, 0.0, False) == (0.0, 0.0)
