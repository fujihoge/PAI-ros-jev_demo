"""b1_node: task B1 (supply / ship / collect scrap) in Gazebo, replaying the decisions of a recorded episode.

  Line / engine   : factory_b.engine.Sim through factory_sim.b1_runner.B1Runner (no Jev, no API call)
  Decisions       : ReplayPolicy over `trace` (recorded/b1x_seed5_empty.json "decisions"), in order
  Driving         : factory_sim.nav_driver.NavDriver -> /cmd_vel_raw (safety layer); position /model_states
  Time            : /clock (use_sim_time); the episode starts when the robot is placed at the store

Publishes (std_msgs/String JSON): /factory/state (1 Hz + changes), /dispatch/decision, /transport/status, /metrics (once),
and the visual commands /m5/lamp_cmd /m5/work_cmd /m5/stock_cmd /m5/cargo_cmd /m5/banner_cmd (world factory_b1.world)
plus /m5/flow_cmd (visual-only workpieces moving on the belts, factory_sim.b1_flow; parameter flow:=false turns it off).
Decision: parameter decider = node (default) | inprocess. node: when a decision is wanted the request is put into /factory/state
(`decision_request`, factory_sim.decision_core) and the answer of decision_node comes on /dispatch/decision; the robot and the
simulation keep running meanwhile; no valid answer within decision_timeout_s (wall) -> timeout fallback (smallest slack), logged. inprocess: the
ReplayPolicy runs here and the executed decision is published on /dispatch/decision (old behaviour).
Output folder: decisions.jsonl, legs.jsonl, tasks.jsonl, metrics.json.
"""
from __future__ import annotations

import dataclasses
import json
import math
import sys
import time
from pathlib import Path

import rclpy
from gazebo_msgs.msg import EntityState, ModelStates
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Bool, Int32, String

from . import b1_flow, b1_visuals, colors, decision_core, messages, paths
from .b1_runner import B1Runner, ReplayPolicy
from .nav_types import NavObs
from .nav_driver import NavDriver, NavParams

paths.ensure_importable()
from factory_b import make_scenario  # noqa: E402

ROBOT = "factory_robot"
# frozen setting (ct_scale 0.6, defect rate 10 %, NG stop 60 s, input lock on, handling 2 s, overhead 2 s, short layout)
CT_SCALE, RATE, STOP_S, LOCK = 0.6, 0.10, 60.0, True


def yaw_of(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class B1Node(Node):
    def __init__(self) -> None:
        super().__init__("b1_node")
        d = self.declare_parameter
        root = paths.find_project_root()
        self.trace_path = d("trace", str(root / "recorded" / "b1x_seed5_empty.json")).value
        self.duration = float(d("duration", 1800.0).value)
        self.seed = int(d("seed", 5).value)
        self.start = d("start", "empty").value
        self.out_dir = Path(d("out", str(root / "out" / "b1_replay")).value)
        self.pace = bool(d("pace", True).value)
        self.max_wait_s = float(d("replay_max_wait_s", 20.0).value)
        self.control_period = float(d("control_period", 0.05).value)
        self.startup_wall_s = float(d("startup_wall_s", 180.0).value)
        self.max_wall_s = float(d("max_wall_s", 0.0).value)
        self.visuals = bool(d("visuals", True).value)
        self.decider = d("decider", "node").value                   # node | inprocess
        if self.decider not in ("node", "inprocess"):
            raise ValueError(f"decider must be node or inprocess, not {self.decider!r}")
        self.decision_timeout_s = float(d("decision_timeout_s", 8.0).value)
        self.debug_decisions = bool(d("debug_decisions", False).value)
        self.flow = bool(d("flow", True).value)                    # visual-only workpieces moving on the belts (b1_flow)
        self.flow_paths_file = d("flow_paths", "").value
        nav_defaults = NavParams()
        kw = {}
        for f in dataclasses.fields(NavParams):
            kw[f.name] = type(getattr(nav_defaults, f.name))(d("nav_" + f.name, getattr(nav_defaults, f.name)).value)
        self.nav_params = NavParams(**kw)

        self.trace = json.loads(Path(self.trace_path).read_text(encoding="utf-8"))
        self.scn = make_scenario("B1", "ii", ct_scale=CT_SCALE).with_condition(RATE, STOP_S, LOCK).with_start(self.start)
        self.store_xy = tuple(self.scn.transport.positions["store"])
        self.lamp_colors = colors.merge_lamp_colors(None)

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        reliable = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE)
        self.pub_state = self.create_publisher(String, "/factory/state", reliable)
        if self.decider == "inprocess":
            self.pub_decision = self.create_publisher(String, "/dispatch/decision", reliable)
        else:
            self.create_subscription(String, "/dispatch/decision", self._on_decision, reliable)
        self.pub_transport = self.create_publisher(String, "/transport/status", reliable)
        self.pub_metrics = self.create_publisher(String, "/metrics", latched)
        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel_raw", 10)
        self.pub_visual = {t: self.create_publisher(String, t, 10) for t in b1_visuals.TOPIC.values()}
        self.create_subscription(ModelStates, "/model_states", self._on_models, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(Bool, "/safety/stop", lambda m: setattr(self, "safety_stop", m.data), 10)
        self.create_subscription(Int32, "/safety/stop_count", lambda m: setattr(self, "safety_count", m.data), 10)
        self.set_state = self.create_client(SetEntityState, "/set_entity_state")

        self.pose = None
        self.odom_v = (0.0, 0.0)
        self.safety_stop = False
        self.safety_count: int | None = None
        self.versions = messages.Versioner()
        self.driver = NavDriver(self.nav_params)
        self.tracker = b1_visuals.Tracker(self.lamp_colors)
        self.banner = colors.CompletionBanner()
        self.flow_tracker: b1_flow.FlowTracker | None = None
        self.runner: B1Runner | None = None
        self.remote: decision_core.RemotePolicy | None = None
        self.phase = "wait"
        self.done = False
        self.exit_code = 0
        self._place_future = None
        self._place_tries = 0
        self._phase_t = 0.0
        self._last_state_t = -1e9
        self._last_sig = None
        self._last_visual_all = -1e9
        self.wall_start = time.monotonic()
        self.wall_ep0: float | None = None
        self.t0 = self.sim0 = 0.0
        self._last_sim_seen = (0.0, time.monotonic())
        self.create_timer(self.control_period, self._tick)
        self.create_timer(1.0, self._watchdog, clock=Clock(clock_type=ClockType.SYSTEM_TIME))
        self.get_logger().info(f"b1_node: decider={self.decider} replay of {self.trace_path} ({len(self.trace['decisions'])} decisions) "
                               f"duration={self.duration}s pace={self.pace} out={self.out_dir}")

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_models(self, m: ModelStates) -> None:
        if ROBOT in m.name:
            p = m.pose[m.name.index(ROBOT)]
            self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))

    def _on_odom(self, m: Odometry) -> None:
        self.odom_v = (m.twist.twist.linear.x, m.twist.twist.angular.z)

    def _send_request(self, req: dict) -> None:
        """RemotePolicy.send: the request rides on /factory/state (published at once, then repeated with every state message)."""
        w0 = time.monotonic()
        self._publish_state(req["time_s"], force=True)
        if self.debug_decisions:
            self.get_logger().info(f"DBG publish took {(time.monotonic() - w0) * 1000:.1f} ms; send {req['request_id']} t={req['time_s']:.3f} now={self.now() - self.t0:.3f} wall={time.monotonic():.3f}")

    def _on_decision(self, m: String) -> None:
        if self.remote is None:
            return
        try:
            ans = json.loads(m.data)
        except ValueError:
            return
        if self.debug_decisions:
            self.get_logger().info(f"DBG recv {ans.get('request_id')} now={self.now() - self.t0:.3f} wall={time.monotonic():.3f} phase={self.phase}")
        if self.remote.on_message(ans) and self.phase == "run" and not self.done:
            self._run(self.now())          # act on the answer now, not at the next 50 ms tick

    def _pub(self, pub, kind: str, body: dict) -> dict:
        msg = messages.envelope(self.versions.next(), kind, body)
        s = String()
        s.data = messages.dumps(msg)
        pub.publish(s)
        return msg

    def _send_visuals(self, items) -> None:
        if not self.visuals:
            return
        for topic, text in items:
            s = String()
            s.data = text
            self.pub_visual[topic].publish(s)

    def _load_flow_paths(self) -> dict:
        f = self.flow_paths_file
        if not f:
            try:
                from ament_index_python.packages import get_package_share_directory
                f = str(Path(get_package_share_directory("factory_sim")) / "worlds" / "factory_b1.flow.json")
            except Exception:
                f = ""
            if not f or not Path(f).exists():
                f = str(Path(__file__).resolve().parents[1] / "worlds" / "factory_b1.flow.json")
        return json.loads(Path(f).read_text(encoding="utf-8"))

    def _view(self, t: float) -> dict:
        view = self.runner.visual_view()
        if self.flow_tracker:
            self.flow_tracker.adjust_view(view, self.runner.ids, t)
        view["banner"] = self.banner.update(t, view.pop("completed"))
        return view

    def _cmd(self, v: float, w: float) -> None:
        m = Twist()
        m.linear.x, m.angular.z = float(v), float(w)
        self.pub_cmd.publish(m)

    # ------------------------------------------------------------------ main loop
    def _tick(self) -> None:
        if self.done:
            return
        now = self.now()
        if now <= 0.0:
            return
        if self.phase == "wait":
            self._cmd(0.0, 0.0)
            ready = (self.pose is not None and (self.safety_count is not None or now > 5.0) and self.set_state.service_is_ready()
                     and (self.decider == "inprocess" or self.count_publishers("/dispatch/decision") > 0))
            if ready:
                self._place()
        elif self.phase == "placing":
            if self._place_future.done():
                self.phase, self._phase_t = "settling", now
        elif self.phase == "settling":
            self._cmd(0.0, 0.0)
            if now - self._phase_t >= 1.5:
                x, y, _ = self.pose
                near = math.hypot(x - self.store_xy[0], y - self.store_xy[1]) < 0.05
                still = abs(self.odom_v[0]) < 0.01 and abs(self.odom_v[1]) < 0.01
                if near and still:
                    self._begin_episode(now)
                elif now - self._phase_t > 10.0:
                    if self._place_tries >= 3:
                        self._fail("could not place the robot at the store")
                    else:
                        self._place()
        elif self.phase == "run":
            self._run(now)

    def _place(self) -> None:
        self._place_tries += 1
        st = EntityState()
        st.name = ROBOT
        st.pose.position.x, st.pose.position.y, st.pose.position.z = self.store_xy[0], self.store_xy[1], 0.02
        st.pose.orientation.w = 1.0
        st.reference_frame = "world"
        rq = SetEntityState.Request()
        rq.state = st
        self._place_future = self.set_state.call_async(rq)
        self.phase = "placing"

    def _begin_episode(self, now: float) -> None:
        self.t0 = self.sim0 = now
        self.wall_ep0 = time.monotonic()
        if self.decider == "node":
            self.remote = decision_core.RemotePolicy(self._send_request, timeout_s=self.decision_timeout_s)
            policy = self.remote
        else:
            policy = ReplayPolicy(self.trace["decisions"], self.max_wait_s)
        self.runner = B1Runner(self.scn, self.seed, policy, self.duration, pace=self.pace)
        if self.flow:
            self.flow_tracker = b1_flow.FlowTracker(self._load_flow_paths(), len(self.runner.ids), self.scn.wip0)
        self.phase = "run"
        self.get_logger().info(f"episode start at sim {now:.2f} s; robot at {self.pose[0]:.3f},{self.pose[1]:.3f}")
        self.runner.update(0.0, NavObs(pose=self.pose))
        self._send_flow(0.0)
        self._flush_events()
        self._publish_state(0.0, force=True)
        self._send_visuals(self.tracker.changed(self._view(0.0)))

    def _run(self, now: float) -> None:
        r = self.runner
        t = now - self.t0
        obs = NavObs(goal_id=self.driver.goal_id, status=self.driver.status, result=self.driver.result, pose=self.pose)
        try:
            r.update(t, obs)
        except Exception as e:
            self.get_logger().error(f"b1 runner failed: {e!r}")
            r.abort(f"exception: {e!r}")
            self.exit_code = 1
        self._send_flow(t)
        g = r.nav_goal
        if g is None:
            if self.driver.status == "running":
                self.driver.cancel()
        elif g.goal_id != self.driver.goal_id or self.driver.status == "idle":
            self.driver.start(g.goal_id, now, self.pose, g.xy[0], g.xy[1], f"{g.from_name}->{g.name}")
        if self.driver.status == "running":
            v, w = self.driver.step(now, self.pose, self.odom_v[0], self.odom_v[1], self.safety_stop)
        else:
            v, w = 0.0, 0.0
        self._cmd(v, w)
        self._flush_events()
        self._publish_state(t)
        self._send_visuals(self.tracker.changed(self._view(t)))
        if now - self._last_visual_all >= 2.0:
            self._send_visuals(self.tracker.everything())
            self._last_visual_all = now
        if r.ended:
            self._finish(now)

    def _send_flow(self, t: float) -> None:
        events = self.runner.drain_flow()
        if self.flow_tracker:
            self._send_visuals([(b1_visuals.TOPIC["flow"], c) for c in self.flow_tracker.feed(events, t)])

    def _flush_events(self) -> None:
        for kind, body in self.runner.drain():
            if kind == "decision":
                if self.decider == "inprocess":
                    self._pub(self.pub_decision, "dispatch_decision", body)
            elif kind == "transport":
                self._pub(self.pub_transport, "transport_status", body)

    def _publish_state(self, t: float, force: bool = False) -> None:
        sig = self.runner.state_signature()
        if force or sig != self._last_sig or t - self._last_state_t >= 1.0:
            body = self.runner.state_body(t)
            req = self.remote.pending_request if self.remote else None
            if req is not None:
                body = {**body, "decision_request": None}      # placeholder keeps key order; the request itself is not rounded
            msg = messages.envelope(self.versions.next(), "factory_state", body)
            if req is not None:
                msg["decision_request"] = req
            s = String()
            s.data = messages.dumps(msg)
            self.pub_state.publish(s)
            self._last_sig, self._last_state_t = sig, t

    def _finish(self, now: float) -> None:
        r = self.runner
        self._cmd(0.0, 0.0)
        wall = time.monotonic() - self.wall_ep0
        sim = now - self.sim0
        rtf = sim / wall if wall > 0 else None
        metrics = r.metrics(mean_rtf=rtf, extra={"wall_s": round(wall, 1), "sim_s": round(sim, 1), "result_dir": str(self.out_dir),
                                                  "trace": self.trace_path, "replay_calc_model_completed": 20,
                                                  "decider": self.decider, **({"decision_node": r.policy.stats()} if self.remote else {}),
                                                  "safety_stops": (self.safety_count or 0),
                                                  "nav_params": dataclasses.asdict(self.nav_params)})
        self._flush_events()
        self._pub(self.pub_metrics, "metrics", metrics)
        self.out_dir.mkdir(parents=True, exist_ok=True)

        def wjl(name, rows):
            (self.out_dir / name).write_text("".join(json.dumps(x, ensure_ascii=False) + "\n" for x in rows), encoding="utf-8")

        wjl("decisions.jsonl", r.decisions)
        wjl("legs.jsonl", r.legs)
        wjl("tasks.jsonl", r.tasks)
        (self.out_dir / "metrics.json").write_text(json.dumps(metrics, ensure_ascii=False, indent=1), encoding="utf-8")
        self.get_logger().info(f"EPISODE END b1_replay completed={metrics['completed']} (calc model 20) decisions={metrics['decisions']} "
                               f"skipped={len(metrics['replay_skipped'])} legs={metrics['legs']} "
                               f"over_plan_mean={metrics['leg_over_plan_mean_s']} nav_failures={r.nav_failures} rtf={rtf} -> {self.out_dir}")
        self.phase = "done"
        self.done = True

    def _fail(self, why: str) -> None:
        self.get_logger().error(why)
        self.exit_code = 1
        self.done = True

    def _watchdog(self) -> None:
        if self.done:
            return
        wall = time.monotonic() - self.wall_start
        if self.phase in ("wait", "placing", "settling"):
            if wall > self.startup_wall_s:
                self._fail(f"startup timeout ({wall:.0f} s wall) in phase {self.phase}")
            return
        if self.phase == "run":
            limit = self.max_wall_s or (self.duration * 6 + 180)
            if time.monotonic() - self.wall_ep0 > limit:
                self.get_logger().error(f"wall time limit {limit:.0f} s exceeded")
                self.runner.abort(f"wall_timeout_{limit:.0f}s")
                self.exit_code = 1
            now = self.now()
            if now != self._last_sim_seen[0]:
                self._last_sim_seen = (now, time.monotonic())
            elif time.monotonic() - self._last_sim_seen[1] > 60.0:
                self.get_logger().error("simulation clock stalled for 60 s")
                self.runner.abort("sim_clock_stalled")
                self.exit_code = 1
                self._finish(now)


def main(argv=None) -> int:
    rclpy.init(args=argv)
    node = B1Node()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
        t_end = time.monotonic() + 1.5
        while time.monotonic() < t_end and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        code = node.exit_code
        node.destroy_node()
        rclpy.try_shutdown()
    return code


if __name__ == "__main__":
    sys.exit(main())
