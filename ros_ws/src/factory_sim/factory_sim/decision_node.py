"""decision_node: the dispatch decision of B1 as its own ROS 2 node.

  subscribes  /factory/state    (JSON; a message with a `decision_request` key asks for a decision, see factory_sim.decision_core)
  publishes   /dispatch/decision (JSON; request_id, choice | null (+ wait), policy, source, rule, info incl. Jev probabilities)
Parameters  : policy = replay | jev_b1x
              replay   : trace (recorded decisions, default recorded/b1x_seed5_empty.json), replay_max_wait_s
              jev_b1x  : jev_transport = mock (default, no network) | sdk (real API; start it on purpose only),
                         jev_sdk_path, jev_call_limit, jev_timeout_s. The Jev call runs in a worker thread, so the node keeps spinning.
Duplicates of a request (the state message repeats it every second) are answered once.
"""
from __future__ import annotations

import json
import queue
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from . import decision_core as dc
from . import paths

paths.ensure_importable()


def build_decider(policy: str, *, trace: str, max_wait_s: float, jev_transport: str = "mock", jev_sdk_path: str = "",
                  jev_call_limit: int = 150, jev_timeout_s: float = 10.0):
    if policy == "replay":
        return dc.ReplayDecider(json.loads(Path(trace).read_text(encoding="utf-8"))["decisions"], max_wait_s)
    if policy == "jev_b1x":
        from policies.jev_b1 import JevB1Policy
        if jev_transport == "mock":
            from adapters.jev import MockTransport
            inner = MockTransport()
        else:
            if jev_sdk_path and jev_sdk_path not in sys.path:
                sys.path.insert(0, jev_sdk_path)
            from adapters.jev import SdkTransport
            inner = SdkTransport()
        from .call_limit import CountingTransport
        return dc.JevDecider(JevB1Policy(CountingTransport(inner, limit=jev_call_limit), "b1x", timeout_s=jev_timeout_s), "b1x")
    raise ValueError(f"unknown policy {policy!r} (replay | jev_b1x)")


class DecisionNode(Node):
    def __init__(self) -> None:
        super().__init__("decision_node")
        d = self.declare_parameter
        root = paths.find_project_root()
        self.policy_name = d("policy", "replay").value
        trace = d("trace", str(root / "recorded" / "b1x_seed5_empty.json")).value
        self.decider = build_decider(
            self.policy_name, trace=trace, max_wait_s=float(d("replay_max_wait_s", 20.0).value),
            jev_transport=d("jev_transport", "mock").value, jev_sdk_path=d("jev_sdk_path", "").value,
            jev_call_limit=int(d("jev_call_limit", 150).value), jev_timeout_s=float(d("jev_timeout_s", 10.0).value))
        self.async_policy = self.policy_name == "jev_b1x"
        self.seen: set[str] = set()
        self.results: queue.Queue = queue.Queue()
        self.pool = ThreadPoolExecutor(max_workers=1) if self.async_policy else None
        self.lock = threading.Lock()
        reliable = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(String, "/dispatch/decision", reliable)
        self.create_subscription(String, "/factory/state", self._on_state, reliable)
        self.create_timer(0.005, self._drain)
        self.n = 0
        self.get_logger().info(f"decision_node: policy={self.policy_name} ({getattr(self.decider, 'name', '')})")

    def _on_state(self, m: String) -> None:
        try:
            req = json.loads(m.data).get("decision_request")
        except ValueError:
            return
        if req is None:
            return
        if not dc.valid_request(req):
            self.get_logger().warning("ignored an invalid decision_request")
            return
        rid = req["request_id"]
        if rid in self.seen:                       # repeated request: answered (or being answered) already
            return
        self.seen.add(rid)
        if self.pool is not None:
            self.pool.submit(self._work, req)
        else:
            self._work(req)

    def _work(self, req: dict) -> None:
        try:
            ans = self.decider.decide(req)
        except Exception as e:                     # never leave b1_node waiting for the timeout without a reason
            self.get_logger().error(f"decide failed for {req['request_id']}: {e!r}")
            ans = dc.make_answer(req, self.policy_name, None, "error", f"exception: {e!r}"[:200], {}, wait=True)
        self.results.put(ans)
        if self.pool is None:
            self._drain()

    def _drain(self) -> None:
        while True:
            try:
                ans = self.results.get_nowait()
            except queue.Empty:
                return
            s = String()
            s.data = json.dumps(ans, ensure_ascii=False, separators=(",", ":"), allow_nan=False)
            self.pub.publish(s)
            self.n += 1


def main(argv=None) -> int:
    rclpy.init(args=argv)
    node = DecisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
