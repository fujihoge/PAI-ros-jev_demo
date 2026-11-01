#!/usr/bin/env python3
"""低レベルの安全層: LiDAR による衝突停止。判断ロジックは含まない。

  購読  /cmd_vel_raw (geometry_msgs/Twist) : 上位（b1_node の航法部）からの指令
        /scan        (sensor_msgs/LaserScan): lidar_link 基準（+x が前方）
  配信  /cmd_vel     (geometry_msgs/Twist) : ロボット（diff_drive）への指令
        /safety/stop       (std_msgs/Bool)  : 安全停止が効いている間 true（10 Hz と変化時）
        /safety/stop_count (std_msgs/Int32) : 停止の発動回数（立ち上がりの累計。後続の「衝突・航法失敗」の指標用）
        /safety/margin     (std_msgs/Float32): 進行方向の「障害物までの隙間 - 必要な隙間」[m]（負で停止。進行なしは 9.9）

方式:
  進行方向（指令 vx の符号）の矩形の通路（ロボット中心線から左右 half_width=0.18 m、前方向のみ）にある LiDAR 点のうち、
  車体の端（前 front_edge / 後 rear_edge を LiDAR 原点から測る）に最も近いものまでの隙間 gap を求める。
  必要な隙間 req(v) = margin + (latency + stop_per_v) * |v| = 0.10 + 0.45*|v| [m]（v=0.6 で 0.37、1.0 で 0.55）
  ... 速度に比例して縮むので、
  到着直前の低速域では自然にしきい値が小さくなる（正規の停止位置で誤って止まらない）。
  gap < req なら vx を 0 にして出す（旋回 wz はそのまま通す）。gap > req + hysteresis で解除。
  後退（vx<0）も同じ規則で後方の通路を見る（後方が空いていれば後退して脱出できる）。旋回は常に許可。
  LiDAR の最小距離 0.12 m の不感帯に入らないよう、停止後の隙間は margin(=0.10 m 既定) 程度を狙う。
  LiDAR の面は z=0.28 m。これより低い物（棚の最下段の板など）や高い物（トレイ z=0.40〜0.45）は見えない。
  停止中は直近の進行方向で障害物の有無を見続け、消える（隙間 > req + hysteresis）まで停止状態を保つ。
  /cmd_vel_raw が一度も来ていない間は何も出さない（他の /cmd_vel の出し手を邪魔しない）。
  /scan が scan_timeout 秒以上来ないとき、/cmd_vel_raw が cmd_timeout 秒以上来ないときは、安全側（前進 0 / 停止）。
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, Int32


def required_gap(v, margin, latency, stop_per_v):
    return margin + (latency + stop_per_v) * abs(v)


def corridor_gap(angle_min, angle_inc, ranges, rmin, rmax, direction, half_width, edge):
    """進行方向 direction(+1 前 / -1 後) の通路内で、車体端までの最小の隙間 [m]。無ければ inf。"""
    best = math.inf
    a = angle_min
    for r in ranges:
        if math.isfinite(r) and rmin <= r <= rmax:
            x = direction * r * math.cos(a)
            y = r * math.sin(a)
            if x > 0.0 and abs(y) < half_width:
                g = x - edge
                if g < best:
                    best = g
        a += angle_inc
    return best


class SafetyStop(Node):
    def __init__(self):
        super().__init__('safety_stop')
        p = self.declare_parameter
        self.cmd_in = p('cmd_in', '/cmd_vel_raw').value
        self.cmd_out = p('cmd_out', '/cmd_vel').value
        self.half_width = p('half_width', 0.18).value
        self.front_edge = p('front_edge', 0.05).value   # 前端 0.17 - LiDAR 取り付け x 0.12
        self.rear_edge = p('rear_edge', 0.29).value     # 後端 0.17 + 0.12
        self.margin = p('margin', 0.10).value
        self.latency = p('latency', 0.28).value
        self.stop_per_v = p('stop_per_v', 0.17).value   # 停止距離 ≒ 0.17 s * v（実測 v=0.6 で 0.10 m、1.0 で 0.17 m）
        self.hyst = p('hysteresis', 0.03).value
        self.scan_timeout = p('scan_timeout', 1.0).value
        self.cmd_timeout = p('cmd_timeout', 0.5).value
        self.enabled = p('enabled', True).value
        self.scan = None
        self.scan_t = None
        self.cmd = Twist()
        self.cmd_t = None
        self.stopped = False
        self.last_v = 0.0      # 直近の非ゼロ指令（停止中に指令が 0 になっても、障害物が消えるまでは停止状態を保つ）
        self.count = 0
        self.margin_val = 9.9
        self.pub = self.create_publisher(Twist, self.cmd_out, 10)
        self.pub_stop = self.create_publisher(Bool, '/safety/stop', 10)
        self.pub_cnt = self.create_publisher(Int32, '/safety/stop_count', 10)
        self.pub_margin = self.create_publisher(Float32, '/safety/margin', 10)
        self.create_subscription(Twist, self.cmd_in, self._on_cmd, 10)
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        self.create_timer(0.1, self._tick)
        self.get_logger().info(
            f'safety_stop: {self.cmd_in} -> {self.cmd_out}, half_width={self.half_width} margin={self.margin} '
            f'req(v)=margin+{self.latency + self.stop_per_v:.2f}*|v|, edges front={self.front_edge} rear={self.rear_edge}')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_cmd(self, m):
        self.cmd, self.cmd_t = m, self.now()
        self._apply()

    def _on_scan(self, m):
        self.scan, self.scan_t = m, self.now()
        self._apply()

    def _tick(self):
        t = self.now()
        if self.cmd_t is not None and t - self.cmd_t > self.cmd_timeout and (self.cmd.linear.x or self.cmd.angular.z):
            self.cmd = Twist()
            self.pub.publish(Twist())   # 上位が止まったら止める（watchdog）
        b = Bool()
        b.data = self.stopped
        self.pub_stop.publish(b)

    def _apply(self):
        if self.cmd_t is None:   # 上位から一度も指令が来ていない間は何も出さない（他の /cmd_vel の出し手を邪魔しない）
            return
        v = self.cmd.linear.x
        out = Twist()
        out.linear.y, out.linear.z = self.cmd.linear.y, self.cmd.linear.z
        out.angular = self.cmd.angular
        out.linear.x = v
        blocked, gap, req, why = False, math.inf, 0.0, ''
        moving = abs(v) > 1e-3
        if moving:
            self.last_v = v
        vv = v if moving else (self.last_v if self.stopped else 0.0)   # 停止中は直近の向きで障害物の有無を見続ける
        if self.enabled and abs(vv) > 1e-3:
            d = 1.0 if vv > 0 else -1.0
            if self.scan is None or self.now() - self.scan_t > self.scan_timeout:
                blocked, why = True, 'scan_timeout'
            else:
                s = self.scan
                gap = corridor_gap(s.angle_min, s.angle_increment, s.ranges, s.range_min, s.range_max,
                                   d, self.half_width, self.front_edge if d > 0 else self.rear_edge)
                req = required_gap(vv, self.margin, self.latency, self.stop_per_v)
                thr = req + (self.hyst if self.stopped else 0.0)
                blocked = gap < thr
                why = f'gap={gap:.3f} < req={req:.3f}'
            if not moving:
                out.linear.x = 0.0
            self.margin_val = (gap - req) if math.isfinite(gap) else 9.9
        else:
            self.margin_val = 9.9
        if blocked:
            out.linear.x = 0.0
        if blocked and not self.stopped:
            self.count += 1
            self.get_logger().warn(f'SAFETY STOP #{self.count}: vx={v:.2f} {why}')
        elif self.stopped and not blocked:
            self.get_logger().info('safety stop released')
        self.stopped = blocked
        self.pub.publish(out)
        c = Int32()
        c.data = self.count
        self.pub_cnt.publish(c)
        mg = Float32()
        mg.data = float(max(-9.9, min(9.9, self.margin_val)))
        self.pub_margin.publish(mg)


def main():
    rclpy.init()
    n = SafetyStop()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
