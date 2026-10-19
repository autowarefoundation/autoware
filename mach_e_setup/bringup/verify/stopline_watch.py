#!/usr/bin/env python3
"""Live stop-line watcher: prints one line per notable event (mode change, stop_line
planning factor appearing/disappearing, ego crossing any map stop line)."""
import math, time, xml.etree.ElementTree as ET
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry
from autoware_internal_planning_msgs.msg import PlanningFactorArray
from autoware_adapi_v1_msgs.msg import OperationModeState
from autoware_planning_msgs.msg import Trajectory
from autoware_control_msgs.msg import Control

OSM = '/home/rancslab/autoware_map/PD2LAB/lanelet2_map.osm'
MODES = {0: 'UNKNOWN', 1: 'STOP', 2: 'AUTONOMOUS', 3: 'LOCAL', 4: 'REMOTE'}


def load_stop_lines():
    r = ET.parse(OSM).getroot()
    tags = lambda e: {t.get('k'): t.get('v') for t in e.findall('tag')}
    nodes = {n.get('id'): (float(tags(n)['local_x']), float(tags(n)['local_y'])) for n in r.findall('node')}
    lines = {}
    for w in r.findall('way'):
        if tags(w).get('type') == 'stop_line':
            lines[w.get('id')] = [nodes[nd.get('ref')] for nd in w.findall('nd')]
    owner = {}
    for rel in r.findall('relation'):
        t = tags(rel)
        if t.get('type') == 'regulatory_element':
            for m in rel.findall('member'):
                if m.get('role') == 'ref_line':
                    owner[m.get('ref')] = f"RE{rel.get('id')}/{t.get('subtype')}"
    return {k: (v, owner.get(k, 'unreferenced')) for k, v in lines.items()}


def cross(o, p, q):
    return (p[0] - o[0]) * (q[1] - o[1]) - (p[1] - o[1]) * (q[0] - o[0])


def seg_hit(a, b, c, d):
    return cross(a, b, c) * cross(a, b, d) < 0 and cross(c, d, a) * cross(c, d, b) < 0


class W(Node):
    def __init__(self):
        super().__init__('claude_stopline_watch')
        self.lines = load_stop_lines()
        self.mode = None
        self.prev = None
        self.speed = 0.0
        self.factor = None          # last stop_line factor (dist, t)
        self.factor_on = False
        self.last_factor_t = 0.0
        self.traj_min_v = None
        self.cmd_acc = None
        self.min_speed_hist = []    # (t, x, y, v)
        tl = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OperationModeState, '/api/operation_mode/state', self.on_mode, tl)
        self.create_subscription(Odometry, '/localization/kinematic_state', self.on_odom, 10)
        self.create_subscription(PlanningFactorArray, '/planning/planning_factors/stop_line', self.on_factor, 10)
        self.create_subscription(Trajectory, '/planning/trajectory', self.on_traj, 1)
        self.create_subscription(Control, '/control/command/control_cmd', self.on_cmd, 1)
        self.p(f"watching {len(self.lines)} stop lines")

    def p(self, s):
        print(time.strftime('%H:%M:%S ') + s, flush=True)

    def on_mode(self, m):
        if m.mode != self.mode:
            self.p(f"MODE {MODES.get(self.mode)} -> {MODES.get(m.mode)} (aw_ctrl={m.is_autoware_control_enabled}) v={self.speed*3.6:.1f}km/h")
            self.mode = m.mode

    def on_cmd(self, c):
        self.cmd_acc = c.longitudinal.acceleration

    def on_traj(self, t):
        if t.points:
            self.traj_min_v = min(p.longitudinal_velocity_mps for p in t.points[:200])

    def on_factor(self, msg):
        now = time.time()
        stops = [f for f in msg.factors if f.behavior == 3 and f.control_points]
        if stops:
            d = min(cp.distance for f in stops for cp in f.control_points)
            self.factor = d
            self.last_factor_t = now
            if not self.factor_on:
                self.p(f"STOP_LINE factor ON: dist {d:.1f} m, ego {self.speed*3.6:.1f} km/h, mode {MODES.get(self.mode)}, traj_min_v {self.traj_min_v}")
                self.factor_on = True
        elif self.factor_on:
            self.p(f"STOP_LINE factor OFF (last dist {self.factor:.1f} m), ego {self.speed*3.6:.1f} km/h")
            self.factor_on = False

    def on_odom(self, o):
        now = time.time()
        x, y = o.pose.pose.position.x, o.pose.pose.position.y
        self.speed = o.twist.twist.linear.x
        self.min_speed_hist.append((now, x, y, abs(self.speed)))
        self.min_speed_hist = [h for h in self.min_speed_hist if now - h[0] < 15]
        if self.prev is not None and math.dist(self.prev, (x, y)) < 5:
            for lid, (pts, owner) in self.lines.items():
                if any(seg_hit(self.prev, (x, y), pts[i], pts[i + 1]) for i in range(len(pts) - 1)):
                    near = [h[3] for h in self.min_speed_hist if math.dist((h[1], h[2]), (x, y)) < 12]
                    vmin = min(near) if near else float('nan')
                    stopped = vmin < 0.3
                    tag = 'OK (stopped)' if stopped else '*** OVERRUN (no stop) ***'
                    self.p(f"CROSSED stop line {lid} [{owner}] at {self.speed*3.6:.1f} km/h, min speed in last 12 m {vmin*3.6:.1f} km/h, "
                           f"mode {MODES.get(self.mode)}, stop factor seen {now - self.last_factor_t:.1f}s ago, "
                           f"traj_min_v {self.traj_min_v}, cmd_acc {self.cmd_acc}  -> {tag}")
        self.prev = (x, y)


def main():
    rclpy.init()
    rclpy.spin(W())


if __name__ == '__main__':
    main()
