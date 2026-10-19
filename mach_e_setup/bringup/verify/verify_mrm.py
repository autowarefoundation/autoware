#!/usr/bin/env python3
"""Count MRM episodes while engaged; print first episodes and a summary every 2 min of driving."""
import time, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from autoware_adapi_v1_msgs.msg import MrmState, OperationModeState
rclpy.init(); n = Node('claude_verify_mrm')
st = {'auto': False, 'mrm': False, 'eps': 0, 'mrm_t': 0.0, 'auto_t': 0.0, 't': time.time(), 'last_sum': time.time(), 'tot_eps': 0}
def p(s): print(time.strftime('%H:%M:%S ') + s, flush=True)
def on_mode(m):
    a = (m.mode == 2 and m.is_autoware_control_enabled)
    if a != st['auto']: p(f"ENGAGED={a}")
    st['auto'] = a
def on_mrm(m):
    now = time.time(); dt = now - st['t']; st['t'] = now
    if st['auto']:
        st['auto_t'] += dt
        if st['mrm']: st['mrm_t'] += dt
    bad = m.state != 1
    if bad and not st['mrm'] and st['auto']:
        st['eps'] += 1; st['tot_eps'] += 1
        if st['tot_eps'] <= 3: p(f"MRM episode while engaged: state {m.state} behavior {m.behavior}")
    st['mrm'] = bad
    if now - st['last_sum'] > 120 and st['auto_t'] > 5:
        p(f"SUMMARY last ~2 min: engaged {st['auto_t']:.0f}s, MRM episodes {st['eps']}, MRM time {st['mrm_t']:.1f}s (total episodes {st['tot_eps']})")
        st.update(eps=0, mrm_t=0.0, auto_t=0.0, last_sum=now)
tl = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
n.create_subscription(OperationModeState, '/api/operation_mode/state', on_mode, tl)
n.create_subscription(MrmState, '/system/fail_safe/mrm_state', on_mrm, 10)
p('watching MRM'); rclpy.spin(n)
