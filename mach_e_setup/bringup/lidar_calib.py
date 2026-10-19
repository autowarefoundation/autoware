#!/usr/bin/env python3
"""Check / correct the Ouster -> base_link extrinsic (sensor_kit_calibration.yaml: ouster_base_link).

  lidar_calib.py ground            parked on locally flat ground (asphalt, no curbs right next to
                                   the car): fits the ground plane -> lidar z, roll, pitch
  lidar_calib.py drive [--seconds 180]
                                   while driving (Autoware localization running): straights at
                                   >2 m/s and some normal turns. No-slip rule: base_link (rear axle)
                                   moves along its own heading and has no sideways velocity.
                                   -> lidar yaw and x from NDT, plus ADMA heading offset and
                                   where ADMA POI 1 sits along the car
  lidar_calib.py drive --file F    re-analyze a saved drive recording

base_link is the rear-axle centre on the ground, so the ground plane must be z = 0 and level in
base_link. x, y, yaw are not observable from the ground; y is best measured with a tape.
"""
import argparse
import math
import os
import sys
import time

import numpy as np
import yaml

CONFIG_DIR = os.path.expanduser(
    '~/autoware.mach_e/src/launcher/autoware_launch/sensor_kit/mach_e_sensor_kit_launch/'
    'mach_e_sensor_kit_description/config')
LIDAR_KEY = 'ouster_base_link'
LIDAR_TOPIC = '/sensing/lidar/top/pointcloud_raw_ex'
OUT_DIR = os.path.expanduser('~/autoware_logs/calib')


# --- transforms (URDF rpy: R = Rz(yaw) Ry(pitch) Rx(roll)) -------------------------------------
def rpy_to_R(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def R_to_rpy(R):
    return (math.atan2(R[2, 1], R[2, 2]), math.asin(-max(-1.0, min(1.0, R[2, 0]))),
            math.atan2(R[1, 0], R[0, 0]))


def load_tf(entry):
    return rpy_to_R(entry['roll'], entry['pitch'], entry['yaw']), np.array([entry['x'], entry['y'], entry['z']])


def load_calibration():
    with open(os.path.join(CONFIG_DIR, 'sensors_calibration.yaml')) as f:
        R_b_s, t_b_s = load_tf(yaml.safe_load(f)['base_link']['sensor_kit_base_link'])
    with open(os.path.join(CONFIG_DIR, 'sensor_kit_calibration.yaml')) as f:
        lidar = yaml.safe_load(f)['sensor_kit_base_link'][LIDAR_KEY]
    R_s_l, t_s_l = load_tf(lidar)
    return lidar, (R_b_s, t_b_s), (R_s_l, t_s_l)


def print_entry(title, x, y, z, roll, pitch, yaw):
    print(f'{title}\n  {LIDAR_KEY}:\n    x: {x:.4f}\n    y: {y:.4f}\n    z: {z:.4f}\n'
          f'    roll: {roll:.4f}\n    pitch: {pitch:.4f}\n    yaw: {yaw:.4f}')


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


# --- ground: lidar z / roll / pitch ------------------------------------------------------------
def collect_scans(n_scans):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2

    rclpy.init()
    node = rclpy.create_node('lidar_calib_ground')
    clouds, frames = [], set()

    def cb(msg):
        if len(clouds) < n_scans:
            frames.add(msg.header.frame_id)
            # read_points_numpy needs all fields to share a datatype; the Ouster cloud mixes them
            rec = point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
            clouds.append(np.stack([rec[n].astype(np.float64) for n in ('x', 'y', 'z')], axis=1))

    node.create_subscription(PointCloud2, LIDAR_TOPIC, cb, qos_profile_sensor_data)
    end = time.time() + 20
    while len(clouds) < n_scans and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()
    if not clouds:
        sys.exit(f'no scans on {LIDAR_TOPIC} - is the lidar driver running?')
    return np.vstack(clouds), frames


def fit_plane(P, iters=300, thresh=0.05, rng=np.random.default_rng(0)):
    best = None
    for _ in range(iters):
        a, b, c = P[rng.choice(len(P), 3, replace=False)]
        n = np.cross(b - a, c - a)
        if np.linalg.norm(n) < 1e-6:
            continue
        n /= np.linalg.norm(n)
        inl = np.abs((P - a) @ n) < thresh
        if best is None or inl.sum() > best.sum():
            best = inl
    Q = P[best]
    c = Q.mean(axis=0)
    n = np.linalg.svd(Q - c, full_matrices=False)[2][-1]
    n = n if n[2] > 0 else -n
    return n, c, best


def ground(args):
    lidar, (R_b_s, t_b_s), (R_s_l, t_s_l) = load_calibration()
    P_l, frames = collect_scans(args.scans)
    if frames != {'os_sensor'}:
        print(f'WARNING: cloud frame_id {frames}, expected os_sensor')
    R_b_l, t_b_l = R_b_s @ R_s_l, R_b_s @ t_s_l + t_b_s
    P_b = P_l @ R_b_l.T + t_b_l

    r = np.hypot(P_b[:, 0] - 1.5, P_b[:, 1])  # distance from roughly the car centre
    cand = (r > args.min_range) & (r < args.max_range) & (np.abs(P_b[:, 2]) < 0.6)
    P = P_b[cand]
    if len(P) > 300000:
        P = P[np.random.default_rng(1).choice(len(P), 300000, replace=False)]
    if len(P) < 2000:
        sys.exit(f'only {len(P)} candidate ground points near z=0 - is the current z badly off?')
    n, c, inl = fit_plane(P)
    Q = P[inl]
    rms = np.sqrt(np.mean(((Q - c) @ n) ** 2))
    tilt = math.degrees(math.acos(min(1.0, n[2])))
    print(f'{len(P_l)} points from {args.scans} scans, {inl.sum()} ground inliers '
          f'({100 * inl.mean():.0f}% of candidates), plane RMS {100 * rms:.1f} cm')
    # quadrant check: a sloped driveway or crown shows up as different heights per side
    for name, m in (('front', Q[:, 0] > 1.5), ('rear', Q[:, 0] <= 1.5), ('left', Q[:, 1] > 0), ('right', Q[:, 1] <= 0)):
        if m.sum() > 100:
            print(f'  {name:5s} mean ground z {100 * Q[m, 2].mean():+6.1f} cm  ({m.sum()} pts)')
    print(f'ground in base_link with current calibration: tilt {tilt:.2f} deg, '
          f'base_link origin {-100 * (c @ n):+.1f} cm above the plane (should be 0)')

    # rotate the lidar so the plane normal becomes +z (no yaw change), then fix z so the plane is z=0
    v = np.cross(n, [0, 0, 1.0])
    s, cth = np.linalg.norm(v), n[2]
    K = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    R_c = np.eye(3) + K + K @ K * ((1 - cth) / s ** 2) if s > 1e-9 else np.eye(3)
    R_b_l_new = R_c @ R_b_l
    t_b_l_new = t_b_l.copy()
    z_ground = (Q - t_b_l) @ R_c.T @ np.array([0, 0, 1.0])  # ground z relative to the lidar origin
    t_b_l_new[2] = -np.median(z_ground)
    R_s_l_new = R_b_s.T @ R_b_l_new
    t_s_l_new = R_b_s.T @ (t_b_l_new - t_b_s)
    rr, pp, yy = R_to_rpy(R_s_l_new)
    print_entry('\ncurrent sensor_kit_calibration.yaml:', lidar['x'], lidar['y'], lidar['z'],
                lidar['roll'], lidar['pitch'], lidar['yaw'])
    print_entry('suggested (only z, roll, pitch are measured here):', lidar['x'], lidar['y'],
                t_s_l_new[2], rr, pp, lidar['yaw'] + wrap(yy - lidar['yaw']))
    print(f'change: z {100 * (t_s_l_new[2] - lidar["z"]):+.1f} cm, roll {math.degrees(wrap(rr - lidar["roll"])):+.2f} deg, '
          f'pitch {math.degrees(wrap(pp - lidar["pitch"])):+.2f} deg')
    if rms > 0.04:
        print('WARNING: plane RMS > 4 cm - ground is not flat here, move to a flatter spot')
    print('Repeat with the car turned 180 deg on the same spot: real calibration errors stay the same,\n'
          'ground slope flips sign. Average the two.')


# --- drive: lidar yaw / x, ADMA heading offset / POI 1 position ----------------------------------
def record_drive(seconds):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from adma_ros_driver_msgs.msg import AdmaDataScaled

    rclpy.init()
    node = rclpy.create_node('lidar_calib_drive')
    ndt, adma = [], []

    def on_ndt(m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        ndt.append((m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec, p.x, p.y,
                    math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))))

    last = [0.0]

    def on_adma(m):
        t = m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec
        if t - last[0] >= 0.05:  # 20 Hz is plenty
            last[0] = t
            adma.append((t, m.poi_1.ins_lat_abs, m.poi_1.ins_lon_abs, math.radians(90.0 - m.ins_yaw)))

    node.create_subscription(PoseWithCovarianceStamped, '/localization/pose_estimator/pose_with_covariance', on_ndt, 10)
    node.create_subscription(AdmaDataScaled, '/genesys/adma/data_scaled', on_adma, qos_profile_sensor_data)
    print(f'recording {seconds:.0f} s - drive straights (>2 m/s) and some normal turns')
    end = time.time() + seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
    os.makedirs(OUT_DIR, exist_ok=True)
    path = os.path.join(OUT_DIR, time.strftime('drive_%Y%m%d_%H%M%S.npz'))
    np.savez(path, ndt=np.array(ndt), adma=np.array(adma))
    print(f'saved {len(ndt)} NDT poses, {len(adma)} ADMA samples to {path}')
    return path


def motion_samples(t, x, y, yaw, dt=0.5):
    """velocity / yaw rate over ~dt windows -> (speed, v_lat along body-left, yaw_rate)."""
    out = []
    j = 0
    for i in range(len(t)):
        while j < len(t) and t[j] - t[i] < dt:
            j += 1
        if j >= len(t):
            break
        T = t[j] - t[i]
        if T > 2 * dt:
            continue
        vx, vy = (x[j] - x[i]) / T, (y[j] - y[i]) / T
        h = math.atan2(math.sin(yaw[i]) + math.sin(yaw[j]), math.cos(yaw[i]) + math.cos(yaw[j]))
        v_fwd = vx * math.cos(h) + vy * math.sin(h)
        v_lat = -vx * math.sin(h) + vy * math.cos(h)
        out.append((v_fwd, v_lat, wrap(yaw[j] - yaw[i]) / T))
    return np.array(out)


def fit_no_slip(S, name):
    """v_lat = a*v_fwd + c*omega: a = heading error (rad), c = point's x relative to rear axle."""
    if len(S) == 0:
        print(f'{name}: no data')
        return None
    m = (S[:, 0] > 2.0) & (np.abs(S[:, 0] * S[:, 2]) < 2.5)  # moving forward, lateral accel < 2.5 m/s^2
    S = S[m]
    straight = np.abs(S[:, 2]) < 0.02
    turning = np.abs(S[:, 2]) > 0.08
    if straight.sum() < 20 or turning.sum() < 20:
        print(f'{name}: not enough data ({straight.sum()} straight, {turning.sum()} turning samples; '
              f'need 20+ each) - drive more straights and turns')
        return None
    A = np.column_stack([S[:, 0], S[:, 2]])
    (a, c), *_ = np.linalg.lstsq(A, S[:, 1], rcond=None)
    res = S[:, 1] - A @ [a, c]
    med_straight = np.median(np.arctan2(S[straight, 1], S[straight, 0]))
    print(f'{name}: {len(S)} samples ({straight.sum()} straight, {turning.sum()} turning), residual {res.std():.3f} m/s\n'
          f'   heading correction (true - reported) {math.degrees(a):+.2f} deg (straights only: {math.degrees(med_straight):+.2f} deg), '
          f'point is {c:+.2f} m ahead of the rear axle')
    return a, c


def drive(args):
    path = args.file or record_drive(args.seconds)
    d = np.load(path)
    lidar, *_ = load_calibration()
    print()
    ndt = d['ndt']
    res = None
    if len(ndt) > 10:
        S = motion_samples(ndt[:, 0], ndt[:, 1], ndt[:, 2], ndt[:, 3])
        res = fit_no_slip(S, 'NDT base_link')
    adma = d['adma']
    if len(adma) > 10:
        lat0, lon0 = adma[0, 1], adma[0, 2]
        x = (adma[:, 2] - lon0) * 111320 * math.cos(math.radians(lat0))
        y = (adma[:, 1] - lat0) * 110790
        fit_no_slip(motion_samples(adma[:, 0], x, y, adma[:, 3]), 'ADMA POI 1')
        print('   -> ADMA heading error: set yaw_offset_deg in gnss.launch.xml to the ADMA heading correction above;\n'
              '      POI 1 position: should match gnss_link x in sensor_kit_calibration.yaml')
    if res:
        a, c = res
        # base_link heading error a means the lidar yaw in the TF is off by a; base_link sitting c ahead
        # of the rear axle means the TF lidar x is c too small.
        print_entry('\nsuggested for NDT (yaw and x measured here; keep z/roll/pitch from "ground"):',
                    lidar['x'] + c, lidar['y'], lidar['z'], lidar['roll'], lidar['pitch'], lidar['yaw'] - a)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    g = sub.add_parser('ground', help='parked: lidar z / roll / pitch from the ground plane')
    g.add_argument('--scans', type=int, default=10)
    g.add_argument('--min-range', type=float, default=4.0)
    g.add_argument('--max-range', type=float, default=15.0)
    dr = sub.add_parser('drive', help='driving: lidar yaw / x, ADMA heading offset')
    dr.add_argument('--seconds', type=float, default=180.0)
    dr.add_argument('--file', help='analyze a saved recording instead of recording')
    args = ap.parse_args()
    ground(args) if args.cmd == 'ground' else drive(args)


if __name__ == '__main__':
    main()
