#!/usr/bin/env python3
"""Measure where the ADMA/GNSS initial pose disagrees with NDT, and why.

Park at a spot with NDT converged, then record a few seconds there:
    ndt_gnss_offset.py record --label lot_north_facing_east
Repeat at 3-4 spots spread across the map, with different headings (also record one spot
twice, facing opposite ways). Then fit the error models:
    ndt_gnss_offset.py analyze

Models (NDT pose minus GNSS-derived base_link pose, map frame):
    A  map offset    constant (dx, dy) in the map         -> map origin / georeference shift
    B  lever arm     constant (fwd, left) in the vehicle  -> ADMA POI vs gnss_link TF mismatch
    C  map rotation  rotation about the map origin + shift -> projection / map heading rotation
    D  C + B
The one with the smallest RMS (and sensible parameters) is the likely cause. The yaw column
(NDT yaw minus ADMA heading in ENU) separates a rotated map (yaw diff ~= C's rotation) from
an ADMA heading/mounting bias (yaw diff without a matching C rotation).
Needs: ADMA driver + Autoware localization running, vehicle at standstill.
"""
import argparse
import csv
import math
import os
import sys
import time

DEFAULT_CSV = os.path.expanduser('~/autoware_logs/ndt_gnss_offset.csv')
FIELDS = ['time', 'label', 'n_ndt', 'n_gnss', 'gnss_x', 'gnss_y', 'gnss_z', 'ndt_x', 'ndt_y',
          'ndt_z', 'ndt_yaw_deg', 'adma_yaw_enu_deg', 'adma_stddev_pos', 'adma_stddev_yaw',
          'nvtl']
NVTL_OK = 1.5  # ndt_scan_matcher converged_param_nearest_voxel_transformation_likelihood (lab fork; upstream 2.3)


def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


def circ_mean_deg(angles):
    s = sum(math.sin(math.radians(a)) for a in angles)
    c = sum(math.cos(math.radians(a)) for a in angles)
    return math.degrees(math.atan2(s, c))


def quat_yaw_deg(q):
    return math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))


def record(args):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
    from adma_ros_driver_msgs.msg import AdmaDataScaled
    from autoware_internal_debug_msgs.msg import Float32Stamped

    rclpy.init()
    node = rclpy.create_node('ndt_gnss_offset')
    ndt, gnss, adma, nvtl = [], [], [], []
    node.create_subscription(
        PoseWithCovarianceStamped, '/localization/pose_estimator/pose_with_covariance',
        lambda m: ndt.append(m.pose.pose), 10)
    node.create_subscription(PoseStamped, '/sensing/gnss/pose', lambda m: gnss.append(m.pose), 10)
    node.create_subscription(
        AdmaDataScaled, '/genesys/adma/data_scaled', adma.append, qos_profile_sensor_data)
    node.create_subscription(
        Float32Stamped, '/localization/pose_estimator/nearest_voxel_transformation_likelihood',
        lambda m: nvtl.append(m.data), qos_profile_sensor_data)

    print(f'recording {args.seconds:.0f} s at "{args.label}" - keep the vehicle still')
    end = time.time() + args.seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

    missing = [n for n, buf in (('NDT pose', ndt), ('GNSS pose', gnss), ('ADMA', adma)) if not buf]
    if missing:
        sys.exit('no data from: ' + ', '.join(missing) + ' (is localization initialized and '
                 'is the lidar pipeline publishing /localization/util/downsample/pointcloud?)')
    speed = max(math.hypot(a.ins_vel_hor.x, a.ins_vel_hor.y) for a in adma)
    if speed > 0.1:
        sys.exit(f'vehicle moved (ADMA speed up to {speed:.2f} m/s) - record at standstill')

    def mean(vals):
        return sum(vals) / len(vals)

    row = {
        'time': time.strftime('%Y-%m-%d %H:%M:%S'), 'label': args.label,
        'n_ndt': len(ndt), 'n_gnss': len(gnss),
        'gnss_x': mean([p.position.x for p in gnss]), 'gnss_y': mean([p.position.y for p in gnss]),
        'gnss_z': mean([p.position.z for p in gnss]),
        'ndt_x': mean([p.position.x for p in ndt]), 'ndt_y': mean([p.position.y for p in ndt]),
        'ndt_z': mean([p.position.z for p in ndt]),
        'ndt_yaw_deg': circ_mean_deg([quat_yaw_deg(p.orientation) for p in ndt]),
        'adma_yaw_enu_deg': circ_mean_deg([90.0 - a.ins_yaw for a in adma]),
        'adma_stddev_pos': max(max(a.ins_stddev_lat, a.ins_stddev_long) for a in adma),
        'adma_stddev_yaw': max(a.ins_stddev_yaw for a in adma),
        'nvtl': mean(nvtl) if nvtl else float('nan'),
    }
    new = not os.path.exists(args.csv)
    os.makedirs(os.path.dirname(args.csv), exist_ok=True)
    with open(args.csv, 'a', newline='') as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        if new:
            w.writeheader()
        w.writerow({k: (f'{v:.4f}' if isinstance(v, float) else v) for k, v in row.items()})

    print_spot(row)
    if not row['nvtl'] >= NVTL_OK:
        print(f'WARNING: NVTL {row["nvtl"]:.2f} < {NVTL_OK} - NDT may not be converged here')
    if row['adma_stddev_pos'] > 0.05:
        print(f'WARNING: ADMA position stddev {row["adma_stddev_pos"]:.2f} m - no RTK fix?')
    print(f'appended to {args.csv}')


def print_spot(r):
    dx, dy = float(r['ndt_x']) - float(r['gnss_x']), float(r['ndt_y']) - float(r['gnss_y'])
    yaw = math.radians(float(r['ndt_yaw_deg']))
    fwd = dx * math.cos(yaw) + dy * math.sin(yaw)
    left = -dx * math.sin(yaw) + dy * math.cos(yaw)
    dyaw = wrap_deg(float(r['ndt_yaw_deg']) - float(r['adma_yaw_enu_deg']))
    dist = math.hypot(float(r['ndt_x']), float(r['ndt_y']))
    print(f'{r["label"][:24]:24s} {dist:6.0f}m  map d=({dx:+6.2f},{dy:+6.2f})  '
          f'vehicle fwd {fwd:+6.2f} left {left:+6.2f}  dz {float(r["ndt_z"]) - float(r["gnss_z"]):+6.2f}  '
          f'yaw NDT-ADMA {dyaw:+6.2f} deg  NVTL {float(r["nvtl"]):.2f}')


def analyze(args):
    import numpy as np

    with open(args.csv) as f:
        rows = list(csv.DictReader(f))
    if not rows:
        sys.exit(f'no spots in {args.csv}')
    print(f'{"spot":24s} {"origin":>7s}')
    for r in rows:
        print_spot(r)

    g = np.array([[float(r['gnss_x']), float(r['gnss_y'])] for r in rows])
    n = np.array([[float(r['ndt_x']), float(r['ndt_y'])] for r in rows])
    yaw = np.radians([float(r['ndt_yaw_deg']) for r in rows])
    d = (n - g).reshape(-1)  # [dx0, dy0, dx1, dy1, ...]

    # Linearized columns (small rotation): rotation about the map origin, map shift, lever arm.
    cols = {
        'rot': np.column_stack([-g[:, 1], g[:, 0]]).reshape(-1),
        'tx': np.tile([1.0, 0.0], len(rows)), 'ty': np.tile([0.0, 1.0], len(rows)),
        'fwd': np.column_stack([np.cos(yaw), np.sin(yaw)]).reshape(-1),
        'left': np.column_stack([-np.sin(yaw), np.cos(yaw)]).reshape(-1),
    }
    models = [('A map offset', ['tx', 'ty']), ('B lever arm', ['fwd', 'left']),
              ('C map rotation', ['rot', 'tx', 'ty']), ('D rotation + lever', ['rot', 'tx', 'ty', 'fwd', 'left'])]

    print(f'\nresidual of NDT - GNSS without a model: RMS {np.sqrt(np.mean(d ** 2)):.2f} m')
    for name, keys in models:
        if len(d) <= len(keys):
            print(f'{name:20s} needs more than {len(keys) // 2 + len(keys) % 2} spots')
            continue
        A = np.column_stack([cols[k] for k in keys])
        p, *_ = np.linalg.lstsq(A, d, rcond=None)
        rms = np.sqrt(np.mean((d - A @ p) ** 2))
        desc = []
        for k, v in zip(keys, p):
            desc.append(f'rot {math.degrees(v):+.3f} deg' if k == 'rot' else f'{k} {v:+.2f} m')
        print(f'{name:20s} RMS {rms:5.2f} m   ' + ', '.join(desc))

    dyaw = [wrap_deg(float(r['ndt_yaw_deg']) - float(r['adma_yaw_enu_deg'])) for r in rows]
    print(f'yaw NDT - ADMA: mean {np.mean(dyaw):+.2f} deg, spread {np.std(dyaw):.2f} deg')
    lat0, lon0 = args.origin_lat, args.origin_lon
    if lat0 is not None and lon0 is not None:
        zone_cm = math.floor((lon0 + 180) / 6) * 6 - 180 + 3
        gam = math.degrees(math.atan(math.tan(math.radians(lon0 - zone_cm)) * math.sin(math.radians(lat0))))
        print(f'UTM grid convergence at the map origin: {gam:+.3f} deg '
              '(a LocalCartesianUTM map built in a true-north frame is rotated by about this much)')
    print('\nRead it as: lowest RMS with plausible parameters wins. A rotation in C that matches the\n'
          'yaw mean -> rotated map/projection; a yaw mean with no C rotation -> ADMA heading or\n'
          'mounting bias (set yaw_offset_deg in gnss.launch.xml); B fwd/left -> fix POI 1 or gnss_link.')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--csv', default=DEFAULT_CSV, help=f'spot log (default {DEFAULT_CSV})')
    sub = ap.add_subparsers(dest='cmd', required=True)
    r = sub.add_parser('record', help='record one spot (vehicle at standstill)')
    r.add_argument('--label', default=time.strftime('spot_%H%M%S'))
    r.add_argument('--seconds', type=float, default=5.0)
    a = sub.add_parser('analyze', help='fit error models to all recorded spots')
    a.add_argument('--origin-lat', type=float, help='map_projector_info origin, for the UTM check')
    a.add_argument('--origin-lon', type=float)
    args = ap.parse_args()
    record(args) if args.cmd == 'record' else analyze(args)


if __name__ == '__main__':
    main()
