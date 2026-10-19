#!/usr/bin/env python3
"""Throttled, subsampled copies of live pointclouds for RViz only.

rviz2 (Humble) leaks GPU memory on every PointCloud2 message it draws (~800 MB/min for a 40k-point
cloud at 10 Hz, 2026-10-15). The leak scales with points x rate, so the general lidar view is
throttled; the obstacle cloud stays at full rate. Autoware itself still uses the full clouds.

    display_cloud_throttle.py [--rate 2.0] [--points 20000] [--obstacle-points 50000]

  /sensing/lidar/concatenated/pointcloud        -> /display/lidar               (--rate Hz, --points)
  /perception/obstacle_segmentation/pointcloud  -> /display/obstacle_pointcloud (every message, i.e. full
      rate, capped at --obstacle-points: the safety driver must see obstacles without delay)
Output points are x, y, z, intensity (float32), same frame and stamp as the input.
"""
import argparse

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField

LIDAR = ('/sensing/lidar/concatenated/pointcloud', '/display/lidar')
OBSTACLE = ('/perception/obstacle_segmentation/pointcloud', '/display/obstacle_pointcloud')
NP_TYPES = {PointField.INT8: 'i1', PointField.UINT8: 'u1', PointField.INT16: 'i2', PointField.UINT16: 'u2',
            PointField.INT32: 'i4', PointField.UINT32: 'u4', PointField.FLOAT32: 'f4', PointField.FLOAT64: 'f8'}
OUT_FIELDS = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
              for i, n in enumerate(['x', 'y', 'z', 'intensity'])]


def to_xyzi(msg):
    """(N, 4) float32 array of x, y, z, intensity (0 if the cloud has no intensity)."""
    names = [f.name for f in msg.fields]
    dtype = np.dtype({'names': names,
                      'formats': [('>' if msg.is_bigendian else '<') + NP_TYPES[f.datatype] for f in msg.fields],
                      'offsets': [f.offset for f in msg.fields],
                      'itemsize': msg.point_step})
    pts = np.frombuffer(msg.data, dtype=dtype, count=msg.width * msg.height)
    out = np.zeros((len(pts), 4), np.float32)
    for i, n in enumerate(['x', 'y', 'z', 'intensity']):
        if n in names:
            out[:, i] = pts[n]
    return out[np.isfinite(out[:, :3]).all(axis=1)]


class DisplayCloudThrottle(Node):
    def __init__(self, rate, points, obstacle_points):
        super().__init__('display_cloud_throttle')
        self.rng = np.random.default_rng()
        self.latest_lidar = None
        self.lidar_pub = self.create_publisher(PointCloud2, LIDAR[1], qos_profile_sensor_data)
        self.obstacle_pub = self.create_publisher(PointCloud2, OBSTACLE[1], qos_profile_sensor_data)
        self.create_subscription(PointCloud2, LIDAR[0], lambda m: setattr(self, 'latest_lidar', m),
                                 qos_profile_sensor_data)
        self.create_subscription(
            PointCloud2, OBSTACLE[0],
            lambda m: self.obstacle_pub.publish(self.subsample(m, obstacle_points)), qos_profile_sensor_data)
        self.create_timer(1.0 / rate, lambda: self.publish_lidar(points))

    def publish_lidar(self, points):
        if self.latest_lidar is not None:
            self.lidar_pub.publish(self.subsample(self.latest_lidar, points))
            self.latest_lidar = None   # publish each input cloud at most once

    def subsample(self, msg, points):
        xyzi = to_xyzi(msg)
        if len(xyzi) > points:
            xyzi = xyzi[self.rng.choice(len(xyzi), points, replace=False)]
        out = PointCloud2()
        out.header = msg.header
        out.height, out.width = 1, len(xyzi)
        out.fields = OUT_FIELDS
        out.is_bigendian, out.point_step, out.row_step, out.is_dense = False, 16, 16 * len(xyzi), True
        out.data = xyzi.tobytes()
        return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--rate', type=float, default=2.0, help='output rate [Hz]')
    ap.add_argument('--points', type=int, default=20000, help='max points per lidar view cloud')
    ap.add_argument('--obstacle-points', type=int, default=50000, help='max points per obstacle cloud')
    args, _ = ap.parse_known_args()
    rclpy.init()
    rclpy.spin(DisplayCloudThrottle(args.rate, args.points, args.obstacle_points))


if __name__ == '__main__':
    main()
