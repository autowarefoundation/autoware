#!/usr/bin/env python3
"""Publish a voxel-downsampled copy of a map's pointcloud_map.pcd for RViz only.

    display_map_pub.py <map_dir> [voxel_m]

The downsampled cloud is cached next to this script as <map>_display_<voxel>m.npy and published
once on /display/pointcloud_map (frame map, transient local), so RViz uploads it to the GPU once
instead of the full map. Autoware itself still uses the full /map/pointcloud_map.
"""
import os
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


def read_pcd_xyzi(path):
    with open(path, 'rb') as f:
        header = {}
        while True:
            line = f.readline().decode('ascii').strip()
            k, _, v = line.partition(' ')
            header[k] = v.split()
            if k == 'DATA':
                break
        if header['DATA'][0] != 'binary':
            raise SystemExit(f'{path}: only binary pcd supported')
        fields, sizes, types = header['FIELDS'], header['SIZE'], header['TYPE']
        dtype = np.dtype([(n, {'F': 'f', 'U': 'u', 'I': 'i'}[t] + s) for n, s, t in zip(fields, sizes, types)])
        data = np.fromfile(f, dtype=dtype, count=int(header['POINTS'][0]))
    xyz = np.stack([data['x'], data['y'], data['z']], axis=1).astype(np.float32)
    inten = data['intensity'].astype(np.float32) if 'intensity' in fields else np.zeros(len(data), np.float32)
    return xyz, inten


def voxel_downsample(xyz, inten, voxel):
    ok = np.isfinite(xyz).all(axis=1)
    xyz, inten = xyz[ok], inten[ok]
    keys = np.floor(xyz / voxel).astype(np.int64)
    keys -= keys.min(axis=0)
    dims = keys.max(axis=0) + 1
    flat = (keys[:, 0] * dims[1] + keys[:, 1]) * dims[2] + keys[:, 2]
    _, idx = np.unique(flat, return_index=True)
    return xyz[idx], inten[idx]


def main():
    map_dir = sys.argv[1].rstrip('/')
    voxel = float(sys.argv[2]) if len(sys.argv) > 2 else 0.3
    name = os.path.basename(map_dir)
    cache = os.path.join(os.path.dirname(os.path.abspath(__file__)), f'{name}_display_{voxel:g}m.npy')
    pcd = os.path.join(map_dir, 'pointcloud_map.pcd')
    if os.path.exists(cache) and os.path.getmtime(cache) > os.path.getmtime(pcd):
        pts = np.load(cache)
    else:
        xyz, inten = read_pcd_xyzi(pcd)
        xyz, inten = voxel_downsample(xyz, inten, voxel)
        pts = np.concatenate([xyz, inten[:, None]], axis=1).astype(np.float32)
        np.save(cache, pts)

    rclpy.init()
    node = Node('display_map_pub')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    pub = node.create_publisher(PointCloud2, '/display/pointcloud_map', qos)
    msg = PointCloud2()
    msg.header.frame_id = 'map'
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.height, msg.width = 1, len(pts)
    msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate(['x', 'y', 'z', 'intensity'])]
    msg.is_bigendian, msg.point_step, msg.row_step, msg.is_dense = False, 16, 16 * len(pts), True
    msg.data = pts.tobytes()
    pub.publish(msg)
    node.get_logger().info(f'published {len(pts)} points ({voxel} m voxel) from {pcd}')
    rclpy.spin(node)


if __name__ == '__main__':
    main()
