################################################################################
#
# Copyright (c) 2017 University of Oxford
# Authors:
#  Geoff Pascoe (gmp@robots.ox.ac.uk)
#
# This work is licensed under the Creative Commons
# Attribution-NonCommercial-ShareAlike 4.0 International License.
# To view a copy of this license, visit
# http://creativecommons.org/licenses/by-nc-sa/4.0/ or send a letter to
# Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
#
################################################################################

import os
import re
import numpy as np
from transform import build_se3_transform

from interpolate_poses import interpolate_vo_poses, interpolate_ins_poses


def build_pointcloud(lidar_dir, poses_file, extrinsics_dir, start_time, end_time, origin_time=-1):
    """Builds a pointcloud by combining multiple LIDAR scans with odometry information.

    Args:
        lidar_dir (str): Directory containing LIDAR scans.
        poses_file (str): Path to a file containing pose information. Can be VO or INS data.
        extrinsics_dir (str): Directory containing extrinsic calibrations.
        start_time (int): UNIX timestamp of the start of the window over which to build the pointcloud.
        end_time (int): UNIX timestamp of the end of the window over which to build the pointcloud.
        origin_time (int): UNIX timestamp of origin frame. Pointcloud coordinates are relative to this frame.

    Returns:
        numpy.ndarray: 3xn array of (x, y, z) coordinates of pointcloud
        numpy.array: array of n reflectance values or None if no reflectance values are recorded (LDMRS)

    Raises:
        ValueError: if specified window doesn't contain any laser scans.
        IOError: if scan files are not found.

    """
    if origin_time < 0:
        origin_time = start_time

    lidar = re.search('(lms_front|lms_rear|ldmrs)', lidar_dir).group(0)
    timestamps_path = os.path.join(lidar_dir, os.pardir, lidar + '.timestamps')

    timestamps = []
    with open(timestamps_path) as timestamps_file:
        for line in timestamps_file:
            timestamp = int(line.split(' ')[0])
            if start_time <= timestamp <= end_time:
                timestamps.append(timestamp)

    if len(timestamps) == 0:
        raise ValueError("No LIDAR data in the given time bracket.")

    with open(os.path.join(extrinsics_dir, lidar + '.txt')) as extrinsics_file:
        extrinsics = next(extrinsics_file)
    G_posesource_laser = build_se3_transform([float(x) for x in extrinsics.split(' ')])

    poses_type = re.search('(vo|ins)\.csv', poses_file).group(1)

    if poses_type == 'ins':
        with open(os.path.join(extrinsics_dir, 'ins.txt')) as extrinsics_file:
            extrinsics = next(extrinsics_file)
            G_posesource_laser = np.linalg.solve(build_se3_transform([float(x) for x in extrinsics.split(' ')]),
                                                 G_posesource_laser)

        poses = interpolate_ins_poses(poses_file, timestamps, origin_time)
    else:
        # sensor is VO, which is located at the main vehicle frame
        poses = interpolate_vo_poses(poses_file, timestamps, origin_time)

    pointcloud = np.array([[0], [0], [0], [0]])
    if lidar == 'ldmrs':
        reflectance = None
    else:
        reflectance = np.empty((0))

    for i in range(0, len(poses)):
        scan_path = os.path.join(lidar_dir, str(timestamps[i]) + '.bin')
        if not os.path.isfile(scan_path):
            continue

        scan_file = open(scan_path)
        scan = np.fromfile(scan_file, np.double)
        scan_file.close()

        scan = scan.reshape((len(scan) // 3, 3)).transpose()

        if lidar != 'ldmrs':
            # LMS scans are tuples of (x, y, reflectance)
            reflectance = np.concatenate((reflectance, np.ravel(scan[2, :])))
            scan[2, :] = np.zeros((1, scan.shape[1]))

        scan = np.dot(np.dot(poses[i], G_posesource_laser), np.vstack([scan, np.ones((1, scan.shape[1]))]))
        pointcloud = np.hstack([pointcloud, scan])

    pointcloud = pointcloud[:, 1:]
    if pointcloud.shape[1] == 0:
        raise IOError("Could not find scan files for given time range in directory " + lidar_dir)

    return pointcloud, reflectance


if __name__ == "__main__":
    import argparse
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D

    parser = argparse.ArgumentParser(description='Build and display a pointcloud')
    parser.add_argument('--poses_file', type=str, default=None, help='File containing relative or absolute poses')
    parser.add_argument('--extrinsics_dir', type=str, default=None,
                        help='Directory containing extrinsic calibrations')
    parser.add_argument('--laser_dir', type=str, default=None, help='Directory containing LIDAR data')

    args = parser.parse_args()

    lidar = re.search('(lms_front|lms_rear|ldmrs)', args.laser_dir).group(0)
    timestamps_path = os.path.join(args.laser_dir, os.pardir, lidar + '.timestamps')
    with open(timestamps_path) as timestamps_file:
        start_time = int(next(timestamps_file).split(' ')[0])

    end_time = start_time + 2e7

    pointcloud, reflectance = build_pointcloud(args.laser_dir, args.poses_file,
                                               args.extrinsics_dir, start_time, end_time)

    from pcd import create_pointcloud_file
    create_pointcloud_file (pointcloud.transpose(), '/tmp/test.pcd')
    sys.exit()

    if reflectance is not None:
        colours = (reflectance - reflectance.min()) / (reflectance.max() - reflectance.min())
        colours = 1 / (1 + np.exp(-10 * (colours - colours.mean())))
    else:
        colours = 'gray'

    x = np.ravel(pointcloud[0, :])
    y = np.ravel(pointcloud[1, :])
    z = np.ravel(pointcloud[2, :])

    xmin = x.min()
    ymin = y.min()
    zmin = z.min()
    xmax = x.max()
    ymax = y.max()
    zmax = z.max()
    xmid = (xmax + xmin) * 0.5
    ymid = (ymax + ymin) * 0.5
    zmid = (zmax + zmin) * 0.5

    max_range = max(xmax - xmin, ymax - ymin, zmax - zmin)
    x_range = [xmid - 0.5 * max_range, xmid + 0.5 * max_range]
    y_range = [ymid - 0.5 * max_range, ymid + 0.5 * max_range]
    z_range = [zmid - 0.5 * max_range, zmid + 0.5 * max_range]

    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.set_aspect('equal')
    ax.scatter(-y, -x, -z, marker=',', s=1, c=colours, cmap='gray', edgecolors='none')
    ax.set_xlim(-y_range[1], -y_range[0])
    ax.set_ylim(-x_range[1], -x_range[0])
    ax.set_zlim(-z_range[1], -z_range[0])
    plt.show()
