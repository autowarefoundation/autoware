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

import bisect
import csv
import numpy as np
import numpy.matlib as ml
from transform import *


def interpolate_vo_poses(vo_path, pose_timestamps, origin_timestamp):
    """Interpolate poses from visual odometry.

    Args:
        vo_path (str): path to file containing relative poses from visual odometry.
        pose_timestamps (list[int]): UNIX timestamps at which interpolated poses are required.
        origin_timestamp (int): UNIX timestamp of origin frame. Poses will be reported relative to this frame.

    Returns:
        list[numpy.matrixlib.defmatrix.matrix]: SE3 matrix representing interpolated pose for each requested timestamp.

    """
    with open(vo_path) as vo_file:
        vo_reader = csv.reader(vo_file)
        headers = next(vo_file)

        vo_timestamps = [0]
        abs_poses = [ml.identity(4)]

        lower_timestamp = min(min(pose_timestamps), origin_timestamp)
        upper_timestamp = max(max(pose_timestamps), origin_timestamp)

        for row in vo_reader:
            timestamp = int(row[0])
            if timestamp < lower_timestamp:
                vo_timestamps[0] = timestamp
                continue

            vo_timestamps.append(timestamp)

            xyzrpy = [float(v) for v in row[2:8]]
            rel_pose = build_se3_transform(xyzrpy)
            abs_pose = abs_poses[-1] * rel_pose
            abs_poses.append(abs_pose)

            if timestamp >= upper_timestamp:
                break

    return interpolate_poses(vo_timestamps, abs_poses, pose_timestamps, origin_timestamp)


def interpolate_ins_poses(ins_path, pose_timestamps, origin_timestamp):
    """Interpolate poses from INS.

    Args:
        ins_path (str): path to file containing poses from INS.
        pose_timestamps (list[int]): UNIX timestamps at which interpolated poses are required.
        origin_timestamp (int): UNIX timestamp of origin frame. Poses will be reported relative to this frame.

    Returns:
        list[numpy.matrixlib.defmatrix.matrix]: SE3 matrix representing interpolated pose for each requested timestamp.

    """
    with open(ins_path) as ins_file:
        ins_reader = csv.reader(ins_file)
        headers = next(ins_file)

        ins_timestamps = [0]
        abs_poses = [ml.identity(4)]

        upper_timestamp = max(max(pose_timestamps), origin_timestamp)

        for row in ins_reader:
            timestamp = int(row[0])
            ins_timestamps.append(timestamp)

            xyzrpy = [float(v) for v in row[2:8]]
            abs_pose = build_se3_transform(xyzrpy)
            abs_poses.append(abs_pose)

            if timestamp >= upper_timestamp:
                break

    ins_timestamps = ins_timestamps[1:]
    abs_poses = abs_poses[1:]

    return interpolate_poses(ins_timestamps, abs_poses, pose_timestamps, origin_timestamp)


def interpolate_poses(pose_timestamps, abs_poses, requested_timestamps, origin_timestamp):
    """Interpolate between absolute poses.

    Args:
        pose_timestamps (list[int]): Timestamps of supplied poses. Must be in ascending order.
        abs_poses (list[numpy.matrixlib.defmatrix.matrix]): SE3 matrices representing poses at the timestamps specified.
        requested_timestamps (list[int]): Timestamps for which interpolated timestamps are required.
        origin_timestamp (int): UNIX timestamp of origin frame. Poses will be reported relative to this frame.

    Returns:
        list[numpy.matrixlib.defmatrix.matrix]: SE3 matrix representing interpolated pose for each requested timestamp.

    Raises:
        ValueError: if pose_timestamps and abs_poses are not the same length
        ValueError: if pose_timestamps is not in ascending order

    """
    requested_timestamps.insert(0, origin_timestamp)
    requested_timestamps = np.array(requested_timestamps)
    pose_timestamps = np.array(pose_timestamps)

    if len(pose_timestamps) != len(abs_poses):
        raise ValueError('Must supply same number of timestamps as poses')

    abs_quaternions = np.zeros((4, len(abs_poses)))
    abs_positions = np.zeros((3, len(abs_poses)))
    for i, pose in enumerate(abs_poses):
        if i > 0 and pose_timestamps[i-1] >= pose_timestamps[i]:
            raise ValueError('Pose timestamps must be in ascending order')

        abs_quaternions[:, i] = so3_to_quaternion(pose[0:3, 0:3])
        abs_positions[:, i] = np.ravel(pose[0:3, 3])

    upper_indices = [bisect.bisect(pose_timestamps, pt) for pt in requested_timestamps]
    lower_indices = [u - 1 for u in upper_indices]

    if max(upper_indices) >= len(pose_timestamps):
        upper_indices = [min(i, len(pose_timestamps) - 1) for i in upper_indices]

    fractions = (requested_timestamps - pose_timestamps[lower_indices]) / \
                (pose_timestamps[upper_indices] - pose_timestamps[lower_indices])

    quaternions_lower = abs_quaternions[:, lower_indices]
    quaternions_upper = abs_quaternions[:, upper_indices]

    d_array = (quaternions_lower * quaternions_upper).sum(0)

    linear_interp_indices = np.nonzero(d_array >= 1)
    sin_interp_indices = np.nonzero(d_array < 1)

    scale0_array = np.zeros(d_array.shape)
    scale1_array = np.zeros(d_array.shape)

    scale0_array[linear_interp_indices] = 1 - fractions[linear_interp_indices]
    scale1_array[linear_interp_indices] = fractions[linear_interp_indices]

    theta_array = np.arccos(np.abs(d_array[sin_interp_indices]))

    scale0_array[sin_interp_indices] = \
        np.sin((1 - fractions[sin_interp_indices]) * theta_array) / np.sin(theta_array)
    scale1_array[sin_interp_indices] = \
        np.sin(fractions[sin_interp_indices] * theta_array) / np.sin(theta_array)

    negative_d_indices = np.nonzero(d_array < 0)
    scale1_array[negative_d_indices] = -scale1_array[negative_d_indices]

    quaternions_interp = np.tile(scale0_array, (4, 1)) * quaternions_lower \
                         + np.tile(scale1_array, (4, 1)) * quaternions_upper

    positions_lower = abs_positions[:, lower_indices]
    positions_upper = abs_positions[:, upper_indices]

    positions_interp = np.multiply(np.tile((1 - fractions), (3, 1)), positions_lower) \
                       + np.multiply(np.tile(fractions, (3, 1)), positions_upper)

    poses_mat = ml.zeros((4, 4 * len(requested_timestamps)))

    poses_mat[0, 0::4] = 1 - 2 * np.square(quaternions_interp[2, :]) - \
                         2 * np.square(quaternions_interp[3, :])
    poses_mat[0, 1::4] = 2 * np.multiply(quaternions_interp[1, :], quaternions_interp[2, :]) - \
                         2 * np.multiply(quaternions_interp[3, :], quaternions_interp[0, :])
    poses_mat[0, 2::4] = 2 * np.multiply(quaternions_interp[1, :], quaternions_interp[3, :]) + \
                         2 * np.multiply(quaternions_interp[2, :], quaternions_interp[0, :])

    poses_mat[1, 0::4] = 2 * np.multiply(quaternions_interp[1, :], quaternions_interp[2, :]) \
                         + 2 * np.multiply(quaternions_interp[3, :], quaternions_interp[0, :])
    poses_mat[1, 1::4] = 1 - 2 * np.square(quaternions_interp[1, :]) \
                         - 2 * np.square(quaternions_interp[3, :])
    poses_mat[1, 2::4] = 2 * np.multiply(quaternions_interp[2, :], quaternions_interp[3, :]) - \
                         2 * np.multiply(quaternions_interp[1, :], quaternions_interp[0, :])

    poses_mat[2, 0::4] = 2 * np.multiply(quaternions_interp[1, :], quaternions_interp[3, :]) - \
                         2 * np.multiply(quaternions_interp[2, :], quaternions_interp[0, :])
    poses_mat[2, 1::4] = 2 * np.multiply(quaternions_interp[2, :], quaternions_interp[3, :]) + \
                         2 * np.multiply(quaternions_interp[1, :], quaternions_interp[0, :])
    poses_mat[2, 2::4] = 1 - 2 * np.square(quaternions_interp[1, :]) - \
                         2 * np.square(quaternions_interp[2, :])

    poses_mat[0:3, 3::4] = positions_interp
    poses_mat[3, 3::4] = 1

    poses_mat = np.linalg.solve(poses_mat[0:4, 0:4], poses_mat)

    poses_out = [0] * (len(requested_timestamps) - 1)
    for i in range(1, len(requested_timestamps)):
        poses_out[i - 1] = poses_mat[0:4, i * 4:(i + 1) * 4]

    return poses_out
