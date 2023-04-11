#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2022 TIER IV, Inc. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from autoware_auto_planning_msgs.msg import Trajectory
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from matplotlib import animation
import matplotlib.pyplot as plt
import message_filters
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

PLOT_MAX_ARCLENGTH = 200
PATH_ORIGIN_FRAME = "map"
SELF_POSE_FRAME = "base_link"


class TrajectoryVisualizer(Node):
    def __init__(self):
        super().__init__("trajectory_visualizer")

        self.fig = plt.figure()

        self.max_vel = 0.0
        self.min_vel = 0.0

        # update flag
        self.update_sv_traj = False
        self.update_traj = False
        self.update_max_traj = False
        self.update_boundary = False
        self.update_optimized_st_graph = False

        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(
            self.tf_buffer, self, spin_thread=True
        )  # For get self-position
        self.self_pose = Pose()
        self.self_pose_received = False
        self.localization_twist = Twist()
        self.vehicle_twist = Twist()

        self.sv_trajectory = Trajectory()
        self.trajectory = Trajectory()
        self.max_trajectory = Trajectory()
        self.boundary = Trajectory()
        self.optimized_st_graph = Trajectory()

        self.sub_localization_twist = self.create_subscription(
            Odometry, "/localization/kinematic_state", self.CallbackLocalizationOdom, 1
        )
        self.sub_vehicle_twist = self.create_subscription(
            TwistStamped, "/vehicle/status/twist", self.CallbackVehicleTwist, 1
        )

        topic_header = "/planning/scenario_planning/lane_driving/motion_planning/"
        traj0 = "obstacle_cruise_planner/optimized_sv_trajectory"
        self.sub_status0 = message_filters.Subscriber(self, Trajectory, topic_header + traj0)
        traj1 = "/planning/scenario_planning/lane_driving/trajectory"
        self.sub_status1 = message_filters.Subscriber(self, Trajectory, traj1)
        traj2 = "surround_obstacle_checker/trajectory"
        self.sub_status2 = message_filters.Subscriber(self, Trajectory, topic_header + traj2)
        traj3 = "obstacle_cruise_planner/boundary"
        self.sub_status3 = message_filters.Subscriber(self, Trajectory, topic_header + traj3)
        traj4 = "obstacle_cruise_planner/optimized_st_graph"
        self.sub_status4 = message_filters.Subscriber(self, Trajectory, topic_header + traj4)

        self.ts1 = message_filters.ApproximateTimeSynchronizer(
            [self.sub_status0, self.sub_status1, self.sub_status2], 30, 0.5
        )
        self.ts1.registerCallback(self.CallbackMotionVelOptTraj)
        self.ts2 = message_filters.ApproximateTimeSynchronizer(
            [self.sub_status3, self.sub_status4], 30, 0.5
        )
        self.ts2.registerCallback(self.CallbackMotionVelObsTraj)

        # Main Process
        self.ani = animation.FuncAnimation(
            self.fig, self.plotTrajectoryVelocity, interval=100, blit=True
        )
        self.setPlotTrajectoryVelocity()

        plt.show()
        return

    def CallbackLocalizationOdom(self, cmd):
        self.localization_twist = cmd.twist.twist

    def CallbackVehicleTwist(self, cmd):
        self.vehicle_twist = cmd.twist

    def CallbackMotionVelOptTraj(self, cmd0, cmd1, cmd2):
        self.CallbackSVTraj(cmd0)
        self.CallbackTraj(cmd1)
        self.CallbackMaxTraj(cmd2)

    def CallbackMotionVelObsTraj(self, cmd1, cmd2):
        self.CallbackBoundary(cmd1)
        self.CallbackOptimizedSTGraph(cmd2)

    def CallbackSVTraj(self, cmd):
        self.sv_trajectory = cmd
        self.update_sv_traj = True

    def CallbackTraj(self, cmd):
        self.trajectory = cmd
        self.update_traj = True

    def CallbackMaxTraj(self, cmd):
        self.max_trajectory = cmd
        self.update_max_traj = True

    def CallbackBoundary(self, cmd):
        self.boundary = cmd
        self.update_boundary = True

    def CallbackOptimizedSTGraph(self, cmd):
        self.optimized_st_graph = cmd
        self.update_optimized_st_graph = True

    def setPlotTrajectoryVelocity(self):
        self.ax1 = plt.subplot(2, 1, 1)  # row, col, index(<raw*col)
        (self.im0,) = self.ax1.plot(
            [], [], label="Optimized SV Velocity", marker="", color="purple"
        )
        (self.im1,) = self.ax1.plot([], [], label="Input Velocity", marker="", color="black")
        (self.im2,) = self.ax1.plot([], [], label="Output Velocity", marker="", color="blue")
        (self.im3,) = self.ax1.plot(
            [], [], label="localization twist vx", color="r", marker="*", ls=":", markersize=10
        )
        (self.im4,) = self.ax1.plot(
            [], [], label="vehicle twist vx", color="k", marker="+", ls=":", markersize=10
        )
        self.ax1.set_title("velocity on trajectory")
        self.ax1.legend()
        self.ax1.set_xlim([0, PLOT_MAX_ARCLENGTH])
        self.ax1.set_ylabel("vel [m/s]")

        self.ax2 = plt.subplot(2, 1, 2)
        (self.im5,) = self.ax2.plot(
            [], [], label="Original", marker="", color="black", linewidth=3.0
        )
        (self.im6,) = self.ax2.plot([], [], label="Optimum", marker="", color="blue")
        (self.im7,) = self.ax2.plot([], [], label="Boundary", marker="", color="green")
        (self.im8,) = self.ax2.plot([], [], label="Optimized ST Graph", marker="", color="purple")
        self.ax2.set_title("st graph")
        self.ax2.legend()
        self.ax2.set_xlim([0, 30])
        self.ax2.set_ylim([0, PLOT_MAX_ARCLENGTH])
        self.ax2.set_xlabel("t [s]")
        self.ax2.set_ylabel("s [m]")

        return (
            self.im0,
            self.im1,
            self.im2,
            self.im3,
            self.im4,
            self.im5,
            self.im6,
            self.im7,
            self.im8,
        )

    def plotTrajectoryVelocity(self, data):
        self.updatePose(PATH_ORIGIN_FRAME, SELF_POSE_FRAME)
        if self.self_pose_received is False:
            print("plot start but self pose is not received")
        self.get_logger().info("plot called")

        # copy
        trajectory = self.trajectory
        max_trajectory = self.max_trajectory

        if self.update_sv_traj:
            self.update_sv_traj = False
            s_list = []
            v_list = []
            for p in self.sv_trajectory.points:
                s_list.append(p.pose.position.x)
                v_list.append(p.pose.position.y)
            self.im0.set_data(s_list, v_list)

        if self.update_traj:
            x = self.CalcArcLength(trajectory)
            y = self.ToVelList(trajectory)
            self.im2.set_data(x, y)
            self.update_traj = False

            opt_closest = self.calcClosestTrajectory(trajectory)
            if opt_closest >= 0:
                x_closest = x[opt_closest]
                self.im3.set_data(x_closest, self.localization_twist.linear.x)
                self.im4.set_data(x_closest, self.vehicle_twist.linear.x)

                opt_zero_vel_id = -1
                for i in range(opt_closest, len(trajectory.points)):
                    if trajectory.points[i].longitudinal_velocity_mps < 1e-3:
                        opt_zero_vel_id = i
                        break
                else:
                    opt_zero_vel_id = len(trajectory.points) - 1

                opt_pos = self.CalcPartArcLength(trajectory, opt_closest, opt_zero_vel_id + 1)
                opt_time = self.CalcTime(trajectory, opt_closest, opt_zero_vel_id + 1)
                self.im6.set_data(opt_time, opt_pos)

        if self.update_max_traj:
            x = self.CalcArcLength(max_trajectory)
            y = self.ToVelList(max_trajectory)
            self.im1.set_data(x, y)
            self.update_max_traj = False

            max_closest = self.calcClosestTrajectory(max_trajectory)
            if max_closest >= 0:
                max_zero_vel_id = -1
                for i in range(max_closest, len(max_trajectory.points)):
                    if max_trajectory.points[i].longitudinal_velocity_mps < 1e-3:
                        max_zero_vel_id = i
                        break
                else:
                    max_zero_vel_id = len(max_trajectory.points) - 1

                max_pos = self.CalcPartArcLength(max_trajectory, max_closest, max_zero_vel_id + 1)
                max_time = self.CalcTime(max_trajectory, max_closest, max_zero_vel_id + 1)
                self.im5.set_data(max_time, max_pos)

        if self.update_boundary:
            self.update_boundary = False
            s_list = []
            t_list = []
            for p in self.boundary.points:
                s_list.append(p.pose.position.x)
                t_list.append(p.pose.position.y)
            self.im7.set_data(t_list, s_list)

        if self.update_optimized_st_graph:
            self.update_optimized_st_graph = False
            s_list = []
            t_list = []
            for p in self.optimized_st_graph.points:
                s_list.append(p.pose.position.x)
                t_list.append(p.pose.position.y)
            self.im8.set_data(t_list, s_list)

        # change y-range
        self.ax1.set_ylim([-1.0, 50.0])

        return (
            self.im0,
            self.im1,
            self.im2,
            self.im3,
            self.im4,
            self.im5,
            self.im6,
            self.im7,
            self.im8,
        )

    def CalcArcLength(self, traj):
        return self.CalcPartArcLength(traj, 0, len(traj))

    def CalcPartArcLength(self, traj, start, end):
        assert start <= end
        s_arr = []
        ds = 0.0
        s_sum = 0.0

        if len(traj) > 0:
            s_arr.append(s_sum)

        for i in range(start + 1, end):
            p0 = traj[i - 1]
            p1 = traj[i]
            dx = p1.pose.position.x - p0.pose.position.x
            dy = p1.pose.position.y - p0.pose.position.y
            ds = np.sqrt(dx**2 + dy**2)
            s_sum += ds
            s_arr.append(s_sum)
        return s_arr

    def CalcTrajectoryInterval(self, traj, start, end):
        ds_arr = []

        for i in range(start + 1, end):
            p0 = traj[i - 1]
            p1 = traj[i]
            dx = p1.pose.position.x - p0.pose.position.x
            dy = p1.pose.position.y - p0.pose.position.y
            ds = np.sqrt(dx**2 + dy**2)
            ds_arr.append(ds)
        return ds_arr

    def CalcTime(self, traj, start, end):
        t_arr = []
        t_sum = 0.0
        ds_arr = self.CalcTrajectoryInterval(traj, start, end)

        if len(traj) > 0:
            t_arr.append(t_sum)

        for i in range(start, end - 1):
            v = traj[i].longitudinal_velocity_mps
            ds = ds_arr[i - start]
            dt = ds / max(v, 0.1)
            t_sum += dt
            t_arr.append(t_sum)
        return t_arr

    def ToVelList(self, traj):
        v_list = []
        for p in traj:
            v_list.append(p.longitudinal_velocity_mps)
        return v_list

    def updatePose(self, from_link, to_link):
        try:
            tf = self.tf_buffer.lookup_transform(from_link, to_link, rclpy.time.Time())
            self.self_pose.position.x = tf.transform.translation.x
            self.self_pose.position.y = tf.transform.translation.y
            self.self_pose.position.z = tf.transform.translation.z
            self.self_pose.orientation.x = tf.transform.rotation.x
            self.self_pose.orientation.y = tf.transform.rotation.y
            self.self_pose.orientation.z = tf.transform.rotation.z
            self.self_pose.orientation.w = tf.transform.rotation.w
            self.self_pose_received = True
            return
        except BaseException:
            self.get_logger().warn(
                "lookup transform failed: from {} to {}".format(from_link, to_link)
            )
            return

    def calcClosestTrajectory(self, path):
        closest = -1
        min_dist_squared = 1.0e10
        for i in range(0, len(path.points)):
            dist_sq = self.calcSquaredDist2d(self.self_pose, path.points[i].pose)
            if dist_sq < min_dist_squared:
                min_dist_squared = dist_sq
                closest = i
        return closest

    def calcSquaredDist2d(self, p1, p2):
        dx = p1.position.x - p2.position.x
        dy = p1.position.y - p2.position.y
        return dx * dx + dy * dy


def main(args=None):
    try:
        rclpy.init(args=args)
        node = TrajectoryVisualizer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
