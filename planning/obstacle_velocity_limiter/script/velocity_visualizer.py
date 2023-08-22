#!/usr/bin/env python3

# Copyright 2022 TIER IV, Inc.
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
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node


# cspell: ignore axhline, relim
class TrajectoryVisualizer(Node):
    def __init__(self):
        super().__init__("trajectory_visualizer")

        self.fig = plt.figure()

        self.sub_original_traj = self.create_subscription(
            Trajectory,
            "/planning/scenario_planning/lane_driving/motion_planning/obstacle_avoidance_planner/trajectory",
            self.plotOriginalTrajectory,
            1,
        )
        self.sub_adjusted_traj = self.create_subscription(
            Trajectory,
            "/planning/scenario_planning/lane_driving/motion_planning/obstacle_velocity_limiter/trajectory",
            self.plotAdjustedTrajectory,
            1,
        )
        self.sub_odom = self.create_subscription(
            Odometry,
            "/localization/kinematic_state",
            self.plotEgoVel,
            1,
        )

        self.initPlotTrajectoryVelocity()

    def plotOriginalTrajectory(self, original):
        x = self.CalcArcLength(original)
        y = self.ToVelList(original)
        self.im1.set_data(x, y)
        self.ax1.relim()
        self.ax1.autoscale_view(True, True, True)
        plt.draw()
        plt.pause(0.01)

    def plotAdjustedTrajectory(self, adjusted):
        x = self.CalcArcLength(adjusted)
        y = self.ToVelList(adjusted)
        self.im2.set_data(x, y)
        self.ax1.relim()
        self.ax1.autoscale_view(True, True, True)
        plt.draw()
        plt.pause(0.01)

    def plotEgoVel(self, odom_msg):
        self.ego_vel.set_ydata(
            [odom_msg.twist.twist.linear.x for _ in range(len(self.ego_vel.get_xdata()))]
        )

    def initPlotTrajectoryVelocity(self):
        self.ax1 = plt.subplot(1, 1, 1)  # row, col, index(<raw*col)
        (self.im1,) = self.ax1.plot([], [], label="0: original trajectory", marker="", ls="-")
        (self.im2,) = self.ax1.plot([], [], label="1: adjusted trajectory", marker="", ls="--")
        self.ego_vel = self.ax1.axhline(y=0, color="blue", linestyle=":")
        self.ax1.set_title("trajectory's velocity")
        self.ax1.legend()
        self.ax1.set_ylabel("vel [m/s]")
        self.ax1.autoscale()

    def CalcArcLength(self, traj):
        s_arr = []
        ds = 0.0
        s_sum = 0.0

        if len(traj.points) > 0:
            s_arr.append(s_sum)

        for i in range(1, len(traj.points)):
            p0 = traj.points[i - 1]
            p1 = traj.points[i]
            dx = p1.pose.position.x - p0.pose.position.x
            dy = p1.pose.position.y - p0.pose.position.y
            ds = np.sqrt(dx**2 + dy**2)
            s_sum += ds
            s_arr.append(s_sum)
        return s_arr

    def ToVelList(self, traj):
        v_list = []
        for p in traj.points:
            v_list.append(p.longitudinal_velocity_mps)
        return v_list


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
