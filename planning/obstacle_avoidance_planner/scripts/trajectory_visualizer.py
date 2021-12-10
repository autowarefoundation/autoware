#!/usr/bin/env python3

# Copyright 2020 Tier IV, Inc.
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

# TODO(kosuke murakami): write ros2 visualizer
# import rospy
# from tier4_planning_msgs.msg import Trajectory
# from tier4_planning_msgs.msg import TrajectoryPoint
# import matplotlib.pyplot as plt
# import numpy as np
# import tf
# from geometry_msgs.msg import Vector3


# def quaternion_to_euler(quaternion):
#     """Convert Quaternion to Euler Angles

#     quaternion: geometry_msgs/Quaternion
#     euler: geometry_msgs/Vector3
#     """
#     e = tf.transformations.euler_from_quaternion(
#         (quaternion.x, quaternion.y, quaternion.z, quaternion.w))
#     return Vector3(x=e[0], y=e[1], z=e[2])


# class TrajectoryVisualizer():

#     def __init__(self):
#         self.in_trajectory = Trajectory()
#         self.debug_trajectory = Trajectory()
#         self.debug_fixed_trajectory = Trajectory()

#         self.plot_done1 = True
#         self.plot_done2 = True
#         self.plot_done3 = True

#         self.length = 50

#         self.sub_status1 = rospy.Subscriber(
#           "/planning/scenario_planning/lane_driving/motion_planning/obstacle_avoidance_planner/trajectory",
#           Trajectory, self.CallBackTraj, queue_size=1, tcp_nodelay=True)
#         rospy.Timer(rospy.Duration(0.3), self.timerCallback)

#     def CallBackTraj(self, cmd):
#         if (self.plot_done1):
#             self.in_trajectory = cmd
#             self.plot_done1 = False

#     def CallBackDebugTraj(self, cmd):
#         if (self.plot_done2):
#             self.debug_trajectory = cmd
#             self.plot_done2 = False

#     def CallBackDebugFixedTraj(self, cmd):
#         if (self.plot_done3):
#             self.debug_fixed_trajectory = cmd
#             self.plot_done3 = False

#     def timerCallback(self, event):
#         self.plotTrajectory()
#         self.plot_done1 = True
#         self.plot_done2 = True
#         self.plot_done3 = True

#     def CalcArcLength(self, traj):
#         s_arr = []
#         ds = 0.0
#         s_sum = 0.0

#         if len(traj.points) > 0:
#             s_arr.append(s_sum)

#         for i in range(1, len(traj.points)):
#             p0 = traj.points[i-1]
#             p1 = traj.points[i]
#             dx = p1.pose.position.x - p0.pose.position.x
#             dy = p1.pose.position.y - p0.pose.position.y
#             ds = np.sqrt(dx**2 + dy**2)
#             s_sum += ds
#             if(s_sum > self.length):
#                 break
#             s_arr.append(s_sum)
#         return s_arr

#     def CalcX(self, traj):
#         v_list = []
#         for p in traj.points:
#             v_list.append(p.pose.position.x)
#         return v_list

#     def CalcY(self, traj):
#         v_list = []
#         for p in traj.points:
#             v_list.append(p.pose.position.y)
#         return v_list

#     def CalcYaw(self, traj, s_arr):
#         v_list = []
#         for p in traj.points:
#             v_list.append(quaternion_to_euler(p.pose.orientation).z)
#         return v_list[0: len(s_arr)]

#     def plotTrajectory(self):
#         plt.clf()
#         ax3 = plt.subplot(1, 1, 1)
#         x = self.CalcArcLength(self.in_trajectory)
#         y = self.CalcYaw(self.in_trajectory, x)
#         if len(x) == len(y):
#             ax3.plot(x, y, label="final",  marker="*")
#         ax3.set_xlabel("arclength [m]")
#         ax3.set_ylabel("yaw")
#         plt.pause(0.01)


# def main():
#     rospy.init_node("trajectory_visualizer")
#     TrajectoryVisualizer()
#     rospy.spin()


# if __name__ == "__main__":
#     main()
