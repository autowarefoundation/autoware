// Copyright 2022 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TIER4_AUTOWARE_UTILS__ROS__MSG_COVARIANCE_HPP_
#define TIER4_AUTOWARE_UTILS__ROS__MSG_COVARIANCE_HPP_

namespace tier4_autoware_utils
{
namespace xyz_covariance_index
{
/// Covariance for x-y-z.
/// Used at
/// - sensor_msgs/msg/Imu.msg: msg.linear_acceleration_covariance
enum XYZ_COV_IDX {
  X_X = 0,
  X_Y = 1,
  X_Z = 2,
  Y_X = 3,
  Y_Y = 4,
  Y_Z = 5,
  Z_X = 6,
  Z_Y = 7,
  Z_Z = 8,
};
}  // namespace xyz_covariance_index

namespace rpy_covariance_index
{
/// Covariance for roll-pitch-yaw.
/// Used at
/// - sensor_msgs/msg/Imu.msg: msg.angular_velocity_covariance
/// - sensor_msgs/msg/Imu.msg: msg.orientation_covariance
enum RPY_COV_IDX {
  ROLL_ROLL = 0,
  ROLL_PITCH = 1,
  ROLL_YAW = 2,
  PITCH_ROLL = 3,
  PITCH_PITCH = 4,
  PITCH_YAW = 5,
  YAW_ROLL = 6,
  YAW_PITCH = 7,
  YAW_YAW = 8
};
}  // namespace rpy_covariance_index

namespace xyzrpy_covariance_index
{
/// Covariance for 6-DOF.
/// Used at
/// - geometry_msgs/msg/AccelWithCovariance.msg: msg.covariance
/// - geometry_msgs/msg/TwistWithCovariance.msg: msg.covariance
/// - geometry_msgs/msg/PoseWithCovariance.msg: msg.covariance
enum XYZRPY_COV_IDX {
  X_X = 0,
  X_Y = 1,
  X_Z = 2,
  X_ROLL = 3,
  X_PITCH = 4,
  X_YAW = 5,
  Y_X = 6,
  Y_Y = 7,
  Y_Z = 8,
  Y_ROLL = 9,
  Y_PITCH = 10,
  Y_YAW = 11,
  Z_X = 12,
  Z_Y = 13,
  Z_Z = 14,
  Z_ROLL = 15,
  Z_PITCH = 16,
  Z_YAW = 17,
  ROLL_X = 18,
  ROLL_Y = 19,
  ROLL_Z = 20,
  ROLL_ROLL = 21,
  ROLL_PITCH = 22,
  ROLL_YAW = 23,
  PITCH_X = 24,
  PITCH_Y = 25,
  PITCH_Z = 26,
  PITCH_ROLL = 27,
  PITCH_PITCH = 28,
  PITCH_YAW = 29,
  YAW_X = 30,
  YAW_Y = 31,
  YAW_Z = 32,
  YAW_ROLL = 33,
  YAW_PITCH = 34,
  YAW_YAW = 35
};
}  // namespace xyzrpy_covariance_index

namespace xyz_upper_covariance_index
{
/// Upper-triangle covariance about the x, y, z axes
/// Used at
/// - radar_msgs/msg/RadarTrack.msg: msg.{position, velocity, acceleration}_covariance
enum XYZ_UPPER_COV_IDX {
  X_X = 0,
  X_Y = 1,
  X_Z = 2,
  Y_Y = 3,
  Y_Z = 4,
  Z_Z = 5,
};
}  // namespace xyz_upper_covariance_index
}  // namespace tier4_autoware_utils

#endif  // TIER4_AUTOWARE_UTILS__ROS__MSG_COVARIANCE_HPP_
