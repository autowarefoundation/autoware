// Copyright 2023- Autoware Foundation
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

#ifndef POSE_INSTABILITY_DETECTOR_HPP_
#define POSE_INSTABILITY_DETECTOR_HPP_

#include <rclcpp/rclcpp.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <vector>

class PoseInstabilityDetector : public rclcpp::Node
{
  using Quaternion = geometry_msgs::msg::Quaternion;
  using Twist = geometry_msgs::msg::Twist;
  using TwistWithCovarianceStamped = geometry_msgs::msg::TwistWithCovarianceStamped;
  using Pose = geometry_msgs::msg::Pose;
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using Odometry = nav_msgs::msg::Odometry;
  using KeyValue = diagnostic_msgs::msg::KeyValue;
  using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
  using DiagnosticArray = diagnostic_msgs::msg::DiagnosticArray;

public:
  explicit PoseInstabilityDetector(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void callback_odometry(Odometry::ConstSharedPtr odometry_msg_ptr);
  void callback_twist(TwistWithCovarianceStamped::ConstSharedPtr twist_msg_ptr);
  void callback_timer();

  // subscribers and timer
  rclcpp::Subscription<Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<TwistWithCovarianceStamped>::SharedPtr twist_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // publisher
  rclcpp::Publisher<PoseStamped>::SharedPtr diff_pose_pub_;
  rclcpp::Publisher<DiagnosticArray>::SharedPtr diagnostics_pub_;

  // parameters
  const double threshold_diff_position_x_;
  const double threshold_diff_position_y_;
  const double threshold_diff_position_z_;
  const double threshold_diff_angle_x_;
  const double threshold_diff_angle_y_;
  const double threshold_diff_angle_z_;

  // variables
  std::optional<Odometry> latest_odometry_ = std::nullopt;
  std::optional<Odometry> prev_odometry_ = std::nullopt;
  std::vector<TwistWithCovarianceStamped> twist_buffer_;
};

#endif  // POSE_INSTABILITY_DETECTOR_HPP_
