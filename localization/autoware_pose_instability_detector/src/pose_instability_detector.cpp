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

#include "autoware/pose_instability_detector/pose_instability_detector.hpp"

#include "autoware/universe_utils/geometry/geometry.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

namespace autoware::pose_instability_detector
{
PoseInstabilityDetector::PoseInstabilityDetector(const rclcpp::NodeOptions & options)
: rclcpp::Node("pose_instability_detector", options),
  timer_period_(this->declare_parameter<double>("timer_period")),
  heading_velocity_maximum_(this->declare_parameter<double>("heading_velocity_maximum")),
  heading_velocity_scale_factor_tolerance_(
    this->declare_parameter<double>("heading_velocity_scale_factor_tolerance")),
  angular_velocity_maximum_(this->declare_parameter<double>("angular_velocity_maximum")),
  angular_velocity_scale_factor_tolerance_(
    this->declare_parameter<double>("angular_velocity_scale_factor_tolerance")),
  angular_velocity_bias_tolerance_(
    this->declare_parameter<double>("angular_velocity_bias_tolerance")),
  pose_estimator_longitudinal_tolerance_(
    this->declare_parameter<double>("pose_estimator_longitudinal_tolerance")),
  pose_estimator_lateral_tolerance_(
    this->declare_parameter<double>("pose_estimator_lateral_tolerance")),
  pose_estimator_vertical_tolerance_(
    this->declare_parameter<double>("pose_estimator_vertical_tolerance")),
  pose_estimator_angular_tolerance_(
    this->declare_parameter<double>("pose_estimator_angular_tolerance"))
{
  // Define subscribers, publishers and a timer.
  odometry_sub_ = this->create_subscription<Odometry>(
    "~/input/odometry", 10,
    std::bind(&PoseInstabilityDetector::callback_odometry, this, std::placeholders::_1));

  twist_sub_ = this->create_subscription<TwistWithCovarianceStamped>(
    "~/input/twist", 10,
    std::bind(&PoseInstabilityDetector::callback_twist, this, std::placeholders::_1));

  timer_ = rclcpp::create_timer(
    this, this->get_clock(), std::chrono::duration<double>(timer_period_),
    std::bind(&PoseInstabilityDetector::callback_timer, this));

  diff_pose_pub_ = this->create_publisher<PoseStamped>("~/debug/diff_pose", 10);
  diagnostics_pub_ = this->create_publisher<DiagnosticArray>("/diagnostics", 10);
}

void PoseInstabilityDetector::callback_odometry(Odometry::ConstSharedPtr odometry_msg_ptr)
{
  latest_odometry_ = *odometry_msg_ptr;
}

void PoseInstabilityDetector::callback_twist(
  TwistWithCovarianceStamped::ConstSharedPtr twist_msg_ptr)
{
  twist_buffer_.push_back(*twist_msg_ptr);
}

void PoseInstabilityDetector::callback_timer()
{
  // odometry callback and timer callback has to be called at least once
  if (latest_odometry_ == std::nullopt) {
    return;
  }
  if (prev_odometry_ == std::nullopt) {
    prev_odometry_ = latest_odometry_;
    return;
  }

  // twist callback has to be called at least once
  if (twist_buffer_.empty()) {
    return;
  }

  // time variables
  const rclcpp::Time latest_odometry_time = rclcpp::Time(latest_odometry_->header.stamp);
  const rclcpp::Time prev_odometry_time = rclcpp::Time(prev_odometry_->header.stamp);

  // define lambda function to convert quaternion to rpy
  auto quat_to_rpy = [](const Quaternion & quat) {
    tf2::Quaternion tf2_quat(quat.x, quat.y, quat.z, quat.w);
    tf2::Matrix3x3 mat(tf2_quat);
    double roll{};
    double pitch{};
    double yaw{};
    mat.getRPY(roll, pitch, yaw);
    return std::make_tuple(roll, pitch, yaw);
  };

  // delete twist data older than prev_odometry_ (but preserve the one right before prev_odometry_)
  while (twist_buffer_.size() > 1) {
    if (rclcpp::Time(twist_buffer_[1].header.stamp) < prev_odometry_time) {
      twist_buffer_.pop_front();
    } else {
      break;
    }
  }

  // dead reckoning from prev_odometry_ to latest_odometry_
  PoseStamped::SharedPtr prev_pose = std::make_shared<PoseStamped>();
  prev_pose->header = prev_odometry_->header;
  prev_pose->pose = prev_odometry_->pose.pose;

  Pose::SharedPtr dr_pose = std::make_shared<Pose>();
  dead_reckon(prev_pose, latest_odometry_time, twist_buffer_, dr_pose);

  // compare dead reckoning pose and latest_odometry_
  const Pose latest_ekf_pose = latest_odometry_->pose.pose;
  const Pose ekf_to_dr = autoware::universe_utils::inverseTransformPose(*dr_pose, latest_ekf_pose);
  const geometry_msgs::msg::Point pos = ekf_to_dr.position;
  const auto [ang_x, ang_y, ang_z] = quat_to_rpy(ekf_to_dr.orientation);
  const std::vector<double> values = {pos.x, pos.y, pos.z, ang_x, ang_y, ang_z};

  // publish diff_pose for debug
  PoseStamped diff_pose;
  diff_pose.header.stamp = latest_odometry_time;
  diff_pose.header.frame_id = "base_link";
  diff_pose.pose = ekf_to_dr;
  diff_pose_pub_->publish(diff_pose);

  // publish diagnostics
  ThresholdValues threshold_values =
    calculate_threshold((latest_odometry_time - prev_odometry_time).seconds());

  const std::vector<double> thresholds = {threshold_values.position_x, threshold_values.position_y,
                                          threshold_values.position_z, threshold_values.angle_x,
                                          threshold_values.angle_y,    threshold_values.angle_z};

  const std::vector<std::string> labels = {"diff_position_x", "diff_position_y", "diff_position_z",
                                           "diff_angle_x",    "diff_angle_y",    "diff_angle_z"};

  DiagnosticStatus status;
  status.name = "localization: pose_instability_detector";
  status.hardware_id = this->get_name();
  bool all_ok = true;

  for (size_t i = 0; i < values.size(); ++i) {
    const bool ok = (std::abs(values[i]) < thresholds[i]);
    all_ok &= ok;
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = labels[i] + ":threshold";
    kv.value = std::to_string(thresholds[i]);
    status.values.push_back(kv);
    kv.key = labels[i] + ":value";
    kv.value = std::to_string(values[i]);
    status.values.push_back(kv);
    kv.key = labels[i] + ":status";
    kv.value = (ok ? "OK" : "WARN");
    status.values.push_back(kv);
  }
  status.level = (all_ok ? DiagnosticStatus::OK : DiagnosticStatus::WARN);
  status.message = (all_ok ? "OK" : "WARN");

  DiagnosticArray diagnostics;
  diagnostics.header.stamp = latest_odometry_time;
  diagnostics.status.emplace_back(status);
  diagnostics_pub_->publish(diagnostics);

  // prepare for next loop
  prev_odometry_ = latest_odometry_;
}

PoseInstabilityDetector::ThresholdValues PoseInstabilityDetector::calculate_threshold(
  double interval_sec) const
{
  // Calculate maximum longitudinal difference
  const double longitudinal_difference =
    heading_velocity_maximum_ * heading_velocity_scale_factor_tolerance_ * 0.01 * interval_sec;

  // Calculate maximum lateral and vertical difference
  double lateral_difference = 0.0;

  const std::vector<double> heading_velocity_signs = {1.0, -1.0, -1.0, 1.0};
  const std::vector<double> angular_velocity_signs = {1.0, 1.0, -1.0, -1.0};

  const double nominal_variation_x = heading_velocity_maximum_ / angular_velocity_maximum_ *
                                     sin(angular_velocity_maximum_ * interval_sec);
  const double nominal_variation_y = heading_velocity_maximum_ / angular_velocity_maximum_ *
                                     (1 - cos(angular_velocity_maximum_ * interval_sec));

  for (int i = 0; i < 4; i++) {
    const double edge_heading_velocity =
      heading_velocity_maximum_ *
      (1 + heading_velocity_signs[i] * heading_velocity_scale_factor_tolerance_ * 0.01);
    const double edge_angular_velocity =
      angular_velocity_maximum_ *
        (1 + angular_velocity_signs[i] * angular_velocity_scale_factor_tolerance_ * 0.01) +
      angular_velocity_signs[i] * angular_velocity_bias_tolerance_;

    const double edge_variation_x =
      edge_heading_velocity / edge_angular_velocity * sin(edge_angular_velocity * interval_sec);
    const double edge_variation_y = edge_heading_velocity / edge_angular_velocity *
                                    (1 - cos(edge_angular_velocity * interval_sec));

    const double diff_variation_x = edge_variation_x - nominal_variation_x;
    const double diff_variation_y = edge_variation_y - nominal_variation_y;

    const double lateral_difference_candidate = abs(
      diff_variation_x * sin(angular_velocity_maximum_ * interval_sec) -
      diff_variation_y * cos(angular_velocity_maximum_ * interval_sec));
    lateral_difference = std::max(lateral_difference, lateral_difference_candidate);
  }

  const double vertical_difference = lateral_difference;

  // Calculate maximum angular difference
  const double roll_difference =
    (angular_velocity_maximum_ * angular_velocity_scale_factor_tolerance_ * 0.01 +
     angular_velocity_bias_tolerance_) *
    interval_sec;
  const double pitch_difference = roll_difference;
  const double yaw_difference = roll_difference;

  // Set thresholds
  ThresholdValues result_values{};
  result_values.position_x = longitudinal_difference + pose_estimator_longitudinal_tolerance_;
  result_values.position_y = lateral_difference + pose_estimator_lateral_tolerance_;
  result_values.position_z = vertical_difference + pose_estimator_vertical_tolerance_;
  result_values.angle_x = roll_difference + pose_estimator_angular_tolerance_;
  result_values.angle_y = pitch_difference + pose_estimator_angular_tolerance_;
  result_values.angle_z = yaw_difference + pose_estimator_angular_tolerance_;

  return result_values;
}

void PoseInstabilityDetector::dead_reckon(
  PoseStamped::SharedPtr & initial_pose, const rclcpp::Time & end_time,
  const std::deque<TwistWithCovarianceStamped> & twist_deque, Pose::SharedPtr & estimated_pose)
{
  // get start time
  rclcpp::Time start_time = rclcpp::Time(initial_pose->header.stamp);

  // initialize estimated_pose
  estimated_pose->position = initial_pose->pose.position;
  estimated_pose->orientation = initial_pose->pose.orientation;

  // cut out necessary twist data
  std::deque<TwistWithCovarianceStamped> sliced_twist_deque =
    clip_out_necessary_twist(twist_deque, start_time, end_time);

  // dead reckoning
  rclcpp::Time prev_odometry_time = rclcpp::Time(sliced_twist_deque.front().header.stamp);
  tf2::Quaternion prev_orientation;
  tf2::fromMsg(estimated_pose->orientation, prev_orientation);

  for (size_t i = 1; i < sliced_twist_deque.size(); ++i) {
    const rclcpp::Time curr_time = rclcpp::Time(sliced_twist_deque[i].header.stamp);
    const double time_diff_sec = (curr_time - prev_odometry_time).seconds();

    const Twist twist = sliced_twist_deque[i - 1].twist.twist;

    // variation of orientation (rpy update)
    tf2::Quaternion delta_orientation;
    tf2::Vector3 rotation_axis(twist.angular.x, twist.angular.y, twist.angular.z);
    if (rotation_axis.length() > 0.0) {
      delta_orientation.setRotation(
        rotation_axis.normalized(), rotation_axis.length() * time_diff_sec);
    } else {
      delta_orientation.setValue(0.0, 0.0, 0.0, 1.0);
    }

    tf2::Quaternion curr_orientation;
    curr_orientation = prev_orientation * delta_orientation;
    curr_orientation.normalize();

    // average quaternion of two frames
    tf2::Quaternion average_quat = prev_orientation.slerp(curr_orientation, 0.5);

    // Convert twist to world frame (take average of two frames)
    tf2::Vector3 linear_velocity(twist.linear.x, twist.linear.y, twist.linear.z);
    linear_velocity = tf2::quatRotate(average_quat, linear_velocity);

    // xyz update
    estimated_pose->position.x += linear_velocity.x() * time_diff_sec;
    estimated_pose->position.y += linear_velocity.y() * time_diff_sec;
    estimated_pose->position.z += linear_velocity.z() * time_diff_sec;

    // update previous variables
    prev_odometry_time = curr_time;
    prev_orientation = curr_orientation;
  }
  estimated_pose->orientation.x = prev_orientation.x();
  estimated_pose->orientation.y = prev_orientation.y();
  estimated_pose->orientation.z = prev_orientation.z();
  estimated_pose->orientation.w = prev_orientation.w();
}

std::deque<PoseInstabilityDetector::TwistWithCovarianceStamped>
PoseInstabilityDetector::clip_out_necessary_twist(
  const std::deque<TwistWithCovarianceStamped> & twist_buffer, const rclcpp::Time & start_time,
  const rclcpp::Time & end_time)
{
  // If there is only one element in the twist_buffer, return a deque that has the same twist
  // from the start till the end
  if (twist_buffer.size() == 1) {
    TwistWithCovarianceStamped twist = twist_buffer.front();
    std::deque<TwistWithCovarianceStamped> simple_twist_deque;

    twist.header.stamp = start_time;
    simple_twist_deque.push_back(twist);

    twist.header.stamp = end_time;
    simple_twist_deque.push_back(twist);

    return simple_twist_deque;
  }

  // get iterator to the element that is right before start_time (if it does not exist, start_it =
  // twist_buffer.begin())
  auto start_it = twist_buffer.begin();

  for (auto it = twist_buffer.begin(); it != twist_buffer.end(); ++it) {
    if (rclcpp::Time(it->header.stamp) > start_time) {
      break;
    }
    start_it = it;
  }

  // get iterator to the element that is right after end_time (if it does not exist, end_it =
  // twist_buffer.end())
  auto end_it = twist_buffer.end();
  end_it--;
  for (auto it = end_it; it != twist_buffer.begin(); --it) {
    if (rclcpp::Time(it->header.stamp) < end_time) {
      break;
    }
    end_it = it;
  }

  // Create result deque
  std::deque<TwistWithCovarianceStamped> result_deque(start_it, end_it);

  // If the first element is later than start_time, add the first element to the front of the
  // result_deque
  if (rclcpp::Time(result_deque.front().header.stamp) > start_time) {
    TwistWithCovarianceStamped start_twist = *start_it;
    start_twist.header.stamp = start_time;
    result_deque.push_front(start_twist);
  } else {
    if (result_deque.size() < 2) {
      return result_deque;
    }
    // If the first element is earlier than start_time, interpolate the first element
    rclcpp::Time time0 = rclcpp::Time(result_deque[0].header.stamp);
    rclcpp::Time time1 = rclcpp::Time(result_deque[1].header.stamp);
    double ratio = (start_time - time0).seconds() / (time1 - time0).seconds();
    Twist twist0 = result_deque[0].twist.twist;
    Twist twist1 = result_deque[1].twist.twist;
    result_deque[0].twist.twist.linear.x = twist1.linear.x * ratio + twist0.linear.x * (1 - ratio);
    result_deque[0].twist.twist.linear.y = twist1.linear.y * ratio + twist0.linear.y * (1 - ratio);
    result_deque[0].twist.twist.linear.z = twist1.linear.z * ratio + twist0.linear.z * (1 - ratio);
    result_deque[0].twist.twist.angular.x =
      twist1.angular.x * ratio + twist0.angular.x * (1 - ratio);
    result_deque[0].twist.twist.angular.y =
      twist1.angular.y * ratio + twist0.angular.y * (1 - ratio);
    result_deque[0].twist.twist.angular.z =
      twist1.angular.z * ratio + twist0.angular.z * (1 - ratio);

    result_deque[0].header.stamp = start_time;
  }

  // If the last element is earlier than end_time, add the last element to the back of the
  // result_deque
  if (rclcpp::Time(result_deque.back().header.stamp) < end_time) {
    TwistWithCovarianceStamped end_twist = *end_it;
    end_twist.header.stamp = end_time;
    result_deque.push_back(end_twist);
  } else {
    if (result_deque.size() < 2) {
      return result_deque;
    }
    // If the last element is later than end_time, interpolate the last element
    rclcpp::Time time0 = rclcpp::Time(result_deque[result_deque.size() - 2].header.stamp);
    rclcpp::Time time1 = rclcpp::Time(result_deque[result_deque.size() - 1].header.stamp);
    double ratio = (end_time - time0).seconds() / (time1 - time0).seconds();
    Twist twist0 = result_deque[result_deque.size() - 2].twist.twist;
    Twist twist1 = result_deque[result_deque.size() - 1].twist.twist;
    result_deque[result_deque.size() - 1].twist.twist.linear.x =
      twist1.linear.x * ratio + twist0.linear.x * (1 - ratio);
    result_deque[result_deque.size() - 1].twist.twist.linear.y =
      twist1.linear.y * ratio + twist0.linear.y * (1 - ratio);
    result_deque[result_deque.size() - 1].twist.twist.linear.z =
      twist1.linear.z * ratio + twist0.linear.z * (1 - ratio);
    result_deque[result_deque.size() - 1].twist.twist.angular.x =
      twist1.angular.x * ratio + twist0.angular.x * (1 - ratio);
    result_deque[result_deque.size() - 1].twist.twist.angular.y =
      twist1.angular.y * ratio + twist0.angular.y * (1 - ratio);
    result_deque[result_deque.size() - 1].twist.twist.angular.z =
      twist1.angular.z * ratio + twist0.angular.z * (1 - ratio);

    result_deque[result_deque.size() - 1].header.stamp = end_time;
  }
  return result_deque;
}
}  // namespace autoware::pose_instability_detector

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::pose_instability_detector::PoseInstabilityDetector)
