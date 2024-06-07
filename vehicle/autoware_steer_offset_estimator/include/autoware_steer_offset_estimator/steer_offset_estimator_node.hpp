// Copyright 2022 Tier IV, Inc.
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

#ifndef AUTOWARE_STEER_OFFSET_ESTIMATOR__STEER_OFFSET_ESTIMATOR_NODE_HPP_
#define AUTOWARE_STEER_OFFSET_ESTIMATOR__STEER_OFFSET_ESTIMATOR_NODE_HPP_

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"

#include "autoware_vehicle_msgs/msg/steering_report.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "tier4_debug_msgs/msg/float32_stamped.hpp"

#include <memory>

namespace autoware::steer_offset_estimator
{
using geometry_msgs::msg::TwistStamped;
using tier4_debug_msgs::msg::Float32Stamped;
using Steering = autoware_vehicle_msgs::msg::SteeringReport;
using diagnostic_updater::DiagnosticStatusWrapper;
using diagnostic_updater::Updater;

class SteerOffsetEstimatorNode : public rclcpp::Node
{
private:
  rclcpp::Publisher<Float32Stamped>::SharedPtr pub_steer_offset_;
  rclcpp::Publisher<Float32Stamped>::SharedPtr pub_steer_offset_cov_;
  rclcpp::Subscription<TwistStamped>::SharedPtr sub_twist_;
  rclcpp::Subscription<Steering>::SharedPtr sub_steer_;
  rclcpp::TimerBase::SharedPtr timer_;

  TwistStamped::ConstSharedPtr twist_ptr_;
  Steering::ConstSharedPtr steer_ptr_;
  double wheel_base_;
  double update_hz_;
  double estimated_steer_offset_{0.0};
  double covariance_;
  double forgetting_factor_;
  double valid_min_velocity_;
  double valid_max_steer_;
  double warn_steer_offset_abs_error_;

  // Diagnostic Updater
  std::shared_ptr<Updater> updater_ptr_;

  void onTwist(const TwistStamped::ConstSharedPtr msg);
  void onSteer(const Steering::ConstSharedPtr msg);
  void onTimer();
  bool updateSteeringOffset();
  void monitorSteerOffset(DiagnosticStatusWrapper & stat);

public:
  explicit SteerOffsetEstimatorNode(const rclcpp::NodeOptions & node_options);
};
}  // namespace autoware::steer_offset_estimator

#endif  // AUTOWARE_STEER_OFFSET_ESTIMATOR__STEER_OFFSET_ESTIMATOR_NODE_HPP_
