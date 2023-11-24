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

#include "steer_offset_estimator/steer_offset_estimator_node.hpp"

#include "vehicle_info_util/vehicle_info_util.hpp"

#include <memory>
#include <utility>

namespace steer_offset_estimator
{
SteerOffsetEstimatorNode::SteerOffsetEstimatorNode(const rclcpp::NodeOptions & node_options)
: Node("steer_offset_estimator", node_options)
{
  using std::placeholders::_1;
  // get parameter
  const auto vehicle_info = vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo();
  wheel_base_ = vehicle_info.wheel_base_m;
  covariance_ = this->declare_parameter<double>("initial_covariance");
  forgetting_factor_ = this->declare_parameter<double>("forgetting_factor");
  update_hz_ = this->declare_parameter<double>("steer_update_hz");
  valid_min_velocity_ = this->declare_parameter<double>("valid_min_velocity");
  valid_max_steer_ = this->declare_parameter<double>("valid_max_steer");
  warn_steer_offset_abs_error_ =
    this->declare_parameter<double>("warn_steer_offset_deg") * M_PI / 180.0;

  // publisher
  pub_steer_offset_ = this->create_publisher<Float32Stamped>("~/output/steering_offset", 1);
  pub_steer_offset_cov_ =
    this->create_publisher<Float32Stamped>("~/output/steering_offset_covariance", 1);

  // subscriber
  sub_twist_ = this->create_subscription<TwistStamped>(
    "~/input/twist", 1, std::bind(&SteerOffsetEstimatorNode::onTwist, this, _1));
  sub_steer_ = this->create_subscription<Steering>(
    "~/input/steer", 1, std::bind(&SteerOffsetEstimatorNode::onSteer, this, _1));

  /* Diagnostic Updater */
  updater_ptr_ = std::make_shared<Updater>(this, 1.0 / update_hz_);
  updater_ptr_->setHardwareID("steer_offset_estimator");
  updater_ptr_->add("steer_offset_estimator", this, &SteerOffsetEstimatorNode::monitorSteerOffset);

  // timer
  {
    auto on_timer = std::bind(&SteerOffsetEstimatorNode::onTimer, this);
    const auto period = rclcpp::Rate(update_hz_).period();
    timer_ = std::make_shared<rclcpp::GenericTimer<decltype(on_timer)>>(
      this->get_clock(), period, std::move(on_timer),
      this->get_node_base_interface()->get_context());
    this->get_node_timers_interface()->add_timer(timer_, nullptr);
  }
}

// function for diagnostics
void SteerOffsetEstimatorNode::monitorSteerOffset(DiagnosticStatusWrapper & stat)
{
  using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;
  const double eps = 1e-3;
  if (covariance_ > eps) {
    stat.summary(DiagStatus::OK, "Preparation");
    return;
  }
  if (std::abs(estimated_steer_offset_) > warn_steer_offset_abs_error_) {
    stat.summary(DiagStatus::WARN, "Steer offset is larger than tolerance");
    return;
  }
  stat.summary(DiagStatus::OK, "Calibration OK");
}

void SteerOffsetEstimatorNode::onTwist(const TwistStamped::ConstSharedPtr msg)
{
  twist_ptr_ = msg;
}

void SteerOffsetEstimatorNode::onSteer(const Steering::ConstSharedPtr msg)
{
  steer_ptr_ = msg;
}

bool SteerOffsetEstimatorNode::updateSteeringOffset()
{
  // RLS; recursive least-squares algorithm

  if (!twist_ptr_ || !steer_ptr_) {
    // null input
    return false;
  }

  const double vel = twist_ptr_->twist.linear.x;
  const double wz = twist_ptr_->twist.angular.z;
  const double steer = steer_ptr_->steering_tire_angle;

  if (std::abs(vel) < valid_min_velocity_ || std::abs(steer) > valid_max_steer_) {
    // invalid velocity/steer value for estimating steering offset
    return false;
  }

  // use following approximation: tan(a+b) = tan(a) + tan(b) = a + b

  const double phi = vel / wheel_base_;
  covariance_ = (covariance_ - (covariance_ * phi * phi * covariance_) /
                                 (forgetting_factor_ + phi * covariance_ * phi)) /
                forgetting_factor_;

  const double coef = (covariance_ * phi) / (forgetting_factor_ + phi * covariance_ * phi);
  const double measured_wz_offset = wz - phi * steer;
  const double error_wz_offset = measured_wz_offset - phi * estimated_steer_offset_;

  estimated_steer_offset_ = estimated_steer_offset_ + coef * error_wz_offset;

  return true;
}

void SteerOffsetEstimatorNode::onTimer()
{
  if (updateSteeringOffset()) {
    auto msg = std::make_unique<Float32Stamped>();
    msg->stamp = this->now();
    msg->data = estimated_steer_offset_;
    pub_steer_offset_->publish(std::move(msg));

    auto cov_msg = std::make_unique<Float32Stamped>();
    cov_msg->stamp = this->now();
    cov_msg->data = covariance_;
    pub_steer_offset_cov_->publish(std::move(cov_msg));
  }
}
}  // namespace steer_offset_estimator

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(steer_offset_estimator::SteerOffsetEstimatorNode)
