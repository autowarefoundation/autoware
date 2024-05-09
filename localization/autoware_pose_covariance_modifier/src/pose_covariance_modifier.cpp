// Copyright 2024 The Autoware Foundation
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

#include "include/pose_covariance_modifier.hpp"

#include <interpolation/linear_interpolation.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

namespace autoware::pose_covariance_modifier
{
using PoseSource = PoseCovarianceModifierNode::PoseSource;

PoseCovarianceModifierNode::PoseCovarianceModifierNode(const rclcpp::NodeOptions & node_options)
: Node("PoseCovarianceModifierNode", node_options),
  gnss_pose_received_time_last_(this->now()),
  pose_source_(PoseSource::NDT)
{
  // parameters
  threshold_gnss_stddev_yaw_deg_max_ =
    this->declare_parameter<double>("threshold_gnss_stddev_yaw_deg_max");
  threshold_gnss_stddev_z_max_ = this->declare_parameter<double>("threshold_gnss_stddev_z_max");
  threshold_gnss_stddev_xy_bound_lower_ =
    this->declare_parameter<double>("threshold_gnss_stddev_xy_bound_lower");
  threshold_gnss_stddev_xy_bound_upper_ =
    this->declare_parameter<double>("threshold_gnss_stddev_xy_bound_upper");
  ndt_std_dev_bound_lower_ = this->declare_parameter<double>("ndt_std_dev_bound_lower");
  ndt_std_dev_bound_upper_ = this->declare_parameter<double>("ndt_std_dev_bound_upper");
  gnss_pose_timeout_sec_ = this->declare_parameter<double>("gnss_pose_timeout_sec");
  debug_mode_ = this->declare_parameter<bool>("enable_debug_topics");

  // subscribers
  sub_gnss_pose_with_cov_ =
    this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "input_gnss_pose_with_cov_topic", 10,
      std::bind(
        &PoseCovarianceModifierNode::callback_gnss_pose_with_cov, this, std::placeholders::_1));

  sub_ndt_pose_with_cov_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "input_ndt_pose_with_cov_topic", 10,
    std::bind(
      &PoseCovarianceModifierNode::callback_ndt_pose_with_cov, this, std::placeholders::_1));

  // publishers
  pub_pose_with_covariance_stamped_ =
    this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "output_pose_with_covariance_topic", 10);

  pub_str_pose_source_ = this->create_publisher<std_msgs::msg::String>("~/selected_pose_type", 10);

  if (debug_mode_) {
    pub_double_ndt_position_stddev_ =
      this->create_publisher<std_msgs::msg::Float64>("~/debug/ndt_position_stddev", 10);
    pub_double_gnss_position_stddev_ =
      this->create_publisher<std_msgs::msg::Float64>("~/debug/gnss_position_stddev", 10);
  }
}

void PoseCovarianceModifierNode::callback_gnss_pose_with_cov(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & msg_pose_with_cov_in)
{
  // will be used to check if GNSS pose has timed out in the NDT pose callback
  gnss_pose_received_time_last_ = this->now();

  // if the pose source is not GNSS, it will be used to calculate the NDT covariance in the NDT pose
  // callback
  gnss_pose_with_cov_last_ = msg_pose_with_cov_in;

  const double gnss_pose_yaw_stddev_deg =
    std::sqrt(msg_pose_with_cov_in->pose.covariance[YAW_POS_IDX_]) * 180 / M_PI;

  const double gnss_pose_stddev_z = std::sqrt(msg_pose_with_cov_in->pose.covariance[Z_POS_IDX_]);

  const double gnss_pose_stddev_xy =
    (std::sqrt(msg_pose_with_cov_in->pose.covariance[X_POS_IDX_]) +
     std::sqrt(msg_pose_with_cov_in->pose.covariance[Y_POS_IDX_])) /
    2;

  pose_source_ =
    pose_source_from_gnss_stddev(gnss_pose_yaw_stddev_deg, gnss_pose_stddev_z, gnss_pose_stddev_xy);
  publish_pose_type(pose_source_);

  if (pose_source_ == PoseSource::NDT) {
    // if the pose source is only NDT, don't publish GNSS poses
    return;
  }

  pub_pose_with_covariance_stamped_->publish(*msg_pose_with_cov_in);

  if (debug_mode_) {
    std_msgs::msg::Float64 msg_double;
    msg_double.data = gnss_pose_stddev_xy;
    pub_double_gnss_position_stddev_->publish(msg_double);
  }
}

void PoseCovarianceModifierNode::callback_ndt_pose_with_cov(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & msg_pose_with_cov_in)
{
  if (pose_source_ == PoseSource::GNSS) {
    // if the pose source is only gnss, GNSS pose will be used in the GNSS pose callback
    return;
  }
  geometry_msgs::msg::PoseWithCovarianceStamped msg_pose_with_cov_out;

  // pose_source_ was determined in the GNSS callback
  if (gnss_pose_has_timed_out(gnss_pose_received_time_last_) || pose_source_ == PoseSource::NDT) {
    msg_pose_with_cov_out = *msg_pose_with_cov_in;
  } else if (pose_source_ == PoseSource::GNSS_NDT) {
    auto ndt_pose_with_cov_updated = *msg_pose_with_cov_in;
    ndt_pose_with_cov_updated.pose.covariance =
      update_ndt_covariances_from_gnss(msg_pose_with_cov_in->pose.covariance);
    msg_pose_with_cov_out = ndt_pose_with_cov_updated;
  }

  pub_pose_with_covariance_stamped_->publish(msg_pose_with_cov_out);

  if (debug_mode_) {
    std_msgs::msg::Float64 msg_double;
    msg_double.data = (std::sqrt(msg_pose_with_cov_out.pose.covariance[X_POS_IDX_]) +
                       std::sqrt(msg_pose_with_cov_out.pose.covariance[Y_POS_IDX_])) /
                      2.0;
    pub_double_ndt_position_stddev_->publish(msg_double);
  }
}

bool PoseCovarianceModifierNode::gnss_pose_has_timed_out(
  const rclcpp::Time & gnss_pose_received_time_last)
{
  auto duration = this->now() - gnss_pose_received_time_last;
  if (duration.seconds() > gnss_pose_timeout_sec_) {
    RCLCPP_WARN(this->get_logger(), "GNSS pose has timed out");
    return true;
  }
  return false;
}

PoseSource PoseCovarianceModifierNode::pose_source_from_gnss_stddev(
  const double gnss_pose_yaw_stddev_deg, const double gnss_pose_stddev_z,
  const double gnss_pose_stddev_xy) const
{
  // If the GNSS pose z or yaw has a high standard deviation, use NDT pose
  if (
    gnss_pose_yaw_stddev_deg > threshold_gnss_stddev_yaw_deg_max_ ||
    gnss_pose_stddev_z > threshold_gnss_stddev_z_max_) {
    return PoseSource::NDT;
  }
  if (gnss_pose_stddev_xy <= threshold_gnss_stddev_xy_bound_lower_) {
    return PoseSource::GNSS;
  }
  if (gnss_pose_stddev_xy <= threshold_gnss_stddev_xy_bound_upper_) {
    return PoseSource::GNSS_NDT;
  }
  // If the gnss xy standard deviation is above the upper bound, use NDT pose
  return PoseSource::NDT;
}

std::array<double, 36> PoseCovarianceModifierNode::update_ndt_covariances_from_gnss(
  const std::array<double, 36> & ndt_covariance_in)
{
  // See the ../README.md#NDT-covariance-calculation for detailed explanation

  auto lerp_range_to_range = [](double x, double x_min, double x_max, double y_min, double y_max) {
    // Normalize input value to range [0, 1]
    const double input_normalized = (x - x_min) / (x_max - x_min);

    // Interpolate to the output range
    return interpolation::lerp(y_min, y_max, input_normalized);
  };

  auto ndt_variance_from_gnss_variance = [&](double ndt_variance, double gnss_variance) {
    // Check NDT stddev bound values.
    double ndt_stddev = std::sqrt(ndt_variance);
    if (ndt_stddev > ndt_std_dev_bound_upper_ || ndt_stddev < ndt_std_dev_bound_lower_) {
      RCLCPP_ERROR(
        get_logger(),
        "Input variance of NDT exceeds bound values. Variance values of NDT were not modified. "
        "Check your bound values for NDT stddev.");
      return ndt_variance;
    }
    // calculate NDT covariance value based on gnss covariance
    const double gnss_std_dev = std::sqrt(gnss_variance);

    // interpolate the gnss_std_dev from gnss ranges to ndt ranges
    const double interpolated_std_dev = lerp_range_to_range(
      gnss_std_dev, threshold_gnss_stddev_xy_bound_lower_, threshold_gnss_stddev_xy_bound_upper_,
      ndt_std_dev_bound_lower_, ndt_std_dev_bound_upper_);

    // As the gnss error increases, the ndt error should decrease
    const double reversed_std_dev =
      ndt_std_dev_bound_lower_ + ndt_std_dev_bound_upper_ - interpolated_std_dev;

    const double interpolated_variance = std::pow(reversed_std_dev, 2);

    // Make sure the ndt covariance is not below the lower bounds of ndt covariance value and return
    return (std::max(interpolated_variance, std::pow(ndt_std_dev_bound_lower_, 2)));
  };

  std::array<double, 36> ndt_covariance = ndt_covariance_in;
  std::array<int, 3> indices = {X_POS_IDX_, Y_POS_IDX_, Z_POS_IDX_};
  for (int idx : indices) {
    ndt_covariance[idx] = ndt_variance_from_gnss_variance(
      ndt_covariance_in[idx], gnss_pose_with_cov_last_->pose.covariance[idx]);
  }

  return ndt_covariance;
}

void PoseCovarianceModifierNode::publish_pose_type(const PoseSource & pose_source)
{
  std_msgs::msg::String selected_pose_type;
  switch (pose_source) {
    case PoseSource::GNSS:
      selected_pose_type.data = "GNSS";
      break;
    case PoseSource::GNSS_NDT:
      selected_pose_type.data = "GNSS_NDT";
      break;
    case PoseSource::NDT:
      selected_pose_type.data = "NDT";
      break;
    default:
      selected_pose_type.data = "NOT_DEFINED";
      break;
  }
  pub_str_pose_source_->publish(selected_pose_type);
}

}  // namespace autoware::pose_covariance_modifier

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::pose_covariance_modifier::PoseCovarianceModifierNode)
