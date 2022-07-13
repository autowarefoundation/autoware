// Copyright 2021 Tier IV, Inc.
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

#include "tier4_debug_tools/lateral_error_publisher.hpp"

#include <limits>

LateralErrorPublisher::LateralErrorPublisher(const rclcpp::NodeOptions & node_options)
: Node("lateral_error_publisher", node_options)
{
  using std::placeholders::_1;

  /* Parameters */
  yaw_threshold_to_search_closest_ =
    declare_parameter("yaw_threshold_to_search_closest", M_PI / 4.0);

  /* Publishers and Subscribers */
  sub_trajectory_ = create_subscription<autoware_auto_planning_msgs::msg::Trajectory>(
    "~/input/reference_trajectory", rclcpp::QoS{1},
    std::bind(&LateralErrorPublisher::onTrajectory, this, _1));
  sub_vehicle_pose_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "~/input/vehicle_pose_with_covariance", rclcpp::QoS{1},
    std::bind(&LateralErrorPublisher::onVehiclePose, this, _1));
  sub_ground_truth_pose_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "~/input/ground_truth_pose_with_covariance", rclcpp::QoS{1},
    std::bind(&LateralErrorPublisher::onGroundTruthPose, this, _1));
  pub_control_lateral_error_ =
    create_publisher<tier4_debug_msgs::msg::Float32Stamped>("~/control_lateral_error", 1);
  pub_localization_lateral_error_ =
    create_publisher<tier4_debug_msgs::msg::Float32Stamped>("~/localization_lateral_error", 1);
  pub_lateral_error_ =
    create_publisher<tier4_debug_msgs::msg::Float32Stamped>("~/lateral_error", 1);
}

void LateralErrorPublisher::onTrajectory(
  const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr msg)
{
  current_trajectory_ptr_ = msg;
}

void LateralErrorPublisher::onVehiclePose(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  current_vehicle_pose_ptr_ = msg;
}

void LateralErrorPublisher::onGroundTruthPose(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  current_ground_truth_pose_ptr_ = msg;

  // Guard
  if (current_trajectory_ptr_ == nullptr || current_vehicle_pose_ptr_ == nullptr) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000 /* ms */,
      "Reference trajectory or EKF pose is not received");
    return;
  }
  if (current_trajectory_ptr_->points.size() < 2) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000 /* ms */, "Reference trajectory is too short");
    return;
  }

  // Search closest trajectory point with vehicle pose
  const auto closest_index = motion_utils::findNearestIndex(
    current_trajectory_ptr_->points, current_vehicle_pose_ptr_->pose.pose,
    std::numeric_limits<double>::max(), yaw_threshold_to_search_closest_);
  if (!closest_index) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000 /* ms */, "Failed to search closest index");
    return;
  }

  // Calculate the normal vector in the reference trajectory direction
  size_t base_index = 0;
  size_t next_index = 0;
  if (*closest_index == current_trajectory_ptr_->points.size() - 1) {
    base_index = *closest_index - 1;
    next_index = *closest_index;
  } else {
    base_index = *closest_index;
    next_index = *closest_index + 1;
  }

  const auto & base_pose = current_trajectory_ptr_->points.at(base_index).pose;
  const auto & next_pose = current_trajectory_ptr_->points.at(next_index).pose;
  const double dx = next_pose.position.x - base_pose.position.x;
  const double dy = next_pose.position.y - base_pose.position.y;
  const Eigen::Vector2d trajectory_direction(dx, dy);
  RCLCPP_DEBUG(this->get_logger(), "trajectory direction: (%f, %f)", dx, dy);

  const auto rotation = Eigen::Rotation2Dd(M_PI_2);
  const Eigen::Vector2d normal_direction = rotation * trajectory_direction;
  RCLCPP_DEBUG(
    this->get_logger(), "normal direction: (%f, %f)", normal_direction(0), normal_direction(1));
  const Eigen::Vector2d unit_normal_direction = normal_direction.normalized();
  RCLCPP_DEBUG(
    this->get_logger(), "unit normal direction: (%f, %f)", unit_normal_direction(0),
    unit_normal_direction(1));

  // Calculate control lateral error
  const auto & closest_pose = current_trajectory_ptr_->points.at(*closest_index).pose;
  const auto & vehicle_pose = current_vehicle_pose_ptr_->pose.pose;
  const Eigen::Vector2d closest_to_vehicle(
    vehicle_pose.position.x - closest_pose.position.x,
    vehicle_pose.position.y - closest_pose.position.y);
  const auto control_lateral_error = closest_to_vehicle.dot(unit_normal_direction);
  RCLCPP_DEBUG(this->get_logger(), "control_lateral_error: %f", control_lateral_error);

  // Calculate localization lateral error
  const auto ground_truth_pose = current_ground_truth_pose_ptr_->pose.pose;
  const Eigen::Vector2d vehicle_to_ground_truth(
    ground_truth_pose.position.x - vehicle_pose.position.x,
    ground_truth_pose.position.y - vehicle_pose.position.y);
  const auto localization_lateral_error = vehicle_to_ground_truth.dot(unit_normal_direction);
  RCLCPP_DEBUG(this->get_logger(), "localization_lateral_error: %f", localization_lateral_error);

  const auto lateral_error = control_lateral_error + localization_lateral_error;
  RCLCPP_DEBUG(this->get_logger(), "localization_error: %f", lateral_error);

  // Publish lateral errors
  tier4_debug_msgs::msg::Float32Stamped control_msg;
  control_msg.stamp = this->now();
  control_msg.data = static_cast<float>(control_lateral_error);
  pub_control_lateral_error_->publish(control_msg);

  tier4_debug_msgs::msg::Float32Stamped localization_msg;
  localization_msg.stamp = this->now();
  localization_msg.data = static_cast<float>(localization_lateral_error);
  pub_localization_lateral_error_->publish(localization_msg);

  tier4_debug_msgs::msg::Float32Stamped sum_msg;
  sum_msg.stamp = this->now();
  sum_msg.data = static_cast<float>(lateral_error);
  pub_lateral_error_->publish(sum_msg);
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(LateralErrorPublisher)
