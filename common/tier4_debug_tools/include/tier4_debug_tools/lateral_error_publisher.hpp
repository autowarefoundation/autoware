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

#ifndef TIER4_DEBUG_TOOLS__LATERAL_ERROR_PUBLISHER_HPP_
#define TIER4_DEBUG_TOOLS__LATERAL_ERROR_PUBLISHER_HPP_

#define EIGEN_MPL2_ONLY

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <motion_utils/trajectory/trajectory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tier4_debug_msgs/msg/float32_stamped.hpp>

class LateralErrorPublisher : public rclcpp::Node
{
public:
  explicit LateralErrorPublisher(const rclcpp::NodeOptions & node_options);

private:
  /* Parameters */
  double yaw_threshold_to_search_closest_;

  /* States */
  autoware_auto_planning_msgs::msg::Trajectory::SharedPtr
    current_trajectory_ptr_;  //!< @brief reference trajectory
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr
    current_vehicle_pose_ptr_;  //!< @brief current EKF pose
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr
    current_ground_truth_pose_ptr_;  //!< @brief current GNSS pose

  /* Publishers and Subscribers */
  rclcpp::Subscription<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr
    sub_trajectory_;  //!< @brief subscription for reference trajectory
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    sub_vehicle_pose_;  //!< @brief subscription for vehicle pose
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    sub_ground_truth_pose_;  //!< @brief subscription for gnss pose
  rclcpp::Publisher<tier4_debug_msgs::msg::Float32Stamped>::SharedPtr
    pub_control_lateral_error_;  //!< @brief publisher for control lateral error
  rclcpp::Publisher<tier4_debug_msgs::msg::Float32Stamped>::SharedPtr
    pub_localization_lateral_error_;  //!< @brief publisher for localization lateral error
  rclcpp::Publisher<tier4_debug_msgs::msg::Float32Stamped>::SharedPtr
    pub_lateral_error_;  //!< @brief publisher for lateral error (control + localization)

  /**
   * @brief set current_trajectory_ with received message
   */
  void onTrajectory(const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr);
  /**
   * @brief set current_vehicle_pose_ with received message
   */
  void onVehiclePose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  /**
   * @brief set current_ground_truth_pose_ and calculate lateral error
   */
  void onGroundTruthPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
};

#endif  // TIER4_DEBUG_TOOLS__LATERAL_ERROR_PUBLISHER_HPP_
