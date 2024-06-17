// Copyright 2023 TIER IV, Inc.
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

#ifndef AUTOWARE__CONTROL_VALIDATOR__CONTROL_VALIDATOR_HPP_
#define AUTOWARE__CONTROL_VALIDATOR__CONTROL_VALIDATOR_HPP_

#include "autoware/control_validator/debug_marker.hpp"
#include "autoware_control_validator/msg/control_validator_status.hpp"
#include "autoware_vehicle_info_utils/vehicle_info_utils.hpp"
#include "tier4_autoware_utils/ros/polling_subscriber.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <memory>
#include <string>

namespace autoware::control_validator
{
using autoware_control_validator::msg::ControlValidatorStatus;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using diagnostic_updater::DiagnosticStatusWrapper;
using diagnostic_updater::Updater;
using nav_msgs::msg::Odometry;

struct ValidationParams
{
  double max_distance_deviation_threshold;
};

class ControlValidator : public rclcpp::Node
{
public:
  explicit ControlValidator(const rclcpp::NodeOptions & options);

  void onPredictedTrajectory(const Trajectory::ConstSharedPtr msg);

  bool checkValidMaxDistanceDeviation(const Trajectory & predicted_trajectory);

private:
  void setupDiag();

  void setupParameters();

  bool isDataReady();

  void validate(const Trajectory & trajectory);

  void publishPredictedTrajectory();
  void publishDebugInfo();
  void displayStatus();

  void setStatus(DiagnosticStatusWrapper & stat, const bool & is_ok, const std::string & msg);

  rclcpp::Subscription<Trajectory>::SharedPtr sub_predicted_traj_;
  tier4_autoware_utils::InterProcessPollingSubscriber<Odometry> sub_kinematics_{
    this, "~/input/kinematics"};
  tier4_autoware_utils::InterProcessPollingSubscriber<Trajectory> sub_reference_traj_{
    this, "~/input/reference_trajectory"};
  rclcpp::Publisher<ControlValidatorStatus>::SharedPtr pub_status_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;

  // system parameters
  int diag_error_count_threshold_ = 0;
  bool display_on_terminal_ = true;

  Updater diag_updater_{this};

  ControlValidatorStatus validation_status_;
  ValidationParams validation_params_;  // for thresholds

  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;

  bool isAllValid(const ControlValidatorStatus & status);

  Trajectory::ConstSharedPtr current_reference_trajectory_;
  Trajectory::ConstSharedPtr current_predicted_trajectory_;

  Odometry::ConstSharedPtr current_kinematics_;

  std::shared_ptr<ControlValidatorDebugMarkerPublisher> debug_pose_publisher_;
};
}  // namespace autoware::control_validator

#endif  // AUTOWARE__CONTROL_VALIDATOR__CONTROL_VALIDATOR_HPP_
