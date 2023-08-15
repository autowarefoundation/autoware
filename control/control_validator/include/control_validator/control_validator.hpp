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

#ifndef CONTROL_VALIDATOR__CONTROL_VALIDATOR_HPP_
#define CONTROL_VALIDATOR__CONTROL_VALIDATOR_HPP_

#include "control_validator/debug_marker.hpp"
#include "control_validator/msg/control_validator_status.hpp"
#include "vehicle_info_util/vehicle_info_util.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <memory>
#include <string>

namespace control_validator
{
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using control_validator::msg::ControlValidatorStatus;
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

  void onReferenceTrajectory(const Trajectory::ConstSharedPtr msg);
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

  rclcpp::Subscription<Odometry>::SharedPtr sub_kinematics_;
  rclcpp::Subscription<Trajectory>::SharedPtr sub_reference_traj_;
  rclcpp::Subscription<Trajectory>::SharedPtr sub_predicted_traj_;
  rclcpp::Publisher<ControlValidatorStatus>::SharedPtr pub_status_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;

  // system parameters
  bool publish_diag_ = true;
  int diag_error_count_threshold_ = 0;
  bool display_on_terminal_ = true;

  Updater diag_updater_{this};

  ControlValidatorStatus validation_status_;
  ValidationParams validation_params_;  // for thresholds

  vehicle_info_util::VehicleInfo vehicle_info_;

  bool isAllValid(const ControlValidatorStatus & status);

  Trajectory::ConstSharedPtr current_reference_trajectory_;
  Trajectory::ConstSharedPtr current_predicted_trajectory_;

  Odometry::ConstSharedPtr current_kinematics_;

  std::shared_ptr<ControlValidatorDebugMarkerPublisher> debug_pose_publisher_;
};
}  // namespace control_validator

#endif  // CONTROL_VALIDATOR__CONTROL_VALIDATOR_HPP_
