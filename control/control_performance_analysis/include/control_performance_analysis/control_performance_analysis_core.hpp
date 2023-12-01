// Copyright 2021 - 2022 Tier IV, Inc., Leo Drive Teknoloji A.Ş.
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

#ifndef CONTROL_PERFORMANCE_ANALYSIS__CONTROL_PERFORMANCE_ANALYSIS_CORE_HPP_
#define CONTROL_PERFORMANCE_ANALYSIS__CONTROL_PERFORMANCE_ANALYSIS_CORE_HPP_

#include "control_performance_analysis/control_performance_analysis_utils.hpp"
#include "control_performance_analysis/msg/driving_monitor_stamped.hpp"
#include "control_performance_analysis/msg/error_stamped.hpp"
#include "control_performance_analysis/msg/float_stamped.hpp"
#include "motion_utils/trajectory/trajectory.hpp"

#include <Eigen/Core>
#include <rclcpp/time.hpp>

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <memory>
#include <optional>
#include <utility>
#include <vector>

namespace control_performance_analysis
{
using autoware_auto_control_msgs::msg::AckermannControlCommand;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_vehicle_msgs::msg::SteeringReport;
using control_performance_analysis::msg::DrivingMonitorStamped;
using control_performance_analysis::msg::Error;
using control_performance_analysis::msg::ErrorStamped;
using control_performance_analysis::msg::FloatStamped;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseArray;
using geometry_msgs::msg::Twist;
using nav_msgs::msg::Odometry;

struct Params
{
  double wheelbase_;
  double curvature_interval_length_;
  uint odom_interval_;
  double acceptable_max_distance_to_waypoint_;
  double acceptable_max_yaw_difference_rad_;
  double prevent_zero_division_value_;
  double lpf_gain_;
};

class ControlPerformanceAnalysisCore
{
public:
  // See https://eigen.tuxfamily.org/dox/group__TopicStructHavingEigenMembers.html
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ControlPerformanceAnalysisCore();
  explicit ControlPerformanceAnalysisCore(Params & p);

  // Setters
  void setCurrentPose(const Pose & msg);
  void setCurrentWaypoints(const Trajectory & trajectory);
  void setCurrentControlValue(const AckermannControlCommand & msg);
  void setInterpolatedVars(
    const Pose & interpolated_pose, const double & interpolated_velocity,
    const double & interpolated_acceleration, const double & interpolated_steering_angle);
  void setOdomHistory(const Odometry & odom);
  void setSteeringStatus(const SteeringReport & steering);

  std::optional<int32_t> findCurveRefIdx();
  std::pair<bool, int32_t> findClosestPrevWayPointIdx_path_direction();
  double estimateCurvature();
  double estimatePurePursuitCurvature();

  // Getters
  bool isDataReady() const;
  bool calculateErrorVars();
  bool calculateDrivingVars();
  Pose getPrevWPPose() const;  // It is not used!
  std::pair<bool, Pose> calculateClosestPose();

  // Output variables
  ErrorStamped error_vars;
  DrivingMonitorStamped driving_status_vars;

private:
  Params p_;

  // Variables Received Outside
  std::shared_ptr<autoware_auto_planning_msgs::msg::Trajectory> current_trajectory_ptr_;
  std::shared_ptr<Pose> current_vec_pose_ptr_;
  std::shared_ptr<std::vector<Odometry>> odom_history_ptr_;  // velocities at k-2, k-1, k, k+1
  std::shared_ptr<AckermannControlCommand> current_control_ptr_;
  std::shared_ptr<SteeringReport> current_vec_steering_msg_ptr_;

  // State holder

  std_msgs::msg::Header last_odom_header;
  std_msgs::msg::Header last_steering_report;

  // Variables computed

  std::unique_ptr<int32_t> idx_prev_wp_;  // the waypoint index, vehicle
  std::unique_ptr<int32_t> idx_next_wp_;  //  the next waypoint index, vehicle heading to
  std::unique_ptr<ErrorStamped> prev_target_vars_{};
  std::unique_ptr<DrivingMonitorStamped> prev_driving_vars_{};
  std::shared_ptr<Pose> interpolated_pose_ptr_;
  std::shared_ptr<double> interpolated_velocity_ptr_;
  std::shared_ptr<double> interpolated_acceleration_ptr_;
  std::shared_ptr<double> interpolated_steering_angle_ptr_;

  // V = xPx' ; Value function from DARE Lyap matrix P
  Eigen::Matrix2d const lyap_P_ = (Eigen::MatrixXd(2, 2) << 2.342, 8.60, 8.60, 64.29).finished();
  double const contR{10.0};  // Control weight in LQR

  rclcpp::Logger logger_{rclcpp::get_logger("control_performance_analysis")};
  rclcpp::Clock clock_{RCL_ROS_TIME};
};
}  // namespace control_performance_analysis

#endif  // CONTROL_PERFORMANCE_ANALYSIS__CONTROL_PERFORMANCE_ANALYSIS_CORE_HPP_
