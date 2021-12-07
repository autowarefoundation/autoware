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

#ifndef CONTROL_PERFORMANCE_ANALYSIS__CONTROL_PERFORMANCE_ANALYSIS_CORE_HPP_
#define CONTROL_PERFORMANCE_ANALYSIS__CONTROL_PERFORMANCE_ANALYSIS_CORE_HPP_

#include "control_performance_analysis/control_performance_analysis_utils.hpp"

#include <eigen3/Eigen/Core>

#include <autoware_auto_control_msgs/msg/ackermann_lateral_command.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <memory>
#include <utility>
#include <vector>

namespace control_performance_analysis
{
using autoware_auto_control_msgs::msg::AckermannLateralCommand;
using autoware_auto_planning_msgs::msg::Trajectory;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseArray;
using geometry_msgs::msg::Twist;

struct TargetPerformanceMsgVars
{
  double lateral_error;
  double heading_error;
  double control_effort_energy;
  double error_energy;
  double value_approximation;
  double curvature_estimate;
  double curvature_estimate_pp;
  double lateral_error_velocity;
  double lateral_error_acceleration;
};

class ControlPerformanceAnalysisCore
{
public:
  // See https://eigen.tuxfamily.org/dox/group__TopicStructHavingEigenMembers.html
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ControlPerformanceAnalysisCore();
  ControlPerformanceAnalysisCore(double wheelbase, double curvature_interval_length);

  // Setters
  void setCurrentPose(const Pose & msg);
  void setCurrentWaypoints(const Trajectory & trajectory);
  void setCurrentVelocities(const Twist & twist_msg);
  void setCurrentControlValue(const AckermannLateralCommand & msg);
  void setInterpolatedPose(Pose & interpolated_pose);

  void findCurveRefIdx();
  std::pair<bool, int32_t> findClosestPrevWayPointIdx_path_direction();
  double estimateCurvature();
  double estimatePurePursuitCurvature();

  // Getters
  bool isDataReady() const;
  std::pair<bool, TargetPerformanceMsgVars> getPerformanceVars();
  Pose getPrevWPPose() const;
  std::pair<bool, Pose> calculateClosestPose();

private:
  double wheelbase_;
  double curvature_interval_length_;

  // Variables Received Outside
  std::shared_ptr<PoseArray> current_waypoints_ptr_;
  std::shared_ptr<Pose> current_vec_pose_ptr_;
  std::shared_ptr<std::vector<double>> current_velocities_ptr_;  // [Vx, Heading rate]
  std::shared_ptr<AckermannLateralCommand> current_control_ptr_;

  // Variables computed
  std::unique_ptr<int32_t> idx_prev_wp_;       // the waypoint index, vehicle
  std::unique_ptr<int32_t> idx_curve_ref_wp_;  // index of waypoint corresponds to front axle center
  std::unique_ptr<int32_t> idx_next_wp_;       //  the next waypoint index, vehicle heading to
  std::unique_ptr<TargetPerformanceMsgVars> prev_target_vars_{};
  std::shared_ptr<Pose> interpolated_pose_ptr_;
  // V = xPx' ; Value function from DARE Lyap matrix P
  Eigen::Matrix2d const lyap_P_ = (Eigen::MatrixXd(2, 2) << 2.342, 8.60, 8.60, 64.29).finished();
  double const contR{10.0};  // Control weight in LQR

  rclcpp::Logger logger_{rclcpp::get_logger("control_performance_analysis")};
  rclcpp::Clock clock_{RCL_ROS_TIME};
};
}  // namespace control_performance_analysis

#endif  // CONTROL_PERFORMANCE_ANALYSIS__CONTROL_PERFORMANCE_ANALYSIS_CORE_HPP_
