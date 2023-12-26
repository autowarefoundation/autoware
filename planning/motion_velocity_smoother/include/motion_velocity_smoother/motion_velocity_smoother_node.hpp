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

#ifndef MOTION_VELOCITY_SMOOTHER__MOTION_VELOCITY_SMOOTHER_NODE_HPP_
#define MOTION_VELOCITY_SMOOTHER__MOTION_VELOCITY_SMOOTHER_NODE_HPP_

#include "motion_utils/trajectory/conversion.hpp"
#include "motion_utils/trajectory/trajectory.hpp"
#include "motion_velocity_smoother/resample.hpp"
#include "motion_velocity_smoother/smoother/analytical_jerk_constrained_smoother/analytical_jerk_constrained_smoother.hpp"
#include "motion_velocity_smoother/smoother/jerk_filtered_smoother.hpp"
#include "motion_velocity_smoother/smoother/l2_pseudo_jerk_smoother.hpp"
#include "motion_velocity_smoother/smoother/linf_pseudo_jerk_smoother.hpp"
#include "motion_velocity_smoother/smoother/smoother_base.hpp"
#include "motion_velocity_smoother/trajectory_utils.hpp"
#include "osqp_interface/osqp_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.h"
#include "tf2_ros/transform_listener.h"
#include "tier4_autoware_utils/geometry/geometry.hpp"
#include "tier4_autoware_utils/math/unit_conversion.hpp"
#include "tier4_autoware_utils/ros/logger_level_configure.hpp"
#include "tier4_autoware_utils/ros/self_pose_listener.hpp"
#include "tier4_autoware_utils/system/stop_watch.hpp"

#include "autoware_adapi_v1_msgs/msg/operation_mode_state.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory_point.hpp"
#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tier4_debug_msgs/msg/float32_stamped.hpp"         // temporary
#include "tier4_planning_msgs/msg/stop_speed_exceeded.hpp"  // temporary
#include "tier4_planning_msgs/msg/velocity_limit.hpp"       // temporary
#include "visualization_msgs/msg/marker_array.hpp"

#include <iostream>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace motion_velocity_smoother
{
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;
using autoware_adapi_v1_msgs::msg::OperationModeState;
using geometry_msgs::msg::AccelWithCovarianceStamped;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseStamped;
using nav_msgs::msg::Odometry;
using tier4_debug_msgs::msg::Float32Stamped;        // temporary
using tier4_planning_msgs::msg::StopSpeedExceeded;  // temporary
using tier4_planning_msgs::msg::VelocityLimit;      // temporary
using visualization_msgs::msg::MarkerArray;

struct Motion
{
  double vel = 0.0;
  double acc = 0.0;

  Motion() {}
  Motion(const double v, const double a) : vel(v), acc(a) {}
};

class MotionVelocitySmootherNode : public rclcpp::Node
{
public:
  explicit MotionVelocitySmootherNode(const rclcpp::NodeOptions & node_options);

private:
  rclcpp::Publisher<Trajectory>::SharedPtr pub_trajectory_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_virtual_wall_;
  rclcpp::Publisher<StopSpeedExceeded>::SharedPtr pub_over_stop_velocity_;
  rclcpp::Subscription<Odometry>::SharedPtr sub_current_odometry_;
  rclcpp::Subscription<AccelWithCovarianceStamped>::SharedPtr sub_current_acceleration_;
  rclcpp::Subscription<Trajectory>::SharedPtr sub_current_trajectory_;
  rclcpp::Subscription<VelocityLimit>::SharedPtr sub_external_velocity_limit_;
  rclcpp::Subscription<OperationModeState>::SharedPtr sub_operation_mode_;

  Odometry::ConstSharedPtr current_odometry_ptr_;  // current odometry
  AccelWithCovarianceStamped::ConstSharedPtr current_acceleration_ptr_;
  VelocityLimit::ConstSharedPtr external_velocity_limit_ptr_{
    nullptr};                                     // external velocity limit message
  Trajectory::ConstSharedPtr base_traj_raw_ptr_;  // current base_waypoints
  double max_velocity_with_deceleration_;         // maximum velocity with deceleration
                                                  // for external velocity limit
  double wheelbase_;                              // wheelbase
  double base_link2front_;                        // base_link to front

  TrajectoryPoints prev_output_;  // previously published trajectory

  // previous trajectory point closest to ego vehicle
  boost::optional<TrajectoryPoint> prev_closest_point_{};
  boost::optional<TrajectoryPoint> current_closest_point_from_prev_output_{};

  bool is_reverse_;

  // check if the vehicle is under control of the planning module
  OperationModeState operation_mode_;

  enum class AlgorithmType {
    INVALID = 0,
    JERK_FILTERED = 1,
    L2 = 2,
    LINF = 3,
    ANALYTICAL = 4,
  };

  enum class InitializeType {
    EGO_VELOCITY = 0,
    LARGE_DEVIATION_REPLAN = 1,
    ENGAGING = 2,
    NORMAL = 3,
  };

  struct Param
  {
    bool enable_lateral_acc_limit;
    bool enable_steering_rate_limit;

    double max_velocity;                              // max velocity [m/s]
    double margin_to_insert_external_velocity_limit;  // for external velocity limit [m]
    double replan_vel_deviation;                      // if speed error exceeds this [m/s],
                                                      // replan from current velocity
    double engage_velocity;                           // use this speed when start moving [m/s]
    double engage_acceleration;           // use this acceleration when start moving [m/ss]
    double engage_exit_ratio;             // exit engage sequence
                                          // when the speed exceeds ratio x engage_vel.
    double stopping_velocity;             // change target velocity to this value before v=0 point.
    double stopping_distance;             // distance for the stopping_velocity
    double extract_ahead_dist;            // forward waypoints distance from current position [m]
    double extract_behind_dist;           // backward waypoints distance from current position [m]
    double stop_dist_to_prohibit_engage;  // prevent to move toward close stop point
    double ego_nearest_dist_threshold;    // for ego's closest index calculation
    double ego_nearest_yaw_threshold;     // for ego's closest index calculation

    resampling::ResampleParam post_resample_param;
    AlgorithmType algorithm_type;  // Option : JerkFiltered, Linf, L2

    bool plan_from_ego_speed_on_manual_mode = true;
  } node_param_{};

  struct ExternalVelocityLimit
  {
    double velocity{0.0};  // current external_velocity_limit
    double dist{0.0};      // distance to set external velocity limit
    std::string sender{""};
  };
  ExternalVelocityLimit
    external_velocity_limit_;  // velocity and distance constraint  of external velocity limit

  std::shared_ptr<SmootherBase> smoother_;

  bool publish_debug_trajs_;  // publish planned trajectories

  double over_stop_velocity_warn_thr_;  // threshold to publish over velocity warn

  mutable rclcpp::Clock::SharedPtr clock_;

  void setupSmoother(const double wheelbase);

  // parameter update
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  rcl_interfaces::msg::SetParametersResult onParameter(
    const std::vector<rclcpp::Parameter> & parameters);

  // topic callback
  void onCurrentOdometry(const Odometry::ConstSharedPtr msg);

  void onCurrentTrajectory(const Trajectory::ConstSharedPtr msg);

  void onExternalVelocityLimit(const VelocityLimit::ConstSharedPtr msg);

  void calcExternalVelocityLimit();

  // publish methods
  void publishTrajectory(const TrajectoryPoints & traj) const;

  void publishStopDistance(const TrajectoryPoints & trajectory) const;

  // non-const methods
  void publishClosestState(const TrajectoryPoints & trajectory);

  void updatePrevValues(const TrajectoryPoints & final_result);

  // const methods
  bool checkData() const;

  void updateDataForExternalVelocityLimit();

  AlgorithmType getAlgorithmType(const std::string & algorithm_name) const;

  TrajectoryPoints calcTrajectoryVelocity(const TrajectoryPoints & traj_input) const;

  bool smoothVelocity(
    const TrajectoryPoints & input, const size_t input_closest,
    TrajectoryPoints & traj_smoothed) const;

  std::pair<Motion, InitializeType> calcInitialMotion(
    const TrajectoryPoints & input_traj, const size_t input_closest) const;

  void applyExternalVelocityLimit(TrajectoryPoints & traj) const;

  void insertBehindVelocity(
    const size_t output_closest, const InitializeType type, TrajectoryPoints & output) const;

  void applyStopApproachingVelocity(TrajectoryPoints & traj) const;

  void overwriteStopPoint(const TrajectoryPoints & input, TrajectoryPoints & output) const;

  double calcTravelDistance() const;

  bool isEngageStatus(const double target_vel) const;

  void publishDebugTrajectories(const std::vector<TrajectoryPoints> & debug_trajectories) const;

  void publishClosestVelocity(
    const TrajectoryPoints & trajectory, const Pose & current_pose,
    const rclcpp::Publisher<Float32Stamped>::SharedPtr pub) const;

  Trajectory toTrajectoryMsg(
    const TrajectoryPoints & points, const std_msgs::msg::Header * header = nullptr) const;

  TrajectoryPoint calcProjectedTrajectoryPoint(
    const TrajectoryPoints & trajectory, const Pose & pose) const;
  TrajectoryPoint calcProjectedTrajectoryPointFromEgo(const TrajectoryPoints & trajectory) const;

  // parameter handling
  void initCommonParam();

  // debug
  tier4_autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch_;
  std::shared_ptr<rclcpp::Time> prev_time_;
  double prev_acc_;
  rclcpp::Publisher<Float32Stamped>::SharedPtr pub_dist_to_stopline_;
  rclcpp::Publisher<Trajectory>::SharedPtr pub_trajectory_raw_;
  rclcpp::Publisher<VelocityLimit>::SharedPtr pub_velocity_limit_;
  rclcpp::Publisher<Trajectory>::SharedPtr pub_trajectory_vel_lim_;
  rclcpp::Publisher<Trajectory>::SharedPtr pub_trajectory_latacc_filtered_;
  rclcpp::Publisher<Trajectory>::SharedPtr pub_trajectory_steering_rate_limited_;
  rclcpp::Publisher<Trajectory>::SharedPtr pub_trajectory_resampled_;
  rclcpp::Publisher<Float32Stamped>::SharedPtr debug_closest_velocity_;
  rclcpp::Publisher<Float32Stamped>::SharedPtr debug_closest_acc_;
  rclcpp::Publisher<Float32Stamped>::SharedPtr debug_closest_jerk_;
  rclcpp::Publisher<Float32Stamped>::SharedPtr debug_calculation_time_;
  rclcpp::Publisher<Float32Stamped>::SharedPtr debug_closest_max_velocity_;

  // For Jerk Filtered Algorithm Debug
  rclcpp::Publisher<Trajectory>::SharedPtr pub_forward_filtered_trajectory_;
  rclcpp::Publisher<Trajectory>::SharedPtr pub_backward_filtered_trajectory_;
  rclcpp::Publisher<Trajectory>::SharedPtr pub_merged_filtered_trajectory_;
  rclcpp::Publisher<Float32Stamped>::SharedPtr pub_closest_merged_velocity_;

  // helper functions
  size_t findNearestIndexFromEgo(const TrajectoryPoints & points) const;
  bool isReverse(const TrajectoryPoints & points) const;
  void flipVelocity(TrajectoryPoints & points) const;
  void publishStopWatchTime();

  std::unique_ptr<tier4_autoware_utils::LoggerLevelConfigure> logger_configure_;
};
}  // namespace motion_velocity_smoother

#endif  // MOTION_VELOCITY_SMOOTHER__MOTION_VELOCITY_SMOOTHER_NODE_HPP_
