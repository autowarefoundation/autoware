// Copyright 2021 Tier IV, Inc. All rights reserved.
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

#ifndef TRAJECTORY_FOLLOWER__PID_LONGITUDINAL_CONTROLLER_HPP_
#define TRAJECTORY_FOLLOWER__PID_LONGITUDINAL_CONTROLLER_HPP_

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "trajectory_follower/debug_values.hpp"
#include "trajectory_follower/longitudinal_controller_base.hpp"
#include "trajectory_follower/longitudinal_controller_utils.hpp"
#include "trajectory_follower/lowpass_filter.hpp"
#include "trajectory_follower/pid.hpp"
#include "trajectory_follower/smooth_stop.hpp"
#include "vehicle_info_util/vehicle_info_util.hpp"

#include "autoware_auto_control_msgs/msg/longitudinal_command.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "autoware_auto_vehicle_msgs/msg/vehicle_odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tier4_debug_msgs/msg/float32_multi_array_stamped.hpp"

#include <deque>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware
{
namespace motion
{
namespace control
{
namespace trajectory_follower
{

namespace trajectory_follower = ::autoware::motion::control::trajectory_follower;

/// \class PidLongitudinalController
/// \brief The node class used for generating longitudinal control commands (velocity/acceleration)
class TRAJECTORY_FOLLOWER_PUBLIC PidLongitudinalController : public LongitudinalControllerBase
{
public:
  explicit PidLongitudinalController(rclcpp::Node & node);

private:
  struct Motion
  {
    double vel{0.0};
    double acc{0.0};
  };

  enum class Shift { Forward = 0, Reverse };

  struct ControlData
  {
    bool is_far_from_trajectory{false};
    size_t nearest_idx{0};  // nearest_idx = 0 when nearest_idx is not found with findNearestIdx
    Motion current_motion{};
    Shift shift{Shift::Forward};  // shift is used only to calculate the sign of pitch compensation
    double stop_dist{0.0};  // signed distance that is positive when car is before the stopline
    double slope_angle{0.0};
    double dt{0.0};
  };
  rclcpp::Node * node_;
  // ros variables
  rclcpp::Publisher<tier4_debug_msgs::msg::Float32MultiArrayStamped>::SharedPtr m_pub_slope;
  rclcpp::Publisher<tier4_debug_msgs::msg::Float32MultiArrayStamped>::SharedPtr m_pub_debug;

  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr m_set_param_res;
  rcl_interfaces::msg::SetParametersResult paramCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  // pointers for ros topic
  nav_msgs::msg::Odometry::ConstSharedPtr m_current_kinematic_state_ptr{nullptr};
  geometry_msgs::msg::AccelWithCovarianceStamped::ConstSharedPtr m_current_accel_ptr{nullptr};
  autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr m_trajectory_ptr{nullptr};

  // vehicle info
  double m_wheel_base;

  // control state
  enum class ControlState { DRIVE = 0, STOPPING, STOPPED, EMERGENCY };
  ControlState m_control_state{ControlState::STOPPED};
  std::string toStr(const ControlState s)
  {
    if (s == ControlState::DRIVE) return "DRIVE";
    if (s == ControlState::STOPPING) return "STOPPING";
    if (s == ControlState::STOPPED) return "STOPPED";
    if (s == ControlState::EMERGENCY) return "EMERGENCY";
    return "UNDEFINED";
  };

  // control period
  double m_longitudinal_ctrl_period;

  // delay compensation
  double m_delay_compensation_time;

  // enable flags
  bool m_enable_smooth_stop;
  bool m_enable_overshoot_emergency;
  bool m_enable_slope_compensation;
  bool m_enable_large_tracking_error_emergency;
  bool m_enable_keep_stopped_until_steer_convergence;

  // smooth stop transition
  struct StateTransitionParams
  {
    // drive
    double drive_state_stop_dist;
    double drive_state_offset_stop_dist;
    // stopping
    double stopping_state_stop_dist;
    // stop
    double stopped_state_entry_duration_time;
    double stopped_state_entry_vel;
    double stopped_state_entry_acc;
    // emergency
    double emergency_state_overshoot_stop_dist;
    double emergency_state_traj_trans_dev;
    double emergency_state_traj_rot_dev;
  };
  StateTransitionParams m_state_transition_params;

  // drive
  trajectory_follower::PIDController m_pid_vel;
  std::shared_ptr<trajectory_follower::LowpassFilter1d> m_lpf_vel_error{nullptr};
  double m_current_vel_threshold_pid_integrate;
  bool m_enable_brake_keeping_before_stop;
  double m_brake_keeping_acc;

  // smooth stop
  trajectory_follower::SmoothStop m_smooth_stop;

  // stop
  struct StoppedStateParams
  {
    double vel;
    double acc;
    double jerk;
  };
  StoppedStateParams m_stopped_state_params;

  // emergency
  struct EmergencyStateParams
  {
    double vel;
    double acc;
    double jerk;
  };
  EmergencyStateParams m_emergency_state_params;

  // acceleration limit
  double m_max_acc;
  double m_min_acc;

  // jerk limit
  double m_max_jerk;
  double m_min_jerk;

  // slope compensation
  bool m_use_traj_for_pitch;
  std::shared_ptr<trajectory_follower::LowpassFilter1d> m_lpf_pitch{nullptr};
  double m_max_pitch_rad;
  double m_min_pitch_rad;

  // ego nearest index search
  double m_ego_nearest_dist_threshold;
  double m_ego_nearest_yaw_threshold;

  // buffer of send command
  std::vector<autoware_auto_control_msgs::msg::LongitudinalCommand> m_ctrl_cmd_vec;

  // for calculating dt
  std::shared_ptr<rclcpp::Time> m_prev_control_time{nullptr};

  // shift mode
  Shift m_prev_shift{Shift::Forward};

  // diff limit
  Motion m_prev_ctrl_cmd{};      // with slope compensation
  Motion m_prev_raw_ctrl_cmd{};  // without slope compensation
  std::vector<std::pair<rclcpp::Time, double>> m_vel_hist;

  // debug values
  trajectory_follower::DebugValues m_debug_values;

  std::shared_ptr<rclcpp::Time> m_last_running_time{std::make_shared<rclcpp::Time>(node_->now())};

  // Diagnostic

  diagnostic_updater::Updater diagnostic_updater_;
  struct DiagnosticData
  {
    double trans_deviation{0.0};  // translation deviation between nearest point and current_pose
    double rot_deviation{0.0};    // rotation deviation between nearest point and current_pose
  };
  DiagnosticData m_diagnostic_data;
  void setupDiagnosticUpdater();
  void checkControlState(diagnostic_updater::DiagnosticStatusWrapper & stat);

  /**
   * @brief set current and previous velocity with received message
   * @param [in] msg current state message
   */
  void setKinematicState(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

  /**
   * @brief set current acceleration with received message
   * @param [in] msg trajectory message
   */
  void setCurrentAcceleration(
    const geometry_msgs::msg::AccelWithCovarianceStamped::ConstSharedPtr msg);

  /**
   * @brief set reference trajectory with received message
   * @param [in] msg trajectory message
   */
  void setTrajectory(const autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr msg);

  /**
   * @brief compute control command, and publish periodically
   */
  boost::optional<LongitudinalOutput> run() override;

  /**
   * @brief set input data like current odometry and trajectory.
   */
  void setInputData(InputData const & input_data) override;

  /**
   * @brief calculate data for controllers whose type is ControlData
   * @param [in] current_pose current ego pose
   */
  ControlData getControlData(const geometry_msgs::msg::Pose & current_pose);

  /**
   * @brief calculate control command in emergency state
   * @param [in] dt time between previous and current one
   */
  Motion calcEmergencyCtrlCmd(const double dt) const;

  /**
   * @brief update control state according to the current situation
   * @param [in] control_data control data
   */
  void updateControlState(const ControlData & control_data);

  /**
   * @brief calculate control command based on the current control state
   * @param [in] current_pose current ego pose
   * @param [in] control_data control data
   */
  Motion calcCtrlCmd(
    const geometry_msgs::msg::Pose & current_pose, const ControlData & control_data);

  /**
   * @brief publish control command
   * @param [in] ctrl_cmd calculated control command to control velocity
   * @param [in] current_vel current velocity of the vehicle
   */
  autoware_auto_control_msgs::msg::LongitudinalCommand createCtrlCmdMsg(
    const Motion & ctrl_cmd, const double & current_vel);

  /**
   * @brief publish debug data
   * @param [in] ctrl_cmd calculated control command to control velocity
   * @param [in] control_data data for control calculation
   */
  void publishDebugData(const Motion & ctrl_cmd, const ControlData & control_data);

  /**
   * @brief calculate time between current and previous one
   */
  double getDt();

  /**
   * @brief calculate current velocity and acceleration
   */
  Motion getCurrentMotion() const;

  /**
   * @brief calculate direction (forward or backward) that vehicle moves
   * @param [in] nearest_idx nearest index on trajectory to vehicle
   */
  enum Shift getCurrentShift(const size_t nearest_idx) const;

  /**
   * @brief filter acceleration command with limitation of acceleration and jerk, and slope
   * compensation
   * @param [in] raw_acc acceleration before filtered
   * @param [in] control_data data for control calculation
   */
  double calcFilteredAcc(const double raw_acc, const ControlData & control_data);

  /**
   * @brief store acceleration command before slope compensation
   * @param [in] accel command before slope compensation
   */
  void storeAccelCmd(const double accel);

  /**
   * @brief add acceleration to compensate for slope
   * @param [in] acc acceleration before slope compensation
   * @param [in] pitch pitch angle (upward is negative)
   * @param [in] shift direction that vehicle move (forward or backward)
   */
  double applySlopeCompensation(const double acc, const double pitch, const Shift shift) const;

  /**
   * @brief keep target motion acceleration negative before stop
   * @param [in] traj reference trajectory
   * @param [in] motion delay compensated target motion
   */
  Motion keepBrakeBeforeStop(
    const autoware_auto_planning_msgs::msg::Trajectory & traj, const Motion & target_motion,
    const size_t nearest_idx) const;

  /**
   * @brief interpolate trajectory point that is nearest to vehicle
   * @param [in] traj reference trajectory
   * @param [in] point vehicle position
   * @param [in] nearest_idx index of the trajectory point nearest to the vehicle position
   */
  autoware_auto_planning_msgs::msg::TrajectoryPoint calcInterpolatedTargetValue(
    const autoware_auto_planning_msgs::msg::Trajectory & traj,
    const geometry_msgs::msg::Pose & pose) const;

  /**
   * @brief calculate predicted velocity after time delay based on past control commands
   * @param [in] current_motion current velocity and acceleration of the vehicle
   * @param [in] delay_compensation_time predicted time delay
   */
  double predictedVelocityInTargetPoint(
    const Motion current_motion, const double delay_compensation_time) const;

  /**
   * @brief calculate velocity feedback with feed forward and pid controller
   * @param [in] target_motion reference velocity and acceleration. This acceleration will be used
   * as feed forward.
   * @param [in] dt time step to use
   * @param [in] current_vel current velocity of the vehicle
   */
  double applyVelocityFeedback(
    const Motion target_motion, const double dt, const double current_vel);

  /**
   * @brief update variables for debugging about pitch
   * @param [in] pitch current pitch of the vehicle (filtered)
   * @param [in] traj_pitch current trajectory pitch
   * @param [in] raw_pitch current raw pitch of the vehicle (unfiltered)
   */
  void updatePitchDebugValues(const double pitch, const double traj_pitch, const double raw_pitch);

  /**
   * @brief update variables for velocity and acceleration
   * @param [in] ctrl_cmd latest calculated control command
   * @param [in] current_pose current pose of the vehicle
   * @param [in] control_data data for control calculation
   */
  void updateDebugVelAcc(
    const Motion & ctrl_cmd, const geometry_msgs::msg::Pose & current_pose,
    const ControlData & control_data);
};
}  // namespace trajectory_follower
}  // namespace control
}  // namespace motion
}  // namespace autoware

#endif  // TRAJECTORY_FOLLOWER__PID_LONGITUDINAL_CONTROLLER_HPP_
