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

#ifndef AUTOWARE__PID_LONGITUDINAL_CONTROLLER__PID_LONGITUDINAL_CONTROLLER_HPP_
#define AUTOWARE__PID_LONGITUDINAL_CONTROLLER__PID_LONGITUDINAL_CONTROLLER_HPP_

#include "autoware/pid_longitudinal_controller/debug_values.hpp"
#include "autoware/pid_longitudinal_controller/longitudinal_controller_utils.hpp"
#include "autoware/pid_longitudinal_controller/lowpass_filter.hpp"
#include "autoware/pid_longitudinal_controller/pid.hpp"
#include "autoware/pid_longitudinal_controller/smooth_stop.hpp"
#include "autoware/trajectory_follower_base/longitudinal_controller_base.hpp"
#include "autoware/universe_utils/ros/marker_helper.hpp"
#include "autoware_vehicle_info_utils/vehicle_info_utils.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "autoware_adapi_v1_msgs/msg/operation_mode_state.hpp"
#include "autoware_control_msgs/msg/longitudinal.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tier4_debug_msgs/msg/float32_multi_array_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::motion::control::pid_longitudinal_controller
{
using autoware::universe_utils::createDefaultMarker;
using autoware::universe_utils::createMarkerColor;
using autoware::universe_utils::createMarkerScale;
using autoware_adapi_v1_msgs::msg::OperationModeState;
using visualization_msgs::msg::Marker;

namespace trajectory_follower = ::autoware::motion::control::trajectory_follower;

/// \class PidLongitudinalController
/// \brief The node class used for generating longitudinal control commands (velocity/acceleration)
class PidLongitudinalController : public trajectory_follower::LongitudinalControllerBase
{
public:
  /// \param node Reference to the node used only for the component and parameter initialization.
  explicit PidLongitudinalController(
    rclcpp::Node & node, std::shared_ptr<diagnostic_updater::Updater> diag_updater);

private:
  struct Motion
  {
    double vel{0.0};
    double acc{0.0};
  };
  struct StateAfterDelay
  {
    StateAfterDelay(const double velocity, const double acceleration, const double distance)
    : vel(velocity), acc(acceleration), running_distance(distance)
    {
    }
    double vel{0.0};
    double acc{0.0};
    double running_distance{0.0};
  };
  enum class Shift { Forward = 0, Reverse };

  struct ControlData
  {
    bool is_far_from_trajectory{false};
    autoware_planning_msgs::msg::Trajectory interpolated_traj{};
    size_t nearest_idx{0};  // nearest_idx = 0 when nearest_idx is not found with findNearestIdx
    size_t target_idx{0};
    StateAfterDelay state_after_delay{0.0, 0.0, 0.0};
    Motion current_motion{};
    Shift shift{Shift::Forward};  // shift is used only to calculate the sign of pitch compensation
    double stop_dist{0.0};  // signed distance that is positive when car is before the stopline
    double slope_angle{0.0};
    double dt{0.0};
  };
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_;
  // ros variables
  rclcpp::Publisher<tier4_debug_msgs::msg::Float32MultiArrayStamped>::SharedPtr m_pub_slope;
  rclcpp::Publisher<tier4_debug_msgs::msg::Float32MultiArrayStamped>::SharedPtr m_pub_debug;
  rclcpp::Publisher<Marker>::SharedPtr m_pub_stop_reason_marker;

  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr m_set_param_res;
  rcl_interfaces::msg::SetParametersResult paramCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  // pointers for ros topic
  nav_msgs::msg::Odometry m_current_kinematic_state;
  geometry_msgs::msg::AccelWithCovarianceStamped m_current_accel;
  autoware_planning_msgs::msg::Trajectory m_trajectory;
  OperationModeState m_current_operation_mode;

  // vehicle info
  double m_wheel_base{0.0};
  double m_vehicle_width{0.0};
  double m_front_overhang{0.0};
  bool m_prev_vehicle_is_under_control{false};
  std::shared_ptr<rclcpp::Time> m_under_control_starting_time{nullptr};

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
  PIDController m_pid_vel;
  std::shared_ptr<LowpassFilter1d> m_lpf_vel_error{nullptr};
  bool m_enable_integration_at_low_speed;
  double m_current_vel_threshold_pid_integrate;
  double m_time_threshold_before_pid_integrate;
  bool m_enable_brake_keeping_before_stop;
  double m_brake_keeping_acc;

  // smooth stop
  SmoothStop m_smooth_stop;

  // stop
  struct StoppedStateParams
  {
    double vel;
    double acc;
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

  // acc feedback
  double m_acc_feedback_gain;
  std::shared_ptr<LowpassFilter1d> m_lpf_acc_error{nullptr};

  // acceleration limit
  double m_max_acc;
  double m_min_acc;

  // jerk limit
  double m_max_jerk;
  double m_min_jerk;
  double m_max_acc_cmd_diff;

  // slope compensation
  enum class SlopeSource { RAW_PITCH = 0, TRAJECTORY_PITCH, TRAJECTORY_ADAPTIVE };
  SlopeSource m_slope_source{SlopeSource::RAW_PITCH};
  double m_adaptive_trajectory_velocity_th;
  std::shared_ptr<LowpassFilter1d> m_lpf_pitch{nullptr};
  double m_max_pitch_rad;
  double m_min_pitch_rad;

  // ego nearest index search
  double m_ego_nearest_dist_threshold;
  double m_ego_nearest_yaw_threshold;

  // buffer of send command
  std::vector<autoware_control_msgs::msg::Longitudinal> m_ctrl_cmd_vec;

  // for calculating dt
  std::shared_ptr<rclcpp::Time> m_prev_control_time{nullptr};

  // shift mode
  Shift m_prev_shift{Shift::Forward};

  // diff limit
  Motion m_prev_ctrl_cmd{};      // with slope compensation
  Motion m_prev_raw_ctrl_cmd{};  // without slope compensation
  std::vector<std::pair<rclcpp::Time, double>> m_vel_hist;

  // debug values
  DebugValues m_debug_values;

  std::optional<bool> m_prev_keep_stopped_condition{std::nullopt};

  std::shared_ptr<rclcpp::Time> m_last_running_time{std::make_shared<rclcpp::Time>(clock_->now())};

  // Diagnostic
  std::shared_ptr<diagnostic_updater::Updater>
    diag_updater_{};  // Diagnostic updater for publishing diagnostic data.
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
  void setKinematicState(const nav_msgs::msg::Odometry & msg);

  /**
   * @brief set current acceleration with received message
   * @param [in] msg trajectory message
   */
  void setCurrentAcceleration(const geometry_msgs::msg::AccelWithCovarianceStamped & msg);

  /**
   * @brief set current operation mode with received message
   * @param [in] msg operation mode report message
   */
  void setCurrentOperationMode(const OperationModeState & msg);

  /**
   * @brief set reference trajectory with received message
   * @param [in] msg trajectory message
   */
  void setTrajectory(const autoware_planning_msgs::msg::Trajectory & msg);

  bool isReady(const trajectory_follower::InputData & input_data) override;

  /**
   * @brief compute control command, and publish periodically
   */
  trajectory_follower::LongitudinalOutput run(
    trajectory_follower::InputData const & input_data) override;

  /**
   * @brief calculate data for controllers whose type is ControlData
   * @param [in] current_pose current ego pose
   */
  ControlData getControlData(const geometry_msgs::msg::Pose & current_pose);

  /**
   * @brief calculate control command in emergency state
   * @param [in] dt time between previous and current one
   */
  Motion calcEmergencyCtrlCmd(const double dt);

  /**
   * @brief update control state according to the current situation
   * @param [in] control_data control data
   */
  void updateControlState(const ControlData & control_data);

  /**
   * @brief calculate control command based on the current control state
   * @param [in] control_data control data
   */
  Motion calcCtrlCmd(const ControlData & control_data);

  /**
   * @brief publish control command
   * @param [in] ctrl_cmd calculated control command to control velocity
   * @param [in] current_vel current velocity of the vehicle
   */
  autoware_control_msgs::msg::Longitudinal createCtrlCmdMsg(
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
   * @param [in] control_data data for control calculation
   */
  enum Shift getCurrentShift(const ControlData & control_data) const;

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
    const ControlData & control_data, const Motion & target_motion, const size_t nearest_idx) const;

  /**
   * @brief interpolate trajectory point that is nearest to vehicle
   * @param [in] traj reference trajectory
   * @param [in] point vehicle position
   * @param [in] nearest_idx index of the trajectory point nearest to the vehicle position
   */
  std::pair<autoware_planning_msgs::msg::TrajectoryPoint, size_t>
  calcInterpolatedTrajPointAndSegment(
    const autoware_planning_msgs::msg::Trajectory & traj,
    const geometry_msgs::msg::Pose & pose) const;

  /**
   * @brief calculate predicted velocity after time delay based on past control commands
   * @param [in] current_motion current velocity and acceleration of the vehicle
   * @param [in] delay_compensation_time predicted time delay
   */
  StateAfterDelay predictedStateAfterDelay(
    const Motion current_motion, const double delay_compensation_time) const;

  /**
   * @brief calculate velocity feedback with feed forward and pid controller
   * @param [in] control_data data for control calculation
   */
  double applyVelocityFeedback(const ControlData & control_data);

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
   * @param [in] control_data data for control calculation
   */
  void updateDebugVelAcc(const ControlData & control_data);

  double getTimeUnderControl();
};
}  // namespace autoware::motion::control::pid_longitudinal_controller

#endif  // AUTOWARE__PID_LONGITUDINAL_CONTROLLER__PID_LONGITUDINAL_CONTROLLER_HPP_
