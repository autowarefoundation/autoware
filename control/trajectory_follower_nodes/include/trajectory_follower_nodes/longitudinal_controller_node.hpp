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

#ifndef TRAJECTORY_FOLLOWER_NODES__LONGITUDINAL_CONTROLLER_NODE_HPP_
#define TRAJECTORY_FOLLOWER_NODES__LONGITUDINAL_CONTROLLER_NODE_HPP_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "autoware_auto_control_msgs/msg/longitudinal_command.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "autoware_auto_vehicle_msgs/msg/vehicle_odometry.hpp"
#include "autoware_auto_system_msgs/msg/float32_multi_array_diagnostic.hpp"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "motion_common/motion_common.hpp"
#include "motion_common/trajectory_common.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "trajectory_follower/debug_values.hpp"
#include "trajectory_follower/longitudinal_controller_utils.hpp"
#include "trajectory_follower/lowpass_filter.hpp"
#include "trajectory_follower/pid.hpp"
#include "trajectory_follower/smooth_stop.hpp"
#include "vehicle_info_util/vehicle_info_util.hpp"

namespace autoware
{
namespace motion
{
namespace control
{
namespace trajectory_follower_nodes
{
using autoware::common::types::float64_t;
using autoware::common::types::bool8_t;
namespace trajectory_follower = ::autoware::motion::control::trajectory_follower;
namespace motion_common = ::autoware::motion::motion_common;

/// \class LongitudinalController
/// \brief The node class used for generating longitudinal control commands (velocity/acceleration)
class TRAJECTORY_FOLLOWER_PUBLIC LongitudinalController : public rclcpp::Node
{
public:
  explicit LongitudinalController(const rclcpp::NodeOptions & node_options);

private:
  struct Motion
  {
    float64_t vel{0.0};
    float64_t acc{0.0};
  };

  enum class Shift { Forward = 0, Reverse };

  struct ControlData
  {
    bool8_t is_far_from_trajectory{false};
    size_t nearest_idx{0};  // nearest_idx = 0 when nearest_idx is not found with findNearestIdx
    Motion current_motion{};
    Shift shift{Shift::Forward};  // shift is used only to calculate the sign of pitch compensation
    float64_t stop_dist{0.0};  // signed distance that is positive when car is before the stopline
    float64_t slope_angle{0.0};
    float64_t dt{0.0};
  };

  // ros variables
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
    m_sub_current_velocity;
  rclcpp::Subscription<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr m_sub_trajectory;
  rclcpp::Publisher<autoware_auto_control_msgs::msg::LongitudinalCommand>::SharedPtr m_pub_control_cmd;
  rclcpp::Publisher<autoware_auto_system_msgs::msg::Float32MultiArrayDiagnostic>::SharedPtr m_pub_slope;
  rclcpp::Publisher<autoware_auto_system_msgs::msg::Float32MultiArrayDiagnostic>::SharedPtr m_pub_debug;
  rclcpp::TimerBase::SharedPtr m_timer_control;

  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr m_tf_sub;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr m_tf_static_sub;
  tf2::BufferCore m_tf_buffer{tf2::BUFFER_CORE_DEFAULT_CACHE_TIME};
  tf2_ros::TransformListener m_tf_listener{m_tf_buffer};

  OnSetParametersCallbackHandle::SharedPtr m_set_param_res;
  rcl_interfaces::msg::SetParametersResult paramCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  // pointers for ros topic
  std::shared_ptr<nav_msgs::msg::Odometry> m_current_velocity_ptr{nullptr};
  std::shared_ptr<nav_msgs::msg::Odometry> m_prev_velocity_ptr{nullptr};
  std::shared_ptr<autoware_auto_planning_msgs::msg::Trajectory> m_trajectory_ptr{nullptr};

  // vehicle info
  float64_t m_wheel_base;

  // control state
  enum class ControlState { DRIVE = 0, STOPPING, STOPPED, EMERGENCY };
  ControlState m_control_state{ControlState::STOPPED};

  // timer callback
  float64_t m_control_rate;

  // delay compensation
  float64_t m_delay_compensation_time;

  // enable flags
  bool8_t m_enable_smooth_stop;
  bool8_t m_enable_overshoot_emergency;
  bool8_t m_enable_slope_compensation;

  // smooth stop transition
  struct StateTransitionParams
  {
    // drive
    float64_t drive_state_stop_dist;
    float64_t drive_state_offset_stop_dist;
    // stopping
    float64_t stopping_state_stop_dist;
    // stop
    float64_t stopped_state_entry_vel;
    float64_t stopped_state_entry_acc;
    // emergency
    float64_t emergency_state_overshoot_stop_dist;
    float64_t emergency_state_traj_trans_dev;
    float64_t emergency_state_traj_rot_dev;
  };
  StateTransitionParams m_state_transition_params;

  // drive
  trajectory_follower::PIDController m_pid_vel;
  std::shared_ptr<trajectory_follower::LowpassFilter1d> m_lpf_vel_error{nullptr};
  float64_t m_current_vel_threshold_pid_integrate;
  bool8_t m_enable_brake_keeping_before_stop;
  float64_t m_brake_keeping_acc;

  // smooth stop
  trajectory_follower::SmoothStop m_smooth_stop;

  // stop
  struct StoppedStateParams
  {
    float64_t vel;
    float64_t acc;
    float64_t jerk;
  };
  StoppedStateParams m_stopped_state_params;

  // emergency
  struct EmergencyStateParams
  {
    float64_t vel;
    float64_t acc;
    float64_t jerk;
  };
  EmergencyStateParams m_emergency_state_params;

  // acceleration limit
  float64_t m_max_acc;
  float64_t m_min_acc;

  // jerk limit
  float64_t m_max_jerk;
  float64_t m_min_jerk;

  // slope compensation
  bool8_t m_use_traj_for_pitch;
  std::shared_ptr<trajectory_follower::LowpassFilter1d> m_lpf_pitch{nullptr};
  float64_t m_max_pitch_rad;
  float64_t m_min_pitch_rad;

  // 1st order lowpass filter for acceleration
  std::shared_ptr<trajectory_follower::LowpassFilter1d> m_lpf_acc{nullptr};

  // buffer of send command
  std::vector<autoware_auto_control_msgs::msg::LongitudinalCommand> m_ctrl_cmd_vec;

  // for calculating dt
  std::shared_ptr<rclcpp::Time> m_prev_control_time{nullptr};

  // shift mode
  Shift m_prev_shift{Shift::Forward};

  // diff limit
  Motion m_prev_ctrl_cmd{};      // with slope compensation
  Motion m_prev_raw_ctrl_cmd{};  // without slope compensation
  std::vector<std::pair<rclcpp::Time, float64_t>> m_vel_hist;

  // debug values
  trajectory_follower::DebugValues m_debug_values;

  std::shared_ptr<rclcpp::Time> m_last_running_time{std::make_shared<rclcpp::Time>(this->now())};

  /**
   * @brief set current and previous velocity with received message
   * @param [in] msg current state message
   */
  void callbackCurrentVelocity(
    const nav_msgs::msg::Odometry::ConstSharedPtr msg);

  /**
   * @brief set reference trajectory with received message
   * @param [in] msg trajectory message
   */
  void callbackTrajectory(const autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr msg);

  /**
   * @brief compute control command, and publish periodically
   */
  void callbackTimerControl();

  /**
   * @brief calculate data for controllers whose type is ControlData
   * @param [in] current_pose current ego pose
   */
  ControlData getControlData(const geometry_msgs::msg::Pose & current_pose);

  /**
   * @brief calculate control command in emergency state
   * @param [in] dt time between previous and current one
   */
  Motion calcEmergencyCtrlCmd(const float64_t dt) const;

  /**
   * @brief update control state according to the current situation
   * @param [in] current_control_state current control state
   * @param [in] control_data control data
   */
  ControlState updateControlState(
    const ControlState current_control_state, const ControlData & control_data);

  /**
   * @brief calculate control command based on the current control state
   * @param [in] current_control_state current control state
   * @param [in] current_pose current ego pose
   * @param [in] control_data control data
   */
  Motion calcCtrlCmd(
    const ControlState & current_control_state, const geometry_msgs::msg::Pose & current_pose,
    const ControlData & control_data);

  /**
   * @brief publish control command
   * @param [in] ctrl_cmd calculated control command to control velocity
   * @param [in] current_vel current velocity of the vehicle
   */
  void publishCtrlCmd(const Motion & ctrl_cmd, const float64_t current_vel);

  /**
   * @brief publish debug data
   * @param [in] ctrl_cmd calculated control command to control velocity
   * @param [in] control_data data for control calculation
   */
  void publishDebugData(const Motion & ctrl_cmd, const ControlData & control_data);

  /**
   * @brief calculate time between current and previous one
   */
  float64_t getDt();

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
   * @brief filter acceleration command with limitation of acceleration and jerk, and slope compensation
   * @param [in] raw_acc acceleration before filtered
   * @param [in] control_data data for control calculation
   */
  float64_t calcFilteredAcc(const float64_t raw_acc, const ControlData & control_data);

  /**
   * @brief store acceleration command before slope compensation
   * @param [in] accel command before slope compensation
   */
  void storeAccelCmd(const float64_t accel);

  /**
   * @brief add acceleration to compensate for slope
   * @param [in] acc acceleration before slope compensation
   * @param [in] pitch pitch angle (upward is negative)
   * @param [in] shift direction that vehicle move (forward or backward)
   */
  float64_t applySlopeCompensation(
    const float64_t acc, const float64_t pitch,
    const Shift shift) const;

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
    const autoware_auto_planning_msgs::msg::Trajectory & traj, const geometry_msgs::msg::Point & point,
    const size_t nearest_idx) const;

  /**
   * @brief calculate predicted velocity after time delay based on past control commands
   * @param [in] current_motion current velocity and acceleration of the vehicle
   * @param [in] delay_compensation_time predicted time delay
   */
  float64_t predictedVelocityInTargetPoint(
    const Motion current_motion, const float64_t delay_compensation_time) const;

  /**
   * @brief calculate velocity feedback with feed forward and pid controller
   * @param [in] target_motion reference velocity and acceleration. This acceleration will be used as feed forward.
   * @param [in] dt time step to use
   * @param [in] current_vel current velocity of the vehicle
   */
  float64_t applyVelocityFeedback(
    const Motion target_motion, const float64_t dt, const float64_t current_vel);

  /**
   * @brief update variables for debugging about pitch
   * @param [in] pitch current pitch of the vehicle (filtered)
   * @param [in] traj_pitch current trajectory pitch
   * @param [in] raw_pitch current raw pitch of the vehicle (unfiltered)
   */
  void updatePitchDebugValues(
    const float64_t pitch, const float64_t traj_pitch,
    const float64_t raw_pitch);

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
}  // namespace trajectory_follower_nodes
}  // namespace control
}  // namespace motion
}  // namespace autoware

#endif  // TRAJECTORY_FOLLOWER_NODES__LONGITUDINAL_CONTROLLER_NODE_HPP_
