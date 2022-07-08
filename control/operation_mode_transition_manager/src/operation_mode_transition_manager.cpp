// Copyright 2022 Autoware Foundation
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

#include "operation_mode_transition_manager/operation_mode_transition_manager.hpp"

#include "tier4_autoware_utils/tier4_autoware_utils.hpp"

#include <algorithm>
#include <cmath>

namespace operation_mode_transition_manager
{

using tier4_autoware_utils::calcDistance2d;
using tier4_autoware_utils::calcYawDeviation;
using tier4_autoware_utils::findNearestIndex;

OperationModeTransitionManager::OperationModeTransitionManager(const rclcpp::NodeOptions & options)
: Node("operation_mode_transition_manager", options)
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  operation_mode_transition_manager_ = std::make_unique<ManualDirectState>(this);
  data_ = std::make_shared<Data>();
  data_->requested_state = State::MANUAL_DIRECT;
  data_->vehicle_info = vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo();

  pub_operation_mode_ = create_publisher<OperationMode>("operation_mode", 1);
  pub_auto_available_ = create_publisher<IsAutonomousAvailable>("is_autonomous_available", 1);
  pub_debug_info_ = create_publisher<OperationModeTransitionManagerDebug>("~/debug_info", 1);

  sub_vehicle_kinematics_ = create_subscription<Odometry>(
    "kinematics", 1, [this](const Odometry::SharedPtr msg) { data_->kinematics = *msg; });

  sub_trajectory_ = create_subscription<Trajectory>(
    "trajectory", 1, [this](const Trajectory::SharedPtr msg) { data_->trajectory = *msg; });

  sub_control_cmd_ = create_subscription<AckermannControlCommand>(
    "control_cmd", 1,
    [this](const AckermannControlCommand::SharedPtr msg) { data_->control_cmd = *msg; });

  sub_control_mode_ = create_subscription<ControlModeReport>(
    "control_mode_report", 1,
    [this](const ControlModeReport::SharedPtr msg) { data_->current_control_mode = *msg; });

  sub_gate_operation_mode_ = create_subscription<OperationMode>(
    "gate_operation_mode", 1,
    [this](const OperationMode::SharedPtr msg) { data_->current_gate_operation_mode = *msg; });

  srv_mode_change_server_ = create_service<OperationModeRequest>(
    "operation_mode_request",
    std::bind(&OperationModeTransitionManager::onOperationModeRequest, this, _1, _2));

  {
    auto & p = engage_acceptable_param_;
    p.allow_autonomous_in_stopped =
      declare_parameter<bool>("engage_acceptable_limits.allow_autonomous_in_stopped");
    p.dist_threshold = declare_parameter<double>("engage_acceptable_limits.dist_threshold");
    p.speed_upper_threshold =
      declare_parameter<double>("engage_acceptable_limits.speed_upper_threshold");
    p.speed_lower_threshold =
      declare_parameter<double>("engage_acceptable_limits.speed_lower_threshold");
    p.yaw_threshold = declare_parameter<double>("engage_acceptable_limits.yaw_threshold");
    p.acc_threshold = declare_parameter<double>("engage_acceptable_limits.acc_threshold");
    p.lateral_acc_threshold =
      declare_parameter<double>("engage_acceptable_limits.lateral_acc_threshold");
    p.lateral_acc_diff_threshold =
      declare_parameter<double>("engage_acceptable_limits.lateral_acc_diff_threshold");
  }

  {
    auto & p = stable_check_param_;
    p.duration = declare_parameter<double>("stable_check.duration");
    p.dist_threshold = declare_parameter<double>("stable_check.dist_threshold");
    p.speed_upper_threshold = declare_parameter<double>("stable_check.speed_upper_threshold");
    p.speed_lower_threshold = declare_parameter<double>("stable_check.speed_lower_threshold");
    p.yaw_threshold = declare_parameter<double>("stable_check.yaw_threshold");
  }

  {
    const auto hz = declare_parameter<double>("frequency_hz");
    const auto period_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / hz));
    timer_ = rclcpp::create_timer(
      this, get_clock(), period_ns, std::bind(&OperationModeTransitionManager::onTimer, this));
  }
}

void OperationModeTransitionManager::onOperationModeRequest(
  const OperationModeRequest::Request::SharedPtr request,
  const OperationModeRequest::Response::SharedPtr response)
{
  const auto req_state = toEnum(request->mode);

  // invalid case
  if (req_state == State::TRANSITION_TO_AUTO) {
    RCLCPP_WARN(
      get_logger(), "mode change to TRANSITION_TO_AUTO is not supported. Request ignored.");
    response->success = false;
    return;
  }

  data_->requested_state = toEnum(request->mode);

  const auto state = updateState(data_);

  // write success/failure condition one by one for the state transition result.
  if (req_state == state) {
    response->success = true;
    return;
  }
  if (isAuto(req_state) && state == State::TRANSITION_TO_AUTO) {
    response->success = true;
    return;
  }

  // not satisfy any success conditions.
  response->success = false;
  data_->requested_state = operation_mode_transition_manager_->getCurrentState();
  RCLCPP_WARN(get_logger(), "mode change failed. Request was declined.");
  return;
}

void OperationModeTransitionManager::onTimer()
{
  data_->is_auto_available = checkEngageAvailable();

  updateState(data_);

  publishData();
}

void OperationModeTransitionManager::publishData()
{
  const auto time = now();

  OperationMode mode;
  mode.stamp = time;
  mode.mode = toMsg(operation_mode_transition_manager_->getCurrentState());
  pub_operation_mode_->publish(mode);

  IsAutonomousAvailable msg;
  msg.stamp = time;
  msg.is_autonomous_available = data_->is_auto_available;
  pub_auto_available_->publish(msg);

  debug_info_.stamp = time;
  debug_info_.requested_state = toStr(data_->requested_state);
  debug_info_.current_state = toStr(operation_mode_transition_manager_->getCurrentState());
  pub_debug_info_->publish(debug_info_);
}

bool OperationModeTransitionManager::hasDangerAcceleration()
{
  debug_info_.target_control_acceleration = data_->control_cmd.longitudinal.acceleration;

  const bool is_stopping = std::abs(data_->kinematics.twist.twist.linear.x) < 0.01;
  if (is_stopping) {
    return false;  // any acceleration is ok when stopped
  }

  const bool has_large_acc =
    std::abs(data_->control_cmd.longitudinal.acceleration) > engage_acceptable_param_.acc_threshold;
  return has_large_acc;
}

std::pair<bool, bool> OperationModeTransitionManager::hasDangerLateralAcceleration()
{
  const auto wheelbase = data_->vehicle_info.wheel_base_m;
  const auto curr_vx = data_->kinematics.twist.twist.linear.x;
  const auto curr_wz = data_->kinematics.twist.twist.angular.z;

  // Calculate angular velocity from kinematics model.
  // Use current_vx to focus on the steering behavior.
  const auto target_wz =
    curr_vx * std::tan(data_->control_cmd.lateral.steering_tire_angle) / wheelbase;

  const auto curr_lat_acc = curr_vx * curr_wz;
  const auto target_lat_acc = curr_vx * target_wz;

  const bool has_large_lat_acc =
    std::abs(curr_lat_acc) > engage_acceptable_param_.lateral_acc_threshold;
  const bool has_large_lat_acc_diff =
    std::abs(curr_lat_acc - target_lat_acc) > engage_acceptable_param_.lateral_acc_diff_threshold;

  debug_info_.lateral_acceleration = curr_lat_acc;
  debug_info_.lateral_acceleration_deviation = curr_lat_acc - target_lat_acc;

  return {has_large_lat_acc, has_large_lat_acc_diff};
}

bool OperationModeTransitionManager::checkEngageAvailable()
{
  constexpr auto dist_max = 100.0;
  constexpr auto yaw_max = M_PI_4;

  const auto current_speed = data_->kinematics.twist.twist.linear.x;
  const auto target_control_speed = data_->control_cmd.longitudinal.speed;
  const auto & param = engage_acceptable_param_;

  if (data_->trajectory.points.size() < 2) {
    RCLCPP_WARN(get_logger(), "Engage unavailable: trajectory size must be > 2");
    debug_info_ = OperationModeTransitionManagerDebug{};  // all false
    return false;
  }

  const auto closest_idx =
    findNearestIndex(data_->trajectory.points, data_->kinematics.pose.pose, dist_max, yaw_max);
  if (!closest_idx) {
    RCLCPP_INFO(get_logger(), "Engage unavailable: closest point not found");
    debug_info_ = OperationModeTransitionManagerDebug{};  // all false
    return false;                                         // closest trajectory point not found.
  }
  const auto closest_point = data_->trajectory.points.at(*closest_idx);
  const auto target_planning_speed = closest_point.longitudinal_velocity_mps;
  debug_info_.trajectory_available_ok = true;

  // No engagement is lateral control error is large
  const auto lateral_deviation = calcDistance2d(closest_point.pose, data_->kinematics.pose.pose);
  const bool lateral_deviation_ok = lateral_deviation < param.dist_threshold;

  // No engagement is yaw control error is large
  const auto yaw_deviation = calcYawDeviation(closest_point.pose, data_->kinematics.pose.pose);
  const bool yaw_deviation_ok = yaw_deviation < param.yaw_threshold;

  // No engagement if speed control error is large
  const auto speed_deviation = current_speed - target_planning_speed;
  const bool speed_upper_deviation_ok = speed_deviation <= param.speed_upper_threshold;
  const bool speed_lower_deviation_ok = speed_deviation >= param.speed_lower_threshold;

  // No engagement if the vehicle is moving but the target speed is zero.
  const bool stop_ok = !(std::abs(current_speed) > 0.1 && std::abs(target_control_speed) < 0.01);

  // No engagement if the large acceleration is commanded.
  const bool large_acceleration_ok = !hasDangerAcceleration();

  // No engagement if the lateral acceleration is over threshold
  const auto [has_large_lat_acc, has_large_lat_acc_diff] = hasDangerLateralAcceleration();
  const auto large_lateral_acceleration_ok = !has_large_lat_acc;
  const auto large_lateral_acceleration_diff_ok = !has_large_lat_acc_diff;

  // No engagement if a stop is expected within a certain period of time
  // TODO(Horibe): write me
  // ...

  const bool is_all_ok = lateral_deviation_ok && yaw_deviation_ok && speed_upper_deviation_ok &&
                         speed_lower_deviation_ok && stop_ok && large_acceleration_ok &&
                         large_lateral_acceleration_ok && large_lateral_acceleration_diff_ok;

  // set for debug info
  {
    debug_info_.is_all_ok = is_all_ok;
    debug_info_.lateral_deviation_ok = lateral_deviation_ok;
    debug_info_.yaw_deviation_ok = yaw_deviation_ok;
    debug_info_.speed_upper_deviation_ok = speed_upper_deviation_ok;
    debug_info_.speed_lower_deviation_ok = speed_lower_deviation_ok;
    debug_info_.stop_ok = stop_ok;
    debug_info_.large_acceleration_ok = large_acceleration_ok;
    debug_info_.large_lateral_acceleration_ok = large_lateral_acceleration_ok;
    debug_info_.large_lateral_acceleration_diff_ok = large_lateral_acceleration_diff_ok;

    debug_info_.current_speed = current_speed;
    debug_info_.target_control_speed = target_control_speed;
    debug_info_.target_planning_speed = target_planning_speed;

    debug_info_.lateral_deviation = lateral_deviation;
    debug_info_.yaw_deviation = yaw_deviation;
    debug_info_.speed_deviation = speed_deviation;
  }

  // Engagement is ready if the vehicle is stopped.
  // (this is checked in the end to calculate some debug values.)
  if (param.allow_autonomous_in_stopped && std::abs(current_speed) < 0.01) {
    debug_info_.is_all_ok = true;
    debug_info_.engage_allowed_for_stopped_vehicle = true;
    return true;
  }

  return is_all_ok;
}

State OperationModeTransitionManager::updateState(const std::shared_ptr<Data> data)
{
  const auto current_state = operation_mode_transition_manager_->getCurrentState();

  operation_mode_transition_manager_->setData(data);
  const auto next_state = operation_mode_transition_manager_->update();

  // no state change
  if (next_state == current_state) {
    return current_state;
  }

  // transit state
  switch (next_state) {
    case State::STOP:
      operation_mode_transition_manager_ = std::make_unique<StopState>(this);
      break;
    case State::REMOTE_OPERATOR:
      operation_mode_transition_manager_ = std::make_unique<RemoteOperatorState>(this);
      break;
    case State::MANUAL_DIRECT:
      operation_mode_transition_manager_ = std::make_unique<ManualDirectState>(this);
      break;
    case State::LOCAL_OPERATOR:
      operation_mode_transition_manager_ = std::make_unique<LocalOperatorState>(this);
      break;
    case State::TRANSITION_TO_AUTO:
      operation_mode_transition_manager_ = std::make_unique<TransitionToAutoState>(this);
      break;
    case State::AUTONOMOUS:
      operation_mode_transition_manager_ = std::make_unique<AutonomousState>(this);
      break;
  }
  operation_mode_transition_manager_->setParam(stable_check_param_);

  if (next_state != operation_mode_transition_manager_->getCurrentState()) {
    throw std::runtime_error("operation_mode_transition_manager: unexpected state change!");
  }

  return operation_mode_transition_manager_->getCurrentState();
}

}  // namespace operation_mode_transition_manager

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(operation_mode_transition_manager::OperationModeTransitionManager)
