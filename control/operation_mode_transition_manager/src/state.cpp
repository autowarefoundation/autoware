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

#include "operation_mode_transition_manager/state.hpp"

#include "tier4_autoware_utils/tier4_autoware_utils.hpp"

#include <algorithm>
#include <cmath>

namespace operation_mode_transition_manager
{

using tier4_autoware_utils::calcDistance2d;
using tier4_autoware_utils::calcYawDeviation;
using tier4_autoware_utils::findNearestIndex;

EngageStateBase::EngageStateBase(const State state, rclcpp::Node * node)
: logger_(node->get_logger()), clock_(node->get_clock()), state_(state)
{
  // TODO(Horibe): move to manager.
  srv_mode_change_client_ = node->create_client<ControlModeRequest>("control_mode_request");
}

State EngageStateBase::defaultUpdateOnManual()
{
  const bool all_engage_requirements_are_satisfied = data_->is_auto_available;
  const bool is_engage_requested = isAuto(data_->requested_state);

  // manual to manual: change state directly
  if (!is_engage_requested) {
    return isManual(data_->requested_state) ? data_->requested_state : getCurrentState();
  }

  // manual to auto: control_more_request will be sent in TRANSITION_TO_AUTO state.
  if (all_engage_requirements_are_satisfied) {
    return State::TRANSITION_TO_AUTO;
  } else {
    RCLCPP_WARN(logger_, "engage requirements are not satisfied. Engage prohibited.");
    return getCurrentState();
  }
}

bool EngageStateBase::sendAutonomousModeRequest()
{
  bool success = true;

  auto request = std::make_shared<ControlModeRequest::Request>();
  request->mode.header.stamp = clock_->now();
  request->mode.data = ControlMode::AUTO;

  const auto callback = [&](rclcpp::Client<ControlModeRequest>::SharedFuture future) {
    success = future.get()->success;
    if (!success) {
      RCLCPP_WARN(logger_, "Autonomous mode change was rejected.");
    }
  };

  srv_mode_change_client_->async_send_request(request, callback);

  // TODO(Horibe): handle request failure. Now, only timeout check is running in Transition state.
  // auto future = srv_mode_change_client_->async_send_request(request, callback);
  // rclcpp::spin_until_future_complete(node_, future);

  return success;
}

bool TransitionToAutoState::checkVehicleOverride()
{
  const auto mode = data_->current_control_mode.mode;

  if (mode == ControlModeReport::AUTONOMOUS) {
    is_vehicle_mode_change_done_ = true;
  }

  if (is_vehicle_mode_change_done_) {
    if (mode != ControlModeReport::AUTONOMOUS) {
      return true;
    }
  }
  return false;
}

bool TransitionToAutoState::checkTransitionTimeout() const
{
  if (data_->current_control_mode.mode == ControlModeReport::AUTONOMOUS) {
    return false;
  }

  constexpr auto timeout_thr = 3.0;
  if ((clock_->now() - transition_requested_time_).seconds() > timeout_thr) {
    return true;
  }
  return false;
}

State TransitionToAutoState::update()
{
  // return to Manual soon if requested.
  const bool is_disengage_requested = isManual(data_->requested_state);
  if (is_disengage_requested) {
    return data_->requested_state;
  }

  // return to Manual when vehicle control_mode is set to Manual after Auto transition is done.
  if (checkVehicleOverride()) {
    data_->requested_state = State::MANUAL_DIRECT;
    return State::MANUAL_DIRECT;
  }

  if (checkTransitionTimeout()) {
    RCLCPP_WARN(logger_, "time out for ControlMode change. Return to MANUAL state.");
    data_->requested_state = State::MANUAL_DIRECT;
    return State::MANUAL_DIRECT;
  }

  // waiting transition of vehicle_cmd_gate
  if (data_->current_gate_operation_mode.mode != OperationMode::TRANSITION_TO_AUTO) {
    RCLCPP_INFO(logger_, "transition check: gate operation_mode is still NOT TransitionToAuto");
    return getCurrentState();
  }

  // send Autonomous mode request to vehicle
  // NOTE: this should be done after gate_operation_mode is set to TRANSITION_TO_AUTO. Otherwise
  // control_cmd with nominal filter will be sent to vehicle.
  if (!is_control_mode_request_send_) {
    sendAutonomousModeRequest();
    is_control_mode_request_send_ = true;
  }

  // waiting transition of vehicle
  if (data_->current_control_mode.mode != ControlModeReport::AUTONOMOUS) {
    RCLCPP_INFO(logger_, "transition check: vehicle control_mode is still NOT Autonomous");
    return getCurrentState();
  }

  const bool is_system_stable = checkSystemStable();

  if (is_system_stable) {
    return State::AUTONOMOUS;
  } else {
    return getCurrentState();
  }
}

bool TransitionToAutoState::checkSystemStable()
{
  constexpr auto dist_max = 5.0;
  constexpr auto yaw_max = M_PI_4;

  const auto unstable = [this]() {
    stable_start_time_.reset();
    return false;
  };

  if (data_->trajectory.points.size() < 2) {
    RCLCPP_INFO(logger_, "Not stable yet: trajectory size must be > 2");
    return unstable();
  }

  const auto closest_idx =
    findNearestIndex(data_->trajectory.points, data_->kinematics.pose.pose, dist_max, yaw_max);
  if (!closest_idx) {
    RCLCPP_INFO(logger_, "Not stable yet: closest point not found");
    return unstable();
  }

  const auto closest_point = data_->trajectory.points.at(*closest_idx);

  // check for lateral deviation
  const auto dist_deviation = calcDistance2d(closest_point.pose, data_->kinematics.pose.pose);
  if (dist_deviation > stable_check_param_.dist_threshold) {
    RCLCPP_INFO(logger_, "Not stable yet: distance deviation is too large: %f", dist_deviation);
    return unstable();
  }

  // check for yaw deviation
  const auto yaw_deviation = calcYawDeviation(closest_point.pose, data_->kinematics.pose.pose);
  if (yaw_deviation > stable_check_param_.yaw_threshold) {
    RCLCPP_INFO(logger_, "Not stable yet: yaw deviation is too large: %f", yaw_deviation);
    return unstable();
  }

  // check for speed deviation
  const auto speed_deviation =
    data_->kinematics.twist.twist.linear.x - closest_point.longitudinal_velocity_mps;
  if (speed_deviation > stable_check_param_.speed_upper_threshold) {
    RCLCPP_INFO(logger_, "Not stable yet: ego speed is too high: %f", speed_deviation);
    return unstable();
  }
  if (speed_deviation < stable_check_param_.speed_lower_threshold) {
    RCLCPP_INFO(logger_, "Not stable yet: ego speed is too low: %f", speed_deviation);
    return unstable();
  }

  // count start.
  if (!stable_start_time_) {
    stable_start_time_ = std::make_unique<rclcpp::Time>(clock_->now());
  }

  // keep being stable for enough time.
  const double stable_time = (clock_->now() - *stable_start_time_).seconds();
  const bool is_system_stable = stable_time > stable_check_param_.duration;
  RCLCPP_INFO(logger_, "Now stable: now duration: %f", stable_time);

  return is_system_stable;
}

State AutonomousState::update()
{
  // check current mode
  if (data_->current_control_mode.mode == ControlModeReport::MANUAL) {
    data_->requested_state = State::MANUAL_DIRECT;
    return State::MANUAL_DIRECT;
  }

  // check request mode
  bool is_disengage_requested = isManual(data_->requested_state);

  if (is_disengage_requested) {
    return data_->requested_state;
  } else {
    return getCurrentState();
  }
}

}  // namespace operation_mode_transition_manager
