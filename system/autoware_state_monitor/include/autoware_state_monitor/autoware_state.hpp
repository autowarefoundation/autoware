// Copyright 2020 Tier IV, Inc.
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

#ifndef AUTOWARE_STATE_MONITOR__AUTOWARE_STATE_HPP_
#define AUTOWARE_STATE_MONITOR__AUTOWARE_STATE_HPP_

#include <autoware_auto_system_msgs/msg/autoware_state.hpp>

#include <string>

enum class AutowareState : int8_t {
  InitializingVehicle,
  WaitingForRoute,
  Planning,
  WaitingForEngage,
  Driving,
  ArrivedGoal,
  Finalizing,
};

inline AutowareState fromMsg(const int state)
{
  using StateMessage = autoware_auto_system_msgs::msg::AutowareState;

  if (state == StateMessage::INITIALIZING) {
    return AutowareState::InitializingVehicle;
  }
  if (state == StateMessage::WAITING_FOR_ROUTE) {
    return AutowareState::WaitingForRoute;
  }
  if (state == StateMessage::PLANNING) {
    return AutowareState::Planning;
  }
  if (state == StateMessage::WAITING_FOR_ENGAGE) {
    return AutowareState::WaitingForEngage;
  }
  if (state == StateMessage::DRIVING) {
    return AutowareState::Driving;
  }
  if (state == StateMessage::ARRIVED_GOAL) {
    return AutowareState::ArrivedGoal;
  }
  if (state == StateMessage::FINALIZING) {
    return AutowareState::Finalizing;
  }

  throw std::runtime_error("invalid state");
}

inline int toMsg(const AutowareState & state)
{
  using StateMessage = autoware_auto_system_msgs::msg::AutowareState;

  if (state == AutowareState::InitializingVehicle) {
    return StateMessage::INITIALIZING;
  }
  if (state == AutowareState::WaitingForRoute) {
    return StateMessage::WAITING_FOR_ROUTE;
  }
  if (state == AutowareState::Planning) {
    return StateMessage::PLANNING;
  }
  if (state == AutowareState::WaitingForEngage) {
    return StateMessage::WAITING_FOR_ENGAGE;
  }
  if (state == AutowareState::Driving) {
    return StateMessage::DRIVING;
  }
  if (state == AutowareState::ArrivedGoal) {
    return StateMessage::ARRIVED_GOAL;
  }
  if (state == AutowareState::Finalizing) {
    return StateMessage::FINALIZING;
  }

  throw std::runtime_error("invalid state");
}

#endif  // AUTOWARE_STATE_MONITOR__AUTOWARE_STATE_HPP_
