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

uint8_t toMsg(const State s)
{
  if (s == State::STOP) {
    return OperationMode::STOP;
  } else if (s == State::REMOTE_OPERATOR) {
    return OperationMode::REMOTE_OPERATOR;
  } else if (s == State::MANUAL_DIRECT) {
    return OperationMode::MANUAL_DIRECT;
  } else if (s == State::LOCAL_OPERATOR) {
    return OperationMode::LOCAL_OPERATOR;
  } else if (s == State::TRANSITION_TO_AUTO) {
    return OperationMode::TRANSITION_TO_AUTO;
  } else if (s == State::AUTONOMOUS) {
    return OperationMode::AUTONOMOUS;
  }
  return OperationMode::STOP;
}

State toEnum(const OperationMode m)
{
  if (m.mode == OperationMode::STOP) {
    return State::STOP;
  } else if (m.mode == OperationMode::REMOTE_OPERATOR) {
    return State::REMOTE_OPERATOR;
  } else if (m.mode == OperationMode::MANUAL_DIRECT) {
    return State::MANUAL_DIRECT;
  } else if (m.mode == OperationMode::LOCAL_OPERATOR) {
    return State::LOCAL_OPERATOR;
  } else if (m.mode == OperationMode::TRANSITION_TO_AUTO) {
    return State::TRANSITION_TO_AUTO;
  } else if (m.mode == OperationMode::AUTONOMOUS) {
    return State::AUTONOMOUS;
  }
  return State::STOP;
}

bool isManual(const State s)
{
  if (
    s == State::STOP || s == State::REMOTE_OPERATOR || s == State::MANUAL_DIRECT ||
    s == State::LOCAL_OPERATOR) {
    return true;
  } else {
    return false;
  }
}

bool isAuto(const State s)
{
  if (s == State::AUTONOMOUS) {
    return true;
  } else {
    return false;
  }
}

std::string toStr(const State s)
{
  if (s == State::STOP) {
    return "STOP";
  } else if (s == State::REMOTE_OPERATOR) {
    return "REMOTE_OPERATOR";
  } else if (s == State::MANUAL_DIRECT) {
    return "MANUAL_DIRECT";
  } else if (s == State::LOCAL_OPERATOR) {
    return "LOCAL_OPERATOR";
  } else if (s == State::TRANSITION_TO_AUTO) {
    return "TRANSITION_TO_AUTO";
  } else if (s == State::AUTONOMOUS) {
    return "AUTONOMOUS";
  } else {
    return "INVALID";
  }
}

}  // namespace operation_mode_transition_manager
