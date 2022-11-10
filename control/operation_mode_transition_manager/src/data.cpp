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

#include "data.hpp"

namespace operation_mode_transition_manager
{

std::string toString(const std::optional<OperationMode> mode)
{
  if (!mode) {
    return "DISENGAGE";
  }
  switch (mode.value()) {
    case OperationMode::STOP:
      return "STOP";
    case OperationMode::AUTONOMOUS:
      return "AUTONOMOUS";
    case OperationMode::LOCAL:
      return "LOCAL";
    case OperationMode::REMOTE:
      return "REMOTE";
  }
  return "UNKNOWN";
}

std::optional<OperationMode> toEnum(const ChangeOperationMode::Request & request)
{
  using ServiceMode = ChangeOperationMode::Request;

  switch (request.mode) {
    case ServiceMode::STOP:
      return OperationMode::STOP;
    case ServiceMode::AUTONOMOUS:
      return OperationMode::AUTONOMOUS;
    case ServiceMode::LOCAL:
      return OperationMode::LOCAL;
    case ServiceMode::REMOTE:
      return OperationMode::REMOTE;
  }
  return std::nullopt;
}

OperationModeValue toMsg(const OperationMode mode)
{
  switch (mode) {
    case OperationMode::STOP:
      return OperationModeState::STOP;
    case OperationMode::AUTONOMOUS:
      return OperationModeState::AUTONOMOUS;
    case OperationMode::LOCAL:
      return OperationModeState::LOCAL;
    case OperationMode::REMOTE:
      return OperationModeState::REMOTE;
  }
  return OperationModeState::UNKNOWN;
}

}  // namespace operation_mode_transition_manager
