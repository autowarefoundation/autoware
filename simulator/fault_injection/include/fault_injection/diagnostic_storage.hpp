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

#ifndef FAULT_INJECTION__DIAGNOSTIC_STORAGE_HPP_
#define FAULT_INJECTION__DIAGNOSTIC_STORAGE_HPP_

#include <diagnostic_aggregator/status_item.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <string>
#include <unordered_map>

namespace fault_injection
{
using diagnostic_msgs::msg::DiagnosticStatus;

struct DiagConfig
{
  std::string sim_name;   // Event name used in the simulation
  std::string diag_name;  // Name of the diagnostic
};

class DiagnosticStorage
{
public:
  void registerEvent(const DiagConfig & diag_config)
  {
    DiagnosticStatus status;
    status.name = diag_config.diag_name;
    status.hardware_id = "fault_injection";
    status.level = DiagnosticStatus::OK;
    status.message = "OK";
    event_diag_map_[diag_config.sim_name] = status;
  }

  bool isEventRegistered(const std::string & event_name)
  {
    return event_diag_map_.find(event_name) != event_diag_map_.end();
  }

  void updateLevel(const std::string & event_name, const int8_t level)
  {
    event_diag_map_.at(event_name).level = diagnostic_aggregator::valToLevel(level);
    event_diag_map_.at(event_name).message = diagnostic_aggregator::valToMsg(level);
  }

  DiagnosticStatus getDiag(const std::string & event_name)
  {
    return event_diag_map_.at(event_name);
  }

private:
  std::unordered_map<std::string, DiagnosticStatus> event_diag_map_;
};

}  // namespace fault_injection

#endif  // FAULT_INJECTION__DIAGNOSTIC_STORAGE_HPP_
