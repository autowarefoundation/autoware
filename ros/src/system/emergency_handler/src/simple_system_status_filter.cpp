/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "simple_system_status_filter.h"

const int DIAG_OK = autoware_system_msgs::DiagnosticStatus::OK;
const int DIAG_WARN = autoware_system_msgs::DiagnosticStatus::WARN;
const int DIAG_ERROR = autoware_system_msgs::DiagnosticStatus::ERROR;
const int DIAG_FATAL = autoware_system_msgs::DiagnosticStatus::FATAL;

const std::map<int, std::string> CommonFilterRule::getBehaviorParam(ros::NodeHandle& pnh)
{
  std::map<std::string, int> emergency_stop_map;
  std::map<int, std::string> behavior_param;
  pnh.getParam("behavior/emergency_stop", emergency_stop_map);
  if (emergency_stop_map.size() == 1)
  {
    const auto& el = emergency_stop_map.begin();
    emergency_stop_ = el->second;
    behavior_param.emplace(el->second, el->first);
  }
  return behavior_param;
}

int CommonFilterRule::emergency_stop_ = 0;

int SimpleHardwareFilter::selectPriority(const SystemStatus& status)
{
  const bool is_emergency = !(checkAllHardwareSimplly(status.hardware_status, DIAG_ERROR));
  return is_emergency ? emergency_stop_ : normal_behavior_;
}

int SimpleNodeFilter::selectPriority(const SystemStatus& status)
{
  vital_monitor_.updateNodeStatus(status.available_nodes);
  SystemStatus updated_status(vital_monitor_.addDeadNodes(status));
  const bool is_emergency = !(checkAllNodeSimplly(updated_status.node_status, DIAG_ERROR));
  return is_emergency ? emergency_stop_ : normal_behavior_;
}
