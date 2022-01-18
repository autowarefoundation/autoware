// Copyright 2020 Autoware Foundation
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

/**
 * @file raspi_cpu_monitor.cpp
 * @brief Raspberry Pi CPU monitor class
 */

#include "system_monitor/cpu_monitor/raspi_cpu_monitor.hpp"

#include "system_monitor/system_monitor_utility.hpp"

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>

#include <string>
#include <vector>

namespace fs = boost::filesystem;

CPUMonitor::CPUMonitor(const rclcpp::NodeOptions & options) : CPUMonitorBase("cpu_monitor", options)
{
}

void CPUMonitor::checkThrottling(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  int level = DiagStatus::OK;
  std::vector<std::string> status;

  const fs::path path("/sys/devices/platform/soc/soc:firmware/get_throttled");
  fs::ifstream ifs(path, std::ios::in);
  if (!ifs) {
    stat.summary(DiagStatus::ERROR, "file open error");
    stat.add("get_throttled", "file open error");
    return;
  }

  int throttled;
  ifs >> std::hex >> throttled;
  ifs.close();

  // Consider only thermal throttling as an error
  if ((throttled & raspiThermalThrottlingMask) == raspiThermalThrottlingMask) {
    level = DiagStatus::ERROR;
  }

  while (throttled) {
    int flag = throttled & ((~throttled) + 1);
    throttled ^= flag;
    status.push_back(throttledToString(flag));
  }
  if (status.empty()) {
    status.emplace_back("All clear");
  }

  stat.add("status", boost::algorithm::join(status, ", "));

  stat.summary(level, thermal_dict_.at(level));
}

void CPUMonitor::getTempNames()
{
  // thermal_zone0
  std::vector<thermal_zone> therms;
  SystemMonitorUtility::getThermalZone("cpu-thermal", &therms);

  for (auto itr = therms.begin(); itr != therms.end(); ++itr) {
    temps_.emplace_back(itr->label_, itr->path_);
  }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(CPUMonitor)
