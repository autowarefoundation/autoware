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

/**
 * @file tegra_cpu_monitor.h
 * @brief TEGRA CPU monitor class
 */

#ifndef SYSTEM_MONITOR__CPU_MONITOR__TEGRA_CPU_MONITOR_HPP_
#define SYSTEM_MONITOR__CPU_MONITOR__TEGRA_CPU_MONITOR_HPP_

#include "system_monitor/cpu_monitor/cpu_monitor_base.hpp"

#include <string>

class CPUMonitor : public CPUMonitorBase
{
public:
  /**
   * @brief constructor
   * @param [in] node_name Name of the node.
   * @param [in] options Options associated with this node.
   */
  explicit CPUMonitor(const rclcpp::NodeOptions & options);

  /**
   * @brief get names for core temperature files
   */
  void getTempNames() override;

protected:
  /**
   * @brief check CPU thermal throttling
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void checkThrottling(
    diagnostic_updater::DiagnosticStatusWrapper & stat) override;  // NOLINT(runtime/references)
};

#endif  // SYSTEM_MONITOR__CPU_MONITOR__TEGRA_CPU_MONITOR_HPP_
