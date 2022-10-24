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
 * @file tegra_gpu_monitor.h
 * @brief Tegra GPU monitor class
 */

#ifndef SYSTEM_MONITOR__GPU_MONITOR__TEGRA_GPU_MONITOR_HPP_
#define SYSTEM_MONITOR__GPU_MONITOR__TEGRA_GPU_MONITOR_HPP_

#include "system_monitor/gpu_monitor/gpu_monitor_base.hpp"

#include <string>
#include <vector>

struct gpu_info
{
  std::string label_;  //!< @brief gpu label
  std::string path_;   //!< @brief sysfs path to gpu temperature

  gpu_info() : label_(), path_() {}
  gpu_info(const std::string & l, const std::string & p) : label_(l), path_(p) {}
};

class GPUMonitor : public GPUMonitorBase
{
public:
  /**
   * @brief constructor
   * @param [in] node_name Name of the node.
   * @param [in] options Options associated with this node.
   */
  explicit GPUMonitor(const rclcpp::NodeOptions & options);

protected:
  /**
   * @brief check GPU temperature
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void checkTemp(
    diagnostic_updater::DiagnosticStatusWrapper & stat) override;  // NOLINT(runtime/references)

  /**
   * @brief check GPU usage
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void checkUsage(
    diagnostic_updater::DiagnosticStatusWrapper & stat) override;  // NOLINT(runtime/references)

  /**
   * @brief check GPU throttling
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void checkThrottling(
    diagnostic_updater::DiagnosticStatusWrapper & stat) override;  // NOLINT(runtime/references)

  /**
   * @brief check GPU frequency
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void checkFrequency(
    diagnostic_updater::DiagnosticStatusWrapper & stat) override;  // NOLINT(runtime/references)

  /**
   * @brief get names for gpu temperature files
   */
  void getTempNames();

  /**
   * @brief get names for gpu load files
   */
  void getLoadNames();

  /**
   * @brief get names for gpu frequency files
   */
  void getFreqNames();

  std::vector<gpu_info> temps_;  //!< @brief GPU list for temperature
  std::vector<gpu_info> loads_;  //!< @brief GPU list for utilization
  std::vector<gpu_info> freqs_;  //!< @brief GPU list for frequency
};

#endif  // SYSTEM_MONITOR__GPU_MONITOR__TEGRA_GPU_MONITOR_HPP_
