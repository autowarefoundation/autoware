// Copyright 2022 Tier IV, Inc.
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
 * @file voltage_monitor.h
 * @brief  voltage monitor class
 */

#ifndef SYSTEM_MONITOR__VOLTAGE_MONITOR__VOLTAGE_MONITOR_HPP_
#define SYSTEM_MONITOR__VOLTAGE_MONITOR__VOLTAGE_MONITOR_HPP_

#include <diagnostic_updater/diagnostic_updater.hpp>

#include <climits>
#include <regex>
#include <string>
class VoltageMonitor : public rclcpp::Node
{
public:
  /**
   * @brief constructor
   * @param [in] options Options associated with this node.
   */
  explicit VoltageMonitor(const rclcpp::NodeOptions & options);

  /**
   * @brief Update the diagnostic state.
   */
  void update();

protected:
  using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;

  diagnostic_updater::Updater updater_;  //!< @brief Updater class which advertises to /diagnostics

  char hostname_[HOST_NAME_MAX + 1];  //!< @brief host name

  /**
   * @brief check CMOS battery
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void checkVoltage(
    diagnostic_updater::DiagnosticStatusWrapper & stat);  // NOLINT(runtime/references)
  /**
   * @brief check CMOS battery
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void checkBatteryStatus(
    diagnostic_updater::DiagnosticStatusWrapper & stat);  // NOLINT(runtime/references)

  float voltage_warn_;
  float voltage_error_;
  std::string voltage_string_;
  std::regex voltage_regex_;
};

#endif  // SYSTEM_MONITOR__VOLTAGE_MONITOR__VOLTAGE_MONITOR_HPP_
