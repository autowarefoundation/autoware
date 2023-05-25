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
 * @file mem_monitor.h
 * @brief Memory monitor class
 */

#ifndef SYSTEM_MONITOR__MEM_MONITOR__MEM_MONITOR_HPP_
#define SYSTEM_MONITOR__MEM_MONITOR__MEM_MONITOR_HPP_

#include <diagnostic_updater/diagnostic_updater.hpp>

#include <climits>
#include <map>
#include <string>

class MemMonitor : public rclcpp::Node
{
public:
  /**
   * @brief constructor
   * @param [in] options Options associated with this node.
   */
  explicit MemMonitor(const rclcpp::NodeOptions & options);

  /**
   * @brief Update the diagnostic state.
   */
  void update();

protected:
  using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;

  /**
   * @brief check Memory usage
   * @param @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void checkUsage(
    diagnostic_updater::DiagnosticStatusWrapper & stat);  // NOLINT(runtime/references)

  /**
   * @brief check Memory ECC
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   */
  void checkEcc(diagnostic_updater::DiagnosticStatusWrapper & stat);

  /**
   * @brief get human-readable output for memory size
   * @param [in] str size with bytes
   * @return human-readable output
   */
  std::string toHumanReadable(const std::string & str);

  diagnostic_updater::Updater updater_;  //!< @brief Updater class which advertises to /diagnostics

  char hostname_[HOST_NAME_MAX + 1];  //!< @brief host name

  size_t available_size_;  //!< @brief Memory available size to generate error

  /**
   * @brief Memory usage status messages
   */
  const std::map<int, const char *> usage_dict_ = {
    {DiagStatus::OK, "OK"}, {DiagStatus::WARN, "high load"}, {DiagStatus::ERROR, "very high load"}};
};

#endif  // SYSTEM_MONITOR__MEM_MONITOR__MEM_MONITOR_HPP_
