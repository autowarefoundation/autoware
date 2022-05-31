// Copyright 2022 The Autoware Contributors
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

#ifndef BLUETOOTH_MONITOR__BLUETOOTH_MONITOR_HPP_
#define BLUETOOTH_MONITOR__BLUETOOTH_MONITOR_HPP_

#include "bluetooth_monitor/service/l2ping_interface.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <map>
#include <string>
#include <vector>

class BluetoothMonitor : public rclcpp::Node
{
public:
  /**
   * @brief Constructor
   * @param [in] options Options associated with this node
   */
  explicit BluetoothMonitor(const rclcpp::NodeOptions & options);

protected:
  /**
   * @brief Connect to L2ping service
   * @param [out] stat Diagnostic message passed directly to diagnostic publish calls
   * @return true on success, false on error
   * @note NOLINT syntax is needed since this function asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  bool connectService(
    diagnostic_updater::DiagnosticStatusWrapper & stat);  // NOLINT(runtime/references)

  /**
   * @brief Send L2ping configuration to L2ping service
   * @param [out] stat Diagnostic message passed directly to diagnostic publish calls
   * @return true on success, false on error
   * @note NOLINT syntax is needed since this function asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  bool sendConfig(
    diagnostic_updater::DiagnosticStatusWrapper & stat);  // NOLINT(runtime/references)

  /**
   * @brief Receive data from L2ping service
   * @param [out] stat Diagnostic message passed directly to diagnostic publish calls
   * @return true on success, false on error
   * @note NOLINT syntax is needed since this function asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  bool receiveData(diagnostic_updater::DiagnosticStatusWrapper & stat);

  /**
   * @brief Close connection with L2ping service
   */
  void closeConnection();

  /**
   * @brief Set error level of diagnostic status
   * @param [out] stat Diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void setErrorLevel(
    diagnostic_updater::DiagnosticStatusWrapper & stat);  // NOLINT(runtime/references)

  /**
   * @brief Obtain diagnostic status and check connection
   * @param [out] stat Diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void checkConnection(
    diagnostic_updater::DiagnosticStatusWrapper & stat);  // NOLINT(runtime/references)

  diagnostic_updater::Updater updater_;  //!< @brief Updater class which advertises to /diagnostics
  int socket_;                           //!< @brief Socket to communicate with L2ping service
  int port_;                             //!< @brief Port number to connect with L2ping service
  L2pingServiceConfig config_;           //!< @brief Configuration of L2ping service
  L2pingStatusList status_list_;         //!< @brief Device status list

  using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;

  static constexpr const char * FUNCTION_ERROR_STR = "Function error";

  const std::map<StatusCode, const char *> status_string_list_ = {
    {StatusCode::OK, "OK"},
    {StatusCode::RTT_WARNING, "RTT warning"},
    {StatusCode::LOST, "Lost"},
    {StatusCode::FUNCTION_ERROR, FUNCTION_ERROR_STR}};

  const std::map<StatusCode, unsigned char> status_error_list_ = {
    {StatusCode::OK, DiagStatus::OK},
    {StatusCode::RTT_WARNING, DiagStatus::WARN},
    {StatusCode::LOST, DiagStatus::ERROR},
    {StatusCode::FUNCTION_ERROR, DiagStatus::ERROR}};
};

#endif  // BLUETOOTH_MONITOR__BLUETOOTH_MONITOR_HPP_
