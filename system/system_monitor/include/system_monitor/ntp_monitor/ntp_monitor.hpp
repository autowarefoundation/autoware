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
 * @file ntp_monitor.h
 * @brief NTP monitor class
 */

#ifndef SYSTEM_MONITOR__NTP_MONITOR__NTP_MONITOR_HPP_
#define SYSTEM_MONITOR__NTP_MONITOR__NTP_MONITOR_HPP_

#include <diagnostic_updater/diagnostic_updater.hpp>

#include <climits>
#include <map>
#include <string>
#include <thread>

class NTPMonitor : public rclcpp::Node
{
public:
  /**
   * @brief constructor
   * @param [in] options Options associated with this node.
   */
  explicit NTPMonitor(const rclcpp::NodeOptions & options);

  /**
   * @brief Update the diagnostic state.
   */
  void update();

protected:
  using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;

  /**
   * @brief check NTP Offset
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void checkOffset(
    diagnostic_updater::DiagnosticStatusWrapper & stat);  // NOLINT(runtime/references)

  /**
   * @brief Timer callback to execute chronyc command
   */
  void onTimer();

  /**
   * @brief Timeout callback function for executing chronyc
   */
  void onTimeout();

  /**
   * @brief function to execute chronyc
   * @param [out] outOffset offset value of NTP time
   * @param [out] out_tracking_map "chronyc tracking" output for diagnostic
   * @param [out] pipe2_err_str if pipe2 error occurred, return error string
   * @return if chronyc error occurred, return error string
   */
  std::string executeChronyc(
    float & outOffset, std::map<std::string, std::string> & out_tracking_map,
    std::string & pipe2_err_str);

  diagnostic_updater::Updater updater_;  //!< @brief Updater class which advertises to /diagnostics

  char hostname_[HOST_NAME_MAX + 1];  //!< @brief host name
  bool chronyc_exists_;               //!< @brief flag if chronyc exists

  float offset_warn_;   //!< @brief NTP offset(sec) to generate warning
  float offset_error_;  //!< @brief NTP offset(sec) to generate error
  int timeout_;         //!< @brief Timeout duration for executing chronyc

  rclcpp::TimerBase::SharedPtr timer_;  //!< @brief Timer to execute chronyc command
  rclcpp::CallbackGroup::SharedPtr timer_callback_group_;  //!< @brief Callback Group
  std::mutex mutex_;           //!< @brief Mutex for output from chronyc command
  std::string error_str_;      //!< @brief Error string
  std::string pipe2_err_str_;  //!< @brief Error string regarding pipe2 function call
  float offset_;               //!< @brief Offset value of NTP time
  std::map<std::string, std::string> tracking_map_;  //!< @brief Output of chronyc tracking
  double elapsed_ms_;                                //!< @brief Execution time of chronyc command
  rclcpp::TimerBase::SharedPtr timeout_timer_;       //!< @brief Timeout for executing chronyc
  std::mutex timeout_mutex_;  //!< @brief Mutex regarding timeout for executing chronyc
  bool timeout_expired_;      //!< @brief Timeout for executing chronyc has expired or not

  /**
   * @brief NTP offset status messages
   */
  const std::map<int, const char *> offset_dict_ = {
    {DiagStatus::OK, "OK"}, {DiagStatus::WARN, "high"}, {DiagStatus::ERROR, "too high"}};
};

#endif  // SYSTEM_MONITOR__NTP_MONITOR__NTP_MONITOR_HPP_
