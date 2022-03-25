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
 * @file cpu_monitor_base.h
 * @brief CPU monitor base class
 */

#ifndef SYSTEM_MONITOR__CPU_MONITOR__CPU_MONITOR_BASE_HPP_
#define SYSTEM_MONITOR__CPU_MONITOR__CPU_MONITOR_BASE_HPP_

#include <diagnostic_updater/diagnostic_updater.hpp>

#include <tier4_external_api_msgs/msg/cpu_status.hpp>
#include <tier4_external_api_msgs/msg/cpu_usage.hpp>

#include <climits>
#include <map>
#include <string>
#include <vector>

/**
 * @brief CPU temperature information
 */
typedef struct cpu_temp_info
{
  std::string label_;  //!< @brief cpu label
  std::string path_;   //!< @brief sysfs path to cpu temperature

  cpu_temp_info() : label_(), path_() {}
  cpu_temp_info(const std::string & label, const std::string & path) : label_(label), path_(path) {}
} cpu_temp_info;

/**
 * @brief CPU frequency information
 */
typedef struct cpu_freq_info
{
  int index_;         //!< @brief cpu index
  std::string path_;  //!< @brief sysfs path to cpu frequency

  cpu_freq_info() : index_(0), path_() {}
  cpu_freq_info(int index, const std::string & path) : index_(index), path_(path) {}
} cpu_freq_info;

class CPUMonitorBase : public rclcpp::Node
{
public:
  /**
   * @brief Update the diagnostic state.
   */
  void update();

  /**
   * @brief get names for core temperature files
   */
  virtual void getTempNames();

  /**
   * @brief get names for cpu frequency files
   */
  virtual void getFreqNames();

protected:
  using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;

  /**
   * @brief constructor
   * @param [in] node_name Name of the node.
   * @param [in] options Options associated with this node.
   */
  CPUMonitorBase(const std::string & node_name, const rclcpp::NodeOptions & options);

  /**
   * @brief check CPU temperature
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  virtual void checkTemp(
    diagnostic_updater::DiagnosticStatusWrapper & stat);  // NOLINT(runtime/references)

  /**
   * @brief check CPU usage
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  virtual void checkUsage(
    diagnostic_updater::DiagnosticStatusWrapper & stat);  // NOLINT(runtime/references)

  /**
   * @brief convert Cpu Usage To diagnostic Level
   * @param [cpu_name] mpstat cpu name
   * @param [usage] cpu usage value
   * @return DiagStatus::OK or WARN or ERROR
   */
  virtual int CpuUsageToLevel(const std::string & cpu_name, float usage);

  /**
   * @brief check CPU load average
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  virtual void checkLoad(
    diagnostic_updater::DiagnosticStatusWrapper & stat);  // NOLINT(runtime/references)

  /**
   * @brief check CPU thermal throttling
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   */
  virtual void checkThrottling(
    diagnostic_updater::DiagnosticStatusWrapper & stat);  // NOLINT(runtime/references)

  /**
   * @brief check CPU frequency
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  virtual void checkFrequency(
    diagnostic_updater::DiagnosticStatusWrapper & stat);  // NOLINT(runtime/references)

  diagnostic_updater::Updater updater_;  //!< @brief Updater class which advertises to /diagnostics

  char hostname_[HOST_NAME_MAX + 1];        //!< @brief host name
  int num_cores_;                           //!< @brief number of cores
  std::vector<cpu_temp_info> temps_;        //!< @brief CPU list for temperature
  std::vector<cpu_freq_info> freqs_;        //!< @brief CPU list for frequency
  std::vector<int> usage_warn_check_cnt_;   //!< @brief CPU list for usage over warn check counter
  std::vector<int> usage_error_check_cnt_;  //!< @brief CPU list for usage over error check counter
  bool mpstat_exists_;                      //!< @brief flag if mpstat exists

  float usage_warn_;       //!< @brief CPU usage(%) to generate warning
  float usage_error_;      //!< @brief CPU usage(%) to generate error
  int usage_warn_count_;   //!< @brief continuous count over usage_warn_ to generate warning
  int usage_error_count_;  //!< @brief continuous count over usage_error_ to generate error
  bool usage_avg_;         //!< @brief Check CPU usage calculated as averages among all processors

  /**
   * @brief CPU temperature status messages
   */
  const std::map<int, const char *> temp_dict_ = {
    {DiagStatus::OK, "OK"}, {DiagStatus::WARN, "warm"}, {DiagStatus::ERROR, "hot"}};

  /**
   * @brief CPU usage status messages
   */
  const std::map<int, const char *> load_dict_ = {
    {DiagStatus::OK, "OK"}, {DiagStatus::WARN, "high load"}, {DiagStatus::ERROR, "very high load"}};

  /**
   * @brief CPU thermal throttling status messages
   */
  const std::map<int, const char *> thermal_dict_ = {
    {DiagStatus::OK, "OK"}, {DiagStatus::WARN, "unused"}, {DiagStatus::ERROR, "throttling"}};

  // Publisher
  rclcpp::Publisher<tier4_external_api_msgs::msg::CpuUsage>::SharedPtr pub_cpu_usage_;

  virtual void publishCpuUsage(tier4_external_api_msgs::msg::CpuUsage usage);
};

#endif  // SYSTEM_MONITOR__CPU_MONITOR__CPU_MONITOR_BASE_HPP_
