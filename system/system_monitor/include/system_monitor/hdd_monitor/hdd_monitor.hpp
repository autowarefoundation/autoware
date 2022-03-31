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
 * @file hdd_monitor.h
 * @brief HDD monitor class
 */

#ifndef SYSTEM_MONITOR__HDD_MONITOR__HDD_MONITOR_HPP_
#define SYSTEM_MONITOR__HDD_MONITOR__HDD_MONITOR_HPP_

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <hdd_reader/hdd_reader.hpp>

#include <climits>
#include <map>
#include <string>
#include <vector>

/**
 * @brief error and warning temperature levels
 */
struct HDDParam
{
  std::string device_;                      //!< @brief device
  float temp_warn_;                         //!< @brief HDD temperature(DegC) to generate warning
  float temp_error_;                        //!< @brief HDD temperature(DegC) to generate error
  int power_on_hours_warn_;                 //!< @brief HDD power on hours to generate warning
  uint64_t total_data_written_warn_;        //!< @brief HDD total data written to generate warning
  float total_data_written_safety_factor_;  //!< @brief safety factor of HDD total data written
  int free_warn_;                           //!< @brief HDD free space(MB) to generate warning
  int free_error_;                          //!< @brief HDD free space(MB) to generate error

  HDDParam()
  : temp_warn_(55.0),
    temp_error_(70.0),
    power_on_hours_warn_(3000000),
    total_data_written_warn_(4915200),
    total_data_written_safety_factor_(0.05),
    free_warn_(5120),
    free_error_(100)
  {
  }
};

/**
 * @brief SMART information items to check
 */
enum class HDDSMARTInfoItem : uint32_t {
  TEMPERATURE = 0,
  POWER_ON_HOURS = 1,
  TOTAL_DATA_WRITTEN = 2,
  SIZE
};

class HDDMonitor : public rclcpp::Node
{
public:
  /**
   * @brief constructor
   * @param [in] options Options associated with this node.
   */
  explicit HDDMonitor(const rclcpp::NodeOptions & options);

protected:
  using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;

  /**
   * @brief check HDD temperature
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void checkSMARTTemperature(
    diagnostic_updater::DiagnosticStatusWrapper & stat);  // NOLINT(runtime/references)

  /**
   * @brief check HDD power on hours
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void checkSMARTPowerOnHours(
    diagnostic_updater::DiagnosticStatusWrapper & stat);  // NOLINT(runtime/references)

  /**
   * @brief check HDD total data written
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void checkSMARTTotalDataWritten(
    diagnostic_updater::DiagnosticStatusWrapper & stat);  // NOLINT(runtime/references)

  /**
   * @brief check S.M.A.R.T. information
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void checkSMART(
    diagnostic_updater::DiagnosticStatusWrapper & stat,
    HDDSMARTInfoItem item);  // NOLINT(runtime/references)

  /**
   * @brief check HDD usage
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void checkUsage(
    diagnostic_updater::DiagnosticStatusWrapper & stat);  // NOLINT(runtime/references)

  /**
   * @brief human readable size string to MB
   * @param [in] human readable size string
   * @return Megabyte
   */
  int HumanReadableToMegaByte(const std::string & str);

  /**
   * @brief get HDD parameters
   */
  void getHDDParams();

  /**
   * @brief get device name from mount point
   * @param [in] mount_point mount point
   * @return device name
   */
  std::string getDeviceFromMountPoint(const std::string & mount_point);

  /**
   * @brief timer callback
   */
  void onTimer();

  /**
   * @brief update HDD information list
   */
  void updateHDDInfoList();

  diagnostic_updater::Updater updater_;  //!< @brief Updater class which advertises to /diagnostics
  rclcpp::TimerBase::SharedPtr timer_;   //!< @brief timer to get HDD information from HDDReader

  char hostname_[HOST_NAME_MAX + 1];  //!< @brief host name

  int hdd_reader_port_;                         //!< @brief port number to connect to hdd_reader
  std::map<std::string, HDDParam> hdd_params_;  //!< @brief list of error and warning levels
  std::vector<HDDDevice> hdd_devices_;          //!< @brief list of devices
  //!< @brief diagnostic of connection
  diagnostic_updater::DiagnosticStatusWrapper connect_diag_;
  HDDInfoList hdd_info_list_;  //!< @brief list of HDD information

  /**
   * @brief HDD SMART status messages
   */
  const std::map<int, const char *> smart_dicts_[static_cast<uint32_t>(HDDSMARTInfoItem::SIZE)] = {
    // temperature
    {{DiagStatus::OK, "OK"}, {DiagStatus::WARN, "hot"}, {DiagStatus::ERROR, "critical hot"}},
    // power on hours
    {{DiagStatus::OK, "OK"}, {DiagStatus::WARN, "lifetime limit"}, {DiagStatus::ERROR, "unused"}},
    // total data written
    {{DiagStatus::OK, "OK"}, {DiagStatus::WARN, "warranty period"}, {DiagStatus::ERROR, "unused"}},
  };

  /**
   * @brief HDD usage status messages
   */
  const std::map<int, const char *> usage_dict_ = {
    {DiagStatus::OK, "OK"},
    {DiagStatus::WARN, "low disk space"},
    {DiagStatus::ERROR, "very low disk space"}};
};

#endif  // SYSTEM_MONITOR__HDD_MONITOR__HDD_MONITOR_HPP_
