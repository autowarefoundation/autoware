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
struct HddParam
{
  std::string part_device_;                 //!< @brief partition device
  std::string disk_device_;                 //!< @brief disk device
  float temp_warn_;                         //!< @brief HDD temperature(DegC) to generate warning
  float temp_error_;                        //!< @brief HDD temperature(DegC) to generate error
  int power_on_hours_warn_;                 //!< @brief HDD power on hours to generate warning
  uint64_t total_data_written_warn_;        //!< @brief HDD total data written to generate warning
  float total_data_written_safety_factor_;  //!< @brief safety factor of HDD total data written
  int recovered_error_warn_;    //!< @brief HDD recovered error count to generate warning
  int free_warn_;               //!< @brief HDD free space(MB) to generate warning
  int free_error_;              //!< @brief HDD free space(MB) to generate error
  float read_data_rate_warn_;   //!< @brief HDD data rate(MB/s) of read to generate warning
  float write_data_rate_warn_;  //!< @brief HDD data rate(MB/s) of write to generate warning
  float read_iops_warn_;        //!< @brief HDD IOPS of read to generate warning
  float write_iops_warn_;       //!< @brief HDD IOPS of write to generate warning
  uint8_t temp_attribute_id_;   //!< @brief S.M.A.R.T attribute ID of temperature
  uint8_t power_on_hours_attribute_id_;  //!< @brief S.M.A.R.T attribute ID of power on hours
  uint8_t
    total_data_written_attribute_id_;     //!< @brief S.M.A.R.T attribute ID of total data written
  uint8_t recovered_error_attribute_id_;  //!< @brief S.M.A.R.T attribute ID of recovered error

  HddParam()
  : temp_warn_(55.0),
    temp_error_(70.0),
    power_on_hours_warn_(3000000),
    total_data_written_warn_(4915200),
    total_data_written_safety_factor_(0.05),
    recovered_error_warn_(1),
    free_warn_(5120),
    free_error_(100),
    read_data_rate_warn_(360.0),
    write_data_rate_warn_(103.5),
    read_iops_warn_(63360.0),
    write_iops_warn_(24120.0),
    temp_attribute_id_(0xC2),
    power_on_hours_attribute_id_(0x09),
    total_data_written_attribute_id_(0xF1),
    recovered_error_attribute_id_(0xC3)
  {
  }
};

/**
 * @brief statistics of sysfs device
 */
struct SysfsDevStat
{
  uint64_t rd_ios_;      //!< @brief number of read operations issued to the device
  uint64_t rd_sectors_;  //!< @brief number of sectors read
  uint64_t wr_ios_;      //!< @brief number of write operations issued to the device
  uint64_t wr_sectors_;  //!< @brief number of sectors written

  SysfsDevStat() : rd_ios_(0), rd_sectors_(0), wr_ios_(0), wr_sectors_(0) {}
};

/**
 * @brief statistics of HDD
 */
struct HddStat
{
  std::string device_;                //!< @brief device
  std::string error_str_;             //!< @brief error string
  float read_data_rate_MBs_;          //!< @brief data rate of read (MB/s)
  float write_data_rate_MBs_;         //!< @brief data rate of write (MB/s)
  float read_iops_;                   //!< @brief IOPS of read
  float write_iops_;                  //!< @brief IOPS of write
  SysfsDevStat last_sysfs_dev_stat_;  //!< @brief last statistics of sysfs device

  HddStat() : read_data_rate_MBs_(0.0), write_data_rate_MBs_(0.0), read_iops_(0.0), write_iops_(0.0)
  {
  }
};

/**
 * @brief SMART information items to check
 */
enum class HddSmartInfoItem : uint32_t {
  TEMPERATURE = 0,
  POWER_ON_HOURS = 1,
  TOTAL_DATA_WRITTEN = 2,
  RECOVERED_ERROR = 3,
  SIZE
};

/**
 * @brief HDD statistics items to check
 */
enum class HddStatItem : uint32_t {
  READ_DATA_RATE = 0,
  WRITE_DATA_RATE = 1,
  READ_IOPS = 2,
  WRITE_IOPS = 3,
  SIZE
};

class HddMonitor : public rclcpp::Node
{
public:
  /**
   * @brief constructor
   * @param [in] options Options associated with this node.
   */
  explicit HddMonitor(const rclcpp::NodeOptions & options);

protected:
  using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;

  /**
   * @brief check HDD temperature
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void checkSmartTemperature(
    diagnostic_updater::DiagnosticStatusWrapper & stat);  // NOLINT(runtime/references)

  /**
   * @brief check HDD power on hours
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void checkSmartPowerOnHours(
    diagnostic_updater::DiagnosticStatusWrapper & stat);  // NOLINT(runtime/references)

  /**
   * @brief check HDD total data written
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void checkSmartTotalDataWritten(
    diagnostic_updater::DiagnosticStatusWrapper & stat);  // NOLINT(runtime/references)

  /**
   * @brief check HDD recovered error count
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void checkSmartRecoveredError(
    diagnostic_updater::DiagnosticStatusWrapper & stat);  // NOLINT(runtime/references)

  /**
   * @brief check S.M.A.R.T. information
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @param [in] item S.M.A.R.T information item to be checked
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void checkSmart(
    diagnostic_updater::DiagnosticStatusWrapper & stat,
    HddSmartInfoItem item);  // NOLINT(runtime/references)

  /**
   * @brief check HDD usage
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void checkUsage(
    diagnostic_updater::DiagnosticStatusWrapper & stat);  // NOLINT(runtime/references)

  /**
   * @brief check HDD data rate of read
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void checkReadDataRate(
    diagnostic_updater::DiagnosticStatusWrapper & stat);  // NOLINT(runtime/references)

  /**
   * @brief check HDD data rate of write
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void checkWriteDataRate(
    diagnostic_updater::DiagnosticStatusWrapper & stat);  // NOLINT(runtime/references)

  /**
   * @brief check HDD IOPS of read
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void checkReadIops(
    diagnostic_updater::DiagnosticStatusWrapper & stat);  // NOLINT(runtime/references)

  /**
   * @brief check HDD IOPS of write
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void checkWriteIops(
    diagnostic_updater::DiagnosticStatusWrapper & stat);  // NOLINT(runtime/references)

  /**
   * @brief check HDD statistics
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @param [in] item statistic item to be checked
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void checkStatistics(
    diagnostic_updater::DiagnosticStatusWrapper & stat,
    HddStatItem item);  // NOLINT(runtime/references)

  /**
   * @brief check HDD connection
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void checkConnection(
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
  void getHddParams();

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
  void updateHddInfoList();

  /**
   * @brief start HDD transfer measurement
   */
  void startHddTransferMeasurement();

  /**
   * @brief update HDD statistics
   */
  void updateHddStatistics();

  /**
   * @brief get increment value of sysfs device stats per second
   * @param [in] cur_val current value
   * @param [in] last_val last value
   * @param [in] duration_sec duration in seconds
   * @return increment value
   */
  double getIncreaseSysfsDeviceStatValuePerSec(
    uint64_t cur_val, uint64_t last_val, double duration_sec);

  /**
   * @brief read stats for current whole device using /sys/block/ directory
   * @param [in] device device name
   * @param [out] sysfs_dev_stat statistics of sysfs device
   * @return result of success or failure
   */
  int readSysfsDeviceStat(const std::string & device, SysfsDevStat & sysfs_dev_stat);

  /**
   * @brief update HDD connections
   */
  void updateHddConnections();

  /**
   * @brief unmount device
   * @param [in] device device name
   * @return result of success or failure
   */
  int unmountDevice(std::string & device);

  diagnostic_updater::Updater updater_;  //!< @brief Updater class which advertises to /diagnostics
  rclcpp::TimerBase::SharedPtr timer_;   //!< @brief timer to get HDD information from HddReader

  char hostname_[HOST_NAME_MAX + 1];  //!< @brief host name

  int hdd_reader_port_;                         //!< @brief port number to connect to hdd_reader
  std::map<std::string, HddParam> hdd_params_;  //!< @brief list of error and warning levels
  std::map<std::string, bool>
    hdd_connected_flags_;  //!< @brief list of flag whether HDD is connected
  std::map<std::string, uint32_t>
    initial_recovered_errors_;                //!< @brief list of initial recovered error count
  std::map<std::string, HddStat> hdd_stats_;  //!< @brief list of HDD statistics
  //!< @brief diagnostic of connection
  diagnostic_updater::DiagnosticStatusWrapper connect_diag_;
  HddInfoList hdd_info_list_;               //!< @brief list of HDD information
  rclcpp::Time last_hdd_stat_update_time_;  //!< @brief last HDD statistics update time

  /**
   * @brief HDD SMART status messages
   */
  const std::map<int, const char *> smart_dicts_[static_cast<uint32_t>(HddSmartInfoItem::SIZE)] = {
    // temperature
    {{DiagStatus::OK, "OK"}, {DiagStatus::WARN, "hot"}, {DiagStatus::ERROR, "critical hot"}},
    // power on hours
    {{DiagStatus::OK, "OK"}, {DiagStatus::WARN, "lifetime limit"}, {DiagStatus::ERROR, "unused"}},
    // total data written
    {{DiagStatus::OK, "OK"}, {DiagStatus::WARN, "warranty period"}, {DiagStatus::ERROR, "unused"}},
    // recovered error count
    {{DiagStatus::OK, "OK"},
     {DiagStatus::WARN, "high soft error rate"},
     {DiagStatus::ERROR, "unused"}},
  };

  /**
   * @brief HDD usage status messages
   */
  const std::map<int, const char *> usage_dict_ = {
    {DiagStatus::OK, "OK"},
    {DiagStatus::WARN, "low disk space"},
    {DiagStatus::ERROR, "very low disk space"}};

  /**
   * @brief HDD statistics status messages
   */
  const std::map<int, const char *> stat_dicts_[static_cast<uint32_t>(HddStatItem::SIZE)] = {
    // data rate of read
    {{DiagStatus::OK, "OK"},
     {DiagStatus::WARN, "high data rate of read"},
     {DiagStatus::ERROR, "unused"}},
    // data rate of write
    {{DiagStatus::OK, "OK"},
     {DiagStatus::WARN, "high data rate of write"},
     {DiagStatus::ERROR, "unused"}},
    // IOPS of read
    {{DiagStatus::OK, "OK"},
     {DiagStatus::WARN, "high IOPS of read"},
     {DiagStatus::ERROR, "unused"}},
    // IOPS of write
    {{DiagStatus::OK, "OK"},
     {DiagStatus::WARN, "high IOPS of write"},
     {DiagStatus::ERROR, "unused"}},
  };

  /**
   * @brief HDD connection status messages
   */
  const std::map<int, const char *> connection_dict_ = {
    {DiagStatus::OK, "OK"}, {DiagStatus::WARN, "not connected"}, {DiagStatus::ERROR, "unused"}};
};

#endif  // SYSTEM_MONITOR__HDD_MONITOR__HDD_MONITOR_HPP_
