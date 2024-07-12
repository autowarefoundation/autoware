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
 * @file hdd_monitor.cpp
 * @brief HDD monitor class
 */

#include "system_monitor/hdd_monitor/hdd_monitor.hpp"

#include "system_monitor/hdd_reader/hdd_reader.hpp"
#include "system_monitor/system_monitor_utility.hpp"

#include <boost/algorithm/string.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/process.hpp>
#include <boost/serialization/vector.hpp>

#include <fmt/format.h>
#include <stdio.h>

#include <algorithm>
#include <filesystem>
#include <string>
#include <vector>

namespace bp = boost::process;

HddMonitor::HddMonitor(const rclcpp::NodeOptions & options)
: Node("hdd_monitor", options),
  updater_(this),
  hdd_reader_port_(declare_parameter<int>("hdd_reader_port", 7635)),
  last_hdd_stat_update_time_{0, 0, this->get_clock()->get_clock_type()}
{
  using namespace std::literals::chrono_literals;

  gethostname(hostname_, sizeof(hostname_));

  getHddParams();

  updater_.setHardwareID(hostname_);
  updater_.add("HDD Temperature", this, &HddMonitor::checkSmartTemperature);
  updater_.add("HDD PowerOnHours", this, &HddMonitor::checkSmartPowerOnHours);
  updater_.add("HDD TotalDataWritten", this, &HddMonitor::checkSmartTotalDataWritten);
  updater_.add("HDD RecoveredError", this, &HddMonitor::checkSmartRecoveredError);
  updater_.add("HDD Usage", this, &HddMonitor::checkUsage);
  updater_.add("HDD ReadDataRate", this, &HddMonitor::checkReadDataRate);
  updater_.add("HDD WriteDataRate", this, &HddMonitor::checkWriteDataRate);
  updater_.add("HDD ReadIOPS", this, &HddMonitor::checkReadIops);
  updater_.add("HDD WriteIOPS", this, &HddMonitor::checkWriteIops);
  updater_.add("HDD Connection", this, &HddMonitor::checkConnection);

  // get HDD connection status
  updateHddConnections();

  // get HDD information from HDD reader for the first time
  updateHddInfoList();

  // start HDD transfer measurement
  startHddTransferMeasurement();

  timer_ = rclcpp::create_timer(this, get_clock(), 1s, std::bind(&HddMonitor::onTimer, this));
}

void HddMonitor::checkSmartTemperature(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  checkSmart(stat, HddSmartInfoItem::TEMPERATURE);
}

void HddMonitor::checkSmartPowerOnHours(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  checkSmart(stat, HddSmartInfoItem::POWER_ON_HOURS);
}

void HddMonitor::checkSmartTotalDataWritten(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  checkSmart(stat, HddSmartInfoItem::TOTAL_DATA_WRITTEN);
}

void HddMonitor::checkSmartRecoveredError(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  checkSmart(stat, HddSmartInfoItem::RECOVERED_ERROR);
}

void HddMonitor::checkSmart(
  diagnostic_updater::DiagnosticStatusWrapper & stat, HddSmartInfoItem item)
{
  // Remember start time to measure elapsed time
  const auto t_start = SystemMonitorUtility::startMeasurement();

  if (hdd_params_.empty()) {
    stat.summary(DiagStatus::ERROR, "invalid disk parameter");
    return;
  }

  // Return error if connection diagnostic indicates error
  if (connect_diag_.level != DiagStatus::OK) {
    stat.summary(connect_diag_.level, connect_diag_.message);
    for (const auto & e : connect_diag_.values) {
      stat.add(e.key, e.value);
    }
    return;
  }

  int level = DiagStatus::OK;
  int whole_level = DiagStatus::OK;
  int index = 0;
  std::string error_str = "";
  std::string key_str = "";
  std::string val_str = "";

  for (auto itr = hdd_params_.begin(); itr != hdd_params_.end(); ++itr, ++index) {
    if (!hdd_connected_flags_[itr->first]) {
      continue;
    }

    // Retrieve HDD information
    auto hdd_itr = hdd_info_list_.find(itr->second.disk_device_);
    if (hdd_itr == hdd_info_list_.end()) {
      stat.add(fmt::format("HDD {}: status", index), "hdd_reader error");
      stat.add(fmt::format("HDD {}: name", index), itr->second.part_device_.c_str());
      stat.add(fmt::format("HDD {}: hdd_reader", index), strerror(ENOENT));
      error_str = "hdd_reader error";
      continue;
    }

    if (hdd_itr->second.error_code_ != 0) {
      stat.add(fmt::format("HDD {}: status", index), "hdd_reader error");
      stat.add(fmt::format("HDD {}: name", index), itr->second.part_device_.c_str());
      stat.add(fmt::format("HDD {}: hdd_reader", index), strerror(hdd_itr->second.error_code_));
      error_str = "hdd_reader error";
      continue;
    }

    switch (item) {
      case HddSmartInfoItem::TEMPERATURE: {
        float temp = static_cast<float>(hdd_itr->second.temp_);

        level = DiagStatus::OK;
        if (temp >= itr->second.temp_error_) {
          level = DiagStatus::ERROR;
        } else if (temp >= itr->second.temp_warn_) {
          level = DiagStatus::WARN;
        }
        key_str = fmt::format("HDD {}: temperature", index);
        if (hdd_itr->second.is_valid_temp_) {
          val_str = fmt::format("{:.1f} DegC", temp);
        } else {
          val_str = "not available";
        }
      } break;
      case HddSmartInfoItem::POWER_ON_HOURS: {
        int64_t power_on_hours = static_cast<int64_t>(hdd_itr->second.power_on_hours_);

        level = DiagStatus::OK;
        if (power_on_hours >= itr->second.power_on_hours_warn_) {
          level = DiagStatus::WARN;
        }
        key_str = fmt::format("HDD {}: power on hours", index);
        if (hdd_itr->second.is_valid_power_on_hours_) {
          val_str = fmt::format("{} Hours", hdd_itr->second.power_on_hours_);
        } else {
          val_str = "not available";
        }
      } break;
      case HddSmartInfoItem::TOTAL_DATA_WRITTEN: {
        uint64_t total_data_written = static_cast<uint64_t>(hdd_itr->second.total_data_written_);

        level = DiagStatus::OK;
        if (total_data_written >= itr->second.total_data_written_warn_) {
          level = DiagStatus::WARN;
        }
        key_str = fmt::format("HDD {}: total data written", index);
        if (hdd_itr->second.is_valid_total_data_written_) {
          val_str = fmt::format("{}", hdd_itr->second.total_data_written_);
        } else {
          val_str = "not available";
        }
      } break;
      case HddSmartInfoItem::RECOVERED_ERROR: {
        int32_t recovered_error = static_cast<int32_t>(hdd_itr->second.recovered_error_);
        if (initial_recovered_errors_.find(itr->first) == initial_recovered_errors_.end()) {
          initial_recovered_errors_[itr->first] = recovered_error;
        }
        recovered_error -= initial_recovered_errors_[itr->first];

        level = DiagStatus::OK;
        if (recovered_error >= itr->second.recovered_error_warn_) {
          level = DiagStatus::WARN;
        }
        key_str = fmt::format("HDD {}: recovered error", index);
        if (hdd_itr->second.is_valid_recovered_error_) {
          val_str = fmt::format("{}", hdd_itr->second.recovered_error_);
        } else {
          val_str = "not available";
        }
      } break;
      default:
        break;
    }

    stat.add(
      fmt::format("HDD {}: status", index), smart_dicts_[static_cast<uint32_t>(item)].at(level));
    stat.add(fmt::format("HDD {}: name", index), itr->second.disk_device_.c_str());
    stat.add(fmt::format("HDD {}: model", index), hdd_itr->second.model_.c_str());
    stat.add(fmt::format("HDD {}: serial", index), hdd_itr->second.serial_.c_str());
    stat.addf(key_str, val_str.c_str());

    whole_level = std::max(whole_level, level);
  }

  if (!error_str.empty()) {
    stat.summary(DiagStatus::ERROR, error_str);
  } else {
    stat.summary(whole_level, smart_dicts_[static_cast<uint32_t>(item)].at(whole_level));
  }

  // Measure elapsed time since start time and report
  SystemMonitorUtility::stopMeasurement(t_start, stat);
}

void HddMonitor::checkUsage(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  // Remember start time to measure elapsed time
  const auto t_start = SystemMonitorUtility::startMeasurement();

  if (hdd_params_.empty()) {
    stat.summary(DiagStatus::ERROR, "invalid disk parameter");
    return;
  }

  int hdd_index = 0;
  int whole_level = DiagStatus::OK;
  std::string error_str = "";

  for (auto itr = hdd_params_.begin(); itr != hdd_params_.end(); ++itr, ++hdd_index) {
    if (!hdd_connected_flags_[itr->first]) {
      continue;
    }

    // Get summary of disk space usage of ext4

    // boost::process create file descriptor without O_CLOEXEC required for multithreading.
    // So create file descriptor with O_CLOEXEC and pass it to boost::process.
    int out_fd[2];
    if (pipe2(out_fd, O_CLOEXEC) != 0) {
      error_str = "pipe2 error";
      stat.add(fmt::format("HDD {}: status", hdd_index), "pipe2 error");
      stat.add(fmt::format("HDD {}: name", hdd_index), itr->first.c_str());
      stat.add(fmt::format("HDD {}: pipe2", hdd_index), strerror(errno));
      continue;
    }
    bp::pipe out_pipe{out_fd[0], out_fd[1]};
    bp::ipstream is_out{std::move(out_pipe)};

    int err_fd[2];
    if (pipe2(err_fd, O_CLOEXEC) != 0) {
      error_str = "pipe2 error";
      stat.add(fmt::format("HDD {}: status", hdd_index), "pipe2 error");
      stat.add(fmt::format("HDD {}: name", hdd_index), itr->first.c_str());
      stat.add(fmt::format("HDD {}: pipe2", hdd_index), strerror(errno));
      continue;
    }
    bp::pipe err_pipe{err_fd[0], err_fd[1]};
    bp::ipstream is_err{std::move(err_pipe)};

    // Invoke shell to use shell wildcard expansion
    bp::child c(
      "/bin/sh", "-c", fmt::format("df -Pm {}*", itr->second.part_device_.c_str()),
      bp::std_out > is_out, bp::std_err > is_err);
    c.wait();

    if (c.exit_code() != 0) {
      std::ostringstream os;
      is_err >> os.rdbuf();
      error_str = "df error";
      stat.add(fmt::format("HDD {}: status", hdd_index), "df error");
      stat.add(fmt::format("HDD {}: name", hdd_index), itr->second.part_device_.c_str());
      stat.add(fmt::format("HDD {}: df", hdd_index), os.str().c_str());
      continue;
    }

    int level = DiagStatus::OK;
    std::string line;
    int index = 0;
    std::vector<std::string> list;
    int avail;

    while (std::getline(is_out, line) && !line.empty()) {
      // Skip header
      if (index <= 0) {
        ++index;
        continue;
      }

      boost::split(list, line, boost::is_space(), boost::token_compress_on);

      try {
        avail = std::stoi(list[3].c_str());
      } catch (std::exception & e) {
        avail = -1;
        error_str = e.what();
        stat.add(fmt::format("HDD {}: status", hdd_index), "avail string error");
      }

      if (avail <= itr->second.free_error_) {
        level = DiagStatus::ERROR;
      } else if (avail <= itr->second.free_warn_) {
        level = DiagStatus::WARN;
      } else {
        level = DiagStatus::OK;
      }

      stat.add(fmt::format("HDD {}: status", hdd_index), usage_dict_.at(level));
      stat.add(fmt::format("HDD {}: filesystem", hdd_index), list[0].c_str());
      stat.add(fmt::format("HDD {}: size", hdd_index), (list[1] + " MiB").c_str());
      stat.add(fmt::format("HDD {}: used", hdd_index), (list[2] + " MiB").c_str());
      stat.add(fmt::format("HDD {}: avail", hdd_index), (list[3] + " MiB").c_str());
      stat.add(fmt::format("HDD {}: use", hdd_index), list[4].c_str());
      std::string mounted_ = list[5];
      if (list.size() > 6) {
        std::string::size_type pos = line.find("% /");
        if (pos != std::string::npos) {
          mounted_ = line.substr(pos + 2);  // 2 is "% " length
        }
      }
      stat.add(fmt::format("HDD {}: mounted on", hdd_index), mounted_.c_str());

      whole_level = std::max(whole_level, level);
      ++index;
    }
  }

  if (!error_str.empty()) {
    stat.summary(DiagStatus::ERROR, error_str);
  } else {
    stat.summary(whole_level, usage_dict_.at(whole_level));
  }

  // Measure elapsed time since start time and report
  SystemMonitorUtility::stopMeasurement(t_start, stat);
}

void HddMonitor::checkReadDataRate(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  checkStatistics(stat, HddStatItem::READ_DATA_RATE);
}

void HddMonitor::checkWriteDataRate(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  checkStatistics(stat, HddStatItem::WRITE_DATA_RATE);
}

void HddMonitor::checkReadIops(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  checkStatistics(stat, HddStatItem::READ_IOPS);
}

void HddMonitor::checkWriteIops(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  checkStatistics(stat, HddStatItem::WRITE_IOPS);
}

void HddMonitor::checkStatistics(
  diagnostic_updater::DiagnosticStatusWrapper & stat, HddStatItem item)
{
  // Remember start time to measure elapsed time
  const auto t_start = SystemMonitorUtility::startMeasurement();

  if (hdd_params_.empty()) {
    stat.summary(DiagStatus::ERROR, "invalid disk parameter");
    return;
  }

  int hdd_index = 0;
  int whole_level = DiagStatus::OK;
  std::string error_str = "";
  std::string key_str = "";
  std::string val_str = "";

  for (auto itr = hdd_params_.begin(); itr != hdd_params_.end(); ++itr, ++hdd_index) {
    if (!hdd_connected_flags_[itr->first]) {
      continue;
    }

    int level = DiagStatus::OK;

    switch (item) {
      case HddStatItem::READ_DATA_RATE: {
        float read_data_rate = hdd_stats_[itr->first].read_data_rate_MBs_;

        if (read_data_rate >= itr->second.read_data_rate_warn_) {
          level = DiagStatus::WARN;
        }
        key_str = fmt::format("HDD {}: data rate of read", hdd_index);
        val_str = fmt::format("{:.2f} MB/s", read_data_rate);
      } break;
      case HddStatItem::WRITE_DATA_RATE: {
        float write_data_rate = hdd_stats_[itr->first].write_data_rate_MBs_;

        if (write_data_rate >= itr->second.write_data_rate_warn_) {
          level = DiagStatus::WARN;
        }
        key_str = fmt::format("HDD {}: data rate of write", hdd_index);
        val_str = fmt::format("{:.2f} MB/s", write_data_rate);
      } break;
      case HddStatItem::READ_IOPS: {
        float read_iops = hdd_stats_[itr->first].read_iops_;

        if (read_iops >= itr->second.read_iops_warn_) {
          level = DiagStatus::WARN;
        }
        key_str = fmt::format("HDD {}: IOPS of read", hdd_index);
        val_str = fmt::format("{:.2f} IOPS", read_iops);
      } break;
      case HddStatItem::WRITE_IOPS: {
        float write_iops = hdd_stats_[itr->first].write_iops_;

        if (write_iops >= itr->second.write_iops_warn_) {
          level = DiagStatus::WARN;
        }
        key_str = fmt::format("HDD {}: IOPS of write", hdd_index);
        val_str = fmt::format("{:.2f} IOPS", write_iops);
      } break;
      default:
        break;
    }

    if (!hdd_stats_[itr->first].error_str_.empty()) {
      error_str = hdd_stats_[itr->first].error_str_;
      stat.add(fmt::format("HDD {}: status", hdd_index), error_str);
      stat.add(fmt::format("HDD {}: name", hdd_index), itr->second.disk_device_.c_str());
    } else {
      stat.add(
        fmt::format("HDD {}: status", hdd_index),
        stat_dicts_[static_cast<uint32_t>(item)].at(level));
      stat.add(fmt::format("HDD {}: name", hdd_index), itr->second.disk_device_.c_str());
      stat.add(key_str, val_str.c_str());
    }

    whole_level = std::max(whole_level, level);
  }

  if (!error_str.empty()) {
    stat.summary(DiagStatus::ERROR, error_str);
  } else {
    stat.summary(whole_level, stat_dicts_[static_cast<uint32_t>(item)].at(whole_level));
  }

  // Measure elapsed time since start time and report
  SystemMonitorUtility::stopMeasurement(t_start, stat);
}

void HddMonitor::checkConnection(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  // Remember start time to measure elapsed time
  const auto t_start = SystemMonitorUtility::startMeasurement();

  if (hdd_params_.empty()) {
    stat.summary(DiagStatus::ERROR, "invalid disk parameter");
    return;
  }

  int hdd_index = 0;
  int whole_level = DiagStatus::OK;

  for (auto itr = hdd_params_.begin(); itr != hdd_params_.end(); ++itr, ++hdd_index) {
    int level = DiagStatus::OK;

    if (!hdd_connected_flags_[itr->first]) {
      level = DiagStatus::WARN;
    }

    stat.add(fmt::format("HDD {}: status", hdd_index), connection_dict_.at(level));
    stat.add(fmt::format("HDD {}: name", hdd_index), itr->second.disk_device_);
    stat.add(fmt::format("HDD {}: mount point", hdd_index), itr->first.c_str());

    whole_level = std::max(whole_level, level);
  }

  stat.summary(whole_level, connection_dict_.at(whole_level));

  // Measure elapsed time since start time and report
  SystemMonitorUtility::stopMeasurement(t_start, stat);
}

void HddMonitor::getHddParams()
{
  const auto num_disks = this->declare_parameter("num_disks", 0);
  for (auto i = 0; i < num_disks; ++i) {
    const auto prefix = "disks.disk" + std::to_string(i);
    const auto mount_point = declare_parameter<std::string>(prefix + ".name", "/");

    HddParam param;
    param.temp_warn_ = declare_parameter<float>(prefix + ".temp_warn", 55.0f);
    param.temp_error_ = declare_parameter<float>(prefix + ".temp_error", 70.0f);
    param.power_on_hours_warn_ = declare_parameter<int>(prefix + ".power_on_hours_warn", 3000000);
    param.total_data_written_safety_factor_ =
      declare_parameter<float>(prefix + ".total_data_written_safety_factor", 0.05f);
    int64_t total_data_written_warn_org =
      declare_parameter<int64_t>(prefix + ".total_data_written_warn", 4915200);
    param.total_data_written_warn_ = static_cast<uint64_t>(
      total_data_written_warn_org * (1.0f - param.total_data_written_safety_factor_));
    param.recovered_error_warn_ = declare_parameter<int>(prefix + ".recovered_error_warn", 1);
    param.free_warn_ = declare_parameter<int>(prefix + ".free_warn", 5120);
    param.free_error_ = declare_parameter<int>(prefix + ".free_error", 100);
    param.read_data_rate_warn_ = declare_parameter<float>(prefix + ".read_data_rate_warn", 360.0);
    param.write_data_rate_warn_ = declare_parameter<float>(prefix + ".write_data_rate_warn", 103.5);
    param.read_iops_warn_ = declare_parameter<float>(prefix + ".read_iops_warn", 63360.0);
    param.write_iops_warn_ = declare_parameter<float>(prefix + ".write_iops_warn", 24120.0);
    param.temp_attribute_id_ =
      static_cast<uint8_t>(declare_parameter<int>(prefix + ".temp_attribute_id", 0xC2));
    param.power_on_hours_attribute_id_ =
      static_cast<uint8_t>(declare_parameter<int>(prefix + ".power_on_hours_attribute_id", 0x09));
    param.total_data_written_attribute_id_ = static_cast<uint8_t>(
      declare_parameter<int>(prefix + ".total_data_written_attribute_id", 0xF1));
    param.recovered_error_attribute_id_ =
      static_cast<uint8_t>(declare_parameter<int>(prefix + ".recovered_error_attribute_id", 0xC3));

    hdd_params_[mount_point] = param;

    HddStat stat;
    hdd_stats_[mount_point] = stat;
  }
}

std::string HddMonitor::getDeviceFromMountPoint(const std::string & mount_point)
{
  std::string ret;

  // boost::process create file descriptor without O_CLOEXEC required for multithreading.
  // So create file descriptor with O_CLOEXEC and pass it to boost::process.
  int out_fd[2];
  if (pipe2(out_fd, O_CLOEXEC) != 0) {
    RCLCPP_ERROR(get_logger(), "Failed to execute pipe2. %s", strerror(errno));
    return "";
  }
  bp::pipe out_pipe{out_fd[0], out_fd[1]};
  bp::ipstream is_out{std::move(out_pipe)};

  int err_fd[2];
  if (pipe2(err_fd, O_CLOEXEC) != 0) {
    RCLCPP_ERROR(get_logger(), "Failed to execute pipe2. %s", strerror(errno));
    return "";
  }
  bp::pipe err_pipe{err_fd[0], err_fd[1]};
  bp::ipstream is_err{std::move(err_pipe)};

  bp::child c(
    "/bin/sh", "-c", fmt::format("findmnt -n -o SOURCE {}", mount_point.c_str()),
    bp::std_out > is_out, bp::std_err > is_err);
  c.wait();

  if (c.exit_code() != 0) {
    RCLCPP_ERROR(get_logger(), "Failed to execute findmnt. %s", mount_point.c_str());
    return "";
  }

  if (!std::getline(is_out, ret)) {
    RCLCPP_ERROR(get_logger(), "Failed to find device name. %s", mount_point.c_str());
    return "";
  }

  return ret;
}

void HddMonitor::onTimer()
{
  updateHddConnections();
  updateHddInfoList();
  updateHddStatistics();
}

void HddMonitor::updateHddInfoList()
{
  // Clear diagnostic of connection
  connect_diag_.clear();
  connect_diag_.clearSummary();

  // Create a new socket
  int sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock < 0) {
    connect_diag_.summary(DiagStatus::ERROR, "socket error");
    connect_diag_.add("socket", strerror(errno));
    return;
  }

  // Specify the receiving timeouts until reporting an error
  struct timeval tv;
  tv.tv_sec = 10;
  tv.tv_usec = 0;
  int ret = setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  if (ret < 0) {
    connect_diag_.summary(DiagStatus::ERROR, "setsockopt error");
    connect_diag_.add("setsockopt", strerror(errno));
    close(sock);
    return;
  }

  // Connect the socket referred to by the file descriptor
  sockaddr_in addr;
  memset(&addr, 0, sizeof(sockaddr_in));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(hdd_reader_port_);
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  // cppcheck-suppress cstyleCast
  ret = connect(sock, (struct sockaddr *)&addr, sizeof(addr));
  if (ret < 0) {
    connect_diag_.summary(DiagStatus::ERROR, "connect error");
    connect_diag_.add("connect", strerror(errno));
    close(sock);
    return;
  }

  uint8_t request_id = HddReaderRequestId::GetHddInfo;
  std::vector<HddDevice> hdd_devices;
  for (auto itr = hdd_params_.begin(); itr != hdd_params_.end(); ++itr) {
    if (!hdd_connected_flags_[itr->first]) {
      continue;
    }

    HddDevice device;
    device.name_ = itr->second.disk_device_;
    device.temp_attribute_id_ = itr->second.temp_attribute_id_;
    device.power_on_hours_attribute_id_ = itr->second.power_on_hours_attribute_id_;
    device.total_data_written_attribute_id_ = itr->second.total_data_written_attribute_id_;
    device.recovered_error_attribute_id_ = itr->second.recovered_error_attribute_id_;

    hdd_devices.push_back(device);
  }

  std::ostringstream oss;
  boost::archive::text_oarchive oa(oss);
  oa & request_id;
  oa & hdd_devices;

  // Write list of devices to FD
  ret = write(sock, oss.str().c_str(), oss.str().length());
  if (ret < 0) {
    connect_diag_.summary(DiagStatus::ERROR, "write error");
    connect_diag_.add("write", strerror(errno));
    RCLCPP_ERROR(get_logger(), "write error");
    close(sock);
    return;
  }

  // Receive messages from a socket
  char buf[1024] = "";
  ret = recv(sock, buf, sizeof(buf) - 1, 0);
  if (ret < 0) {
    connect_diag_.summary(DiagStatus::ERROR, "recv error");
    connect_diag_.add("recv", strerror(errno));
    close(sock);
    return;
  }
  // No data received
  if (ret == 0) {
    connect_diag_.summary(DiagStatus::ERROR, "recv error");
    connect_diag_.add("recv", "No data received");
    close(sock);
    return;
  }

  // Close the file descriptor FD
  ret = close(sock);
  if (ret < 0) {
    connect_diag_.summary(DiagStatus::ERROR, "close error");
    connect_diag_.add("close", strerror(errno));
    return;
  }

  // Restore HDD information list
  try {
    std::istringstream iss(buf);
    boost::archive::text_iarchive ia(iss);
    ia >> hdd_info_list_;
  } catch (const std::exception & e) {
    connect_diag_.summary(DiagStatus::ERROR, "recv error");
    connect_diag_.add("recv", e.what());
    return;
  }
}

void HddMonitor::startHddTransferMeasurement()
{
  for (auto & hdd_stat : hdd_stats_) {
    hdd_stat.second.error_str_ = "";

    if (!hdd_connected_flags_[hdd_stat.first]) {
      continue;
    }

    SysfsDevStat sysfs_dev_stat;
    if (readSysfsDeviceStat(hdd_stat.second.device_, sysfs_dev_stat)) {
      hdd_stat.second.error_str_ = "stat file read error";
      continue;
    }
    hdd_stat.second.last_sysfs_dev_stat_ = sysfs_dev_stat;
  }

  last_hdd_stat_update_time_ = this->now();
}

void HddMonitor::updateHddStatistics()
{
  double duration_sec = (this->now() - last_hdd_stat_update_time_).seconds();

  for (auto & hdd_stat : hdd_stats_) {
    hdd_stat.second.error_str_ = "";

    if (!hdd_connected_flags_[hdd_stat.first]) {
      continue;
    }

    SysfsDevStat sysfs_dev_stat;
    if (readSysfsDeviceStat(hdd_stat.second.device_, sysfs_dev_stat)) {
      hdd_stat.second.error_str_ = "stat file read error";
      continue;
    }

    SysfsDevStat & last_sysfs_dev_stat = hdd_stat.second.last_sysfs_dev_stat_;

    hdd_stat.second.read_data_rate_MBs_ = getIncreaseSysfsDeviceStatValuePerSec(
      sysfs_dev_stat.rd_sectors_, last_sysfs_dev_stat.rd_sectors_, duration_sec);
    hdd_stat.second.read_data_rate_MBs_ /= 2048;
    hdd_stat.second.write_data_rate_MBs_ = getIncreaseSysfsDeviceStatValuePerSec(
      sysfs_dev_stat.wr_sectors_, last_sysfs_dev_stat.wr_sectors_, duration_sec);
    hdd_stat.second.write_data_rate_MBs_ /= 2048;
    hdd_stat.second.read_iops_ = getIncreaseSysfsDeviceStatValuePerSec(
      sysfs_dev_stat.rd_ios_, last_sysfs_dev_stat.rd_ios_, duration_sec);
    hdd_stat.second.write_iops_ = getIncreaseSysfsDeviceStatValuePerSec(
      sysfs_dev_stat.wr_ios_, last_sysfs_dev_stat.wr_ios_, duration_sec);

    hdd_stat.second.last_sysfs_dev_stat_ = sysfs_dev_stat;
  }

  last_hdd_stat_update_time_ = this->now();
}

double HddMonitor::getIncreaseSysfsDeviceStatValuePerSec(
  uint64_t cur_val, uint64_t last_val, double duration_sec)
{
  if (cur_val > last_val && duration_sec > 0.0) {
    return static_cast<double>(cur_val - last_val) / duration_sec;
  }
  return 0.0;
}

int HddMonitor::readSysfsDeviceStat(const std::string & device, SysfsDevStat & sysfs_dev_stat)
{
  int ret = -1;
  unsigned int ios_pgr, tot_ticks, rq_ticks, wr_ticks;
  uint64_t rd_ios, rd_merges_or_rd_sec, wr_ios, wr_merges;
  uint64_t rd_sec_or_wr_ios, wr_sec, rd_ticks_or_wr_sec;
  uint64_t dc_ios, dc_merges, dc_sec, dc_ticks;

  std::string filename("/sys/block/");
  filename += device + "/stat";
  FILE * fp = fopen(filename.c_str(), "r");
  if (fp == NULL) {
    return ret;
  }

  int i = fscanf(
    fp, "%lu %lu %lu %lu %lu %lu %lu %u %u %u %u %lu %lu %lu %lu", &rd_ios, &rd_merges_or_rd_sec,
    &rd_sec_or_wr_ios, &rd_ticks_or_wr_sec, &wr_ios, &wr_merges, &wr_sec, &wr_ticks, &ios_pgr,
    &tot_ticks, &rq_ticks, &dc_ios, &dc_merges, &dc_sec, &dc_ticks);

  if (i >= 7) {
    sysfs_dev_stat.rd_ios_ = rd_ios;
    sysfs_dev_stat.rd_sectors_ = rd_sec_or_wr_ios;
    sysfs_dev_stat.wr_ios_ = wr_ios;
    sysfs_dev_stat.wr_sectors_ = wr_sec;
    ret = 0;
  }

  fclose(fp);
  return ret;
}

void HddMonitor::updateHddConnections()
{
  for (auto & hdd_param : hdd_params_) {
    hdd_connected_flags_[hdd_param.first] = false;

    // Get device name from mount point
    hdd_param.second.part_device_ = getDeviceFromMountPoint(hdd_param.first);
    if (!hdd_param.second.part_device_.empty()) {
      // Check the existence of device file
      std::error_code ec;
      if (std::filesystem::exists(hdd_param.second.part_device_, ec)) {
        hdd_connected_flags_[hdd_param.first] = true;

        // Remove index number of partition for passing device name to hdd-reader
        if (boost::starts_with(hdd_param.second.part_device_, "/dev/sd")) {
          const std::regex pattern("\\d+$");
          hdd_param.second.disk_device_ =
            std::regex_replace(hdd_param.second.part_device_, pattern, "");
        } else if (boost::starts_with(hdd_param.second.part_device_, "/dev/nvme")) {
          const std::regex pattern("p\\d+$");
          hdd_param.second.disk_device_ =
            std::regex_replace(hdd_param.second.part_device_, pattern, "");
        }

        const std::regex raw_pattern(".*/");
        hdd_stats_[hdd_param.first].device_ =
          std::regex_replace(hdd_param.second.disk_device_, raw_pattern, "");
      } else {
        // Deal with the issue that file system remains mounted when a drive is actually
        // disconnected.
        if (unmountDevice(hdd_param.second.part_device_)) {
          RCLCPP_ERROR(
            get_logger(), "Failed to unmount device : %s", hdd_param.second.part_device_.c_str());
        }
      }
    }
  }
}

int HddMonitor::unmountDevice(std::string & device)
{
  // Create a new socket
  int sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock < 0) {
    RCLCPP_ERROR(get_logger(), "socket create error. %s", strerror(errno));
    return -1;
  }

  // Specify the receiving timeouts until reporting an error
  struct timeval tv;
  tv.tv_sec = 10;
  tv.tv_usec = 0;
  int ret = setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  if (ret < 0) {
    RCLCPP_ERROR(get_logger(), "setsockopt error. %s", strerror(errno));
    close(sock);
    return -1;
  }

  // Connect the socket referred to by the file descriptor
  sockaddr_in addr;
  memset(&addr, 0, sizeof(sockaddr_in));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(hdd_reader_port_);
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  // cppcheck-suppress cstyleCast
  ret = connect(sock, (struct sockaddr *)&addr, sizeof(addr));
  if (ret < 0) {
    RCLCPP_ERROR(get_logger(), "socket connect error. %s", strerror(errno));
    close(sock);
    return -1;
  }

  uint8_t request_id = HddReaderRequestId::UnmountDevice;
  std::vector<UnmountDeviceInfo> umount_dev_infos;
  UnmountDeviceInfo dev_info;

  dev_info.part_device_ = device;
  umount_dev_infos.push_back(dev_info);

  std::ostringstream oss;
  boost::archive::text_oarchive oa(oss);
  oa & request_id;
  oa & umount_dev_infos;

  // Write list of devices to FD
  ret = write(sock, oss.str().c_str(), oss.str().length());
  if (ret < 0) {
    RCLCPP_ERROR(get_logger(), "socket write error. %s", strerror(errno));
    close(sock);
    return -1;
  }

  // Receive messages from a socket
  char buf[1024] = "";
  ret = recv(sock, buf, sizeof(buf) - 1, 0);
  if (ret < 0) {
    RCLCPP_ERROR(get_logger(), "socket recv error. %s", strerror(errno));
    close(sock);
    return -1;
  }
  // No data received
  if (ret == 0) {
    RCLCPP_ERROR(get_logger(), "no data received from hdd_reader.");
    close(sock);
    return -1;
  }

  // Close the file descriptor FD
  ret = close(sock);
  if (ret < 0) {
    RCLCPP_ERROR(get_logger(), "socket close error. %s", strerror(errno));
    return -1;
  }

  std::vector<int> responses;

  // Restore responses
  try {
    std::istringstream iss(buf);
    boost::archive::text_iarchive ia(iss);
    ia >> responses;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "restore responses exception. %s", e.what());
    return -1;
  }
  if (responses.empty()) {
    RCLCPP_ERROR(get_logger(), "responses from hdd_reader is empty.");
    return -1;
  }
  return responses[0];
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(HddMonitor)
