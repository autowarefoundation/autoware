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

#include "system_monitor/system_monitor_utility.hpp"

#include <hdd_reader/hdd_reader.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/process.hpp>
#include <boost/serialization/vector.hpp>

#include <fmt/format.h>

#include <algorithm>
#include <string>
#include <vector>

namespace bp = boost::process;

HDDMonitor::HDDMonitor(const rclcpp::NodeOptions & options)
: Node("hdd_monitor", options),
  updater_(this),
  hdd_reader_port_(declare_parameter<int>("hdd_reader_port", 7635))
{
  using namespace std::literals::chrono_literals;

  gethostname(hostname_, sizeof(hostname_));

  getHDDParams();

  updater_.setHardwareID(hostname_);
  updater_.add("HDD Temperature", this, &HDDMonitor::checkSMARTTemperature);
  updater_.add("HDD PowerOnHours", this, &HDDMonitor::checkSMARTPowerOnHours);
  updater_.add("HDD TotalDataWritten", this, &HDDMonitor::checkSMARTTotalDataWritten);
  updater_.add("HDD Usage", this, &HDDMonitor::checkUsage);

  // get HDD information from HDD reader for the first time
  updateHDDInfoList();

  timer_ = rclcpp::create_timer(this, get_clock(), 1s, std::bind(&HDDMonitor::onTimer, this));
}

void HDDMonitor::checkSMARTTemperature(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  checkSMART(stat, HDDSMARTInfoItem::TEMPERATURE);
}

void HDDMonitor::checkSMARTPowerOnHours(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  checkSMART(stat, HDDSMARTInfoItem::POWER_ON_HOURS);
}

void HDDMonitor::checkSMARTTotalDataWritten(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  checkSMART(stat, HDDSMARTInfoItem::TOTAL_DATA_WRITTEN);
}

void HDDMonitor::checkSMART(
  diagnostic_updater::DiagnosticStatusWrapper & stat, HDDSMARTInfoItem item)
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
    // Retrieve HDD information
    auto hdd_itr = hdd_info_list_.find(itr->second.device_);
    if (hdd_itr == hdd_info_list_.end()) {
      stat.add(fmt::format("HDD {}: status", index), "hdd_reader error");
      stat.add(fmt::format("HDD {}: name", index), itr->first.c_str());
      stat.add(fmt::format("HDD {}: hdd_reader", index), strerror(ENOENT));
      error_str = "hdd_reader error";
      continue;
    }

    if (hdd_itr->second.error_code_ != 0) {
      stat.add(fmt::format("HDD {}: status", index), "hdd_reader error");
      stat.add(fmt::format("HDD {}: name", index), itr->first.c_str());
      stat.add(fmt::format("HDD {}: hdd_reader", index), strerror(hdd_itr->second.error_code_));
      error_str = "hdd_reader error";
      continue;
    }

    switch (item) {
      case HDDSMARTInfoItem::TEMPERATURE: {
        float temp = static_cast<float>(hdd_itr->second.temp_);

        level = DiagStatus::OK;
        if (temp >= itr->second.temp_error_) {
          level = DiagStatus::ERROR;
        } else if (temp >= itr->second.temp_warn_) {
          level = DiagStatus::WARN;
        }
        key_str = fmt::format("HDD {}: temperature", index);
        val_str = fmt::format("{:.1f} DegC", temp);
      } break;
      case HDDSMARTInfoItem::POWER_ON_HOURS: {
        int64_t power_on_hours = static_cast<int64_t>(hdd_itr->second.power_on_hours_);

        level = DiagStatus::OK;
        if (power_on_hours >= itr->second.power_on_hours_warn_) {
          level = DiagStatus::WARN;
        }
        key_str = fmt::format("HDD {}: power on hours", index);
        val_str = fmt::format("{} Hours", hdd_itr->second.power_on_hours_);
      } break;
      case HDDSMARTInfoItem::TOTAL_DATA_WRITTEN: {
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
      default:
        break;
    }

    stat.add(
      fmt::format("HDD {}: status", index), smart_dicts_[static_cast<uint32_t>(item)].at(level));
    stat.add(fmt::format("HDD {}: name", index), itr->second.device_.c_str());
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

void HDDMonitor::checkUsage(diagnostic_updater::DiagnosticStatusWrapper & stat)
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
    // Get summary of disk space usage of ext4
    bp::ipstream is_out;
    bp::ipstream is_err;
    // Invoke shell to use shell wildcard expansion
    bp::child c(
      "/bin/sh", "-c", fmt::format("df -Pm {}*", itr->first.c_str()), bp::std_out > is_out,
      bp::std_err > is_err);
    c.wait();

    if (c.exit_code() != 0) {
      std::ostringstream os;
      is_err >> os.rdbuf();
      error_str = "df error";
      stat.add(fmt::format("HDD {}: status", hdd_index), "df error");
      stat.add(fmt::format("HDD {}: name", hdd_index), itr->first.c_str());
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

void HDDMonitor::getHDDParams()
{
  const auto num_disks = this->declare_parameter("num_disks", 0);
  for (auto i = 0; i < num_disks; ++i) {
    const auto prefix = "disks.disk" + std::to_string(i);
    const auto name = declare_parameter<std::string>(prefix + ".name", "/");

    // Get device name from mount point
    const auto device_name = getDeviceFromMountPoint(name);
    if (device_name.empty()) {
      continue;
    }

    HDDParam param;
    param.temp_warn_ = declare_parameter<float>(prefix + ".temp_warn", 55.0f);
    param.temp_error_ = declare_parameter<float>(prefix + ".temp_error", 70.0f);
    param.power_on_hours_warn_ = declare_parameter<int>(prefix + ".power_on_hours_warn", 3000000);
    param.total_data_written_safety_factor_ =
      declare_parameter<float>(prefix + ".total_data_written_safety_factor", 0.05f);
    int64_t total_data_written_warn_org =
      declare_parameter<int64_t>(prefix + ".total_data_written_warn", 4915200);
    param.total_data_written_warn_ = static_cast<uint64_t>(
      total_data_written_warn_org * (1.0f - param.total_data_written_safety_factor_));
    param.free_warn_ = declare_parameter<int>(prefix + ".free_warn", 5120);
    param.free_error_ = declare_parameter<int>(prefix + ".free_error", 100);

    // Remove index number of partition for passing device name to hdd-reader
    if (boost::starts_with(device_name, "/dev/sd")) {
      const std::regex pattern("\\d+$");
      param.device_ = std::regex_replace(device_name, pattern, "");
    } else if (boost::starts_with(device_name, "/dev/nvme")) {
      const std::regex pattern("p\\d+$");
      param.device_ = std::regex_replace(device_name, pattern, "");
    }
    hdd_params_[device_name] = param;

    HDDDevice device;
    device.name_ = param.device_;
    device.total_data_written_attribute_id_ = static_cast<uint8_t>(
      declare_parameter<int>(prefix + ".total_data_written_attribute_id", 0xF1));
    hdd_devices_.push_back(device);
  }
}

std::string HDDMonitor::getDeviceFromMountPoint(const std::string & mount_point)
{
  std::string ret;
  bp::ipstream is_out;
  bp::ipstream is_err;

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

void HDDMonitor::onTimer() { updateHDDInfoList(); }

void HDDMonitor::updateHDDInfoList()
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
  ret = connect(sock, (struct sockaddr *)&addr, sizeof(addr));
  if (ret < 0) {
    connect_diag_.summary(DiagStatus::ERROR, "connect error");
    connect_diag_.add("connect", strerror(errno));
    close(sock);
    return;
  }

  std::ostringstream oss;
  boost::archive::text_oarchive oa(oss);
  oa & hdd_devices_;

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
    boost::archive::text_iarchive oa(iss);
    oa >> hdd_info_list_;
  } catch (const std::exception & e) {
    connect_diag_.summary(DiagStatus::ERROR, "recv error");
    connect_diag_.add("recv", e.what());
    return;
  }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(HDDMonitor)
