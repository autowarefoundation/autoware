// Copyright 2022 Autoware Foundation
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
 * @file _voltage_monitor.cpp
 * @brief  voltage monitor class
 */

#include "system_monitor/voltage_monitor/voltage_monitor.hpp"

#include "system_monitor/msr_reader/msr_reader.hpp"
#include "system_monitor/system_monitor_utility.hpp"

#include <boost/algorithm/string.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/filesystem.hpp>
#include <boost/process.hpp>

#include <fmt/format.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <algorithm>
#include <regex>
#include <string>
#include <vector>

namespace bp = boost::process;

VoltageMonitor::VoltageMonitor(const rclcpp::NodeOptions & options)
: Node("voltage_monitor", options), updater_(this), hostname_()
{
  gethostname(hostname_, sizeof(hostname_));

  updater_.setHardwareID(hostname_);
  // Publisher
  rclcpp::QoS durable_qos{1};
  durable_qos.transient_local();

  voltage_string_ = declare_parameter<std::string>("cmos_battery_label", "");
  voltage_warn_ = declare_parameter<float>("cmos_battery_warn", 2.95);
  voltage_error_ = declare_parameter<float>("cmos_battery_error", 2.75);
  bool sensors_exists = false;
  if (voltage_string_ == "") {
    sensors_exists = false;
  } else {
    // Check if command exists
    fs::path p = bp::search_path("sensors");
    sensors_exists = (p.empty()) ? false : true;
  }
  gethostname(hostname_, sizeof(hostname_));
  auto callback = &VoltageMonitor::checkBatteryStatus;
  if (sensors_exists) {
    try {
      std::regex re(R"((\d+).(\d+))");
      voltage_regex_ = re;
    } catch (std::regex_error & e) {
      // never comes here.
      RCLCPP_WARN(get_logger(), "std::regex_error %d", e.code());
      return;
    }
    callback = &VoltageMonitor::checkVoltage;
  }
  updater_.add("CMOS Battery Status", this, callback);
}

void VoltageMonitor::checkVoltage(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  // Remember start time to measure elapsed time
  const auto t_start = SystemMonitorUtility::startMeasurement();
  float voltage = 0.0;

  int out_fd[2];
  if (RCUTILS_UNLIKELY(pipe2(out_fd, O_CLOEXEC) != 0)) {
    stat.summary(DiagStatus::ERROR, "pipe2 error");
    stat.add("pipe2", strerror(errno));
    return;
  }
  bp::pipe out_pipe{out_fd[0], out_fd[1]};
  bp::ipstream is_out{std::move(out_pipe)};

  int err_fd[2];
  if (RCUTILS_UNLIKELY(pipe2(err_fd, O_CLOEXEC) != 0)) {
    stat.summary(DiagStatus::ERROR, "pipe2 error");
    stat.add("pipe2", strerror(errno));
    return;
  }
  bp::pipe err_pipe{err_fd[0], err_fd[1]};
  bp::ipstream is_err{std::move(err_pipe)};

  bp::child c("sensors", bp::std_out > is_out, bp::std_err > is_err);
  c.wait();

  if (RCUTILS_UNLIKELY(c.exit_code() != 0)) {  // failed to execute sensors
    std::ostringstream os;
    is_err >> os.rdbuf();
    stat.summary(DiagStatus::ERROR, "sensors error");
    stat.add("sensors", os.str().c_str());
    return;
  }
  std::string line;
  while (std::getline(is_out, line)) {
    auto voltageStringPos = line.find(voltage_string_.c_str());
    if (voltageStringPos != std::string::npos) {
      try {
        std::smatch match;
        std::regex_search(line, match, voltage_regex_);
        auto voltageString = match.str();
        voltage = std::stof(voltageString);
      } catch (std::regex_error & e) {
        stat.summary(DiagStatus::WARN, "format error");
        stat.add("exception in std::regex_search ", fmt::format("{}", e.code()));
        return;
      }
      break;
    }
  }
  stat.add("CMOS battery voltage", fmt::format("{}", voltage));
  if (voltage < voltage_error_) {
    stat.summary(DiagStatus::WARN, "Battery Died");
  } else if (voltage < voltage_warn_) {
    stat.summary(DiagStatus::WARN, "Low Battery");
  } else {
    stat.summary(DiagStatus::OK, "OK");
  }

  // Measure elapsed time since start time and report
  SystemMonitorUtility::stopMeasurement(t_start, stat);
}

void VoltageMonitor::checkBatteryStatus(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  // Remember start time to measure elapsed time
  const auto t_start = SystemMonitorUtility::startMeasurement();

  // Get status of RTC
  int out_fd[2];
  if (RCUTILS_UNLIKELY(pipe2(out_fd, O_CLOEXEC) != 0)) {
    stat.summary(DiagStatus::ERROR, "pipe2 error");
    stat.add("pipe2", strerror(errno));
    return;
  }
  bp::pipe out_pipe{out_fd[0], out_fd[1]};
  bp::ipstream is_out{std::move(out_pipe)};

  int err_fd[2];
  if (RCUTILS_UNLIKELY(pipe2(err_fd, O_CLOEXEC) != 0)) {
    stat.summary(DiagStatus::ERROR, "pipe2 error");
    stat.add("pipe2", strerror(errno));
    return;
  }
  bp::pipe err_pipe{err_fd[0], err_fd[1]};
  bp::ipstream is_err{std::move(err_pipe)};

  bp::child c("cat /proc/driver/rtc", bp::std_out > is_out, bp::std_err > is_err);
  c.wait();

  if (RCUTILS_UNLIKELY(c.exit_code() != 0)) {
    std::ostringstream os;
    is_err >> os.rdbuf();
    stat.summary(DiagStatus::ERROR, "rtc error");
    stat.add("rtc", os.str().c_str());
    return;
  }

  std::string line;
  bool status = false;
  while (std::getline(is_out, line)) {
    auto batStatusLine = line.find("batt_status");
    if (batStatusLine != std::string::npos) {
      auto batStatus = line.find("okay");
      if (batStatus != std::string::npos) {
        status = true;
        break;
      }
    }
  }

  if (status) {
    stat.add("CMOS battery status", std::string("OK"));
    stat.summary(DiagStatus::OK, "OK");
  } else {
    stat.add("CMOS battery status", std::string("Battery Dead"));
    stat.summary(DiagStatus::WARN, "Battery Dead");
  }

  // Measure elapsed time since start time and report
  SystemMonitorUtility::stopMeasurement(t_start, stat);
}

void VoltageMonitor::update()
{
  updater_.force_update();
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(VoltageMonitor)
