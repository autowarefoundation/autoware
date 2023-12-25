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
 * @file _cpu_monitor.cpp
 * @brief  CPU monitor class
 */

#include "system_monitor/cpu_monitor/intel_cpu_monitor.hpp"

#include "system_monitor/msr_reader/msr_reader.hpp"
#include "system_monitor/system_monitor_utility.hpp"

#include <boost/algorithm/string.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/filesystem.hpp>

#include <fmt/format.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <algorithm>
#include <regex>
#include <string>
#include <vector>

namespace fs = boost::filesystem;

CPUMonitor::CPUMonitor(const rclcpp::NodeOptions & options) : CPUMonitorBase("cpu_monitor", options)
{
  msr_reader_port_ = declare_parameter<int>("msr_reader_port", 7634);

  this->getTempNames();
  this->getFreqNames();
}

void CPUMonitor::checkThrottling(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  // Remember start time to measure elapsed time
  const auto t_start = SystemMonitorUtility::startMeasurement();

  // Create a new socket
  int sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock < 0) {
    stat.summary(DiagStatus::ERROR, "socket error");
    stat.add("socket", strerror(errno));
    return;
  }

  // Specify the receiving timeouts until reporting an error
  struct timeval tv;
  tv.tv_sec = 10;
  tv.tv_usec = 0;
  int ret = setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  if (ret < 0) {
    stat.summary(DiagStatus::ERROR, "setsockopt error");
    stat.add("setsockopt", strerror(errno));
    close(sock);
    return;
  }

  // Connect the socket referred to by the file descriptor
  sockaddr_in addr;
  memset(&addr, 0, sizeof(sockaddr_in));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(msr_reader_port_);
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  ret = connect(sock, (struct sockaddr *)&addr, sizeof(addr));
  if (ret < 0) {
    stat.summary(DiagStatus::ERROR, "connect error");
    stat.add("connect", strerror(errno));
    close(sock);
    return;
  }

  // Receive messages from a socket
  char buf[1024] = "";
  ret = recv(sock, buf, sizeof(buf) - 1, 0);
  if (ret < 0) {
    stat.summary(DiagStatus::ERROR, "recv error");
    stat.add("recv", strerror(errno));
    close(sock);
    return;
  }
  // No data received
  if (ret == 0) {
    stat.summary(DiagStatus::ERROR, "recv error");
    stat.add("recv", "No data received");
    close(sock);
    return;
  }

  // Close the file descriptor FD
  ret = close(sock);
  if (ret < 0) {
    stat.summary(DiagStatus::ERROR, "close error");
    stat.add("close", strerror(errno));
    return;
  }

  // Restore MSR information
  MSRInfo info;

  try {
    std::istringstream iss(buf);
    boost::archive::text_iarchive oa(iss);
    oa >> info;
  } catch (const std::exception & e) {
    stat.summary(DiagStatus::ERROR, "recv error");
    stat.add("recv", e.what());
    return;
  }

  // msr_reader returns an error
  if (info.error_code_ != 0) {
    stat.summary(DiagStatus::ERROR, "msr_reader error");
    stat.add("msr_reader", strerror(info.error_code_));
    return;
  }

  int level = DiagStatus::OK;
  int whole_level = DiagStatus::OK;
  int index = 0;

  for (auto itr = info.pkg_thermal_status_.begin(); itr != info.pkg_thermal_status_.end();
       ++itr, ++index) {
    if (*itr) {
      level = DiagStatus::ERROR;
    } else {
      level = DiagStatus::OK;
    }

    stat.add(fmt::format("CPU {}: Pkg Thermal Status", index), thermal_dict_.at(level));

    whole_level = std::max(whole_level, level);
  }

  stat.summary(whole_level, thermal_dict_.at(whole_level));

  // Measure elapsed time since start time and report
  SystemMonitorUtility::stopMeasurement(t_start, stat);
}

void CPUMonitor::getTempNames()
{
  const fs::path root("/sys/devices/platform/coretemp.0");

  if (!fs::exists(root)) {
    return;
  }

  for (const fs::path & path : boost::make_iterator_range(
         fs::recursive_directory_iterator(root), fs::recursive_directory_iterator())) {
    if (fs::is_directory(path)) {
      continue;
    }

    std::cmatch match;
    const std::string temp_input = path.generic_string();

    // /sys/devices/platform/coretemp.0/hwmon/hwmon[0-9]/temp[0-9]_input ?
    if (!std::regex_match(temp_input.c_str(), match, std::regex(".*temp(\\d+)_input"))) {
      continue;
    }

    cpu_temp_info temp;
    temp.path_ = temp_input;
    temp.label_ = path.filename().generic_string();

    std::string label = boost::algorithm::replace_all_copy(temp_input, "input", "label");
    const fs::path label_path(label);
    fs::ifstream ifs(label_path, std::ios::in);
    if (ifs) {
      std::string line;
      if (std::getline(ifs, line)) {
        temp.label_ = line;
      }
    }
    ifs.close();
    temps_.push_back(temp);
  }

  std::sort(temps_.begin(), temps_.end(), [](const cpu_temp_info & c1, const cpu_temp_info & c2) {
    std::smatch match;
    const std::regex filter(".*temp(\\d+)_input");
    int n1 = 0;
    int n2 = 0;
    if (std::regex_match(c1.path_, match, filter)) {
      n1 = std::stoi(match[1].str());
    }
    if (std::regex_match(c2.path_, match, filter)) {
      n2 = std::stoi(match[1].str());
    }
    return n1 < n2;
  });  // NOLINT
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(CPUMonitor)
