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
 * @file ntp_monitor.cpp
 * @brief NTP monitor class
 */

#include "system_monitor/ntp_monitor/ntp_monitor.hpp"

#include "system_monitor/system_monitor_utility.hpp"

#include <boost/filesystem.hpp>
#include <boost/process.hpp>

#include <fmt/format.h>

#include <map>
#include <regex>
#include <string>

namespace bp = boost::process;
namespace fs = boost::filesystem;

NTPMonitor::NTPMonitor(const rclcpp::NodeOptions & options)
: Node("ntp_monitor", options),
  updater_(this),
  offset_warn_(declare_parameter<float>("offset_warn", 0.1)),
  offset_error_(declare_parameter<float>("offset_error", 5.0))
{
  gethostname(hostname_, sizeof(hostname_));

  // Check if command exists
  fs::path p = bp::search_path("chronyc");
  chronyc_exists_ = (p.empty()) ? false : true;

  updater_.setHardwareID(hostname_);
  updater_.add("NTP Offset", this, &NTPMonitor::checkOffset);
}

void NTPMonitor::update() { updater_.force_update(); }

void NTPMonitor::checkOffset(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  // Remember start time to measure elapsed time
  const auto t_start = SystemMonitorUtility::startMeasurement();

  if (!chronyc_exists_) {
    stat.summary(DiagStatus::ERROR, "chronyc error");
    stat.add(
      "chronyc", "Command 'chronyc' not found, but can be installed with: sudo apt install chrony");
    return;
  }

  std::string error_str;
  std::string pipe2_err_str;
  float offset = 0.0f;
  std::map<std::string, std::string> tracking_map;
  error_str = executeChronyc(offset, tracking_map, pipe2_err_str);
  if (!pipe2_err_str.empty()) {
    stat.summary(DiagStatus::ERROR, "pipe2 error");
    stat.add("pipe2", pipe2_err_str);
    return;
  }
  if (!error_str.empty()) {
    stat.summary(DiagStatus::ERROR, "chronyc error");
    stat.add("chronyc", error_str);
    return;
  }

  int level = DiagStatus::OK;

  // Check an earlier offset as well
  float abs = std::abs(offset);
  if (abs >= offset_error_) {
    level = DiagStatus::ERROR;
  } else if (abs >= offset_warn_) {
    level = DiagStatus::WARN;
  }
  for (auto itr = tracking_map.begin(); itr != tracking_map.end(); ++itr) {
    stat.add(itr->first, itr->second);
  }
  stat.summary(level, offset_dict_.at(level));

  // Measure elapsed time since start time and report
  SystemMonitorUtility::stopMeasurement(t_start, stat);
}

std::string NTPMonitor::executeChronyc(
  float & out_offset, std::map<std::string, std::string> & out_tracking_map,
  std::string & pipe2_err_str)
{
  std::string result;

  // Tracking chrony status

  // boost::process create file descriptor without O_CLOEXEC required for multithreading.
  // So create file descriptor with O_CLOEXEC and pass it to boost::process.
  int out_fd[2];
  if (pipe2(out_fd, O_CLOEXEC) != 0) {
    pipe2_err_str = std::string(strerror(errno));
    return result;
  }
  bp::pipe out_pipe{out_fd[0], out_fd[1]};
  bp::ipstream is_out{std::move(out_pipe)};

  bp::child c("chronyc tracking", bp::std_out > is_out);
  c.wait();
  if (c.exit_code() != 0) {
    std::ostringstream os;
    is_out >> os.rdbuf();
    result = os.str().c_str();
    return result;
  }

  std::string line;
  std::cmatch match;
  const std::regex filter("^(.+[A-Za-z()]) *: (.*)");
  const std::regex filter_system_time("([0-9.]*) seconds (slow|fast).*");

  while (std::getline(is_out, line) && !line.empty()) {
    if (std::regex_match(line.c_str(), match, filter)) {
      out_tracking_map[match[1].str()] = match[2].str();
    }
  }

  // System time : conversion string to float
  std::string str_system_time = out_tracking_map["System time"];
  if (std::regex_match(str_system_time.c_str(), match, filter_system_time)) {
    out_offset = std::atof(match[1].str().c_str());

    if (match[2].str() == "fast") {
      // "fast" is - value(match to ntpdate)
      out_offset *= -1;
    } else {
      // "slow" is + value(match to ntpdate)
    }
  } else {
    RCLCPP_WARN(get_logger(), "regex_match: illegal result. str = %s", str_system_time.c_str());
  }
  return result;
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(NTPMonitor)
