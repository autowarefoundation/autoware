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
 * @file memory_monitor.cpp
 * @brief Memory monitor class
 */

#include "system_monitor/mem_monitor/mem_monitor.hpp"

#include "system_monitor/system_monitor_utility.hpp"

#include <boost/process.hpp>

#include <fmt/format.h>

#include <string>
#include <vector>

namespace bp = boost::process;

MemMonitor::MemMonitor(const rclcpp::NodeOptions & options)
: Node("mem_monitor", options),
  updater_(this),
  usage_warn_(declare_parameter<float>("usage_warn", 0.95)),
  usage_error_(declare_parameter<float>("usage_error", 0.99))
{
  gethostname(hostname_, sizeof(hostname_));
  updater_.setHardwareID(hostname_);
  updater_.add("Memory Usage", this, &MemMonitor::checkUsage);
}

void MemMonitor::update() { updater_.force_update(); }

void MemMonitor::checkUsage(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  // Remember start time to measure elapsed time
  const auto t_start = SystemMonitorUtility::startMeasurement();

  // Get total amount of free and used memory
  bp::ipstream is_out;
  bp::ipstream is_err;
  bp::child c("free -tb", bp::std_out > is_out, bp::std_err > is_err);
  c.wait();
  if (c.exit_code() != 0) {
    std::ostringstream os;
    is_err >> os.rdbuf();
    stat.summary(DiagStatus::ERROR, "free error");
    stat.add("free", os.str().c_str());
    return;
  }

  int level = DiagStatus::OK;
  std::string line;
  int index = 0;
  std::vector<std::string> list;
  float usage;

  /*
   Output example of `free -tb`

             list[0]     list[1]     list[2]     list[3]     list[4]     list[5]     list[6]
   index 0 |               total        used        free      shared  buff/cache   available
   index 1 | Mem:       32809744    12554780    13090376      292840     7164588    19622092
   index 2 | Swap:      33554428     1767680    31786748
   index 3 | Total:     66364172    14322460    44877124
  */
  while (std::getline(is_out, line) && !line.empty()) {
    // Skip header
    if (index <= 0) {
      ++index;
      continue;
    }

    boost::split(list, line, boost::is_space(), boost::token_compress_on);

    // Physical memory
    if (index == 1) {
      // available divided by total is available memory including calculation for buff/cache,
      // so the subtraction of this from 1 gives real usage.
      usage = 1.0f - std::atof(list[6].c_str()) / std::atof(list[1].c_str());

      if (usage >= usage_error_) {
        level = DiagStatus::ERROR;
      } else if (usage >= usage_warn_) {
        level = DiagStatus::WARN;
      }

      stat.addf(fmt::format("{} usage", list[0]), "%.2f%%", usage * 1e+2);
    }

    stat.add(fmt::format("{} total", list[0]), toHumanReadable(list[1]));
    stat.add(fmt::format("{} used", list[0]), toHumanReadable(list[2]));
    stat.add(fmt::format("{} free", list[0]), toHumanReadable(list[3]));

    // Add an additional information for physical memory
    if (index == 1) {
      stat.add(fmt::format("{} shared", list[0]), toHumanReadable(list[4]));
      stat.add(fmt::format("{} buff/cache", list[0]), toHumanReadable(list[5]));
      stat.add(fmt::format("{} available", list[0]), toHumanReadable(list[6]));
    }
    ++index;
  }

  stat.summary(level, usage_dict_.at(level));

  // Measure elapsed time since start time and report
  SystemMonitorUtility::stopMeasurement(t_start, stat);
}

std::string MemMonitor::toHumanReadable(const std::string & str)
{
  const char * units[] = {"B", "K", "M", "G", "T"};
  int count = 0;
  double size = std::atol(str.c_str());

  while (size > 1024) {
    size /= 1024;
    ++count;
  }
  const char * format = (size > 0 && size < 10) ? "{:.1f}{}" : "{:.0f}{}";
  return fmt::format(format, size, units[count]);
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(MemMonitor)
