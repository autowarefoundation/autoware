// Copyright 2024 The Autoware Foundation
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

#ifndef COMPONENT_MONITOR_NODE_HPP_
#define COMPONENT_MONITOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_msgs/msg/resource_usage_report.hpp>

#include <boost/process.hpp>

#include <cstdint>
#include <sstream>
#include <string>
#include <vector>

namespace autoware::component_monitor
{
class ComponentMonitor : public rclcpp::Node
{
public:
  explicit ComponentMonitor(const rclcpp::NodeOptions & node_options);

private:
  using ResourceUsageReport = autoware_internal_msgs::msg::ResourceUsageReport;
  using VecVecStr = std::vector<std::vector<std::string>>;

  const double publish_rate_;

  std::function<void()> on_timer_tick_wrapped_;

  rclcpp::Publisher<ResourceUsageReport>::SharedPtr usage_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  boost::process::native_environment environment_;

  void on_timer_tick(int pid) const;

  /**
   * @brief Get system usage of the component.
   *
   * @details The output of top -b -n 1 -E k -p PID` is like below:
   *
   * top - 13:57:26 up  3:14,  1 user,  load average: 1,09, 1,10, 1,04
   * Tasks:   1 total,   0 running,   1 sleeping,   0 stopped,   0 zombie
   * %Cpu(s):  0,0 us,  0,8 sy,  0,0 ni, 99,2 id,  0,0 wa,  0,0 hi,  0,0 si,  0,0 st
   * KiB Mem : 65532208 total, 35117428 free, 17669824 used, 12744956 buff/cache
   * KiB Swap: 39062524 total, 39062524 free,        0 used. 45520816 avail Mem
   *
   *     PID USER      PR  NI    VIRT    RES    SHR S  %CPU  %MEM     TIME+ COMMAND
   *    3352 meb       20   0 2905940   1,2g  39292 S   0,0   2,0  23:24.01 awesome
   *
   * We get 5th and 8th fields, which are RES, %CPU, respectively.
   */
  ResourceUsageReport pid_to_report(const pid_t & pid) const;

  /**
   * @brief Run a terminal command and return the standard output.
   *
   * @param cmd The terminal command to run
   * @return  The standard output of the command
   */
  std::stringstream run_system_command(const std::string & cmd) const;

  /**
   * @brief Parses text from a stringstream into separated words by line.
   *
   * @param std_out Reference to the input stringstream.
   * @return Nested vector with each inner vector containing words from one line.
   */
  static VecVecStr parse_lines_into_words(const std::stringstream & std_out);

  /**
   * @brief Parses a memory resource string and converts it to bytes.
   *
   * This function handles memory size strings with suffixes to denote
   * the units (e.g., "k" for kibibytes, "m" for mebibytes, etc.).
   * If the string has no suffix, it is interpreted as plain bytes.
   *
   * @param mem_res A string representing the memory resource with a unit suffix or just bytes.
   * @return uint64_t The memory size in bytes.
   *
   * @exception std::runtime_error Thrown if the suffix is not recognized.
   */
  static std::uint64_t parse_memory_res(const std::string & mem_res);
};

}  // namespace autoware::component_monitor

#endif  // COMPONENT_MONITOR_NODE_HPP_
