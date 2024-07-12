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

#include "component_monitor_node.hpp"

#include "unit_conversions.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_msgs/msg/resource_usage_report.hpp>

#include <boost/process.hpp>

#include <cctype>
#include <cstdint>
#include <exception>
#include <functional>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::component_monitor
{
ComponentMonitor::ComponentMonitor(const rclcpp::NodeOptions & node_options)
: Node("component_monitor", node_options), publish_rate_(declare_parameter<double>("publish_rate"))
{
  usage_pub_ =
    create_publisher<ResourceUsageReport>("~/component_system_usage", rclcpp::SensorDataQoS());

  // Make sure top ins installed and is in path
  const auto path_top = boost::process::search_path("top");
  if (path_top.empty()) {
    RCLCPP_ERROR_STREAM(get_logger(), "Couldn't find 'top' in path.");
    rclcpp::shutdown();
  }

  // Get the PID of the current process
  int pid = getpid();

  environment_ = boost::this_process::environment();
  environment_["LC_NUMERIC"] = "en_US.UTF-8";

  on_timer_tick_wrapped_ = std::bind(&ComponentMonitor::on_timer_tick, this, pid);

  timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Rate(publish_rate_).period(), on_timer_tick_wrapped_);
}

void ComponentMonitor::on_timer_tick(const int pid) const
{
  if (usage_pub_->get_subscription_count() == 0) return;

  try {
    auto usage_msg = pid_to_report(pid);
    usage_msg.header.stamp = this->now();
    usage_msg.pid = pid;
    usage_pub_->publish(usage_msg);
  } catch (std::exception & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
  } catch (...) {
    RCLCPP_ERROR(get_logger(), "An unknown error occurred.");
  }
}

ComponentMonitor::ResourceUsageReport ComponentMonitor::pid_to_report(const pid_t & pid) const
{
  const auto std_out = run_system_command("top -b -n 1 -E k -p " + std::to_string(pid));

  const auto fields = parse_lines_into_words(std_out);

  ResourceUsageReport report;
  report.cpu_cores_utilized = std::stof(fields.back().at(8)) / 100.0f;
  report.total_memory_bytes = unit_conversions::kib_to_bytes(std::stoul(fields.at(3).at(3)));
  report.free_memory_bytes = unit_conversions::kib_to_bytes(std::stoul(fields.at(3).at(5)));
  report.process_memory_bytes = parse_memory_res(fields.back().at(5));

  return report;
}

std::stringstream ComponentMonitor::run_system_command(const std::string & cmd) const
{
  int out_fd[2];
  if (pipe2(out_fd, O_CLOEXEC) != 0) {
    RCLCPP_ERROR_STREAM(get_logger(), "Error setting flags on out_fd");
  }
  boost::process::pipe out_pipe{out_fd[0], out_fd[1]};
  boost::process::ipstream is_out{std::move(out_pipe)};

  int err_fd[2];
  if (pipe2(err_fd, O_CLOEXEC) != 0) {
    RCLCPP_ERROR_STREAM(get_logger(), "Error setting flags on err_fd");
  }
  boost::process::pipe err_pipe{err_fd[0], err_fd[1]};
  boost::process::ipstream is_err{std::move(err_pipe)};

  boost::process::child c(
    cmd, environment_, boost::process::std_out > is_out, boost::process::std_err > is_err);
  c.wait();

  if (c.exit_code() != 0) {
    std::ostringstream os;
    is_err >> os.rdbuf();
    RCLCPP_ERROR_STREAM(get_logger(), "Error running command: " << os.str());
  }

  std::stringstream sstream;
  sstream << is_out.rdbuf();
  return sstream;
}

ComponentMonitor::VecVecStr ComponentMonitor::parse_lines_into_words(
  const std::stringstream & std_out)
{
  VecVecStr fields;
  std::string line;
  std::istringstream input{std_out.str()};

  while (std::getline(input, line)) {
    std::istringstream iss_line{line};
    std::string word;
    std::vector<std::string> words;

    while (iss_line >> word) {
      words.push_back(word);
    }

    fields.push_back(words);
  }

  return fields;
}

std::uint64_t ComponentMonitor::parse_memory_res(const std::string & mem_res)
{
  // example 1: 12.3g
  // example 2: 123 (without suffix, just bytes)
  static const std::unordered_map<char, std::function<std::uint64_t(double)>> unit_map{
    {'k', unit_conversions::kib_to_bytes<double>}, {'m', unit_conversions::mib_to_bytes<double>},
    {'g', unit_conversions::gib_to_bytes<double>}, {'t', unit_conversions::tib_to_bytes<double>},
    {'p', unit_conversions::pib_to_bytes<double>}, {'e', unit_conversions::eib_to_bytes<double>}};

  if (std::isdigit(mem_res.back())) {
    return std::stoull(mem_res);  // Handle plain bytes without any suffix
  }

  // Extract the numeric part and the unit suffix
  double value = std::stod(mem_res.substr(0, mem_res.size() - 1));
  char suffix = mem_res.back();

  // Find the appropriate function from the map
  auto it = unit_map.find(suffix);
  if (it != unit_map.end()) {
    const auto & conversion_function = it->second;
    return conversion_function(value);
  }

  // Throw an exception or handle the error as needed if the suffix is not recognized
  throw std::runtime_error("Unsupported unit suffix: " + std::string(1, suffix));
}

}  // namespace autoware::component_monitor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::component_monitor::ComponentMonitor)
