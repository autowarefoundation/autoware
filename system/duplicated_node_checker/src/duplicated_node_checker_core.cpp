// Copyright 2023 TIER IV, Inc.
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

#include "duplicated_node_checker/duplicated_node_checker_core.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <string>
#include <vector>

namespace duplicated_node_checker
{

DuplicatedNodeChecker::DuplicatedNodeChecker(const rclcpp::NodeOptions & node_options)
: Node("duplicated_node_checker", node_options)
{
  double update_rate = declare_parameter<double>("update_rate");
  add_duplicated_node_names_to_msg_ = declare_parameter<bool>("add_duplicated_node_names_to_msg");
  updater_.setHardwareID("duplicated_node_checker");
  updater_.add("duplicated_node_checker", this, &DuplicatedNodeChecker::produceDiagnostics);

  const auto period_ns = rclcpp::Rate(update_rate).period();
  timer_ = rclcpp::create_timer(
    this, get_clock(), period_ns, std::bind(&DuplicatedNodeChecker::onTimer, this));
}

std::string get_fullname_from_name_ns_pair(std::pair<std::string, std::string> name_and_ns_pair)
{
  std::string full_name;
  const std::string & name = name_and_ns_pair.first;
  const std::string & ns = name_and_ns_pair.second;
  if (ns.back() == '/') {
    full_name = ns + name;
  } else {
    full_name = ns + "/" + name;
  }
  return full_name;
}

void DuplicatedNodeChecker::onTimer()
{
  updater_.force_update();
}

void DuplicatedNodeChecker::produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  using diagnostic_msgs::msg::DiagnosticStatus;

  std::vector<std::string> node_names = this->get_node_names();
  std::vector<std::string> identical_names = findIdenticalNames(node_names);
  std::string msg;
  int level;
  if (identical_names.size() > 0) {
    level = DiagnosticStatus::ERROR;
    msg = "Error: Duplicated nodes detected";
    if (add_duplicated_node_names_to_msg_) {
      std::set<std::string> unique_identical_names(identical_names.begin(), identical_names.end());
      for (const auto & name : unique_identical_names) {
        msg += " " + name;
      }
    }
    for (auto name : identical_names) {
      stat.add("Duplicated Node Name", name);
    }
  } else {
    msg = "OK";
    level = DiagnosticStatus::OK;
  }
  stat.summary(level, msg);
}

}  // namespace duplicated_node_checker

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(duplicated_node_checker::DuplicatedNodeChecker)
