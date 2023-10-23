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

#ifndef DUPLICATED_NODE_CHECKER__DUPLICATED_NODE_CHECKER_CORE_HPP_
#define DUPLICATED_NODE_CHECKER__DUPLICATED_NODE_CHECKER_CORE_HPP_

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>
#include <unordered_set>
#include <vector>

namespace duplicated_node_checker
{
class DuplicatedNodeChecker : public rclcpp::Node
{
public:
  explicit DuplicatedNodeChecker(const rclcpp::NodeOptions & node_options);
  std::vector<std::string> findIdenticalNames(const std::vector<std::string> input_name_lists)
  {
    std::unordered_set<std::string> unique_names;
    std::vector<std::string> identical_names;
    for (auto name : input_name_lists) {
      if (unique_names.find(name) != unique_names.end()) {
        identical_names.push_back(name);
      } else {
        unique_names.insert(name);
      }
    }
    return identical_names;
  }

private:
  void onTimer();
  void produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);

  diagnostic_updater::Updater updater_{this};
  rclcpp::TimerBase::SharedPtr timer_;
  bool add_duplicated_node_names_to_msg_;
};
}  // namespace duplicated_node_checker

#endif  // DUPLICATED_NODE_CHECKER__DUPLICATED_NODE_CHECKER_CORE_HPP_
