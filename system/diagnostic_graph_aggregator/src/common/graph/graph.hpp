// Copyright 2023 The Autoware Contributors
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

#ifndef COMMON__GRAPH__GRAPH_HPP_
#define COMMON__GRAPH__GRAPH_HPP_

#include "types.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace diagnostic_graph_aggregator
{

class Graph
{
public:
  void create(const std::string & file, const std::string & id = "");
  void update(const rclcpp::Time & stamp);  // cppcheck-suppress functionConst
  bool update(const rclcpp::Time & stamp, const DiagnosticStatus & status);
  const auto & nodes() const { return nodes_; }
  const auto & units() const { return units_; }
  DiagGraphStruct create_struct(const rclcpp::Time & stamp) const;
  DiagGraphStatus create_status(const rclcpp::Time & stamp) const;

  Graph();   // For unique_ptr members.
  ~Graph();  // For unique_ptr members.

private:
  // Note: keep order correspondence between links and unit children for viewer.
  std::vector<std::unique_ptr<NodeUnit>> nodes_;
  std::vector<std::unique_ptr<DiagUnit>> diags_;
  std::vector<std::unique_ptr<UnitLink>> links_;
  std::vector<BaseUnit *> units_;
  std::unordered_map<std::string, DiagUnit *> names_;
  std::string id_;
};

}  // namespace diagnostic_graph_aggregator

#endif  // COMMON__GRAPH__GRAPH_HPP_
