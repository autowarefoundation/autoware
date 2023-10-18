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

#ifndef CORE__GRAPH_HPP_
#define CORE__GRAPH_HPP_

#include "types.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace system_diagnostic_graph
{

class Graph final
{
public:
  Graph();
  ~Graph();

  void init(const std::string & file, const std::string & mode);
  void callback(const DiagnosticArray & array, const rclcpp::Time & stamp);
  void update(const rclcpp::Time & stamp);
  DiagnosticGraph message() const;
  std::vector<BaseNode *> nodes() const;

  void debug();

private:
  std::vector<std::unique_ptr<BaseNode>> nodes_;
  std::unordered_map<std::string, DiagNode *> diags_;
  UnknownNode * unknown_;
  rclcpp::Time stamp_;
};

}  // namespace system_diagnostic_graph

#endif  // CORE__GRAPH_HPP_
