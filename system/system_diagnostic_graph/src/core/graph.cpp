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

#include "graph.hpp"

#include "node.hpp"

#include <deque>
#include <map>
#include <memory>
#include <utility>

//
#include <iostream>

namespace system_diagnostic_graph
{

UnitNode * Graph::make_unit(const std::string & name)
{
  const auto key = name;
  auto unit = std::make_unique<UnitNode>(key);
  units_[key] = unit.get();
  nodes_.emplace_back(std::move(unit));
  return units_[key];
}

UnitNode * Graph::find_unit(const std::string & name)
{
  const auto key = name;
  return units_.count(key) ? units_.at(key) : nullptr;
}

DiagNode * Graph::make_diag(const std::string & name, const std::string & hardware)
{
  const auto key = std::make_pair(name, hardware);
  auto diag = std::make_unique<DiagNode>(name, hardware);
  diags_[key] = diag.get();
  nodes_.emplace_back(std::move(diag));
  return diags_[key];
}

DiagNode * Graph::find_diag(const std::string & name, const std::string & hardware)
{
  const auto key = std::make_pair(name, hardware);
  return diags_.count(key) ? diags_.at(key) : nullptr;
}

void Graph::topological_sort()
{
  std::map<BaseNode *, int> degrees;
  std::deque<BaseNode *> nodes;
  std::deque<BaseNode *> result;
  std::deque<BaseNode *> buffer;

  // Create a list of raw pointer nodes.
  for (const auto & node : nodes_) {
    nodes.push_back(node.get());
  }

  // Count degrees of each node.
  for (const auto & node : nodes) {
    for (const auto & link : node->links()) {
      ++degrees[link];
    }
  }

  // Find initial nodes that are zero degrees.
  for (const auto & node : nodes) {
    if (degrees[node] == 0) {
      buffer.push_back(node);
    }
  }

  // Sort by topological order.
  while (!buffer.empty()) {
    const auto node = buffer.front();
    buffer.pop_front();
    for (const auto & link : node->links()) {
      if (--degrees[link] == 0) {
        buffer.push_back(link);
      }
    }
    result.push_back(node);
  }

  // Detect circulation because the result does not include the nodes on the loop.
  if (result.size() != nodes.size()) {
    throw ConfigError("detect graph circulation");
  }

  // Reverse the result to process from leaf node.
  result = std::deque<BaseNode *>(result.rbegin(), result.rend());

  // Replace node vector.
  std::map<BaseNode *, size_t> indices;
  for (size_t i = 0; i < result.size(); ++i) {
    indices[result[i]] = i;
  }
  std::vector<std::unique_ptr<BaseNode>> temp(nodes_.size());
  for (auto & node : nodes_) {
    temp[indices[node.get()]] = std::move(node);
  }
  nodes_ = std::move(temp);
}

}  // namespace system_diagnostic_graph
