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

#include "config.hpp"
#include "error.hpp"
#include "units.hpp"

#include <deque>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

// DEBUG
#include <iostream>

namespace diagnostic_graph_aggregator
{

BaseUnit::UniquePtrList topological_sort(BaseUnit::UniquePtrList && input)
{
  std::unordered_map<BaseUnit *, int> degrees;
  std::deque<BaseUnit *> nodes;
  std::deque<BaseUnit *> result;
  std::deque<BaseUnit *> buffer;

  // Create a list of raw pointer nodes.
  for (const auto & node : input) {
    nodes.push_back(node.get());
  }

  // Count degrees of each node.
  for (const auto & node : nodes) {
    for (const auto & child : node->children()) {
      ++degrees[child];
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
    for (const auto & child : node->children()) {
      if (--degrees[child] == 0) {
        buffer.push_back(child);
      }
    }
    result.push_back(node);
  }

  // Detect circulation because the result does not include the nodes on the loop.
  if (result.size() != nodes.size()) {
    throw error<GraphStructure>("detect graph circulation");
  }

  // Reverse the result to process from leaf node.
  result = std::deque<BaseUnit *>(result.rbegin(), result.rend());

  // Replace node vector.
  std::unordered_map<BaseUnit *, size_t> indices;
  for (size_t i = 0; i < result.size(); ++i) {
    indices[result[i]] = i;
  }
  std::vector<std::unique_ptr<BaseUnit>> sorted(input.size());
  for (auto & node : input) {
    sorted[indices[node.get()]] = std::move(node);
  }
  return sorted;
}

BaseUnit::UniquePtr make_node(const UnitConfig::SharedPtr & config)
{
  if (config->type == "diag") {
    return std::make_unique<DiagUnit>(config->path);
  }
  if (config->type == "and") {
    return std::make_unique<AndUnit>(config->path, false);
  }
  if (config->type == "short-circuit-and") {
    return std::make_unique<AndUnit>(config->path, true);
  }
  if (config->type == "or") {
    return std::make_unique<OrUnit>(config->path);
  }
  if (config->type == "warn-to-ok") {
    return std::make_unique<RemapUnit>(config->path, DiagnosticStatus::OK);
  }
  if (config->type == "warn-to-error") {
    return std::make_unique<RemapUnit>(config->path, DiagnosticStatus::ERROR);
  }
  if (config->type == "ok") {
    return std::make_unique<DebugUnit>(config->path, DiagnosticStatus::OK);
  }
  if (config->type == "warn") {
    return std::make_unique<DebugUnit>(config->path, DiagnosticStatus::WARN);
  }
  if (config->type == "error") {
    return std::make_unique<DebugUnit>(config->path, DiagnosticStatus::ERROR);
  }
  if (config->type == "stale") {
    return std::make_unique<DebugUnit>(config->path, DiagnosticStatus::STALE);
  }
  throw error<UnknownType>("unknown node type", config->type, config->data);
}

Graph::Graph()
{
  // for unique_ptr
}

Graph::~Graph()
{
  // for unique_ptr
}

void Graph::init(const std::string & file)
{
  BaseUnit::UniquePtrList nodes;
  BaseUnit::NodeDict dict;

  for (const auto & config : load_root_config(file).nodes) {
    const auto node = nodes.emplace_back(make_node(config)).get();
    dict.configs[config] = node;
    dict.paths[config->path] = node;
  }
  dict.paths.erase("");

  for (const auto & [config, node] : dict.configs) {
    node->init(config, dict);
  }

  // Sort units in topological order for update dependencies.
  nodes = topological_sort(std::move(nodes));

  // List diag nodes that have diag name.
  for (const auto & node : nodes) {
    const auto diag = dynamic_cast<DiagUnit *>(node.get());
    if (diag) {
      diags_[diag->name()] = diag;
    }
  }

  // List unit nodes that have path name.
  for (const auto & node : nodes) {
    if (!node->path().empty()) {
      units_.push_back(node.get());
    }
  }

  // Set unit index.
  for (size_t index = 0; index < units_.size(); ++index) {
    units_[index]->set_index(index);
  }

  nodes_ = std::move(nodes);
}

void Graph::callback(const rclcpp::Time & stamp, const DiagnosticArray & array)
{
  for (const auto & status : array.status) {
    const auto iter = diags_.find(status.name);
    if (iter != diags_.end()) {
      iter->second->callback(stamp, status);
    } else {
      unknowns_[status.name] = status.level;
    }
  }
}

DiagnosticGraph Graph::report(const rclcpp::Time & stamp)
{
  for (const auto & node : nodes_) {
    node->update(stamp);
  }

  DiagnosticGraph message;
  message.stamp = stamp;
  message.nodes.reserve(units_.size());
  for (const auto & node : units_) {
    const auto report = node->report();
    DiagnosticNode temp;
    temp.status.name = node->path();
    temp.status.level = report.level;
    for (const auto & [ref, used] : report.links) {
      DiagnosticLink link;
      link.index = ref->index();
      link.used = used;
      temp.links.push_back(link);
    }
    message.nodes.push_back(temp);
  }
  return message;
}

std::vector<BaseUnit *> Graph::nodes() const
{
  std::vector<BaseUnit *> result;
  result.reserve(nodes_.size());
  for (const auto & node : nodes_) {
    result.push_back(node.get());
  }
  return result;
}

}  // namespace diagnostic_graph_aggregator
