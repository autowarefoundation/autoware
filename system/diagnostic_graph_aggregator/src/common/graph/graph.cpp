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
#include "loader.hpp"
#include "units.hpp"

#include <unordered_map>

namespace diagnostic_graph_aggregator
{

void Graph::create(const std::string & file, const std::string & id)
{
  GraphLoader graph(file);
  nodes_ = graph.release_nodes();
  diags_ = graph.release_diags();
  links_ = graph.release_links();
  for (const auto & diag : diags_) names_[diag->name()] = diag.get();
  for (const auto & node : nodes_) units_.push_back(node.get());
  for (const auto & diag : diags_) units_.push_back(diag.get());

  id_ = id;
}

void Graph::update(const rclcpp::Time & stamp)
{
  for (const auto & diag : diags_) diag->on_time(stamp);
}

bool Graph::update(const rclcpp::Time & stamp, const DiagnosticStatus & status)
{
  const auto iter = names_.find(status.name);
  if (iter == names_.end()) return false;
  iter->second->on_diag(stamp, status);
  return true;
}

DiagGraphStruct Graph::create_struct(const rclcpp::Time & stamp) const
{
  DiagGraphStruct msg;
  msg.stamp = stamp;
  msg.id = id_;
  for (const auto & node : nodes_) msg.nodes.push_back(node->create_struct());
  for (const auto & diag : diags_) msg.diags.push_back(diag->create_struct());
  for (const auto & link : links_) msg.links.push_back(link->create_struct());
  return msg;
}

DiagGraphStatus Graph::create_status(const rclcpp::Time & stamp) const
{
  DiagGraphStatus msg;
  msg.stamp = stamp;
  msg.id = id_;
  for (const auto & node : nodes_) msg.nodes.push_back(node->create_status());
  for (const auto & diag : diags_) msg.diags.push_back(diag->create_status());
  for (const auto & link : links_) msg.links.push_back(link->create_status());
  return msg;
}

// For unique_ptr members.
Graph::Graph() = default;
Graph::~Graph() = default;

}  // namespace diagnostic_graph_aggregator
