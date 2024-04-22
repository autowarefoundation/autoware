// Copyright 2024 The Autoware Contributors
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

#include "diagnostic_graph_utils/graph.hpp"

namespace diagnostic_graph_utils
{

DiagUnit::DiagnosticStatus DiagNode::create_diagnostic_status() const
{
  DiagnosticStatus status;
  status.level = level();
  status.name = path();
  return status;
}

DiagUnit::DiagnosticStatus DiagLeaf::create_diagnostic_status() const
{
  DiagnosticStatus status;
  status.level = level();
  status.name = path();
  status.message = status_.message;
  status.hardware_id = status_.hardware_id;
  status.values = status_.values;
  return status;
}

void DiagGraph::create(const DiagGraphStruct & msg)
{
  created_stamp_ = msg.stamp;
  id_ = msg.id;
  for (const auto & node : msg.nodes) nodes_.push_back(std::make_unique<DiagNode>(node));
  for (const auto & diag : msg.diags) diags_.push_back(std::make_unique<DiagLeaf>(diag));

  const auto get_child = [this](bool is_leaf, size_t index) -> DiagUnit * {
    if (is_leaf) {
      return diags_.at(index).get();
    } else {
      return nodes_.at(index).get();
    }
  };

  for (const auto & data : msg.links) {
    DiagNode * parent = nodes_.at(data.parent).get();
    DiagUnit * child = get_child(data.is_leaf, data.child);
    const auto link = links_.emplace_back(std::make_unique<DiagLink>(data)).get();
    parent->add_child({link, child});
  }
}

bool DiagGraph::update(const DiagGraphStatus & msg)
{
  if (id_ != msg.id) return false;
  updated_stamp_ = msg.stamp;
  for (size_t i = 0; i < msg.nodes.size(); ++i) nodes_[i]->update(msg.nodes[i]);
  for (size_t i = 0; i < msg.diags.size(); ++i) diags_[i]->update(msg.diags[i]);
  for (size_t i = 0; i < msg.links.size(); ++i) links_[i]->update(msg.links[i]);
  return true;
}

template <class T, class U>
void extend_ptrs(std::vector<T *> & result, const std::vector<std::unique_ptr<U>> & list)
{
  for (const auto & item : list) result.push_back(item.get());
}

template <class T>
std::vector<T *> create_ptrs(const std::vector<std::unique_ptr<T>> & list)
{
  std::vector<T *> result;
  extend_ptrs(result, list);
  return result;
}

std::vector<DiagUnit *> DiagGraph::units() const
{
  std::vector<DiagUnit *> result;
  extend_ptrs(result, nodes_);
  extend_ptrs(result, diags_);
  return result;
}

std::vector<DiagNode *> DiagGraph::nodes() const
{
  return create_ptrs<DiagNode>(nodes_);
}

std::vector<DiagLeaf *> DiagGraph::diags() const
{
  return create_ptrs<DiagLeaf>(diags_);
}

std::vector<DiagLink *> DiagGraph::links() const
{
  return create_ptrs<DiagLink>(links_);
}

}  // namespace diagnostic_graph_utils
