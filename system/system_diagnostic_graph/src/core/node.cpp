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

#include "node.hpp"

#include "expr.hpp"

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <utility>

namespace system_diagnostic_graph
{

UnitNode::UnitNode(const std::string & name)
{
  node_.status.level = DiagnosticStatus::STALE;
  node_.status.name = name;
  node_.status.hardware_id = "";
}

UnitNode::~UnitNode()
{
  // for unique_ptr
}

DiagnosticNode UnitNode::report() const
{
  return node_;
}

void UnitNode::update()
{
  const auto result = expr_->eval();
  node_.status.level = result.level;
  node_.links.clear();
  for (const auto & [node, used] : result.links) {
    DiagnosticLink link;
    link.index = node->index();
    link.used = used;
    node_.links.push_back(link);
  }
}

void UnitNode::create(Graph & graph, const NodeConfig & config)
{
  try {
    expr_ = BaseExpr::create(graph, config->yaml);
  } catch (const ConfigError & error) {
    throw create_error(config, error.what());
  }
}

std::vector<BaseNode *> UnitNode::links() const
{
  return expr_->get_dependency();
}

DiagNode::DiagNode(const std::string & name, const std::string & hardware)
{
  node_.status.level = DiagnosticStatus::STALE;
  node_.status.name = name;
  node_.status.hardware_id = hardware;
}

DiagnosticNode DiagNode::report() const
{
  return node_;
}

void DiagNode::update()
{
  // TODO(Takagi, Isamu): timeout, error hold
  // constexpr double timeout = 1.0; // TODO(Takagi, Isamu): parameterize
}

void DiagNode::callback(const DiagnosticStatus & status)
{
  node_.status = status;
}

}  // namespace system_diagnostic_graph
