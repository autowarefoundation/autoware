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

#include "units.hpp"

#include "error.hpp"

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

namespace diagnostic_graph_aggregator
{

using LinkList = std::vector<std::pair<const BaseUnit *, bool>>;

void merge(LinkList & a, const LinkList & b, bool uses)
{
  for (const auto & [node, used] : b) {
    a.push_back(std::make_pair(node, used && uses));
  }
}

auto resolve(const BaseUnit::NodeDict & dict, const std::vector<UnitConfig::SharedPtr> & children)
{
  std::vector<BaseUnit *> result;
  for (const auto & child : children) {
    result.push_back(dict.configs.at(child));
  }
  return result;
}

BaseUnit::BaseUnit(const std::string & path) : path_(path)
{
  index_ = 0;
  level_ = DiagnosticStatus::OK;
}

BaseUnit::NodeData BaseUnit::status() const
{
  if (path_.empty()) {
    return {level_, links_};
  } else {
    return {level_, {std::make_pair(this, true)}};
  }
}

BaseUnit::NodeData BaseUnit::report() const
{
  return {level_, links_};
}

void DiagUnit::init(const UnitConfig::SharedPtr & config, const NodeDict &)
{
  name_ = config->data.take_text("diag");
  timeout_ = config->data.take<double>("timeout", 1.0);
}

void DiagUnit::update(const rclcpp::Time & stamp)
{
  if (diagnostics_) {
    const auto updated = diagnostics_.value().first;
    const auto elapsed = (stamp - updated).seconds();
    if (timeout_ < elapsed) {
      diagnostics_ = std::nullopt;
    }
  }

  if (diagnostics_) {
    level_ = diagnostics_.value().second.level;
  } else {
    level_ = DiagnosticStatus::STALE;
  }
}

void DiagUnit::callback(const rclcpp::Time & stamp, const DiagnosticStatus & status)
{
  diagnostics_ = std::make_pair(stamp, status);
}

AndUnit::AndUnit(const std::string & path, bool short_circuit) : BaseUnit(path)
{
  short_circuit_ = short_circuit;
}

void AndUnit::init(const UnitConfig::SharedPtr & config, const NodeDict & dict)
{
  children_ = resolve(dict, config->children);
}

void AndUnit::update(const rclcpp::Time &)
{
  if (children_.empty()) {
    return;
  }

  bool uses = true;
  level_ = DiagnosticStatus::OK;
  links_ = LinkList();

  for (const auto & child : children_) {
    const auto status = child->status();
    level_ = std::max(level_, status.level);
    merge(links_, status.links, uses);
    if (short_circuit_ && level_ != DiagnosticStatus::OK) {
      uses = false;
    }
  }
  level_ = std::min(level_, DiagnosticStatus::ERROR);
}

void OrUnit::init(const UnitConfig::SharedPtr & config, const NodeDict & dict)
{
  children_ = resolve(dict, config->children);
}

void OrUnit::update(const rclcpp::Time &)
{
  if (children_.empty()) {
    return;
  }

  level_ = DiagnosticStatus::ERROR;
  links_ = LinkList();

  for (const auto & child : children_) {
    const auto status = child->status();
    level_ = std::min(level_, status.level);
    merge(links_, status.links, true);
  }
  level_ = std::min(level_, DiagnosticStatus::ERROR);
}

RemapUnit::RemapUnit(const std::string & path, DiagnosticLevel remap_warn) : BaseUnit(path)
{
  remap_warn_ = remap_warn;
}

void RemapUnit::init(const UnitConfig::SharedPtr & config, const NodeDict & dict)
{
  if (config->children.size() != 1) {
    throw error<InvalidValue>("list size must be 1", config->data);
  }
  children_ = resolve(dict, config->children);
}

void RemapUnit::update(const rclcpp::Time &)
{
  const auto status = children_.front()->status();
  level_ = status.level;
  links_ = status.links;

  if (level_ == DiagnosticStatus::WARN) level_ = remap_warn_;
}

DebugUnit::DebugUnit(const std::string & path, DiagnosticLevel level) : BaseUnit(path)
{
  level_ = level;  // overwrite
}

void DebugUnit::init(const UnitConfig::SharedPtr &, const NodeDict &)
{
}

void DebugUnit::update(const rclcpp::Time &)
{
}

}  // namespace diagnostic_graph_aggregator
