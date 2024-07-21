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

#include "config.hpp"
#include "error.hpp"
#include "loader.hpp"

#include <algorithm>

namespace diagnostic_graph_aggregator
{

void UnitLink::initialize_object(BaseUnit * parent, BaseUnit * child)
{
  parent_ = parent;
  child_ = child;
}

void UnitLink::initialize_struct()
{
  struct_.parent = parent_->index();
  struct_.child = child_->index();
  struct_.is_leaf = child_->is_leaf();
}

void UnitLink::initialize_status()
{
  // Do nothing.
}

BaseUnit::BaseUnit(const UnitLoader & unit)
{
  index_ = unit.index();
  parents_ = unit.parents();
}

bool BaseUnit::update()
{
  // Update the level of this unit.
  update_status();

  // If the level does not change, it will not affect the parents.
  const auto curr_level = level();
  if (curr_level == prev_level_) return false;
  prev_level_ = curr_level;

  // If the level changes, the parents also need to be updated.
  bool result = false;
  for (const auto & link : parents_) {
    const auto unit = link->parent();
    result = result || unit->update();
  }
  return result;
}

NodeUnit::NodeUnit(const UnitLoader & unit) : BaseUnit(unit)
{
  struct_.path = unit.path();
  status_.level = DiagnosticStatus::STALE;
}

void NodeUnit::initialize_struct()
{
  struct_.type = type();
}

void NodeUnit::initialize_status()
{
  if (child_links().size() == 0) update();
}

LeafUnit::LeafUnit(const UnitLoader & unit) : BaseUnit(unit)
{
  const auto diag_node = unit.data().required("node").text();
  const auto diag_name = unit.data().required("name").text();
  const auto sep = diag_node.empty() ? "" : ": ";

  struct_.path = unit.path();
  struct_.name = diag_node + sep + diag_name;
  status_.level = DiagnosticStatus::STALE;
}

void LeafUnit::initialize_struct()
{
  struct_.type = type();
}

void LeafUnit::initialize_status()
{
  if (child_links().size() == 0) update();
}

DiagUnit::DiagUnit(const UnitLoader & unit) : LeafUnit(unit)
{
  timeout_ = unit.data().optional("timeout").real(1.0);
}

void DiagUnit::update_status()
{
  // Do nothing. The level is updated by on_diag and on_time.
}

bool DiagUnit::on_diag(const rclcpp::Time & stamp, const DiagnosticStatus & status)
{
  last_updated_time_ = stamp;
  status_.level = status.level;
  status_.message = status.message;
  status_.hardware_id = status.hardware_id;
  status_.values = status.values;
  return update();
}

bool DiagUnit::on_time(const rclcpp::Time & stamp)
{
  if (last_updated_time_) {
    const auto updated = last_updated_time_.value();
    const auto elapsed = (stamp - updated).seconds();
    if (timeout_ < elapsed) {
      last_updated_time_ = std::nullopt;
      status_ = DiagLeafStatus();
      status_.level = DiagnosticStatus::STALE;
    }
  }
  return update();
}

MaxUnit::MaxUnit(const UnitLoader & unit) : NodeUnit(unit)
{
  links_ = unit.children();
}

void MaxUnit::update_status()
{
  DiagnosticLevel level = DiagnosticStatus::OK;
  for (const auto * const link : links_) {
    level = std::max(level, link->child()->level());
  }
  status_.level = std::min(level, DiagnosticStatus::ERROR);
}

void ShortCircuitMaxUnit::update_status()
{
  // TODO(Takagi, Isamu): update link flags.
  DiagnosticLevel level = DiagnosticStatus::OK;
  for (const auto * const link : links_) {
    level = std::max(level, link->child()->level());
  }
  status_.level = std::min(level, DiagnosticStatus::ERROR);
}

MinUnit::MinUnit(const UnitLoader & unit) : NodeUnit(unit)
{
  links_ = unit.children();
}

void MinUnit::update_status()
{
  DiagnosticLevel level = DiagnosticStatus::OK;
  if (!links_.empty()) {
    level = DiagnosticStatus::STALE;
    for (const auto * const link : links_) {
      level = std::min(level, link->child()->level());
    }
  }
  status_.level = std::min(level, DiagnosticStatus::ERROR);
}

RemapUnit::RemapUnit(const UnitLoader & unit) : NodeUnit(unit)
{
  link_ = unit.child();
}

void RemapUnit::update_status()
{
  const auto level = link_->child()->level();
  status_.level = (level == level_from_) ? level_to_ : level;
}

WarnToOkUnit::WarnToOkUnit(const UnitLoader & unit) : RemapUnit(unit)
{
  level_from_ = DiagnosticStatus::WARN;
  level_to_ = DiagnosticStatus::OK;
}

WarnToErrorUnit::WarnToErrorUnit(const UnitLoader & unit) : RemapUnit(unit)
{
  level_from_ = DiagnosticStatus::WARN;
  level_to_ = DiagnosticStatus::ERROR;
}

void ConstUnit::update_status()
{
  // Do nothing. This unit always returns the same level.
}

OkUnit::OkUnit(const UnitLoader & unit) : ConstUnit(unit)
{
  status_.level = DiagnosticStatus::OK;
}

WarnUnit::WarnUnit(const UnitLoader & unit) : ConstUnit(unit)
{
  status_.level = DiagnosticStatus::WARN;
}

ErrorUnit::ErrorUnit(const UnitLoader & unit) : ConstUnit(unit)
{
  status_.level = DiagnosticStatus::ERROR;
}

StaleUnit::StaleUnit(const UnitLoader & unit) : ConstUnit(unit)
{
  status_.level = DiagnosticStatus::STALE;
}

}  // namespace diagnostic_graph_aggregator
