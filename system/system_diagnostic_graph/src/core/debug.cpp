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

#include "debug.hpp"

#include "node.hpp"
#include "types.hpp"
#include "update.hpp"

#include <algorithm>
#include <iostream>
#include <unordered_map>
#include <vector>

namespace system_diagnostic_graph
{

const std::unordered_map<DiagnosticLevel, std::string> level_names = {
  {DiagnosticStatus::OK, "OK"},
  {DiagnosticStatus::WARN, "WARN"},
  {DiagnosticStatus::ERROR, "ERROR"},
  {DiagnosticStatus::STALE, "STALE"}};

void DiagGraph::debug()
{
  std::vector<DiagDebugData> lines;
  for (const auto & node : graph_.nodes()) {
    lines.push_back(node->debug());
  }

  std::array<size_t, diag_debug_size> widths = {};
  for (const auto & line : lines) {
    for (size_t i = 0; i < diag_debug_size; ++i) {
      widths[i] = std::max(widths[i], line[i].length());
    }
  }

  const size_t total_width = std::accumulate(widths.begin(), widths.end(), 0);
  std::cout << std::string(total_width + 3 * diag_debug_size + 1, '=') << std::endl;

  for (const auto & line : lines) {
    for (size_t i = 0; i < diag_debug_size; ++i) {
      std::cout << "| " << std::left << std::setw(widths[i]) << line[i] << " ";
    }
    std::cout << "|" << std::endl;
  }
}

DiagDebugData UnitNode::debug() const
{
  const auto & level = node_.status.level;
  const auto & name = node_.status.name;
  return DiagDebugData{std::to_string(index()), "unit", name, "-----", level_names.at(level)};
}

DiagDebugData DiagNode::debug() const
{
  const auto & level = node_.status.level;
  const auto & name = node_.status.name;
  const auto & hardware = node_.status.hardware_id;
  return DiagDebugData{std::to_string(index()), "diag", name, hardware, level_names.at(level)};
}

}  // namespace system_diagnostic_graph
