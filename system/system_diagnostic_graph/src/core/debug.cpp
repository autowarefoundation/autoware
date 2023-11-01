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

#include "graph.hpp"
#include "types.hpp"
#include "units.hpp"

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

void Graph::debug()
{
  std::vector<DiagDebugData> lines;
  for (const auto & node : nodes_) {
    const auto level_name = level_names.at(node->level());
    const auto index_name = std::to_string(node->index());
    lines.push_back({"unit", index_name, level_name, node->path(), "-----"});
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

}  // namespace system_diagnostic_graph
