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

namespace diagnostic_graph_aggregator
{

std::string get_level_text(DiagnosticLevel level)
{
  switch (level) {
    case DiagnosticStatus::OK:
      return "OK";
    case DiagnosticStatus::WARN:
      return "WARN";
    case DiagnosticStatus::ERROR:
      return "ERROR";
    case DiagnosticStatus::STALE:
      return "STALE";
  }
  return "UNKNOWN";
}

void Graph::debug()
{
  std::vector<DiagDebugData> lines;
  for (const auto & node : nodes_) {
    const auto level_name = get_level_text(node->level());
    const auto index_name = std::to_string(node->index());
    lines.push_back({index_name, level_name, node->path(), node->type()});
  }
  for (const auto & [name, level] : unknowns_) {
    const auto level_name = get_level_text(level);
    lines.push_back({"*", level_name, name, "unknown"});
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

}  // namespace diagnostic_graph_aggregator
