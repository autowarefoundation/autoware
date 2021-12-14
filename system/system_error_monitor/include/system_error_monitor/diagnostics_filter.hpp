// Copyright 2020 Tier IV, Inc.
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

#ifndef SYSTEM_ERROR_MONITOR__DIAGNOSTICS_FILTER_HPP_
#define SYSTEM_ERROR_MONITOR__DIAGNOSTICS_FILTER_HPP_

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <string>
#include <unordered_set>
#include <vector>

namespace diagnostics_filter
{
inline std::string splitStringByLastSlash(const std::string & str)
{
  const auto last_slash = str.find_last_of("/");

  if (last_slash == std::string::npos) {
    return "";
  }

  return str.substr(0, last_slash);
}

inline bool isChild(
  const diagnostic_msgs::msg::DiagnosticStatus & child,
  const diagnostic_msgs::msg::DiagnosticStatus & parent)
{
  auto name = splitStringByLastSlash(child.name);
  while (name != "") {
    if (name == parent.name) {
      return true;
    }

    name = splitStringByLastSlash(name);
  }

  return false;
}

inline bool isLeaf(
  const std::unordered_set<std::string> & diag_name_set,
  const diagnostic_msgs::msg::DiagnosticStatus & diag)
{
  return diag_name_set.count(diag.name) == 0;
}

inline std::unordered_set<std::string> createDiagNameSet(
  const std::vector<diagnostic_msgs::msg::DiagnosticStatus> & diagnostics)
{
  std::unordered_set<std::string> diag_name_set;

  for (const auto & diag : diagnostics) {
    diag_name_set.insert(splitStringByLastSlash(diag.name));
  }

  return diag_name_set;
}

inline std::vector<diagnostic_msgs::msg::DiagnosticStatus> extractLeafDiagnostics(
  const std::vector<diagnostic_msgs::msg::DiagnosticStatus> & diagnostics)
{
  const auto diag_name_set = createDiagNameSet(diagnostics);

  std::vector<diagnostic_msgs::msg::DiagnosticStatus> leaf_diagnostics;
  for (const auto & diag : diagnostics) {
    if (isLeaf(diag_name_set, diag)) {
      leaf_diagnostics.emplace_back(diag);
    }
  }

  return leaf_diagnostics;
}

inline std::vector<diagnostic_msgs::msg::DiagnosticStatus> extractLeafChildrenDiagnostics(
  const diagnostic_msgs::msg::DiagnosticStatus & parent,
  const std::vector<diagnostic_msgs::msg::DiagnosticStatus> & diagnostics)
{
  std::vector<diagnostic_msgs::msg::DiagnosticStatus> leaf_children_diagnostics;
  for (const auto & diag : extractLeafDiagnostics(diagnostics)) {
    if (isChild(diag, parent)) {
      leaf_children_diagnostics.emplace_back(diag);
    }
  }

  return leaf_children_diagnostics;
}

}  // namespace diagnostics_filter

#endif  // SYSTEM_ERROR_MONITOR__DIAGNOSTICS_FILTER_HPP_
