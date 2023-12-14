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

#ifndef COMMON__GRAPH__TYPES_HPP_
#define COMMON__GRAPH__TYPES_HPP_

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <tier4_system_msgs/msg/diagnostic_graph.hpp>
#include <tier4_system_msgs/msg/diagnostic_link.hpp>
#include <tier4_system_msgs/msg/diagnostic_node.hpp>

namespace diagnostic_graph_aggregator
{

using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using tier4_system_msgs::msg::DiagnosticGraph;
using tier4_system_msgs::msg::DiagnosticLink;
using tier4_system_msgs::msg::DiagnosticNode;
using DiagnosticLevel = DiagnosticStatus::_level_type;

class BaseUnit;
class DiagUnit;

}  // namespace diagnostic_graph_aggregator

#endif  // COMMON__GRAPH__TYPES_HPP_
