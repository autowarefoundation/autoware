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
#include <tier4_system_msgs/msg/diag_graph_status.hpp>
#include <tier4_system_msgs/msg/diag_graph_struct.hpp>
#include <tier4_system_msgs/msg/diag_leaf_status.hpp>
#include <tier4_system_msgs/msg/diag_leaf_struct.hpp>
#include <tier4_system_msgs/msg/diag_link_status.hpp>
#include <tier4_system_msgs/msg/diag_link_struct.hpp>
#include <tier4_system_msgs/msg/diag_node_status.hpp>
#include <tier4_system_msgs/msg/diag_node_struct.hpp>
#include <tier4_system_msgs/msg/diagnostic_graph.hpp>
#include <tier4_system_msgs/msg/diagnostic_link.hpp>
#include <tier4_system_msgs/msg/diagnostic_node.hpp>

#include <vector>

namespace diagnostic_graph_aggregator
{

using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using tier4_system_msgs::msg::DiagGraphStatus;
using tier4_system_msgs::msg::DiagGraphStruct;
using tier4_system_msgs::msg::DiagLeafStatus;
using tier4_system_msgs::msg::DiagLeafStruct;
using tier4_system_msgs::msg::DiagLinkStatus;
using tier4_system_msgs::msg::DiagLinkStruct;
using tier4_system_msgs::msg::DiagNodeStatus;
using tier4_system_msgs::msg::DiagNodeStruct;
using DiagnosticLevel = DiagnosticStatus::_level_type;

struct PathConfig;
struct EditConfig;
struct UnitConfig;
struct LinkConfig;

class TreeData;
class UnitLink;
class BaseUnit;
class NodeUnit;
class DiagUnit;
class Graph;
class UnitLoader;

}  // namespace diagnostic_graph_aggregator

#endif  // COMMON__GRAPH__TYPES_HPP_
