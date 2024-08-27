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

#ifndef DIAGNOSTICS_HPP_
#define DIAGNOSTICS_HPP_

#include "diagnostic_graph_utils/subscription.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/diag_graph_status.hpp>
#include <autoware_adapi_v1_msgs/msg/diag_graph_struct.hpp>

namespace autoware::default_adapi
{

class DiagnosticsNode : public rclcpp::Node
{
public:
  explicit DiagnosticsNode(const rclcpp::NodeOptions & options);

private:
  using DiagGraph = diagnostic_graph_utils::DiagGraph;
  using DiagUnit = diagnostic_graph_utils::DiagUnit;
  using DiagLink = diagnostic_graph_utils::DiagLink;
  void on_create(DiagGraph::ConstSharedPtr graph);
  void on_update(DiagGraph::ConstSharedPtr graph);
  rclcpp::Publisher<autoware_adapi_v1_msgs::msg::DiagGraphStruct>::SharedPtr pub_struct_;
  rclcpp::Publisher<autoware_adapi_v1_msgs::msg::DiagGraphStatus>::SharedPtr pub_status_;
  diagnostic_graph_utils::DiagGraphSubscription sub_graph_;
};

}  // namespace autoware::default_adapi

#endif  // DIAGNOSTICS_HPP_
