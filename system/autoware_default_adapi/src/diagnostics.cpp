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

#include "diagnostics.hpp"

#include <memory>
#include <unordered_map>

namespace autoware::default_adapi
{

DiagnosticsNode::DiagnosticsNode(const rclcpp::NodeOptions & options) : Node("diagnostics", options)
{
  using std::placeholders::_1;

  pub_struct_ = create_publisher<autoware_adapi_v1_msgs::msg::DiagGraphStruct>(
    "/api/system/diagnostics/struct", rclcpp::QoS(1).transient_local());
  pub_status_ = create_publisher<autoware_adapi_v1_msgs::msg::DiagGraphStatus>(
    "/api/system/diagnostics/status", rclcpp::QoS(1).best_effort());

  sub_graph_.register_create_callback(std::bind(&DiagnosticsNode::on_create, this, _1));
  sub_graph_.register_update_callback(std::bind(&DiagnosticsNode::on_update, this, _1));
  sub_graph_.subscribe(*this, 10);
}
void DiagnosticsNode::on_create(DiagGraph::ConstSharedPtr graph)
{
  const auto & units = graph->units();
  const auto & links = graph->links();

  std::unordered_map<DiagUnit *, size_t> unit_indices_;
  for (size_t i = 0; i < units.size(); ++i) {
    unit_indices_[units[i]] = i;
  }

  autoware_adapi_v1_msgs::msg::DiagGraphStruct msg;
  msg.stamp = graph->created_stamp();
  msg.id = graph->id();
  msg.nodes.reserve(units.size());
  msg.links.reserve(links.size());
  for (const auto & unit : units) {
    msg.nodes.emplace_back();
    msg.nodes.back().path = unit->path();
  }
  for (const auto & link : links) {
    msg.links.emplace_back();
    msg.links.back().parent = unit_indices_.at(link->parent());
    msg.links.back().child = unit_indices_.at(link->child());
  }
  pub_struct_->publish(msg);
}

void DiagnosticsNode::on_update(DiagGraph::ConstSharedPtr graph)
{
  const auto & units = graph->units();

  autoware_adapi_v1_msgs::msg::DiagGraphStatus msg;
  msg.stamp = graph->updated_stamp();
  msg.id = graph->id();
  msg.nodes.reserve(units.size());
  for (const auto & unit : units) {
    msg.nodes.emplace_back();
    msg.nodes.back().level = unit->level();
  }
  pub_status_->publish(msg);
}

}  // namespace autoware::default_adapi

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::default_adapi::DiagnosticsNode)
