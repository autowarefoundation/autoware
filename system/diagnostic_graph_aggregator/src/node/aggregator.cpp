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

#include "aggregator.hpp"

#include <memory>
#include <sstream>
#include <string>

namespace diagnostic_graph_aggregator
{

AggregatorNode::AggregatorNode(const rclcpp::NodeOptions & options) : Node("aggregator", options)
{
  const auto stamp = now();

  // Init diagnostics graph.
  {
    const auto graph_file = declare_parameter<std::string>("graph_file");
    std::ostringstream id;
    id << std::hex << stamp.nanoseconds();
    graph_.create(graph_file, id.str());
  }

  // Init plugins.
  if (declare_parameter<bool>("use_operation_mode_availability")) {
    modes_ = std::make_unique<ModesAvailability>(*this, graph_);
  }

  // Init ros interface.
  {
    const auto qos_input = rclcpp::QoS(declare_parameter<int64_t>("input_qos_depth"));
    const auto qos_unknown = rclcpp::QoS(1);  // TODO(Takagi, Isamu): parameter
    const auto qos_struct = rclcpp::QoS(1).transient_local();
    const auto qos_status = rclcpp::QoS(declare_parameter<int64_t>("graph_qos_depth"));
    const auto callback = std::bind(&AggregatorNode::on_diag, this, std::placeholders::_1);
    sub_input_ = create_subscription<DiagnosticArray>("/diagnostics", qos_input, callback);
    pub_unknown_ = create_publisher<DiagnosticArray>("/diagnostics_graph/unknowns", qos_unknown);
    pub_struct_ = create_publisher<DiagGraphStruct>("/diagnostics_graph/struct", qos_struct);
    pub_status_ = create_publisher<DiagGraphStatus>("/diagnostics_graph/status", qos_status);

    const auto rate = rclcpp::Rate(declare_parameter<double>("rate"));
    timer_ = rclcpp::create_timer(this, get_clock(), rate.period(), [this]() { on_timer(); });
  }

  // Send structure topic once.
  pub_struct_->publish(graph_.create_struct(stamp));
}

AggregatorNode::~AggregatorNode()
{
  // For unique_ptr members.
}

DiagnosticArray AggregatorNode::create_unknown_diags(const rclcpp::Time & stamp)
{
  DiagnosticArray msg;
  msg.header.stamp = stamp;
  for (const auto & [name, diag] : unknown_diags_) msg.status.push_back(diag);
  return msg;
}

void AggregatorNode::on_timer()
{
  // Check timeout of diag units.
  const auto stamp = now();
  graph_.update(stamp);

  // Publish status.
  pub_status_->publish(graph_.create_status(stamp));
  pub_unknown_->publish(create_unknown_diags(stamp));
  if (modes_) modes_->update(stamp);
}

void AggregatorNode::on_diag(const DiagnosticArray & msg)
{
  // Update status. Store it as unknown if it does not exist in the graph.
  const auto & stamp = msg.header.stamp;
  for (const auto & status : msg.status) {
    if (!graph_.update(stamp, status)) {
      unknown_diags_[status.name] = status;
    }
  }

  // TODO(Takagi, Isamu): Publish immediately when graph status changes.
  // pub_status_->publish();
}

}  // namespace diagnostic_graph_aggregator

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(diagnostic_graph_aggregator::AggregatorNode)
