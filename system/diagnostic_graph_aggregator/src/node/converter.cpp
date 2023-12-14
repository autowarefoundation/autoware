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

#include "converter.hpp"

#include <memory>
#include <string>

namespace diagnostic_graph_aggregator
{

std::string level_to_string(DiagnosticLevel level)
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

ToolNode::ToolNode() : Node("diagnostic_graph_aggregator_converter")
{
  using std::placeholders::_1;
  const auto qos_graph = rclcpp::QoS(1);
  const auto qos_array = rclcpp::QoS(1);

  const auto callback = std::bind(&ToolNode::on_graph, this, _1);
  sub_graph_ = create_subscription<DiagnosticGraph>("/diagnostics_graph", qos_graph, callback);
  pub_array_ = create_publisher<DiagnosticArray>("/diagnostics_array", qos_array);
}

void ToolNode::on_graph(const DiagnosticGraph::ConstSharedPtr msg)
{
  DiagnosticArray message;
  message.header.stamp = msg->stamp;
  message.status.reserve(msg->nodes.size());
  for (const auto & node : msg->nodes) {
    message.status.push_back(node.status);
    for (const auto & link : node.links) {
      diagnostic_msgs::msg::KeyValue kv;
      const auto & status = msg->nodes[link.index].status;
      kv.key = status.name;
      kv.value = level_to_string(status.level);
      if (link.used) {
        message.status.back().values.push_back(kv);
      }
    }
  }
  pub_array_->publish(message);
}

}  // namespace diagnostic_graph_aggregator

int main(int argc, char ** argv)
{
  using diagnostic_graph_aggregator::ToolNode;
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<ToolNode>();
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  rclcpp::shutdown();
}
