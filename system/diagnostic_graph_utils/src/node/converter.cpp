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

#include "converter.hpp"

#include <memory>

namespace diagnostic_graph_utils
{

ConverterNode::ConverterNode() : Node("converter")
{
  using std::placeholders::_1;
  pub_array_ = create_publisher<DiagnosticArray>("/diagnostics_array", rclcpp::QoS(1));
  sub_graph_.register_update_callback(std::bind(&ConverterNode::on_update, this, _1));
  sub_graph_.subscribe(*this, 1);
}

void ConverterNode::on_update(DiagGraph::ConstSharedPtr graph)
{
  DiagnosticArray array;
  array.header.stamp = graph->updated_stamp();
  for (const auto & unit : graph->units()) {
    if (unit->path().empty()) continue;
    array.status.push_back(unit->create_diagnostic_status());
  }
  pub_array_->publish(array);
}

}  // namespace diagnostic_graph_utils

int main(int argc, char ** argv)
{
  using diagnostic_graph_utils::ConverterNode;
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<ConverterNode>();
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  rclcpp::shutdown();
}
