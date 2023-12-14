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
#include <string>

namespace diagnostic_graph_aggregator
{

MainNode::MainNode() : Node("diagnostic_graph_aggregator_aggregator")
{
  // Init diagnostics graph.
  {
    const auto file = declare_parameter<std::string>("graph_file");
    graph_.init(file);
  }

  // Init plugins.
  if (declare_parameter<bool>("use_operation_mode_availability")) {
    modes_ = std::make_unique<OperationModes>(*this, graph_.nodes());
  }

  // Init ros interface.
  {
    using std::placeholders::_1;
    const auto qos_input = rclcpp::QoS(declare_parameter<int64_t>("input_qos_depth"));
    const auto qos_graph = rclcpp::QoS(declare_parameter<int64_t>("graph_qos_depth"));

    const auto callback = std::bind(&MainNode::on_diag, this, _1);
    sub_input_ = create_subscription<DiagnosticArray>("/diagnostics", qos_input, callback);
    pub_graph_ = create_publisher<DiagnosticGraph>("/diagnostics_graph", qos_graph);

    const auto rate = rclcpp::Rate(declare_parameter<double>("rate"));
    timer_ = rclcpp::create_timer(this, get_clock(), rate.period(), [this]() { on_timer(); });
  }

  // Init debug mode.
  debug_ = declare_parameter<bool>("use_debug_mode");
}

MainNode::~MainNode()
{
  // for unique_ptr
}

void MainNode::on_timer()
{
  const auto stamp = now();
  pub_graph_->publish(graph_.report(stamp));
  if (debug_) graph_.debug();
  if (modes_) modes_->update(stamp);
}

void MainNode::on_diag(const DiagnosticArray::ConstSharedPtr msg)
{
  graph_.callback(now(), *msg);
}

}  // namespace diagnostic_graph_aggregator

int main(int argc, char ** argv)
{
  using diagnostic_graph_aggregator::MainNode;
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<MainNode>();
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  rclcpp::shutdown();
}
