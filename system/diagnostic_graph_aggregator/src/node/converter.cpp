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

#include <algorithm>

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

std::string parent_path(const std::string & path)
{
  return path.substr(0, path.rfind('/'));
}

auto create_tree(const DiagnosticGraph & graph)
{
  std::map<std::string, std::unique_ptr<TreeNode>, std::greater<std::string>> tree;
  for (const auto & node : graph.nodes) {
    tree.emplace(node.status.name, std::make_unique<TreeNode>(true));
  }
  for (const auto & node : graph.nodes) {
    std::string path = node.status.name;
    while (path = parent_path(path), !path.empty()) {
      if (tree.count(path)) break;
      tree.emplace(path, std::make_unique<TreeNode>(false));
    }
  }
  for (const auto & [path, node] : tree) {
    const auto parent = parent_path(path);
    node->parent = parent.empty() ? nullptr : tree[parent].get();
  }
  return tree;
}

ConverterNode::ConverterNode() : Node("converter")
{
  using std::placeholders::_1;
  const auto qos_graph = rclcpp::QoS(1);
  const auto qos_array = rclcpp::QoS(1);

  const auto callback = std::bind(&ConverterNode::on_graph, this, _1);
  sub_graph_ = create_subscription<DiagnosticGraph>("/diagnostics_graph", qos_graph, callback);
  pub_array_ = create_publisher<DiagnosticArray>("/diagnostics_agg", qos_array);

  initialize_tree_ = false;
  complement_tree_ = declare_parameter<bool>("complement_tree");
}

void ConverterNode::on_graph(const DiagnosticGraph::ConstSharedPtr msg)
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

  if (complement_tree_ && !initialize_tree_) {
    initialize_tree_ = true;
    tree_ = create_tree(*msg);
  }

  if (complement_tree_) {
    for (const auto & [path, node] : tree_) {
      node->level = DiagnosticStatus::OK;
    }
    for (const auto & node : msg->nodes) {
      tree_[node.status.name]->level = node.status.level;
    }
    for (const auto & [path, node] : tree_) {
      if (!node->parent) continue;
      node->parent->level = std::max(node->parent->level, node->level);
    }
    for (const auto & [path, node] : tree_) {
      if (node->leaf) continue;
      message.status.emplace_back();
      message.status.back().name = path;
      message.status.back().level = node->level;
    }
  }

  pub_array_->publish(message);
}

}  // namespace diagnostic_graph_aggregator

int main(int argc, char ** argv)
{
  using diagnostic_graph_aggregator::ConverterNode;
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<ConverterNode>();
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  rclcpp::shutdown();
}
