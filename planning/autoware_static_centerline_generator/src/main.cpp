// Copyright 2022 Tier IV, Inc.
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

#include "static_centerline_generator_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // initialize node
  rclcpp::NodeOptions node_options;
  auto node =
    std::make_shared<autoware::static_centerline_generator::StaticCenterlineGeneratorNode>(
      node_options);

  // get ros parameter
  const auto mode = node->declare_parameter<std::string>("mode");

  // process
  if (mode == "AUTO") {
    node->generate_centerline();
    node->validate();
    node->save_map();
  } else if (mode == "GUI") {
    node->generate_centerline();
  } else if (mode == "VMB") {
    // Do nothing
  } else {
    throw std::invalid_argument("The `mode` is invalid.");
  }

  // NOTE: spin node to keep showing debug path/trajectory in rviz with transient local
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
