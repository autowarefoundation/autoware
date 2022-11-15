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

#include "static_centerline_optimizer/static_centerline_optimizer_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // initialize node
  rclcpp::NodeOptions node_options;
  auto node =
    std::make_shared<static_centerline_optimizer::StaticCenterlineOptimizerNode>(node_options);

  // get ros parameter
  const bool run_background = node->declare_parameter<bool>("run_background");

  // process
  if (!run_background) {
    node->run();
  }

  // NOTE: spin node to keep showing debug path/trajectory in rviz with transient local
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
