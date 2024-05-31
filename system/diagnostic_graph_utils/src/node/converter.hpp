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

#ifndef NODE__CONVERTER_HPP_
#define NODE__CONVERTER_HPP_

#include "diagnostic_graph_utils/subscription.hpp"

#include <rclcpp/rclcpp.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

namespace diagnostic_graph_utils
{

class ConverterNode : public rclcpp::Node
{
public:
  explicit ConverterNode(const rclcpp::NodeOptions & options);

private:
  using DiagnosticArray = diagnostic_msgs::msg::DiagnosticArray;
  using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
  void on_update(DiagGraph::ConstSharedPtr graph);
  rclcpp::Publisher<DiagnosticArray>::SharedPtr pub_array_;
  DiagGraphSubscription sub_graph_;
};

}  // namespace diagnostic_graph_utils

#endif  // NODE__CONVERTER_HPP_
