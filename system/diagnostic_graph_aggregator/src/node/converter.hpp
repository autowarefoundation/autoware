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

#ifndef NODE__CONVERTER_HPP_
#define NODE__CONVERTER_HPP_

#include "graph/types.hpp"

#include <rclcpp/rclcpp.hpp>

namespace diagnostic_graph_aggregator
{

class ToolNode : public rclcpp::Node
{
public:
  ToolNode();

private:
  rclcpp::Subscription<DiagnosticGraph>::SharedPtr sub_graph_;
  rclcpp::Publisher<DiagnosticArray>::SharedPtr pub_array_;
  void on_graph(const DiagnosticGraph::ConstSharedPtr msg);
};

}  // namespace diagnostic_graph_aggregator

#endif  // NODE__CONVERTER_HPP_
