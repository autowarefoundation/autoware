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

#ifndef CONVERTER_HPP_
#define CONVERTER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_system_msgs/msg/hazard_status_stamped.hpp>
#include <tier4_system_msgs/msg/diagnostic_graph.hpp>

#include <string>
#include <unordered_map>
#include <vector>

namespace hazard_status_converter
{

class Converter : public rclcpp::Node
{
public:
  explicit Converter(const rclcpp::NodeOptions & options);

private:
  using DiagnosticGraph = tier4_system_msgs::msg::DiagnosticGraph;
  using HazardStatusStamped = autoware_auto_system_msgs::msg::HazardStatusStamped;
  rclcpp::Subscription<DiagnosticGraph>::SharedPtr sub_graph_;
  rclcpp::Publisher<HazardStatusStamped>::SharedPtr pub_hazard_;
  void on_graph(const DiagnosticGraph::ConstSharedPtr msg);
};

}  // namespace hazard_status_converter

#endif  // CONVERTER_HPP_
