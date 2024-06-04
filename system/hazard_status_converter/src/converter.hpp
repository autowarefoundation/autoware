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

#include <diagnostic_graph_utils/subscription.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_system_msgs/msg/hazard_status_stamped.hpp>

#include <unordered_set>

namespace hazard_status_converter
{

class Converter : public rclcpp::Node
{
public:
  explicit Converter(const rclcpp::NodeOptions & options);

private:
  using HazardStatusStamped = autoware_system_msgs::msg::HazardStatusStamped;
  using DiagGraph = diagnostic_graph_utils::DiagGraph;
  using DiagUnit = diagnostic_graph_utils::DiagUnit;
  void on_create(DiagGraph::ConstSharedPtr graph);
  void on_update(DiagGraph::ConstSharedPtr graph);
  diagnostic_graph_utils::DiagGraphSubscription sub_graph_;
  rclcpp::Publisher<HazardStatusStamped>::SharedPtr pub_hazard_;

  DiagUnit * auto_mode_root_;
  std::unordered_set<DiagUnit *> auto_mode_tree_;
};

}  // namespace hazard_status_converter

#endif  // CONVERTER_HPP_
