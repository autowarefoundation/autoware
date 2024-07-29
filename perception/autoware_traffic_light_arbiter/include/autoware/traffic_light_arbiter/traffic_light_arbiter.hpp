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

#ifndef AUTOWARE__TRAFFIC_LIGHT_ARBITER__TRAFFIC_LIGHT_ARBITER_HPP_
#define AUTOWARE__TRAFFIC_LIGHT_ARBITER__TRAFFIC_LIGHT_ARBITER_HPP_

#include <autoware/traffic_light_arbiter/signal_match_validator.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>

#include <lanelet2_core/Forward.h>

#include <memory>
#include <unordered_set>

namespace autoware
{
class TrafficLightArbiter : public rclcpp::Node
{
public:
  explicit TrafficLightArbiter(const rclcpp::NodeOptions & options);

private:
  using Element = autoware_perception_msgs::msg::TrafficLightElement;
  using LaneletMapBin = autoware_map_msgs::msg::LaneletMapBin;
  using TrafficSignalArray = autoware_perception_msgs::msg::TrafficLightGroupArray;
  using TrafficSignal = autoware_perception_msgs::msg::TrafficLightGroup;

  rclcpp::Subscription<LaneletMapBin>::SharedPtr map_sub_;
  rclcpp::Subscription<TrafficSignalArray>::SharedPtr perception_tlr_sub_;
  rclcpp::Subscription<TrafficSignalArray>::SharedPtr external_tlr_sub_;
  rclcpp::Publisher<TrafficSignalArray>::SharedPtr pub_;

  void onMap(const LaneletMapBin::ConstSharedPtr msg);
  void onPerceptionMsg(const TrafficSignalArray::ConstSharedPtr msg);
  void onExternalMsg(const TrafficSignalArray::ConstSharedPtr msg);
  void arbitrateAndPublish(const builtin_interfaces::msg::Time & stamp);

  std::unique_ptr<std::unordered_set<lanelet::Id>> map_regulatory_elements_set_;

  double external_time_tolerance_;
  double perception_time_tolerance_;
  bool external_priority_;
  bool enable_signal_matching_;

  TrafficSignalArray latest_perception_msg_;
  TrafficSignalArray latest_external_msg_;
  std::unique_ptr<SignalMatchValidator> signal_match_validator_;
};
}  // namespace autoware

#endif  // AUTOWARE__TRAFFIC_LIGHT_ARBITER__TRAFFIC_LIGHT_ARBITER_HPP_
