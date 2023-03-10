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

#ifndef TRAFFIC_LIGHT_SELECTOR_HPP_
#define TRAFFIC_LIGHT_SELECTOR_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <autoware_perception_msgs/msg/traffic_light_array.hpp>
#include <autoware_perception_msgs/msg/traffic_signal_array.hpp>

#include <lanelet2_core/Forward.h>

#include <unordered_map>

class TrafficLightSelector : public rclcpp::Node
{
public:
  explicit TrafficLightSelector(const rclcpp::NodeOptions & options);

private:
  using LaneletMapBin = autoware_auto_mapping_msgs::msg::HADMapBin;
  using TrafficLightArray = autoware_perception_msgs::msg::TrafficLightArray;
  using TrafficSignalArray = autoware_perception_msgs::msg::TrafficSignalArray;
  rclcpp::Subscription<LaneletMapBin>::SharedPtr sub_map_;
  rclcpp::Subscription<TrafficLightArray>::SharedPtr sub_tlr_;
  rclcpp::Publisher<TrafficSignalArray>::SharedPtr pub_;

  void on_map(const LaneletMapBin::ConstSharedPtr msg);
  void on_msg(const TrafficLightArray::ConstSharedPtr msg);

  std::unordered_map<lanelet::Id, lanelet::Id> mapping_;
};

#endif  // TRAFFIC_LIGHT_SELECTOR_HPP_
