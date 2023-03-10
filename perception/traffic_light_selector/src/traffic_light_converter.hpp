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

#ifndef TRAFFIC_LIGHT_CONVERTER_HPP_
#define TRAFFIC_LIGHT_CONVERTER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_perception_msgs/msg/traffic_signal_array.hpp>
#include <autoware_perception_msgs/msg/traffic_light_array.hpp>

class TrafficLightConverter : public rclcpp::Node
{
public:
  explicit TrafficLightConverter(const rclcpp::NodeOptions & options);

private:
  using OldMessage = autoware_auto_perception_msgs::msg::TrafficSignalArray;
  using NewMessage = autoware_perception_msgs::msg::TrafficLightArray;
  rclcpp::Subscription<OldMessage>::SharedPtr sub_;
  rclcpp::Publisher<NewMessage>::SharedPtr pub_;
  void on_msg(const OldMessage::ConstSharedPtr msg);
};

#endif  // TRAFFIC_LIGHT_CONVERTER_HPP_
