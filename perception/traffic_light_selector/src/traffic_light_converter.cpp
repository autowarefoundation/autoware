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

#include "traffic_light_converter.hpp"

#include <unordered_map>
#include <vector>

namespace converter
{

using OldList = autoware_auto_perception_msgs::msg::TrafficSignalArray;
using OldData = autoware_auto_perception_msgs::msg::TrafficSignal;
using OldElem = autoware_auto_perception_msgs::msg::TrafficLight;
using NewList = autoware_perception_msgs::msg::TrafficLightArray;
using NewData = autoware_perception_msgs::msg::TrafficLight;
using NewElem = autoware_perception_msgs::msg::TrafficLightElement;

NewList convert(const OldList & input);
NewData convert(const OldData & input);
NewElem convert(const OldElem & input);

template <class T, class L>
std::vector<T> convert_vector(const L & input)
{
  std::vector<T> output;
  output.reserve(input.size());
  for (const auto & value : input) {
    output.push_back(convert(value));
  }
  return output;
}

NewList convert(const OldList & input)
{
  NewList output;
  output.stamp = input.header.stamp;
  output.lights = convert_vector<NewData>(input.signals);
  return output;
}

NewData convert(const OldData & input)
{
  NewData output;
  output.traffic_light_id = input.map_primitive_id;
  output.elements = convert_vector<NewElem>(input.lights);
  return output;
}

template <class K, class V>
V at_or(const std::unordered_map<K, V> & map, const K & key, const V & value)
{
  return map.count(key) ? map.at(key) : value;
}

NewElem convert(const OldElem & input)
{
  // clang-format off
  static const std::unordered_map<OldElem::_color_type, NewElem::_color_type> color_map({
    {OldElem::RED, NewElem::RED},
    {OldElem::AMBER, NewElem::AMBER},
    {OldElem::GREEN, NewElem::GREEN},
    {OldElem::WHITE, NewElem::WHITE}
  });
  static const std::unordered_map<OldElem::_shape_type, NewElem::_shape_type> shape_map({
    {OldElem::CIRCLE, NewElem::CIRCLE},
    {OldElem::LEFT_ARROW, NewElem::LEFT_ARROW},
    {OldElem::RIGHT_ARROW, NewElem::RIGHT_ARROW},
    {OldElem::UP_ARROW, NewElem::UP_ARROW},
    {OldElem::DOWN_ARROW, NewElem::DOWN_ARROW},
    {OldElem::DOWN_LEFT_ARROW, NewElem::DOWN_LEFT_ARROW},
    {OldElem::DOWN_RIGHT_ARROW, NewElem::DOWN_RIGHT_ARROW},
    {OldElem::CROSS, NewElem::CROSS}
  });
  static const std::unordered_map<OldElem::_status_type, NewElem::_status_type> status_map({
    {OldElem::SOLID_OFF, NewElem::SOLID_OFF},
    {OldElem::SOLID_ON, NewElem::SOLID_ON},
    {OldElem::FLASHING, NewElem::FLASHING}
  });
  // clang-format on

  NewElem output;
  output.color = at_or(color_map, input.color, NewElem::UNKNOWN);
  output.shape = at_or(shape_map, input.shape, NewElem::UNKNOWN);
  output.status = at_or(status_map, input.status, NewElem::UNKNOWN);
  output.confidence = input.confidence;
  return output;
}

}  // namespace converter

TrafficLightConverter::TrafficLightConverter(const rclcpp::NodeOptions & options)
: Node("traffic_light_converter", options)
{
  const auto callback = std::bind(&TrafficLightConverter::on_msg, this, std::placeholders::_1);
  sub_ = create_subscription<OldMessage>("~/sub/traffic_lights", rclcpp::QoS(1), callback);
  pub_ = create_publisher<NewMessage>("~/pub/traffic_lights", rclcpp::QoS(1));
}

void TrafficLightConverter::on_msg(const OldMessage::ConstSharedPtr msg)
{
  pub_->publish(converter::convert(*msg));
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(TrafficLightConverter)
