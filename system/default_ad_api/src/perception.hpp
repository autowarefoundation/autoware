// Copyright 2022 TIER IV, Inc.
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

#ifndef PERCEPTION_HPP_
#define PERCEPTION_HPP_

#include <autoware_ad_api_specs/perception.hpp>
#include <component_interface_specs/perception.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/dynamic_object.hpp>
#include <autoware_adapi_v1_msgs/msg/dynamic_object_path.hpp>
#include <autoware_adapi_v1_msgs/msg/object_classification.hpp>
#include <autoware_auto_perception_msgs/msg/shape.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <unordered_map>
#include <vector>

// This file should be included after messages.
#include "utils/types.hpp"

namespace default_ad_api
{

class PerceptionNode : public rclcpp::Node
{
public:
  explicit PerceptionNode(const rclcpp::NodeOptions & options);

private:
  Pub<autoware_ad_api::perception::DynamicObjectArray> pub_object_recognized_;
  Sub<perception_interface::ObjectRecognition> sub_object_recognized_;
  void object_recognize(const perception_interface::ObjectRecognition::Message::ConstSharedPtr msg);
  uint8_t mapping(
    std::unordered_map<uint8_t, uint8_t> hash_map, uint8_t input, uint8_t default_value);
};

}  // namespace default_ad_api

#endif  // PERCEPTION_HPP_
