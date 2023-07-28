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

#include "perception.hpp"

#include <vector>

namespace default_ad_api
{

using DynamicObjectArray = autoware_ad_api::perception::DynamicObjectArray;
using ObjectClassification = autoware_adapi_v1_msgs::msg::ObjectClassification;
using DynamicObject = autoware_adapi_v1_msgs::msg::DynamicObject;
using DynamicObjectPath = autoware_adapi_v1_msgs::msg::DynamicObjectPath;
using API_Shape = shape_msgs::msg::SolidPrimitive;
using Shape = autoware_auto_perception_msgs::msg::Shape;

std::unordered_map<uint8_t, uint8_t> shape_type_ = {
  {Shape::BOUNDING_BOX, API_Shape::BOX},
  {Shape::CYLINDER, API_Shape::CYLINDER},
  {Shape::POLYGON, API_Shape::PRISM},
};

PerceptionNode::PerceptionNode(const rclcpp::NodeOptions & options) : Node("perception", options)
{
  const auto adaptor = component_interface_utils::NodeAdaptor(this);
  adaptor.init_pub(pub_object_recognized_);
  adaptor.init_sub(sub_object_recognized_, this, &PerceptionNode::object_recognize);
}

uint8_t PerceptionNode::mapping(
  std::unordered_map<uint8_t, uint8_t> hash_map, uint8_t input, uint8_t default_value)
{
  if (hash_map.find(input) == hash_map.end()) {
    return default_value;
  } else {
    return hash_map[input];
  }
}

void PerceptionNode::object_recognize(
  const perception_interface::ObjectRecognition::Message::ConstSharedPtr msg)
{
  DynamicObjectArray::Message objects;
  objects.header = msg->header;
  for (const auto & msg_object : msg->objects) {
    DynamicObject object;
    object.id = msg_object.object_id;
    object.existence_probability = msg_object.existence_probability;
    for (const auto & msg_classification : msg_object.classification) {
      ObjectClassification classification;
      classification.label = msg_classification.label;
      classification.probability = msg_classification.probability;
      object.classification.insert(object.classification.begin(), classification);
    }
    object.kinematics.pose = msg_object.kinematics.initial_pose_with_covariance.pose;
    object.kinematics.twist = msg_object.kinematics.initial_twist_with_covariance.twist;
    object.kinematics.accel = msg_object.kinematics.initial_acceleration_with_covariance.accel;
    for (const auto & msg_predicted_path : msg_object.kinematics.predicted_paths) {
      DynamicObjectPath predicted_path;
      for (const auto & msg_path : msg_predicted_path.path) {
        predicted_path.path.insert(predicted_path.path.begin(), msg_path);
      }
      predicted_path.time_step = msg_predicted_path.time_step;
      predicted_path.confidence = msg_predicted_path.confidence;
      object.kinematics.predicted_paths.insert(
        object.kinematics.predicted_paths.begin(), predicted_path);
    }
    object.shape.type = mapping(shape_type_, msg_object.shape.type, API_Shape::PRISM);
    object.shape.dimensions = {
      msg_object.shape.dimensions.x, msg_object.shape.dimensions.y, msg_object.shape.dimensions.z};
    object.shape.polygon = msg_object.shape.footprint;
    objects.objects.insert(objects.objects.begin(), object);
  }

  pub_object_recognized_->publish(objects);
}

}  // namespace default_ad_api

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(default_ad_api::PerceptionNode)
