// Copyright 2023 TIER IV, Inc.
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

#include "object_velocity_splitter_node.hpp"

#include "autoware/universe_utils/geometry/geometry.hpp"

#include <memory>
#include <string>
#include <vector>

namespace
{
template <class T>
bool update_param(
  const std::vector<rclcpp::Parameter> & params, const std::string & name, T & value)
{
  const auto itr = std::find_if(
    params.cbegin(), params.cend(),
    [&name](const rclcpp::Parameter & p) { return p.get_name() == name; });

  // Not found
  if (itr == params.cend()) {
    return false;
  }

  value = itr->template get_value<T>();
  return true;
}
}  // namespace

namespace autoware::object_velocity_splitter
{
using autoware_perception_msgs::msg::DetectedObject;
using autoware_perception_msgs::msg::DetectedObjects;

ObjectVelocitySplitterNode::ObjectVelocitySplitterNode(const rclcpp::NodeOptions & node_options)
: Node("object_velocity_splitter", node_options)
{
  // Parameter Server
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&ObjectVelocitySplitterNode::onSetParam, this, std::placeholders::_1));

  // Node Parameter
  node_param_.velocity_threshold = declare_parameter<double>("velocity_threshold");

  // Subscriber
  sub_objects_ = create_subscription<DetectedObjects>(
    "~/input/objects", rclcpp::QoS{1},
    std::bind(&ObjectVelocitySplitterNode::onObjects, this, std::placeholders::_1));

  // Publisher
  pub_high_speed_objects_ = create_publisher<DetectedObjects>("~/output/high_speed_objects", 1);
  pub_low_speed_objects_ = create_publisher<DetectedObjects>("~/output/low_speed_objects", 1);
}

void ObjectVelocitySplitterNode::onObjects(const DetectedObjects::ConstSharedPtr msg)
{
  DetectedObjects high_speed_objects;
  DetectedObjects low_speed_objects;
  high_speed_objects.header = msg->header;
  low_speed_objects.header = msg->header;

  for (const auto & object : msg->objects) {
    if (
      std::abs(autoware::universe_utils::calcNorm(
        object.kinematics.twist_with_covariance.twist.linear)) < node_param_.velocity_threshold) {
      low_speed_objects.objects.emplace_back(object);
    } else {
      high_speed_objects.objects.emplace_back(object);
    }
  }
  // publish
  pub_high_speed_objects_->publish(high_speed_objects);
  pub_low_speed_objects_->publish(low_speed_objects);
}

rcl_interfaces::msg::SetParametersResult ObjectVelocitySplitterNode::onSetParam(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;

  try {
    // Node Parameter
    {
      auto & p = node_param_;

      // Update params
      update_param(params, "velocity_threshold", p.velocity_threshold);
    }
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    result.successful = false;
    result.reason = e.what();
    return result;
  }

  result.successful = true;
  result.reason = "success";
  return result;
}

}  // namespace autoware::object_velocity_splitter

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::object_velocity_splitter::ObjectVelocitySplitterNode)
