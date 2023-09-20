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

#include "radar_crossing_objects_noise_filter/radar_crossing_objects_noise_filter_node.hpp"

#include "tier4_autoware_utils/geometry/geometry.hpp"
#include "tier4_autoware_utils/math/normalization.hpp"

#include <tf2/utils.h>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

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

namespace radar_crossing_objects_noise_filter
{
using autoware_auto_perception_msgs::msg::DetectedObject;
using autoware_auto_perception_msgs::msg::DetectedObjects;

RadarCrossingObjectsNoiseFilterNode::RadarCrossingObjectsNoiseFilterNode(
  const rclcpp::NodeOptions & node_options)
: Node("radar_crossing_objects_noise_filter", node_options)
{
  // Parameter Server
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&RadarCrossingObjectsNoiseFilterNode::onSetParam, this, std::placeholders::_1));

  // Node Parameter
  node_param_.angle_threshold = declare_parameter<double>("angle_threshold", 1.0472);
  node_param_.velocity_threshold = declare_parameter<double>("velocity_threshold", 3.0);

  // Subscriber
  sub_objects_ = create_subscription<DetectedObjects>(
    "~/input/objects", rclcpp::QoS{1},
    std::bind(&RadarCrossingObjectsNoiseFilterNode::onObjects, this, std::placeholders::_1));

  // Publisher
  pub_filtered_objects_ = create_publisher<DetectedObjects>("~/output/filtered_objects", 1);
  pub_noise_objects_ = create_publisher<DetectedObjects>("~/output/noise_objects", 1);
}

void RadarCrossingObjectsNoiseFilterNode::onObjects(
  const DetectedObjects::ConstSharedPtr objects_data_)
{
  DetectedObjects filtered_objects;
  DetectedObjects noise_objects;
  filtered_objects.header = objects_data_->header;
  noise_objects.header = objects_data_->header;

  for (const auto & object : objects_data_->objects) {
    if (isNoise(object)) {
      noise_objects.objects.emplace_back(object);
    } else {
      filtered_objects.objects.emplace_back(object);
    }
  }
  // publish
  pub_filtered_objects_->publish(filtered_objects);
  pub_noise_objects_->publish(noise_objects);
}

rcl_interfaces::msg::SetParametersResult RadarCrossingObjectsNoiseFilterNode::onSetParam(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;

  try {
    // Node Parameter
    {
      auto & p = node_param_;

      // Update params
      update_param(params, "angle_threshold", p.angle_threshold);
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

bool RadarCrossingObjectsNoiseFilterNode::isNoise(const DetectedObject & object)
{
  const double velocity =
    std::abs(tier4_autoware_utils::calcNorm(object.kinematics.twist_with_covariance.twist.linear));
  const double object_angle = tf2::getYaw(object.kinematics.pose_with_covariance.pose.orientation);
  const double object_position_angle = std::atan2(
    object.kinematics.pose_with_covariance.pose.position.y,
    object.kinematics.pose_with_covariance.pose.position.x);
  const double crossing_yaw =
    tier4_autoware_utils::normalizeRadian(object_angle - object_position_angle);

  if (
    velocity > node_param_.velocity_threshold &&
    abs(std::cos(crossing_yaw)) < abs(std::cos(node_param_.angle_threshold))) {
    return true;
  } else {
    return false;
  }
}

}  // namespace radar_crossing_objects_noise_filter

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(
  radar_crossing_objects_noise_filter::RadarCrossingObjectsNoiseFilterNode)
