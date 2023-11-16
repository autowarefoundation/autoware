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

#include "radar_object_clustering/radar_object_clustering_node.hpp"

#include "object_recognition_utils/object_recognition_utils.hpp"
#include "tier4_autoware_utils/geometry/geometry.hpp"
#include "tier4_autoware_utils/math/unit_conversion.hpp"

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

double get_distance(const autoware_auto_perception_msgs::msg::DetectedObject & object)
{
  const auto & position = object.kinematics.pose_with_covariance.pose.position;
  return std::hypot(position.x, position.y);
}

}  // namespace

namespace radar_object_clustering
{
using autoware_auto_perception_msgs::msg::DetectedObject;
using autoware_auto_perception_msgs::msg::DetectedObjects;

RadarObjectClusteringNode::RadarObjectClusteringNode(const rclcpp::NodeOptions & node_options)
: Node("radar_object_clustering", node_options)
{
  // Parameter Server
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&RadarObjectClusteringNode::onSetParam, this, std::placeholders::_1));

  // Node Parameter
  node_param_.angle_threshold = declare_parameter<double>("angle_threshold");
  node_param_.distance_threshold = declare_parameter<double>("distance_threshold");
  node_param_.velocity_threshold = declare_parameter<double>("velocity_threshold");
  node_param_.is_fixed_label = declare_parameter<bool>("is_fixed_label");
  node_param_.fixed_label = declare_parameter<std::string>("fixed_label");
  node_param_.is_fixed_size = declare_parameter<bool>("is_fixed_size");
  node_param_.size_x = declare_parameter<double>("size_x");
  node_param_.size_y = declare_parameter<double>("size_y");
  node_param_.size_z = declare_parameter<double>("size_z");

  // Subscriber
  sub_objects_ = create_subscription<DetectedObjects>(
    "~/input/objects", rclcpp::QoS{1},
    std::bind(&RadarObjectClusteringNode::onObjects, this, std::placeholders::_1));

  // Publisher
  pub_objects_ = create_publisher<DetectedObjects>("~/output/objects", 1);
}

void RadarObjectClusteringNode::onObjects(const DetectedObjects::ConstSharedPtr objects_data_)
{
  DetectedObjects output_objects;
  output_objects.header = objects_data_->header;
  std::vector<DetectedObject> objects = objects_data_->objects;

  std::vector<bool> used_flags(objects.size(), false);

  auto func = [](DetectedObject const & lhs, DetectedObject const & rhs) {
    return get_distance(lhs) < get_distance(rhs);
  };
  std::sort(objects.begin(), objects.end(), func);

  for (size_t i = 0; i < objects.size(); i++) {
    if (used_flags.at(i) == true) {
      continue;
    }

    std::vector<DetectedObject> clustered_objects;
    used_flags.at(i) = true;
    clustered_objects.emplace_back(objects.at(i));

    for (size_t j = i; j < objects.size(); ++j) {
      if (used_flags.at(j) == false && isSameObject(objects.at(i), objects.at(j))) {
        used_flags.at(j) = true;
        clustered_objects.emplace_back(objects.at(j));
      }
    }

    // clustering
    DetectedObject clustered_output_object;
    if (clustered_objects.size() == 1) {
      clustered_output_object = clustered_objects.at(0);
    } else {
      auto func_max_confidence = [](const DetectedObject & a, const DetectedObject & b) {
        return a.existence_probability < b.existence_probability;
      };
      auto iter = std::max_element(
        std::begin(clustered_objects), std::end(clustered_objects), func_max_confidence);

      // class label
      clustered_output_object.existence_probability = iter->existence_probability;
      clustered_output_object.classification = iter->classification;

      // kinematics
      clustered_output_object.kinematics = iter->kinematics;

      auto & pose = clustered_output_object.kinematics.pose_with_covariance.pose;
      auto func_sum_x = [](const double & a, const DetectedObject & b) {
        return a + b.kinematics.pose_with_covariance.pose.position.x;
      };
      pose.position.x =
        std::accumulate(
          std::begin(clustered_objects), std::end(clustered_objects), 0.0, func_sum_x) /
        clustered_objects.size();
      auto func_sum_y = [](const double & a, const DetectedObject & b) {
        return a + b.kinematics.pose_with_covariance.pose.position.y;
      };
      pose.position.y =
        std::accumulate(
          std::begin(clustered_objects), std::end(clustered_objects), 0.0, func_sum_y) /
        clustered_objects.size();
      pose.position.z = iter->kinematics.pose_with_covariance.pose.position.z;

      // Shape
      clustered_output_object.shape = iter->shape;
    }

    // Fixed label correction
    if (node_param_.is_fixed_label) {
      clustered_output_object.classification.at(0).label =
        object_recognition_utils::toLabel(node_param_.fixed_label);
    }

    // Fixed size correction
    if (node_param_.is_fixed_size) {
      clustered_output_object.shape.type = autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX;
      clustered_output_object.shape.dimensions.x = node_param_.size_x;
      clustered_output_object.shape.dimensions.y = node_param_.size_y;
      clustered_output_object.shape.dimensions.z = node_param_.size_z;
    }
    output_objects.objects.emplace_back(clustered_output_object);
  }
  pub_objects_->publish(output_objects);
}

bool RadarObjectClusteringNode::isSameObject(
  const DetectedObject & object_1, const DetectedObject & object_2)
{
  const double angle_diff = std::abs(tier4_autoware_utils::normalizeRadian(
    tf2::getYaw(object_1.kinematics.pose_with_covariance.pose.orientation) -
    tf2::getYaw(object_2.kinematics.pose_with_covariance.pose.orientation)));
  const double velocity_diff = std::abs(
    object_1.kinematics.twist_with_covariance.twist.linear.x -
    object_2.kinematics.twist_with_covariance.twist.linear.x);
  const double distance = tier4_autoware_utils::calcDistance2d(
    object_1.kinematics.pose_with_covariance.pose.position,
    object_2.kinematics.pose_with_covariance.pose.position);

  if (
    distance < node_param_.distance_threshold && angle_diff < node_param_.angle_threshold &&
    velocity_diff < node_param_.velocity_threshold) {
    return true;
  } else {
    return false;
  }
}

rcl_interfaces::msg::SetParametersResult RadarObjectClusteringNode::onSetParam(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;

  try {
    // Node Parameter
    {
      auto & p = node_param_;

      // Update params
      update_param(params, "angle_threshold", p.angle_threshold);
      update_param(params, "distance_threshold", p.distance_threshold);
      update_param(params, "velocity_threshold", p.velocity_threshold);
      update_param(params, "is_fixed_label", p.is_fixed_label);
      update_param(params, "fixed_label", p.fixed_label);
      update_param(params, "is_fixed_size", p.is_fixed_size);
      update_param(params, "size_x", p.size_x);
      update_param(params, "size_y", p.size_y);
      update_param(params, "size_z", p.size_z);
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
}  // namespace radar_object_clustering

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(radar_object_clustering::RadarObjectClusteringNode)
