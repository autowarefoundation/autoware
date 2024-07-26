// Copyright 2024 TIER IV, Inc.
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

#include "perception_online_evaluator/utils/marker_utils.hpp"

#include "autoware/universe_utils/geometry/boost_polygon_utils.hpp"
#include "autoware/universe_utils/geometry/geometry.hpp"

#include <autoware/universe_utils/ros/marker_helper.hpp>
#include <autoware/universe_utils/ros/uuid_helper.hpp>

#include <lanelet2_core/primitives/CompoundPolygon.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>

#include <cstdint>

namespace marker_utils
{
using autoware::universe_utils::calcOffsetPose;
using autoware::universe_utils::createDefaultMarker;
using autoware::universe_utils::createMarkerColor;
using autoware::universe_utils::createMarkerOrientation;
using autoware::universe_utils::createMarkerScale;
using autoware::universe_utils::createPoint;
using std_msgs::msg::ColorRGBA;
using visualization_msgs::msg::Marker;

void addFootprintMarker(
  visualization_msgs::msg::Marker & marker, const geometry_msgs::msg::Pose & pose,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info)
{
  const double half_width = vehicle_info.vehicle_width_m / 2.0;
  const double base_to_front = vehicle_info.vehicle_length_m - vehicle_info.rear_overhang_m;
  const double base_to_rear = vehicle_info.rear_overhang_m;

  marker.points.push_back(
    autoware::universe_utils::calcOffsetPose(pose, base_to_front, -half_width, 0.0).position);
  marker.points.push_back(
    autoware::universe_utils::calcOffsetPose(pose, base_to_front, half_width, 0.0).position);
  marker.points.push_back(
    autoware::universe_utils::calcOffsetPose(pose, -base_to_rear, half_width, 0.0).position);
  marker.points.push_back(
    autoware::universe_utils::calcOffsetPose(pose, -base_to_rear, -half_width, 0.0).position);
  marker.points.push_back(marker.points.front());
}

MarkerArray createFootprintMarkerArray(
  const Pose & base_link_pose, const autoware::vehicle_info_utils::VehicleInfo vehicle_info,
  const std::string && ns, const int32_t & id, const float & r, const float & g, const float & b)
{
  const auto current_time = rclcpp::Clock{RCL_ROS_TIME}.now();
  MarkerArray msg;

  Marker marker = createDefaultMarker(
    "map", current_time, ns, id, Marker::LINE_STRIP, createMarkerScale(0.2, 0.2, 0.2),
    createMarkerColor(r, g, b, 0.999));
  marker.lifetime = rclcpp::Duration::from_seconds(1.5);

  MarkerArray marker_array;

  addFootprintMarker(marker, base_link_pose, vehicle_info);
  msg.markers.push_back(marker);
  return msg;
}

MarkerArray createPointsMarkerArray(
  const std::vector<Point> & points, const std::string & ns, const int32_t id, const float r,
  const float g, const float b)
{
  auto marker = createDefaultMarker(
    "map", rclcpp::Clock{RCL_ROS_TIME}.now(), ns, id, Marker::LINE_STRIP,
    createMarkerScale(0.05, 0.0, 0.0), createMarkerColor(r, g, b, 0.999));

  for (const auto & point : points) {
    marker.points.push_back(point);
  }

  MarkerArray msg;
  msg.markers.push_back(marker);
  return msg;
}

MarkerArray createPointsMarkerArray(
  const std::vector<Pose> & poses, const std::string & ns, const int32_t id, const float r,
  const float g, const float b)
{
  auto marker = createDefaultMarker(
    "map", rclcpp::Clock{RCL_ROS_TIME}.now(), ns, id, Marker::LINE_STRIP,
    createMarkerScale(0.05, 0.0, 0.0), createMarkerColor(r, g, b, 0.999));

  for (const auto & pose : poses) {
    marker.points.push_back(pose.position);
  }

  MarkerArray msg;
  msg.markers.push_back(marker);
  return msg;
}

MarkerArray createDeviationLines(
  const std::vector<Pose> & poses1, const std::vector<Pose> & poses2, const std::string & ns,
  const int32_t & first_id, const float r, const float g, const float b)
{
  MarkerArray msg;

  const size_t max_idx = std::min(poses1.size(), poses2.size());
  for (size_t i = 0; i < max_idx; ++i) {
    auto marker = createDefaultMarker(
      "map", rclcpp::Clock{RCL_ROS_TIME}.now(), ns, first_id + i, Marker::LINE_STRIP,
      createMarkerScale(0.01, 0.0, 0.0), createMarkerColor(r, g, b, 0.5));
    marker.points.push_back(poses1.at(i).position);
    marker.points.push_back(poses2.at(i).position);
    msg.markers.push_back(marker);
  }

  return msg;
}

MarkerArray createPoseMarkerArray(
  const Pose & pose, std::string && ns, const int32_t & id, const float & r, const float & g,
  const float & b)
{
  MarkerArray msg;

  Marker marker = createDefaultMarker(
    "map", rclcpp::Clock{RCL_ROS_TIME}.now(), ns, id, Marker::ARROW,
    createMarkerScale(0.7, 0.3, 0.3), createMarkerColor(r, g, b, 0.999));
  marker.pose = pose;
  msg.markers.push_back(marker);

  return msg;
}

MarkerArray createPosesMarkerArray(
  const std::vector<Pose> & poses, std::string && ns, const int32_t & first_id, const float & r,
  const float & g, const float & b, const float & x_scale, const float & y_scale,
  const float & z_scale)
{
  MarkerArray msg;

  for (size_t i = 0; i < poses.size(); ++i) {
    Marker marker = createDefaultMarker(
      "map", rclcpp::Clock{RCL_ROS_TIME}.now(), ns, first_id + i, Marker::ARROW,
      createMarkerScale(x_scale, y_scale, z_scale), createMarkerColor(r, g, b, 0.5));
    marker.pose = poses.at(i);
    msg.markers.push_back(marker);
  }

  return msg;
}

std_msgs::msg::ColorRGBA createColorFromString(const std::string & str)
{
  const auto hash = std::hash<std::string>{}(str);
  const auto r = (hash & 0xFF) / 255.0;
  const auto g = ((hash >> 8) & 0xFF) / 255.0;
  const auto b = ((hash >> 16) & 0xFF) / 255.0;
  return autoware::universe_utils::createMarkerColor(r, g, b, 0.5);
}

MarkerArray createObjectPolygonMarkerArray(
  const PredictedObject & object, std::string && ns, const int32_t & id, const float & r,
  const float & g, const float & b)
{
  MarkerArray msg;

  auto marker = createDefaultMarker(
    "map", rclcpp::Clock{RCL_ROS_TIME}.now(), ns, id, Marker::LINE_STRIP,
    createMarkerScale(0.05, 0.0, 0.0), createMarkerColor(r, g, b, 0.999));

  const double z = object.kinematics.initial_pose_with_covariance.pose.position.z;
  const double height = object.shape.dimensions.z;
  const auto polygon = autoware::universe_utils::toPolygon2d(
    object.kinematics.initial_pose_with_covariance.pose, object.shape);
  for (const auto & p : polygon.outer()) {
    marker.points.push_back(createPoint(p.x(), p.y(), z - height / 2));
    marker.points.push_back(createPoint(p.x(), p.y(), z + height / 2));
  }
  marker.id = id;
  msg.markers.push_back(marker);

  return msg;
}

}  // namespace marker_utils
