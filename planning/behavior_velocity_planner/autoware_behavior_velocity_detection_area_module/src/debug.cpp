// Copyright 2020 Tier IV, Inc.
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

#include "scene.hpp"

#include <autoware/behavior_velocity_planner_common/utilization/debug.hpp>
#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>
#include <autoware/motion_utils/marker/virtual_wall_marker_creator.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware/universe_utils/ros/marker_helper.hpp>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif
#include <std_msgs/msg/color_rgba.hpp>

#include <vector>

namespace autoware::behavior_velocity_planner
{
using autoware::universe_utils::appendMarkerArray;
using autoware::universe_utils::createDefaultMarker;
using autoware::universe_utils::createMarkerColor;
using autoware::universe_utils::createMarkerScale;
using std_msgs::msg::ColorRGBA;

namespace
{
using DebugData = DetectionAreaModule::DebugData;

lanelet::BasicPoint3d getCentroidPoint(const lanelet::BasicPolygon3d & poly)
{
  lanelet::BasicPoint3d p_sum{0.0, 0.0, 0.0};
  for (const auto & p : poly) {
    p_sum += p;
  }
  return p_sum / poly.size();
}

geometry_msgs::msg::Point toMsg(const lanelet::BasicPoint3d & point)
{
  geometry_msgs::msg::Point msg;
  msg.x = point.x();
  msg.y = point.y();
  msg.z = point.z();
  return msg;
}

visualization_msgs::msg::MarkerArray createCorrespondenceMarkerArray(
  const lanelet::autoware::DetectionArea & detection_area_reg_elem, const rclcpp::Time & now,
  ColorRGBA & marker_color)
{
  visualization_msgs::msg::MarkerArray msg;

  const lanelet::ConstLineString3d stop_line = detection_area_reg_elem.stopLine();
  const auto stop_line_center_point =
    (stop_line.front().basicPoint() + stop_line.back().basicPoint()) / 2;

  // ID
  {
    auto marker = createDefaultMarker(
      "map", now, "detection_area_id", detection_area_reg_elem.id(),
      visualization_msgs::msg::Marker::TEXT_VIEW_FACING, createMarkerScale(0.0, 0.0, 1.0),
      createMarkerColor(1.0, 1.0, 1.0, 0.999));
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);

    for (const auto & detection_area : detection_area_reg_elem.detectionAreas()) {
      const auto poly = detection_area.basicPolygon();

      marker.pose.position = toMsg(poly.front());
      marker.pose.position.z += 2.0;
      marker.text = std::to_string(detection_area_reg_elem.id());

      msg.markers.push_back(marker);
    }
  }

  // Polygon
  {
    auto marker = createDefaultMarker(
      "map", now, "detection_area_polygon", detection_area_reg_elem.id(),
      visualization_msgs::msg::Marker::LINE_LIST, createMarkerScale(0.1, 0.0, 0.0),
      createMarkerColor(marker_color.r, marker_color.g, marker_color.b, marker_color.a));
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);

    for (const auto & detection_area : detection_area_reg_elem.detectionAreas()) {
      const auto poly = detection_area.basicPolygon();

      for (size_t i = 0; i < poly.size(); ++i) {
        const auto idx_front = i;
        const auto idx_back = (i == poly.size() - 1) ? 0 : i + 1;

        const auto & p_front = poly.at(idx_front);
        const auto & p_back = poly.at(idx_back);

        marker.points.push_back(toMsg(p_front));
        marker.points.push_back(toMsg(p_back));
      }
    }

    msg.markers.push_back(marker);
  }

  // Polygon to StopLine
  {
    auto marker = createDefaultMarker(
      "map", now, "detection_area_correspondence", detection_area_reg_elem.id(),
      visualization_msgs::msg::Marker::LINE_LIST, createMarkerScale(0.1, 0.0, 0.0),
      createMarkerColor(marker_color.r, marker_color.g, marker_color.b, marker_color.a));
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);

    for (const auto & detection_area : detection_area_reg_elem.detectionAreas()) {
      const auto poly = detection_area.basicPolygon();
      const auto centroid_point = getCentroidPoint(poly);
      for (size_t i = 0; i < poly.size(); ++i) {
        marker.points.push_back(toMsg(centroid_point));
        marker.points.push_back(toMsg(stop_line_center_point));
      }
    }

    msg.markers.push_back(marker);
  }

  return msg;
}
}  // namespace

visualization_msgs::msg::MarkerArray DetectionAreaModule::createDebugMarkerArray()
{
  visualization_msgs::msg::MarkerArray wall_marker;
  const rclcpp::Time now = clock_->now();

  if (!debug_data_.stop_poses.empty()) {
    ColorRGBA marker_color;
    marker_color.r = 1.0;
    marker_color.g = 0.0;
    marker_color.b = 0.0;
    marker_color.a = 1.0;

    appendMarkerArray(
      createCorrespondenceMarkerArray(detection_area_reg_elem_, now, marker_color), &wall_marker);

    appendMarkerArray(
      debug::createPointsMarkerArray(
        debug_data_.obstacle_points, "obstacles", module_id_, now, 0.6, 0.6, 0.6, 1.0, 0.0, 0.0),
      &wall_marker, now);
  } else {
    ColorRGBA marker_color;
    marker_color.r = 0.0;
    marker_color.g = 1.0;
    marker_color.b = 0.0;
    marker_color.a = 1.0;

    appendMarkerArray(
      createCorrespondenceMarkerArray(detection_area_reg_elem_, now, marker_color), &wall_marker);
  }

  return wall_marker;
}

autoware::motion_utils::VirtualWalls DetectionAreaModule::createVirtualWalls()
{
  autoware::motion_utils::VirtualWalls virtual_walls;
  autoware::motion_utils::VirtualWall wall;
  wall.text = "detection_area";

  wall.style = autoware::motion_utils::VirtualWallType::stop;
  for (const auto & p : debug_data_.stop_poses) {
    wall.pose = autoware::universe_utils::calcOffsetPose(p, debug_data_.base_link2front, 0.0, 0.0);
    virtual_walls.push_back(wall);
  }

  wall.style = autoware::motion_utils::VirtualWallType::deadline;
  for (const auto & p : debug_data_.dead_line_poses) {
    wall.pose = autoware::universe_utils::calcOffsetPose(p, debug_data_.base_link2front, 0.0, 0.0);
    virtual_walls.push_back(wall);
  }
  return virtual_walls;
}

}  // namespace autoware::behavior_velocity_planner
