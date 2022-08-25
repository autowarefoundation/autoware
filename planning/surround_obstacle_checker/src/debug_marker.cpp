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

#include "surround_obstacle_checker/debug_marker.hpp"

#include <motion_utils/motion_utils.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>
#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <memory>

namespace surround_obstacle_checker
{

using motion_utils::createStopVirtualWallMarker;
using tier4_autoware_utils::appendMarkerArray;
using tier4_autoware_utils::calcOffsetPose;
using tier4_autoware_utils::createDefaultMarker;
using tier4_autoware_utils::createMarkerColor;
using tier4_autoware_utils::createMarkerScale;
using tier4_autoware_utils::createPoint;

SurroundObstacleCheckerDebugNode::SurroundObstacleCheckerDebugNode(
  const Polygon2d & ego_polygon, const double base_link2front,
  const double & surround_check_distance, const double & surround_check_recover_distance,
  const geometry_msgs::msg::Pose & self_pose, const rclcpp::Clock::SharedPtr clock,
  rclcpp::Node & node)
: ego_polygon_(ego_polygon),
  base_link2front_(base_link2front),
  surround_check_distance_(surround_check_distance),
  surround_check_recover_distance_(surround_check_recover_distance),
  self_pose_(self_pose),
  clock_(clock)
{
  debug_virtual_wall_pub_ =
    node.create_publisher<visualization_msgs::msg::MarkerArray>("~/virtual_wall", 1);
  debug_viz_pub_ = node.create_publisher<visualization_msgs::msg::MarkerArray>("~/debug/marker", 1);
  stop_reason_pub_ = node.create_publisher<StopReasonArray>("~/output/stop_reasons", 1);
  vehicle_footprint_pub_ = node.create_publisher<PolygonStamped>("~/debug/footprint", 1);
  vehicle_footprint_offset_pub_ =
    node.create_publisher<PolygonStamped>("~/debug/footprint_offset", 1);
  vehicle_footprint_recover_offset_pub_ =
    node.create_publisher<PolygonStamped>("~/debug/footprint_recover_offset", 1);
}

bool SurroundObstacleCheckerDebugNode::pushPose(
  const geometry_msgs::msg::Pose & pose, const PoseType & type)
{
  switch (type) {
    case PoseType::NoStart:
      stop_pose_ptr_ = std::make_shared<geometry_msgs::msg::Pose>(pose);
      return true;
    default:
      return false;
  }
}

bool SurroundObstacleCheckerDebugNode::pushObstaclePoint(
  const geometry_msgs::msg::Point & obstacle_point, const PointType & type)
{
  switch (type) {
    case PointType::NoStart:
      stop_obstacle_point_ptr_ = std::make_shared<geometry_msgs::msg::Point>(obstacle_point);
      return true;
    default:
      return false;
  }
}

void SurroundObstacleCheckerDebugNode::publishFootprints()
{
  /* publish vehicle footprint polygon */
  const auto footprint = boostPolygonToPolygonStamped(ego_polygon_, self_pose_.position.z);
  vehicle_footprint_pub_->publish(footprint);

  /* publish vehicle footprint polygon with offset */
  const auto polygon_with_offset =
    createSelfPolygonWithOffset(ego_polygon_, surround_check_distance_);
  const auto footprint_with_offset =
    boostPolygonToPolygonStamped(polygon_with_offset, self_pose_.position.z);
  vehicle_footprint_offset_pub_->publish(footprint_with_offset);

  /* publish vehicle footprint polygon with recover offset */
  const auto polygon_with_recover_offset =
    createSelfPolygonWithOffset(ego_polygon_, surround_check_recover_distance_);
  const auto footprint_with_recover_offset =
    boostPolygonToPolygonStamped(polygon_with_recover_offset, self_pose_.position.z);
  vehicle_footprint_recover_offset_pub_->publish(footprint_with_recover_offset);
}

void SurroundObstacleCheckerDebugNode::publish()
{
  /* publish virtual_wall marker for rviz */
  const auto virtual_wall_msg = makeVirtualWallMarker();
  debug_virtual_wall_pub_->publish(virtual_wall_msg);

  /* publish debug marker for rviz */
  const auto visualization_msg = makeVisualizationMarker();
  debug_viz_pub_->publish(visualization_msg);

  /* publish stop reason for autoware api */
  const auto stop_reason_msg = makeStopReasonArray();
  stop_reason_pub_->publish(stop_reason_msg);

  /* reset variables */
  stop_pose_ptr_ = nullptr;
  stop_obstacle_point_ptr_ = nullptr;
}

MarkerArray SurroundObstacleCheckerDebugNode::makeVirtualWallMarker()
{
  MarkerArray msg;
  rclcpp::Time current_time = this->clock_->now();

  // visualize stop line
  if (stop_pose_ptr_ != nullptr) {
    const auto p = calcOffsetPose(*stop_pose_ptr_, base_link2front_, 0.0, 0.0);
    const auto markers = createStopVirtualWallMarker(p, "surround obstacle", current_time, 0);
    appendMarkerArray(markers, &msg);
  }

  return msg;
}

MarkerArray SurroundObstacleCheckerDebugNode::makeVisualizationMarker()
{
  MarkerArray msg;
  rclcpp::Time current_time = this->clock_->now();

  // visualize surround object
  if (stop_obstacle_point_ptr_ != nullptr) {
    auto marker = createDefaultMarker(
      "map", current_time, "no_start_obstacle_text", 0, Marker::TEXT_VIEW_FACING,
      createMarkerScale(0.0, 0.0, 1.0), createMarkerColor(1.0, 1.0, 1.0, 0.999));
    marker.pose.position = *stop_obstacle_point_ptr_;
    marker.pose.position.z += 2.0;  // add half of the heights of obj roughly
    marker.text = "!";
    msg.markers.push_back(marker);
  }

  return msg;
}

StopReasonArray SurroundObstacleCheckerDebugNode::makeStopReasonArray()
{
  // create header
  std_msgs::msg::Header header;
  header.frame_id = "map";
  header.stamp = this->clock_->now();

  // create stop reason stamped
  StopReason stop_reason_msg;
  stop_reason_msg.reason = StopReason::SURROUND_OBSTACLE_CHECK;
  StopFactor stop_factor;

  if (stop_pose_ptr_ != nullptr) {
    stop_factor.stop_pose = *stop_pose_ptr_;
    if (stop_obstacle_point_ptr_ != nullptr) {
      stop_factor.stop_factor_points.emplace_back(*stop_obstacle_point_ptr_);
    }
    stop_reason_msg.stop_factors.emplace_back(stop_factor);
  }

  // create stop reason array
  StopReasonArray stop_reason_array;
  stop_reason_array.header = header;
  stop_reason_array.stop_reasons.emplace_back(stop_reason_msg);
  return stop_reason_array;
}

Polygon2d SurroundObstacleCheckerDebugNode::createSelfPolygonWithOffset(
  const Polygon2d & base_polygon, const double & offset)
{
  typedef double coordinate_type;
  const double buffer_distance = offset;
  const int points_per_circle = 36;
  boost::geometry::strategy::buffer::distance_symmetric<coordinate_type> distance_strategy(
    buffer_distance);
  boost::geometry::strategy::buffer::join_round join_strategy(points_per_circle);
  boost::geometry::strategy::buffer::end_round end_strategy(points_per_circle);
  boost::geometry::strategy::buffer::point_circle circle_strategy(points_per_circle);
  boost::geometry::strategy::buffer::side_straight side_strategy;
  boost::geometry::model::multi_polygon<Polygon2d> result;
  // Create the buffer of a multi polygon
  boost::geometry::buffer(
    base_polygon, result, distance_strategy, side_strategy, join_strategy, end_strategy,
    circle_strategy);
  return result.front();
}

PolygonStamped SurroundObstacleCheckerDebugNode::boostPolygonToPolygonStamped(
  const Polygon2d & boost_polygon, const double & z)
{
  PolygonStamped polygon_stamped;
  polygon_stamped.header.frame_id = "base_link";
  polygon_stamped.header.stamp = this->clock_->now();

  for (auto const & p : boost_polygon.outer()) {
    geometry_msgs::msg::Point32 gp;
    gp.x = p.x();
    gp.y = p.y();
    gp.z = z;
    polygon_stamped.polygon.points.push_back(gp);
  }

  return polygon_stamped;
}

}  // namespace surround_obstacle_checker
