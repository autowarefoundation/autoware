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

#include "debug_marker.hpp"

#include <autoware/motion_utils/marker/marker_helper.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware/universe_utils/ros/marker_helper.hpp>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <limits>
#include <memory>
#include <vector>

using autoware::motion_utils::createDeletedSlowDownVirtualWallMarker;
using autoware::motion_utils::createDeletedStopVirtualWallMarker;
using autoware::motion_utils::createSlowDownVirtualWallMarker;
using autoware::motion_utils::createStopVirtualWallMarker;
using autoware::universe_utils::appendMarkerArray;
using autoware::universe_utils::calcOffsetPose;
using autoware::universe_utils::createDefaultMarker;
using autoware::universe_utils::createMarkerColor;
using autoware::universe_utils::createMarkerScale;
using autoware::universe_utils::createPoint;

namespace autoware::motion_planning
{
ObstacleStopPlannerDebugNode::ObstacleStopPlannerDebugNode(
  rclcpp::Node * node, const double base_link2front)
: node_(node), base_link2front_(base_link2front)
{
  virtual_wall_pub_ = node_->create_publisher<MarkerArray>("~/virtual_wall", 1);
  debug_viz_pub_ = node_->create_publisher<MarkerArray>("~/debug/marker", 1);
  stop_reason_pub_ = node_->create_publisher<StopReasonArray>("~/output/stop_reasons", 1);
  velocity_factor_pub_ =
    node_->create_publisher<VelocityFactorArray>("/planning/velocity_factors/obstacle_stop", 1);
  pub_debug_values_ =
    node_->create_publisher<Float32MultiArrayStamped>("~/obstacle_stop/debug_values", 1);
}

bool ObstacleStopPlannerDebugNode::pushPolygon(
  const autoware::universe_utils::Polygon2d & polygon, const double z, const PolygonType & type)
{
  std::vector<Eigen::Vector3d> eigen_polygon;
  for (const auto & point : polygon.outer()) {
    Eigen::Vector3d eigen_point;
    eigen_point << point.x(), point.y(), z;
    eigen_polygon.push_back(eigen_point);
  }
  return pushPolygon(eigen_polygon, type);
}

bool ObstacleStopPlannerDebugNode::pushPolygon(
  const std::vector<Eigen::Vector3d> & polygon, const PolygonType & type)
{
  switch (type) {
    case PolygonType::Vehicle:
      if (!polygon.empty()) {
        vehicle_polygons_.push_back(polygon);
      }
      return true;
    case PolygonType::Collision:
      if (!polygon.empty()) {
        collision_polygons_.push_back(polygon);
      }
      return true;
    case PolygonType::SlowDownRange:
      if (!polygon.empty()) {
        slow_down_range_polygons_.push_back(polygon);
      }
      return true;
    case PolygonType::SlowDown:
      if (!polygon.empty()) {
        slow_down_polygons_.push_back(polygon);
      }
      return true;
    case PolygonType::Obstacle:
      if (!polygon.empty()) {
        obstacle_polygons_.push_back(polygon);
      }
      return true;
    default:
      return false;
  }
}

bool ObstacleStopPlannerDebugNode::pushPolyhedron(
  const autoware::universe_utils::Polygon2d & polyhedron, const double z_min, const double z_max,
  const PolygonType & type)
{
  std::vector<Eigen::Vector3d> eigen_polyhedron;
  for (const auto & point : polyhedron.outer()) {
    eigen_polyhedron.emplace_back(point.x(), point.y(), z_min);
    eigen_polyhedron.emplace_back(point.x(), point.y(), z_max);
  }

  return pushPolyhedron(eigen_polyhedron, type);
}

bool ObstacleStopPlannerDebugNode::pushPolyhedron(
  const std::vector<Eigen::Vector3d> & polyhedron, const PolygonType & type)
{
  switch (type) {
    case PolygonType::Vehicle:
      if (!polyhedron.empty()) {
        vehicle_polyhedrons_.push_back(polyhedron);
      }
      return true;
    case PolygonType::Collision:
      if (!polyhedron.empty()) {
        collision_polyhedrons_.push_back(polyhedron);
      }
      return true;
    default:
      return false;
  }
}

bool ObstacleStopPlannerDebugNode::pushPose(const Pose & pose, const PoseType & type)
{
  switch (type) {
    case PoseType::Stop:
      stop_pose_ptr_ = std::make_shared<Pose>(pose);
      return true;
    case PoseType::TargetStop:
      target_stop_pose_ptr_ = std::make_shared<Pose>(pose);
      return true;
    case PoseType::SlowDownStart:
      slow_down_start_pose_ptr_ = std::make_shared<Pose>(pose);
      return true;
    case PoseType::SlowDownEnd:
      slow_down_end_pose_ptr_ = std::make_shared<Pose>(pose);
      return true;
    default:
      return false;
  }
}

bool ObstacleStopPlannerDebugNode::pushObstaclePoint(
  const Point & obstacle_point, const PointType & type)
{
  switch (type) {
    case PointType::Stop:
      stop_obstacle_point_ptr_ = std::make_shared<Point>(obstacle_point);
      return true;
    case PointType::SlowDown:
      slow_down_obstacle_point_ptr_ = std::make_shared<Point>(obstacle_point);
      return true;
    default:
      return false;
  }
}

bool ObstacleStopPlannerDebugNode::pushObstaclePoint(
  const pcl::PointXYZ & obstacle_point, const PointType & type)
{
  Point ros_point;
  ros_point.x = obstacle_point.x;
  ros_point.y = obstacle_point.y;
  ros_point.z = obstacle_point.z;
  return pushObstaclePoint(ros_point, type);
}

void ObstacleStopPlannerDebugNode::publish()
{
  /* publish debug marker for rviz */
  const auto virtual_wall_msg = makeVirtualWallMarker();
  virtual_wall_pub_->publish(virtual_wall_msg);

  /* publish debug marker for rviz */
  const auto visualization_msg = makeVisualizationMarker();
  debug_viz_pub_->publish(visualization_msg);

  /* publish stop reason for autoware api */
  const auto stop_reason_msg = makeStopReasonArray();
  stop_reason_pub_->publish(stop_reason_msg);
  const auto velocity_factor_msg = makeVelocityFactorArray();
  velocity_factor_pub_->publish(velocity_factor_msg);

  // publish debug values
  Float32MultiArrayStamped debug_msg{};
  debug_msg.stamp = node_->now();
  for (const auto & v : debug_values_.getValues()) {
    debug_msg.data.push_back(v);
  }
  pub_debug_values_->publish(debug_msg);

  /* reset variables */
  vehicle_polygons_.clear();
  collision_polygons_.clear();
  slow_down_range_polygons_.clear();
  slow_down_polygons_.clear();
  obstacle_polygons_.clear();
  vehicle_polyhedrons_.clear();
  collision_polyhedrons_.clear();
  stop_pose_ptr_ = nullptr;
  target_stop_pose_ptr_ = nullptr;
  slow_down_start_pose_ptr_ = nullptr;
  slow_down_end_pose_ptr_ = nullptr;
  stop_obstacle_point_ptr_ = nullptr;
  slow_down_obstacle_point_ptr_ = nullptr;
}

MarkerArray ObstacleStopPlannerDebugNode::makeVirtualWallMarker()
{
  MarkerArray msg;
  rclcpp::Time current_time = node_->now();

  if (stop_pose_ptr_ != nullptr) {
    const auto p = calcOffsetPose(*stop_pose_ptr_, base_link2front_, 0.0, 0.0);
    const auto markers = createStopVirtualWallMarker(p, "obstacle on the path", current_time, 0);
    appendMarkerArray(markers, &msg);
  } else {
    const auto markers = createDeletedStopVirtualWallMarker(current_time, 0);
    appendMarkerArray(markers, &msg);
  }

  if (slow_down_start_pose_ptr_ != nullptr && stop_pose_ptr_ == nullptr) {
    const auto p = calcOffsetPose(*slow_down_start_pose_ptr_, base_link2front_, 0.0, 0.0);

    {
      const auto markers =
        createSlowDownVirtualWallMarker(p, "obstacle beside the path", current_time, 0);
      appendMarkerArray(markers, &msg);
    }

    {
      auto markers = createSlowDownVirtualWallMarker(p, "slow down\nstart", current_time, 1);
      markers.markers.front().ns = "slow_down_start_virtual_wall";
      markers.markers.back().ns = "slow_down_start_factor_text";
      appendMarkerArray(markers, &msg);
    }
  } else {
    const auto markers = createDeletedSlowDownVirtualWallMarker(current_time, 0);
    appendMarkerArray(markers, &msg);
  }

  if (slow_down_end_pose_ptr_ != nullptr && stop_pose_ptr_ == nullptr) {
    const auto p = calcOffsetPose(*slow_down_end_pose_ptr_, base_link2front_, 0.0, 0.0);
    auto markers = createSlowDownVirtualWallMarker(p, "slow down\nend", current_time, 2);
    markers.markers.front().ns = "slow_down_end_virtual_wall";
    markers.markers.back().ns = "slow_down_end_factor_text";
    appendMarkerArray(markers, &msg);
  }

  return msg;
}

MarkerArray ObstacleStopPlannerDebugNode::makeVisualizationMarker()
{
  MarkerArray msg;
  rclcpp::Time current_time = node_->now();

  // cube
  if (!vehicle_polyhedrons_.empty()) {
    auto marker = createDefaultMarker(
      "map", current_time, "detection_cubes", 0, Marker::LINE_LIST,
      createMarkerScale(0.01, 0.0, 0.0), createMarkerColor(0.0, 1.0, 0.0, 0.999));

    for (size_t i = 0; i < vehicle_polyhedrons_.size(); ++i) {
      for (size_t j = 0; j < vehicle_polyhedrons_.at(i).size(); ++j) {
        const auto & p = vehicle_polyhedrons_.at(i).at(j);
        marker.points.push_back(createPoint(p.x(), p.y(), p.z()));
      }
    }

    for (size_t i = 0; i < vehicle_polyhedrons_.size(); ++i) {
      for (size_t j = 0; j + 2 < vehicle_polyhedrons_.at(i).size(); ++j) {
        const auto & p = vehicle_polyhedrons_.at(i).at(j);
        marker.points.push_back(createPoint(p.x(), p.y(), p.z()));
        const auto & p1 = vehicle_polyhedrons_.at(i).at(j + 2);
        marker.points.push_back(createPoint(p1.x(), p1.y(), p1.z()));
      }
      const auto & p = vehicle_polyhedrons_.at(i).at(1);
      marker.points.push_back(createPoint(p.x(), p.y(), p.z()));
      const auto & p1 = vehicle_polyhedrons_.at(i).at(vehicle_polyhedrons_.at(i).size() - 1);
      marker.points.push_back(createPoint(p1.x(), p1.y(), p1.z()));
      const auto & p2 = vehicle_polyhedrons_.at(i).at(0);
      marker.points.push_back(createPoint(p2.x(), p2.y(), p2.z()));
      const auto & p3 = vehicle_polyhedrons_.at(i).at(vehicle_polyhedrons_.at(i).size() - 2);
      marker.points.push_back(createPoint(p3.x(), p3.y(), p3.z()));
    }

    msg.markers.push_back(marker);
  }

  if (!collision_polyhedrons_.empty()) {
    auto marker = createDefaultMarker(
      "map", current_time, "collision_cubes", 0, Marker::LINE_LIST,
      createMarkerScale(0.05, 0.0, 0.0), createMarkerColor(1.0, 0.0, 0.0, 0.999));

    for (size_t i = 0; i < collision_polyhedrons_.size(); ++i) {
      for (size_t j = 0; j < collision_polyhedrons_.at(i).size(); ++j) {
        const auto & p = collision_polyhedrons_.at(i).at(j);
        marker.points.push_back(createPoint(p.x(), p.y(), p.z()));
      }
    }

    for (size_t i = 0; i < collision_polyhedrons_.size(); ++i) {
      for (size_t j = 0; j + 2 < collision_polyhedrons_.at(i).size(); ++j) {
        const auto & p = collision_polyhedrons_.at(i).at(j);
        marker.points.push_back(createPoint(p.x(), p.y(), p.z()));
        const auto & p1 = collision_polyhedrons_.at(i).at(j + 2);
        marker.points.push_back(createPoint(p1.x(), p1.y(), p1.z()));
      }
      const auto & p = collision_polyhedrons_.at(i).at(1);
      marker.points.push_back(createPoint(p.x(), p.y(), p.z()));
      const auto & p1 = collision_polyhedrons_.at(i).at(collision_polyhedrons_.at(i).size() - 1);
      marker.points.push_back(createPoint(p1.x(), p1.y(), p1.z()));
      const auto & p2 = collision_polyhedrons_.at(i).at(0);
      marker.points.push_back(createPoint(p2.x(), p2.y(), p2.z()));
      const auto & p3 = collision_polyhedrons_.at(i).at(collision_polyhedrons_.at(i).size() - 2);
      marker.points.push_back(createPoint(p3.x(), p3.y(), p3.z()));
    }

    msg.markers.push_back(marker);
  }

  // polygon
  if (!vehicle_polygons_.empty()) {
    auto marker = createDefaultMarker(
      "map", current_time, "detection_polygons", 0, Marker::LINE_LIST,
      createMarkerScale(0.01, 0.0, 0.0), createMarkerColor(0.0, 1.0, 0.0, 0.999));

    for (size_t i = 0; i < vehicle_polygons_.size(); ++i) {
      for (size_t j = 0; j < vehicle_polygons_.at(i).size(); ++j) {
        {
          const auto & p = vehicle_polygons_.at(i).at(j);
          marker.points.push_back(createPoint(p.x(), p.y(), p.z()));
        }
        if (j + 1 == vehicle_polygons_.at(i).size()) {
          const auto & p = vehicle_polygons_.at(i).at(0);
          marker.points.push_back(createPoint(p.x(), p.y(), p.z()));
        } else {
          const auto & p = vehicle_polygons_.at(i).at(j + 1);
          marker.points.push_back(createPoint(p.x(), p.y(), p.z()));
        }
      }
    }
    msg.markers.push_back(marker);
  }

  if (!collision_polygons_.empty()) {
    auto marker = createDefaultMarker(
      "map", current_time, "collision_polygons", 0, Marker::LINE_LIST,
      createMarkerScale(0.05, 0.0, 0.0), createMarkerColor(1.0, 0.0, 0.0, 0.999));

    for (size_t i = 0; i < collision_polygons_.size(); ++i) {
      for (size_t j = 0; j < collision_polygons_.at(i).size(); ++j) {
        {
          const auto & p = collision_polygons_.at(i).at(j);
          marker.points.push_back(createPoint(p.x(), p.y(), p.z()));
        }
        if (j + 1 == collision_polygons_.at(i).size()) {
          const auto & p = collision_polygons_.at(i).at(0);
          marker.points.push_back(createPoint(p.x(), p.y(), p.z()));
        } else {
          const auto & p = collision_polygons_.at(i).at(j + 1);
          marker.points.push_back(createPoint(p.x(), p.y(), p.z()));
        }
      }
    }
    msg.markers.push_back(marker);
  }

  if (!slow_down_range_polygons_.empty()) {
    auto marker = createDefaultMarker(
      "map", current_time, "slow_down_detection_polygons", 0, Marker::LINE_LIST,
      createMarkerScale(0.01, 0.0, 0.0), createMarkerColor(0.0, 1.0, 0.0, 0.999));

    for (size_t i = 0; i < slow_down_range_polygons_.size(); ++i) {
      for (size_t j = 0; j < slow_down_range_polygons_.at(i).size(); ++j) {
        {
          const auto & p = slow_down_range_polygons_.at(i).at(j);
          marker.points.push_back(createPoint(p.x(), p.y(), p.z()));
        }
        if (j + 1 == slow_down_range_polygons_.at(i).size()) {
          const auto & p = slow_down_range_polygons_.at(i).at(0);
          marker.points.push_back(createPoint(p.x(), p.y(), p.z()));
        } else {
          const auto & p = slow_down_range_polygons_.at(i).at(j + 1);
          marker.points.push_back(createPoint(p.x(), p.y(), p.z()));
        }
      }
    }
    msg.markers.push_back(marker);
  }

  if (!slow_down_polygons_.empty()) {
    auto marker = createDefaultMarker(
      "map", current_time, "slow_down_polygons", 0, Marker::LINE_LIST,
      createMarkerScale(0.05, 0.0, 0.0), createMarkerColor(1.0, 1.0, 0.0, 0.999));

    for (size_t i = 0; i < slow_down_polygons_.size(); ++i) {
      for (size_t j = 0; j < slow_down_polygons_.at(i).size(); ++j) {
        {
          const auto & p = slow_down_polygons_.at(i).at(j);
          marker.points.push_back(createPoint(p.x(), p.y(), p.z()));
        }
        if (j + 1 == slow_down_polygons_.at(i).size()) {
          const auto & p = slow_down_polygons_.at(i).at(0);
          marker.points.push_back(createPoint(p.x(), p.y(), p.z()));
        } else {
          const auto & p = slow_down_polygons_.at(i).at(j + 1);
          marker.points.push_back(createPoint(p.x(), p.y(), p.z()));
        }
      }
    }
    msg.markers.push_back(marker);
  }

  if (!obstacle_polygons_.empty()) {
    auto marker = createDefaultMarker(
      "map", current_time, "obstacle_polygons", 0, Marker::LINE_LIST,
      createMarkerScale(0.05, 0.0, 0.0), createMarkerColor(1.0, 1.0, 0.0, 0.999));

    for (size_t i = 0; i < obstacle_polygons_.size(); ++i) {
      for (size_t j = 0; j < obstacle_polygons_.at(i).size(); ++j) {
        {
          const auto & p = obstacle_polygons_.at(i).at(j);
          marker.points.push_back(createPoint(p.x(), p.y(), p.z()));
        }
        if (j + 1 == obstacle_polygons_.at(i).size()) {
          const auto & p = obstacle_polygons_.at(i).at(0);
          marker.points.push_back(createPoint(p.x(), p.y(), p.z()));
        } else {
          const auto & p = obstacle_polygons_.at(i).at(j + 1);
          marker.points.push_back(createPoint(p.x(), p.y(), p.z()));
        }
      }
    }
    msg.markers.push_back(marker);
  }

  if (target_stop_pose_ptr_ != nullptr) {
    const auto p = calcOffsetPose(*target_stop_pose_ptr_, base_link2front_, 0.0, 0.0);
    const auto markers =
      createStopVirtualWallMarker(p, "obstacle_stop_target_stop_line", current_time, 0);
    appendMarkerArray(markers, &msg);
  } else {
    const auto markers = createDeletedStopVirtualWallMarker(current_time, 0);
    appendMarkerArray(markers, &msg);
  }

  if (stop_obstacle_point_ptr_ != nullptr) {
    auto marker1 = createDefaultMarker(
      "map", current_time, "stop_obstacle_point", 0, Marker::SPHERE,
      createMarkerScale(0.25, 0.25, 0.25), createMarkerColor(1.0, 0.0, 0.0, 0.999));
    marker1.pose.position = *stop_obstacle_point_ptr_;
    msg.markers.push_back(marker1);

    auto marker2 = createDefaultMarker(
      "map", current_time, "stop_obstacle_text", 0, Marker::TEXT_VIEW_FACING,
      createMarkerScale(0.0, 0.0, 1.0), createMarkerColor(1.0, 1.0, 1.0, 0.999));
    marker2.pose.position = *stop_obstacle_point_ptr_;
    marker2.pose.position.z += 2.0;
    marker2.text = "!";
    msg.markers.push_back(marker2);
  }

  if (slow_down_obstacle_point_ptr_ != nullptr) {
    auto marker1 = createDefaultMarker(
      "map", current_time, "slow_down_obstacle_point", 0, Marker::SPHERE,
      createMarkerScale(0.25, 0.25, 0.25), createMarkerColor(1.0, 0.0, 0.0, 0.999));
    marker1.pose.position = *slow_down_obstacle_point_ptr_;
    msg.markers.push_back(marker1);

    auto marker2 = createDefaultMarker(
      "map", current_time, "slow_down_obstacle_text", 0, Marker::TEXT_VIEW_FACING,
      createMarkerScale(0.0, 0.0, 1.0), createMarkerColor(1.0, 1.0, 1.0, 0.999));
    marker2.pose.position = *slow_down_obstacle_point_ptr_;
    marker2.pose.position.z += 2.0;
    marker2.text = "!";
    msg.markers.push_back(marker2);
  }

  return msg;
}

StopReasonArray ObstacleStopPlannerDebugNode::makeStopReasonArray()
{
  // create header
  Header header;
  header.frame_id = "map";
  header.stamp = node_->now();

  // create stop reason stamped
  StopReason stop_reason_msg;
  stop_reason_msg.reason = StopReason::OBSTACLE_STOP;
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

VelocityFactorArray ObstacleStopPlannerDebugNode::makeVelocityFactorArray()
{
  VelocityFactorArray velocity_factor_array;
  velocity_factor_array.header.frame_id = "map";
  velocity_factor_array.header.stamp = node_->now();

  if (stop_pose_ptr_) {
    using distance_type = VelocityFactor::_distance_type;
    VelocityFactor velocity_factor;
    velocity_factor.behavior = PlanningBehavior::ROUTE_OBSTACLE;
    velocity_factor.pose = *stop_pose_ptr_;
    velocity_factor.distance = std::numeric_limits<distance_type>::quiet_NaN();
    velocity_factor.status = VelocityFactor::UNKNOWN;
    velocity_factor.detail = std::string();
    velocity_factor_array.factors.push_back(velocity_factor);
  }
  return velocity_factor_array;
}

}  // namespace autoware::motion_planning
