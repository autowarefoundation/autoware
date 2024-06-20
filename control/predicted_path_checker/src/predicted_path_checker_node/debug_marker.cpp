// Copyright 2023 LeoDrive A.Ş. All rights reserved.
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

#include "predicted_path_checker/debug_marker.hpp"

#include <autoware/motion_utils/marker/marker_helper.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware/universe_utils/ros/marker_helper.hpp>
#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <autoware/universe_utils/geometry/boost_geometry.hpp>

#include <memory>
#include <vector>

using autoware::motion_utils::createDeletedStopVirtualWallMarker;
using autoware::motion_utils::createStopVirtualWallMarker;
using autoware::universe_utils::appendMarkerArray;
using autoware::universe_utils::calcOffsetPose;
using autoware::universe_utils::createDefaultMarker;
using autoware::universe_utils::createMarkerColor;
using autoware::universe_utils::createMarkerOrientation;
using autoware::universe_utils::createMarkerScale;
using autoware::universe_utils::createPoint;

namespace autoware::motion::control::predicted_path_checker
{
PredictedPathCheckerDebugNode::PredictedPathCheckerDebugNode(
  rclcpp::Node * node, const double base_link2front)
: node_(node), base_link2front_(base_link2front)
{
  virtual_wall_pub_ =
    node->create_publisher<visualization_msgs::msg::MarkerArray>("~/debug/virtual_wall", 1);
  debug_viz_pub_ =
    node_->create_publisher<visualization_msgs::msg::MarkerArray>("~/debug/marker", 1);
}

bool PredictedPathCheckerDebugNode::pushPolygon(
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

bool PredictedPathCheckerDebugNode::pushPolygon(
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
    default:
      return false;
  }
}

bool PredictedPathCheckerDebugNode::pushPolyhedron(
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

bool PredictedPathCheckerDebugNode::pushPolyhedron(
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

bool PredictedPathCheckerDebugNode::pushPose(
  const geometry_msgs::msg::Pose & pose, const PoseType & type)
{
  switch (type) {
    case PoseType::Stop:
      stop_pose_ptr_ = std::make_shared<geometry_msgs::msg::Pose>(pose);
      return true;
    case PoseType::Collision:
      collision_pose_ptr_ = std::make_shared<geometry_msgs::msg::Pose>(pose);
      return true;
    default:
      return false;
  }
}

bool PredictedPathCheckerDebugNode::pushObstaclePoint(
  const geometry_msgs::msg::Point & obstacle_point, const PointType & type)
{
  switch (type) {
    case PointType::Stop:
      stop_obstacle_point_ptr_ = std::make_shared<geometry_msgs::msg::Point>(obstacle_point);
      return true;
    default:
      return false;
  }
}

void PredictedPathCheckerDebugNode::publish()
{
  /* publish debug marker for rviz */
  const auto virtual_wall_msg = makeVirtualWallMarker();
  virtual_wall_pub_->publish(virtual_wall_msg);

  /* publish debug marker for rviz */
  const auto visualization_msg = makeVisualizationMarker();
  debug_viz_pub_->publish(visualization_msg);

  /* reset variables */
  vehicle_polygons_.clear();
  collision_polygons_.clear();
  vehicle_polyhedrons_.clear();
  collision_polyhedrons_.clear();
  collision_pose_ptr_ = nullptr;
  stop_pose_ptr_ = nullptr;
  stop_obstacle_point_ptr_ = nullptr;
}

visualization_msgs::msg::MarkerArray PredictedPathCheckerDebugNode::makeVirtualWallMarker()
{
  visualization_msgs::msg::MarkerArray msg;
  rclcpp::Time current_time = node_->now();

  if (stop_pose_ptr_ != nullptr) {
    const auto p = calcOffsetPose(*stop_pose_ptr_, base_link2front_, 0.0, 0.0);
    const auto markers =
      createStopVirtualWallMarker(p, "obstacle on predicted path", current_time, 0);
    appendMarkerArray(markers, &msg);
  } else {
    const auto markers = createDeletedStopVirtualWallMarker(current_time, 0);
    appendMarkerArray(markers, &msg);
  }

  if (collision_pose_ptr_ != nullptr) {
    const auto markers =
      createStopVirtualWallMarker(*collision_pose_ptr_, "collision_point", current_time, 1);
    appendMarkerArray(markers, &msg);
  } else {
    const auto markers = createDeletedStopVirtualWallMarker(current_time, 1);
    appendMarkerArray(markers, &msg);
  }

  return msg;
}

visualization_msgs::msg::MarkerArray PredictedPathCheckerDebugNode::makeVisualizationMarker()
{
  visualization_msgs::msg::MarkerArray msg;
  rclcpp::Time current_time = node_->now();

  // cube
  if (!vehicle_polyhedrons_.empty()) {
    auto marker = createDefaultMarker(
      "map", current_time, "detection_cubes", 0, visualization_msgs::msg::Marker::LINE_LIST,
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
      "map", current_time, "collision_cubes", 0, visualization_msgs::msg::Marker::LINE_LIST,
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
      "map", current_time, "detection_polygons", 0, visualization_msgs::msg::Marker::LINE_LIST,
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
      "map", current_time, "collision_polygons", 0, visualization_msgs::msg::Marker::LINE_LIST,
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

  if (stop_obstacle_point_ptr_ != nullptr) {
    auto marker = createDefaultMarker(
      "map", current_time, "stop_obstacle_point", 0, visualization_msgs::msg::Marker::SPHERE,
      createMarkerScale(0.25, 0.25, 0.25), createMarkerColor(1.0, 0.0, 0.0, 0.999));
    marker.pose.position = *stop_obstacle_point_ptr_;
    msg.markers.push_back(marker);
  }

  if (stop_obstacle_point_ptr_ != nullptr) {
    auto marker = createDefaultMarker(
      "map", current_time, "stop_obstacle_text", 0,
      visualization_msgs::msg::Marker::TEXT_VIEW_FACING, createMarkerScale(0.0, 0.0, 1.0),
      createMarkerColor(1.0, 1.0, 1.0, 0.999));
    marker.pose.position = *stop_obstacle_point_ptr_;
    marker.pose.position.z += 2.0;
    marker.text = "!";
    msg.markers.push_back(marker);
  }

  return msg;
}

}  // namespace autoware::motion::control::predicted_path_checker
