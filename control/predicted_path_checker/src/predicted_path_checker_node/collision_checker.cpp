// Copyright 2022 LeoDrive A.Ş. All rights reserved.
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

#include "predicted_path_checker/collision_checker.hpp"

#include <autoware/universe_utils/ros/marker_helper.hpp>
#include <rclcpp/logging.hpp>

#include <memory>
#include <utility>
#include <vector>

namespace autoware::motion::control::predicted_path_checker
{
CollisionChecker::CollisionChecker(
  rclcpp::Node * node, std::shared_ptr<PredictedPathCheckerDebugNode> debug_ptr)
: debug_ptr_(std::move(debug_ptr)),
  node_(node),
  vehicle_info_(autoware::vehicle_info_utils::VehicleInfoUtils(*node).getVehicleInfo())
{
}

void CollisionChecker::setParam(const CollisionCheckerParam & param)
{
  param_ = param;
}

boost::optional<std::pair<geometry_msgs::msg::Point, PredictedObject>>
CollisionChecker::checkTrajectoryForCollision(
  TrajectoryPoints & predicted_trajectory_array, PredictedObjects::ConstSharedPtr dynamic_objects)
{
  // It checks the collision, if there is a collision, it updates the predicted_trajectory_array and
  // returns the index of the stop point.
  // If there is no collision, it returns boost::none.
  const auto now = node_->now();

  updatePredictedObjectHistory(now);
  if (dynamic_objects->objects.empty() && predicted_object_history_.empty()) {
    return boost::none;
  }

  for (size_t i = 0; i < predicted_trajectory_array.size() - 1; i++) {
    // create one step circle center for vehicle
    const auto & p_front = predicted_trajectory_array.at(i).pose;
    const auto & p_back = predicted_trajectory_array.at(i + 1).pose;
    const auto z_min = p_front.position.z;
    const auto z_max =
      p_front.position.z + vehicle_info_.max_height_offset_m + param_.z_axis_filtering_buffer;

    // create one-step polygon for vehicle
    Polygon2d one_step_move_vehicle_polygon2d =
      utils::createOneStepPolygon(p_front, p_back, vehicle_info_, param_.width_margin);
    if (param_.enable_z_axis_obstacle_filtering) {
      debug_ptr_->pushPolyhedron(
        one_step_move_vehicle_polygon2d, z_min, z_max, PolygonType::Vehicle);
    } else {
      debug_ptr_->pushPolygon(
        one_step_move_vehicle_polygon2d, p_front.position.z, PolygonType::Vehicle);
    }

    // check obstacle history
    auto found_collision_at_history =
      checkObstacleHistory(p_front, one_step_move_vehicle_polygon2d, z_min, z_max);

    auto found_collision_at_dynamic_objects =
      checkDynamicObjects(p_front, dynamic_objects, one_step_move_vehicle_polygon2d, z_min, z_max);

    if (found_collision_at_dynamic_objects || found_collision_at_history) {
      double distance_to_current = std::numeric_limits<double>::max();
      double distance_to_history = std::numeric_limits<double>::max();
      if (found_collision_at_dynamic_objects) {
        distance_to_current = autoware::universe_utils::calcDistance2d(
          p_front, found_collision_at_dynamic_objects.get().first);
      }
      if (found_collision_at_history) {
        distance_to_history =
          autoware::universe_utils::calcDistance2d(p_front, found_collision_at_history.get().first);
      } else {
        predicted_object_history_.clear();
      }

      if (param_.enable_z_axis_obstacle_filtering) {
        debug_ptr_->pushPolyhedron(
          one_step_move_vehicle_polygon2d, z_min, z_max, PolygonType::Collision);
      } else {
        debug_ptr_->pushPolygon(
          one_step_move_vehicle_polygon2d, p_front.position.z, PolygonType::Collision);
      }

      if (distance_to_current > distance_to_history) {
        debug_ptr_->pushObstaclePoint(found_collision_at_history->first, PointType::Stop);
        return found_collision_at_history;
      }

      predicted_object_history_.emplace_back(
        now, found_collision_at_dynamic_objects.get().first,
        found_collision_at_dynamic_objects.get().second);
      debug_ptr_->pushObstaclePoint(found_collision_at_dynamic_objects->first, PointType::Stop);
      return found_collision_at_dynamic_objects;
    }
  }
  return boost::none;
}

boost::optional<std::pair<geometry_msgs::msg::Point, PredictedObject>>
CollisionChecker::checkObstacleHistory(
  const Pose & base_pose, const Polygon2d & one_step_move_vehicle_polygon2d, const double z_min,
  const double z_max)
{
  if (predicted_object_history_.empty()) {
    return boost::none;
  }

  std::vector<std::pair<geometry_msgs::msg::Point, PredictedObject>> collision_points_in_history;
  for (const auto & obj_history : predicted_object_history_) {
    if (param_.enable_z_axis_obstacle_filtering) {
      if (!utils::intersectsInZAxis(obj_history.object, z_min, z_max)) {
        continue;
      }
    }
    const auto & point = obj_history.point;
    const Point2d point2d(point.x, point.y);
    if (bg::within(point2d, one_step_move_vehicle_polygon2d)) {
      collision_points_in_history.emplace_back(point, obj_history.object);
    }
  }
  if (!collision_points_in_history.empty()) {
    double min_norm = 0.0;
    bool is_init = false;
    std::pair<geometry_msgs::msg::Point, PredictedObject> nearest_collision_object_with_point;
    for (const auto & p : collision_points_in_history) {
      double norm = autoware::universe_utils::calcDistance2d(p.first, base_pose);
      if (norm < min_norm || !is_init) {
        min_norm = norm;
        nearest_collision_object_with_point = p;
        is_init = true;
      }
    }
    return boost::make_optional(nearest_collision_object_with_point);
  }
  return boost::none;
}

boost::optional<std::pair<geometry_msgs::msg::Point, PredictedObject>>
CollisionChecker::checkDynamicObjects(
  const Pose & base_pose, PredictedObjects::ConstSharedPtr dynamic_objects,
  const Polygon2d & one_step_move_vehicle_polygon2d, const double z_min, const double z_max)
{
  if (dynamic_objects->objects.empty()) {
    return boost::none;
  }
  double min_norm_collision_norm = 0.0;
  bool is_init = false;
  size_t nearest_collision_object_index = 0;
  geometry_msgs::msg::Point nearest_collision_point;

  for (size_t i = 0; i < dynamic_objects->objects.size(); ++i) {
    const auto & obj = dynamic_objects->objects.at(i);
    if (param_.enable_z_axis_obstacle_filtering) {
      if (!utils::intersectsInZAxis(obj, z_min, z_max)) {
        continue;
      }
    }
    const auto object_polygon = utils::convertObjToPolygon(obj);
    if (object_polygon.outer().empty()) {
      // unsupported type
      continue;
    }

    const auto found_collision_points =
      bg::intersects(one_step_move_vehicle_polygon2d, object_polygon);

    if (found_collision_points) {
      std::vector<Point2d> collision_points;
      PointArray collision_point_array;
      bg::intersection(one_step_move_vehicle_polygon2d, object_polygon, collision_points);
      for (const auto & point : collision_points) {
        geometry_msgs::msg::Point p;
        p.x = point.x();
        p.y = point.y();
        collision_point_array.push_back(p);
      }

      // Also check the corner points

      for (const auto & point : object_polygon.outer()) {
        if (bg::within(point, one_step_move_vehicle_polygon2d)) {
          geometry_msgs::msg::Point p;
          p.x = point.x();
          p.y = point.y();
          collision_point_array.push_back(p);
        }
      }
      geometry_msgs::msg::Point nearest_collision_point_tmp;

      double norm = utils::getNearestPointAndDistanceForPredictedObject(
        collision_point_array, base_pose, &nearest_collision_point_tmp);
      if (norm < min_norm_collision_norm || !is_init) {
        min_norm_collision_norm = norm;
        nearest_collision_point = nearest_collision_point_tmp;
        is_init = true;
        nearest_collision_object_index = i;
      }
    }
  }
  if (is_init) {
    const auto & obj = dynamic_objects->objects.at(nearest_collision_object_index);
    const auto obstacle_polygon = utils::convertObjToPolygon(obj);
    if (param_.enable_z_axis_obstacle_filtering) {
      debug_ptr_->pushPolyhedron(obstacle_polygon, z_min, z_max, PolygonType::Collision);
    } else {
      debug_ptr_->pushPolygon(
        obstacle_polygon, obj.kinematics.initial_pose_with_covariance.pose.position.z,
        PolygonType::Collision);
    }
    return boost::make_optional(std::make_pair(nearest_collision_point, obj));
  }
  return boost::none;
}
}  // namespace autoware::motion::control::predicted_path_checker
