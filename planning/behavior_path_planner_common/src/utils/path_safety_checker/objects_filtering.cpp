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

#include "behavior_path_planner_common/utils/path_safety_checker/objects_filtering.hpp"

#include "behavior_path_planner_common/utils/utils.hpp"
#include "object_recognition_utils/predicted_path_utils.hpp"

#include <motion_utils/trajectory/interpolation.hpp>
#include <tier4_autoware_utils/geometry/boost_polygon_utils.hpp>

#include <boost/geometry/algorithms/distance.hpp>

#include <algorithm>

namespace behavior_path_planner::utils::path_safety_checker::filter
{
bool velocity_filter(const PredictedObject & object, double velocity_threshold, double max_velocity)
{
  const auto v_norm = std::hypot(
    object.kinematics.initial_twist_with_covariance.twist.linear.x,
    object.kinematics.initial_twist_with_covariance.twist.linear.y);
  return (velocity_threshold < v_norm && v_norm < max_velocity);
}

bool position_filter(
  const PredictedObject & object, const std::vector<PathPointWithLaneId> & path_points,
  const geometry_msgs::msg::Point & current_pose, const double forward_distance,
  const double backward_distance)
{
  const auto dist_ego_to_obj = motion_utils::calcSignedArcLength(
    path_points, current_pose, object.kinematics.initial_pose_with_covariance.pose.position);

  return (backward_distance < dist_ego_to_obj && dist_ego_to_obj < forward_distance);
}

bool is_within_circle(
  const geometry_msgs::msg::Point & object_pos, const geometry_msgs::msg::Point & reference_point,
  const double search_radius)
{
  const double dist =
    std::hypot(reference_point.x - object_pos.x, reference_point.y - object_pos.y);
  return dist < search_radius;
}
}  // namespace behavior_path_planner::utils::path_safety_checker::filter

namespace behavior_path_planner::utils::path_safety_checker
{
bool isCentroidWithinLanelet(const PredictedObject & object, const lanelet::ConstLanelet & lanelet)
{
  const auto & object_pos = object.kinematics.initial_pose_with_covariance.pose.position;
  lanelet::BasicPoint2d object_centroid(object_pos.x, object_pos.y);
  return boost::geometry::within(object_centroid, lanelet.polygon2d().basicPolygon());
}

bool isPolygonOverlapLanelet(const PredictedObject & object, const lanelet::ConstLanelet & lanelet)
{
  const auto object_polygon = tier4_autoware_utils::toPolygon2d(object);
  const auto lanelet_polygon = utils::toPolygon2d(lanelet);
  return !boost::geometry::disjoint(lanelet_polygon, object_polygon);
}

PredictedObjects filterObjects(
  const std::shared_ptr<const PredictedObjects> & objects,
  const std::shared_ptr<RouteHandler> & route_handler, const lanelet::ConstLanelets & current_lanes,
  const geometry_msgs::msg::Point & current_pose,
  const std::shared_ptr<ObjectsFilteringParams> & params)
{
  // Guard
  if (objects->objects.empty()) {
    return PredictedObjects();
  }

  const double ignore_object_velocity_threshold = params->ignore_object_velocity_threshold;
  const double object_check_forward_distance = params->object_check_forward_distance;
  const double object_check_backward_distance = params->object_check_backward_distance;
  const ObjectTypesToCheck & target_object_types = params->object_types_to_check;

  PredictedObjects filtered_objects =
    filterObjectsByVelocity(*objects, ignore_object_velocity_threshold, false);

  filterObjectsByClass(filtered_objects, target_object_types);

  const auto path = route_handler->getCenterLinePath(
    current_lanes, object_check_backward_distance, object_check_forward_distance);

  filterObjectsByPosition(
    filtered_objects, path.points, current_pose, object_check_forward_distance,
    object_check_backward_distance);

  return filtered_objects;
}

PredictedObjects filterObjectsByVelocity(
  const PredictedObjects & objects, const double velocity_threshold,
  const bool remove_above_threshold)
{
  if (remove_above_threshold) {
    return filterObjectsByVelocity(objects, -velocity_threshold, velocity_threshold);
  }
  return filterObjectsByVelocity(objects, velocity_threshold, std::numeric_limits<double>::max());
}

PredictedObjects filterObjectsByVelocity(
  const PredictedObjects & objects, double velocity_threshold, double max_velocity)
{
  const auto filter = [&](const auto & object) {
    return filter::velocity_filter(object, velocity_threshold, max_velocity);
  };

  auto filtered = objects;
  filterObjects(filtered, filter);
  return filtered;
}

void filterObjectsByPosition(
  PredictedObjects & objects, const std::vector<PathPointWithLaneId> & path_points,
  const geometry_msgs::msg::Point & current_pose, const double forward_distance,
  const double backward_distance)
{
  const auto filter = [&](const auto & object) {
    return filter::position_filter(
      object, path_points, current_pose, forward_distance, -backward_distance);
  };

  filterObjects(objects, filter);
}

void filterObjectsWithinRadius(
  PredictedObjects & objects, const geometry_msgs::msg::Point & reference_point,
  const double search_radius)
{
  const auto filter = [&](const auto & object) {
    return filter::is_within_circle(
      object.kinematics.initial_pose_with_covariance.pose.position, reference_point, search_radius);
  };

  filterObjects(objects, filter);
}

void filterObjectsByClass(
  PredictedObjects & objects, const ObjectTypesToCheck & target_object_types)
{
  const auto filter = [&](const auto & object) {
    return isTargetObjectType(object, target_object_types);
  };

  filterObjects(objects, filter);
}

std::pair<std::vector<size_t>, std::vector<size_t>> separateObjectIndicesByLanelets(
  const PredictedObjects & objects, const lanelet::ConstLanelets & target_lanelets,
  const std::function<bool(const PredictedObject, const lanelet::ConstLanelet)> & condition)
{
  if (target_lanelets.empty()) {
    return {};
  }

  std::vector<size_t> target_indices;
  std::vector<size_t> other_indices;

  for (size_t i = 0; i < objects.objects.size(); i++) {
    const auto filter = [&](const auto & llt) { return condition(objects.objects.at(i), llt); };
    const auto found = std::find_if(target_lanelets.begin(), target_lanelets.end(), filter);
    if (found != target_lanelets.end()) {
      target_indices.push_back(i);
    } else {
      other_indices.push_back(i);
    }
  }

  return std::make_pair(target_indices, other_indices);
}

std::pair<PredictedObjects, PredictedObjects> separateObjectsByLanelets(
  const PredictedObjects & objects, const lanelet::ConstLanelets & target_lanelets,
  const std::function<bool(const PredictedObject, const lanelet::ConstLanelet)> & condition)
{
  PredictedObjects target_objects;
  PredictedObjects other_objects;

  const auto [target_indices, other_indices] =
    separateObjectIndicesByLanelets(objects, target_lanelets, condition);

  target_objects.objects.reserve(target_indices.size());
  other_objects.objects.reserve(other_indices.size());

  for (const size_t i : target_indices) {
    target_objects.objects.push_back(objects.objects.at(i));
  }

  for (const size_t i : other_indices) {
    other_objects.objects.push_back(objects.objects.at(i));
  }

  return std::make_pair(target_objects, other_objects);
}

std::vector<PredictedPathWithPolygon> getPredictedPathFromObj(
  const ExtendedPredictedObject & obj, const bool & is_use_all_predicted_path)
{
  if (!is_use_all_predicted_path) {
    const auto max_confidence_path = std::max_element(
      obj.predicted_paths.begin(), obj.predicted_paths.end(),
      [](const auto & path1, const auto & path2) { return path1.confidence < path2.confidence; });
    if (max_confidence_path != obj.predicted_paths.end()) {
      return {*max_confidence_path};
    }
  }

  return obj.predicted_paths;
}

std::vector<PoseWithVelocityStamped> createPredictedPath(
  const std::shared_ptr<EgoPredictedPathParams> & ego_predicted_path_params,
  const std::vector<PathPointWithLaneId> & path_points,
  const geometry_msgs::msg::Pose & vehicle_pose, const double current_velocity,
  const size_t ego_seg_idx, const bool is_object_front, const bool limit_to_max_velocity)
{
  if (path_points.empty()) {
    return {};
  }

  const double min_velocity = ego_predicted_path_params->min_velocity;
  const double acceleration = ego_predicted_path_params->acceleration;
  const double max_velocity = limit_to_max_velocity ? ego_predicted_path_params->max_velocity
                                                    : std::numeric_limits<double>::infinity();
  const double time_horizon = is_object_front
                                ? ego_predicted_path_params->time_horizon_for_front_object
                                : ego_predicted_path_params->time_horizon_for_rear_object;
  const double time_resolution = ego_predicted_path_params->time_resolution;
  const double delay_until_departure = ego_predicted_path_params->delay_until_departure;

  std::vector<PoseWithVelocityStamped> predicted_path;
  const auto vehicle_pose_frenet =
    convertToFrenetPoint(path_points, vehicle_pose.position, ego_seg_idx);

  for (double t = 0.0; t < time_horizon; t += time_resolution) {
    double velocity = 0.0;
    double length = 0.0;

    // If t < delay_until_departure, it means ego have not depart yet, therefore the velocity is
    // 0 and there's no change in position.
    if (t >= delay_until_departure) {
      // Adjust time to consider the delay.
      double t_with_delay = t - delay_until_departure;
      velocity =
        std::clamp(current_velocity + acceleration * t_with_delay, min_velocity, max_velocity);
      length = current_velocity * t_with_delay + 0.5 * acceleration * t_with_delay * t_with_delay;
    }

    const auto pose =
      motion_utils::calcInterpolatedPose(path_points, vehicle_pose_frenet.length + length);
    predicted_path.emplace_back(t, pose, velocity);
  }

  return predicted_path;
}

bool isCentroidWithinLanelets(
  const PredictedObject & object, const lanelet::ConstLanelets & target_lanelets)
{
  if (target_lanelets.empty()) {
    return false;
  }

  const auto & object_pos = object.kinematics.initial_pose_with_covariance.pose.position;
  lanelet::BasicPoint2d object_centroid(object_pos.x, object_pos.y);

  const auto is_within = [&](const auto & llt) {
    return boost::geometry::within(object_centroid, llt.polygon2d().basicPolygon());
  };

  return std::any_of(target_lanelets.begin(), target_lanelets.end(), is_within);
}

ExtendedPredictedObject transform(
  const PredictedObject & object, const double safety_check_time_horizon,
  const double safety_check_time_resolution)
{
  ExtendedPredictedObject extended_object;
  extended_object.uuid = object.object_id;
  extended_object.initial_pose = object.kinematics.initial_pose_with_covariance;
  extended_object.initial_twist = object.kinematics.initial_twist_with_covariance;
  extended_object.initial_acceleration = object.kinematics.initial_acceleration_with_covariance;
  extended_object.shape = object.shape;
  extended_object.classification = object.classification;

  const auto obj_velocity = extended_object.initial_twist.twist.linear.x;

  extended_object.predicted_paths.resize(object.kinematics.predicted_paths.size());
  for (size_t i = 0; i < object.kinematics.predicted_paths.size(); ++i) {
    const auto & path = object.kinematics.predicted_paths[i];
    extended_object.predicted_paths[i].confidence = path.confidence;

    // Create path based on time horizon and resolution
    for (double t = 0.0; t < safety_check_time_horizon + 1e-3; t += safety_check_time_resolution) {
      const auto obj_pose = object_recognition_utils::calcInterpolatedPose(path, t);
      if (obj_pose) {
        const auto obj_polygon = tier4_autoware_utils::toPolygon2d(*obj_pose, object.shape);
        extended_object.predicted_paths[i].path.emplace_back(
          t, *obj_pose, obj_velocity, obj_polygon);
      }
    }
  }

  return extended_object;
}

TargetObjectsOnLane createTargetObjectsOnLane(
  const lanelet::ConstLanelets & current_lanes, const std::shared_ptr<RouteHandler> & route_handler,
  const PredictedObjects & filtered_objects, const std::shared_ptr<ObjectsFilteringParams> & params)
{
  const auto & object_lane_configuration = params->object_lane_configuration;
  const bool include_opposite = params->include_opposite_lane;
  const bool invert_opposite = params->invert_opposite_lane;
  const double safety_check_time_horizon = params->safety_check_time_horizon;
  const double safety_check_time_resolution = params->safety_check_time_resolution;

  lanelet::ConstLanelets all_left_lanelets;
  lanelet::ConstLanelets all_right_lanelets;

  // Define lambda functions to update left and right lanelets
  const auto update_left_lanelets = [&](const lanelet::ConstLanelet & target_lane) {
    const auto left_lanelets = route_handler->getAllLeftSharedLinestringLanelets(
      target_lane, include_opposite, invert_opposite);
    all_left_lanelets.insert(all_left_lanelets.end(), left_lanelets.begin(), left_lanelets.end());
  };

  const auto update_right_lanelets = [&](const lanelet::ConstLanelet & target_lane) {
    const auto right_lanelets = route_handler->getAllRightSharedLinestringLanelets(
      target_lane, include_opposite, invert_opposite);
    all_right_lanelets.insert(
      all_right_lanelets.end(), right_lanelets.begin(), right_lanelets.end());
  };

  // Update left and right lanelets for each current lane
  for (const auto & current_lane : current_lanes) {
    update_left_lanelets(current_lane);
    update_right_lanelets(current_lane);
  }

  TargetObjectsOnLane target_objects_on_lane{};
  const auto append_objects_on_lane = [&](auto & lane_objects, const auto & check_lanes) {
    std::for_each(
      filtered_objects.objects.begin(), filtered_objects.objects.end(), [&](const auto & object) {
        if (isCentroidWithinLanelets(object, check_lanes)) {
          lane_objects.push_back(
            transform(object, safety_check_time_horizon, safety_check_time_resolution));
        }
      });
  };

  // TODO(Sugahara): Consider shoulder and other lane objects
  if (object_lane_configuration.check_current_lane && !current_lanes.empty()) {
    append_objects_on_lane(target_objects_on_lane.on_current_lane, current_lanes);
  }
  if (object_lane_configuration.check_left_lane && !all_left_lanelets.empty()) {
    append_objects_on_lane(target_objects_on_lane.on_left_lane, all_left_lanelets);
  }
  if (object_lane_configuration.check_right_lane && !all_right_lanelets.empty()) {
    append_objects_on_lane(target_objects_on_lane.on_right_lane, all_right_lanelets);
  }

  return target_objects_on_lane;
}

bool isTargetObjectType(
  const PredictedObject & object, const ObjectTypesToCheck & target_object_types)
{
  using autoware_auto_perception_msgs::msg::ObjectClassification;
  const auto t = utils::getHighestProbLabel(object.classification);
  return (
    (t == ObjectClassification::CAR && target_object_types.check_car) ||
    (t == ObjectClassification::TRUCK && target_object_types.check_truck) ||
    (t == ObjectClassification::BUS && target_object_types.check_bus) ||
    (t == ObjectClassification::TRAILER && target_object_types.check_trailer) ||
    (t == ObjectClassification::UNKNOWN && target_object_types.check_unknown) ||
    (t == ObjectClassification::BICYCLE && target_object_types.check_bicycle) ||
    (t == ObjectClassification::MOTORCYCLE && target_object_types.check_motorcycle) ||
    (t == ObjectClassification::PEDESTRIAN && target_object_types.check_pedestrian));
}
}  // namespace behavior_path_planner::utils::path_safety_checker
