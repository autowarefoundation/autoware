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

#include "behavior_path_planner/scene_module/dynamic_avoidance/dynamic_avoidance_module.hpp"

#include "behavior_path_planner/utils/path_utils.hpp"
#include "behavior_path_planner/utils/utils.hpp"
#include "signal_processing/lowpass_filter_1d.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace behavior_path_planner
{
namespace
{
geometry_msgs::msg::Point toGeometryPoint(const tier4_autoware_utils::Point2d & point)
{
  geometry_msgs::msg::Point geom_obj_point;
  geom_obj_point.x = point.x();
  geom_obj_point.y = point.y();
  return geom_obj_point;
}

MinMaxValue getMinMaxValues(const std::vector<double> & vec)
{
  const size_t min_idx = std::distance(vec.begin(), std::min_element(vec.begin(), vec.end()));

  const size_t max_idx = std::distance(vec.begin(), std::max_element(vec.begin(), vec.end()));

  return MinMaxValue{vec.at(min_idx), vec.at(max_idx)};
}

void appendObjectMarker(MarkerArray & marker_array, const geometry_msgs::msg::Pose & obj_pose)
{
  auto marker = tier4_autoware_utils::createDefaultMarker(
    "map", rclcpp::Clock{RCL_ROS_TIME}.now(), "dynamic_objects_to_avoid",
    marker_array.markers.size(), visualization_msgs::msg::Marker::CUBE,
    tier4_autoware_utils::createMarkerScale(3.0, 1.0, 1.0),
    tier4_autoware_utils::createMarkerColor(1.0, 0.5, 0.6, 0.8));
  marker.pose = obj_pose;

  marker_array.markers.push_back(marker);
}

void appendExtractedPolygonMarker(
  MarkerArray & marker_array, const tier4_autoware_utils::Polygon2d & obj_poly, const double obj_z)
{
  auto marker = tier4_autoware_utils::createDefaultMarker(
    "map", rclcpp::Clock{RCL_ROS_TIME}.now(), "extracted_polygons", marker_array.markers.size(),
    visualization_msgs::msg::Marker::LINE_STRIP,
    tier4_autoware_utils::createMarkerScale(0.1, 0.0, 0.0),
    tier4_autoware_utils::createMarkerColor(1.0, 0.5, 0.6, 0.8));

  // NOTE: obj_poly.outer() has already duplicated points to close the polygon.
  for (size_t i = 0; i < obj_poly.outer().size(); ++i) {
    const auto & bound_point = obj_poly.outer().at(i);

    geometry_msgs::msg::Point bound_geom_point;
    bound_geom_point.x = bound_point.x();
    bound_geom_point.y = bound_point.y();
    bound_geom_point.z = obj_z;
    marker.points.push_back(bound_geom_point);
  }

  marker_array.markers.push_back(marker);
}

template <typename T>
std::optional<T> getObjectFromUuid(const std::vector<T> & objects, const std::string & target_uuid)
{
  const auto itr = std::find_if(objects.begin(), objects.end(), [&](const auto & object) {
    return object.uuid == target_uuid;
  });

  if (itr == objects.end()) {
    return std::nullopt;
  }
  return *itr;
}

std::pair<double, double> projectObstacleVelocityToTrajectory(
  const std::vector<PathPointWithLaneId> & path_points, const PredictedObject & object)
{
  const auto & obj_pose = object.kinematics.initial_pose_with_covariance.pose;
  const double obj_vel_norm = std::hypot(
    object.kinematics.initial_twist_with_covariance.twist.linear.x,
    object.kinematics.initial_twist_with_covariance.twist.linear.y);

  const size_t obj_idx = motion_utils::findNearestIndex(path_points, obj_pose.position);

  const double obj_yaw = tf2::getYaw(obj_pose.orientation);
  const double obj_vel_yaw =
    obj_yaw + std::atan2(
                object.kinematics.initial_twist_with_covariance.twist.linear.y,
                object.kinematics.initial_twist_with_covariance.twist.linear.x);
  const double path_yaw = tf2::getYaw(path_points.at(obj_idx).point.pose.orientation);

  const double diff_yaw = tier4_autoware_utils::normalizeRadian(obj_vel_yaw - path_yaw);
  return std::make_pair(obj_vel_norm * std::cos(diff_yaw), obj_vel_norm * std::sin(diff_yaw));
}

double calcObstacleMaxLength(const autoware_auto_perception_msgs::msg::Shape & shape)
{
  if (shape.type == autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX) {
    return std::hypot(shape.dimensions.x / 2.0, shape.dimensions.y / 2.0);
  } else if (shape.type == autoware_auto_perception_msgs::msg::Shape::CYLINDER) {
    return shape.dimensions.x / 2.0;
  } else if (shape.type == autoware_auto_perception_msgs::msg::Shape::POLYGON) {
    double max_length_to_point = 0.0;
    for (const auto rel_point : shape.footprint.points) {
      const double length_to_point = std::hypot(rel_point.x, rel_point.y);
      if (max_length_to_point < length_to_point) {
        max_length_to_point = length_to_point;
      }
    }
    return max_length_to_point;
  }

  throw std::logic_error("The shape type is not supported in obstacle_cruise_planner.");
}

double calcDiffAngleAgainstPath(
  const std::vector<PathPointWithLaneId> & path_points,
  const geometry_msgs::msg::Pose & target_pose)
{
  const size_t nearest_idx = motion_utils::findNearestIndex(path_points, target_pose.position);
  const double traj_yaw = tf2::getYaw(path_points.at(nearest_idx).point.pose.orientation);

  const double target_yaw = tf2::getYaw(target_pose.orientation);

  const double diff_yaw = tier4_autoware_utils::normalizeRadian(target_yaw - traj_yaw);
  return diff_yaw;
}

double calcDiffAngleBetweenPaths(
  const std::vector<PathPointWithLaneId> & path_points, const PredictedPath & predicted_path)
{
  const size_t nearest_idx =
    motion_utils::findNearestSegmentIndex(path_points, predicted_path.path.front().position);
  const double ego_yaw = tf2::getYaw(path_points.at(nearest_idx).point.pose.orientation);

  constexpr size_t max_predicted_path_size = 5;
  double signed_max_angle{0.0};
  for (size_t i = 0; i < std::min(max_predicted_path_size, predicted_path.path.size()); ++i) {
    const double obj_yaw = tf2::getYaw(predicted_path.path.at(i).orientation);
    const double diff_yaw = tier4_autoware_utils::normalizeRadian(obj_yaw - ego_yaw);
    if (std::abs(signed_max_angle) < std::abs(diff_yaw)) {
      signed_max_angle = diff_yaw;
    }
  }
  return signed_max_angle;
}

double calcDistanceToPath(
  const std::vector<PathPointWithLaneId> & path_points,
  const geometry_msgs::msg::Point & target_pos)
{
  const size_t target_idx = motion_utils::findNearestIndex(path_points, target_pos);
  if (target_idx == 0 || target_idx == path_points.size() - 1) {
    const double target_yaw = tf2::getYaw(path_points.at(target_idx).point.pose.orientation);
    const double angle_to_target_pos = tier4_autoware_utils::calcAzimuthAngle(
      path_points.at(target_idx).point.pose.position, target_pos);
    const double diff_yaw = tier4_autoware_utils::normalizeRadian(angle_to_target_pos - target_yaw);

    if (
      (target_idx == 0 && (diff_yaw < -M_PI_2 || M_PI_2 < diff_yaw)) ||
      (target_idx == path_points.size() - 1 && (-M_PI_2 < diff_yaw && diff_yaw < M_PI_2))) {
      return tier4_autoware_utils::calcDistance2d(path_points.at(target_idx), target_pos);
    }
  }

  return std::abs(motion_utils::calcLateralOffset(path_points, target_pos));
}

bool isLeft(
  const std::vector<PathPointWithLaneId> & path_points,
  const geometry_msgs::msg::Point & target_pos)
{
  const size_t target_idx = motion_utils::findNearestIndex(path_points, target_pos);
  const double target_yaw = tf2::getYaw(path_points.at(target_idx).point.pose.orientation);
  const double angle_to_target_pos = tier4_autoware_utils::calcAzimuthAngle(
    path_points.at(target_idx).point.pose.position, target_pos);
  const double diff_yaw = tier4_autoware_utils::normalizeRadian(angle_to_target_pos - target_yaw);

  if (0 < diff_yaw) {
    return true;
  }
  return false;
}

template <typename T>
std::optional<T> getObstacleFromUuid(
  const std::vector<T> & obstacles, const std::string & target_uuid)
{
  const auto itr = std::find_if(obstacles.begin(), obstacles.end(), [&](const auto & obstacle) {
    return obstacle.uuid == target_uuid;
  });

  if (itr == obstacles.end()) {
    return std::nullopt;
  }
  return *itr;
}
}  // namespace

DynamicAvoidanceModule::DynamicAvoidanceModule(
  const std::string & name, rclcpp::Node & node,
  std::shared_ptr<DynamicAvoidanceParameters> parameters,
  const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> & rtc_interface_ptr_map)
: SceneModuleInterface{name, node, rtc_interface_ptr_map},
  parameters_{std::move(parameters)},
  target_objects_manager_{TargetObjectsManager(
    parameters_->successive_num_to_entry_dynamic_avoidance_condition,
    parameters_->successive_num_to_exit_dynamic_avoidance_condition)}
{
}

bool DynamicAvoidanceModule::isExecutionRequested() const
{
  RCLCPP_DEBUG(getLogger(), "DYNAMIC AVOIDANCE isExecutionRequested.");

  const auto prev_module_path = getPreviousModuleOutput().path;
  if (!prev_module_path || prev_module_path->points.size() < 2) {
    return false;
  }

  // check if the ego is driving forward
  const auto is_driving_forward = motion_utils::isDrivingForward(prev_module_path->points);
  if (!is_driving_forward || !(*is_driving_forward)) {
    return false;
  }

  // check if the planner is already running
  if (current_state_ == ModuleStatus::RUNNING) {
    return true;
  }

  // check if there is target objects to avoid
  return !target_objects_.empty();
}

bool DynamicAvoidanceModule::isExecutionReady() const
{
  RCLCPP_DEBUG(getLogger(), "DYNAMIC AVOIDANCE isExecutionReady.");
  return true;
}

void DynamicAvoidanceModule::updateData()
{
  // calculate target objects
  updateTargetObjects();

  const auto target_objects_candidate = target_objects_manager_.getValidObjects();
  target_objects_.clear();
  for (const auto & target_object_candidate : target_objects_candidate) {
    if (target_object_candidate.should_be_avoided) {
      target_objects_.push_back(target_object_candidate);
    }
  }
}

ModuleStatus DynamicAvoidanceModule::updateState()
{
  const bool has_avoidance_target = !target_objects_.empty();

  if (!has_avoidance_target) {
    return ModuleStatus::SUCCESS;
  }

  return ModuleStatus::RUNNING;
}

BehaviorModuleOutput DynamicAvoidanceModule::plan()
{
  info_marker_.markers.clear();
  debug_marker_.markers.clear();

  const auto prev_module_path = getPreviousModuleOutput().path;

  // create obstacles to avoid (= extract from the drivable area)
  std::vector<DrivableAreaInfo::Obstacle> obstacles_for_drivable_area;
  for (const auto & object : target_objects_) {
    const auto obstacle_poly = [&]() {
      if (object.polygon_generation_method == PolygonGenerationMethod::EGO_PATH_BASE) {
        return calcEgoPathBasedDynamicObstaclePolygon(object);
      }
      if (object.polygon_generation_method == PolygonGenerationMethod::OBJECT_PATH_BASE) {
        return calcObjectPathBasedDynamicObstaclePolygon(object);
      }
      throw std::logic_error("The polygon_generation_method's string is invalid.");
    }();
    if (obstacle_poly) {
      obstacles_for_drivable_area.push_back(
        {object.pose, obstacle_poly.value(), object.is_collision_left});

      appendObjectMarker(info_marker_, object.pose);
      appendExtractedPolygonMarker(debug_marker_, obstacle_poly.value(), object.pose.position.z);
    }
  }

  DrivableAreaInfo current_drivable_area_info;
  current_drivable_area_info.drivable_lanes =
    getPreviousModuleOutput().drivable_area_info.drivable_lanes;
  current_drivable_area_info.obstacles = obstacles_for_drivable_area;
  current_drivable_area_info.enable_expanding_hatched_road_markings =
    parameters_->use_hatched_road_markings;

  BehaviorModuleOutput output;
  output.path = prev_module_path;
  output.drivable_area_info = utils::combineDrivableAreaInfo(
    current_drivable_area_info, getPreviousModuleOutput().drivable_area_info);
  output.reference_path = getPreviousModuleOutput().reference_path;
  output.turn_signal_info = getPreviousModuleOutput().turn_signal_info;

  return output;
}

CandidateOutput DynamicAvoidanceModule::planCandidate() const
{
  auto candidate_path = utils::generateCenterLinePath(planner_data_);
  return CandidateOutput(*candidate_path);
}

BehaviorModuleOutput DynamicAvoidanceModule::planWaitingApproval()
{
  BehaviorModuleOutput out = plan();
  return out;
}

bool DynamicAvoidanceModule::isLabelTargetObstacle(const uint8_t label) const
{
  using autoware_auto_perception_msgs::msg::ObjectClassification;

  if (label == ObjectClassification::CAR && parameters_->avoid_car) {
    return true;
  }
  if (label == ObjectClassification::TRUCK && parameters_->avoid_truck) {
    return true;
  }
  if (label == ObjectClassification::BUS && parameters_->avoid_bus) {
    return true;
  }
  if (label == ObjectClassification::TRAILER && parameters_->avoid_trailer) {
    return true;
  }
  if (label == ObjectClassification::UNKNOWN && parameters_->avoid_unknown) {
    return true;
  }
  if (label == ObjectClassification::BICYCLE && parameters_->avoid_bicycle) {
    return true;
  }
  if (label == ObjectClassification::MOTORCYCLE && parameters_->avoid_motorcycle) {
    return true;
  }
  if (label == ObjectClassification::PEDESTRIAN && parameters_->avoid_pedestrian) {
    return true;
  }
  return false;
}

void DynamicAvoidanceModule::updateTargetObjects()
{
  const auto prev_module_path = getPreviousModuleOutput().path;
  const auto & predicted_objects = planner_data_->dynamic_object->objects;

  const auto path_points_for_object_polygon = getPreviousModuleOutput().reference_path->points;
  const auto prev_objects = target_objects_manager_.getValidObjects();

  // 1. Rough filtering of target objects
  target_objects_manager_.initialize();
  for (const auto & predicted_object : predicted_objects) {
    const auto obj_uuid = tier4_autoware_utils::toHexString(predicted_object.object_id);
    const auto & obj_pose = predicted_object.kinematics.initial_pose_with_covariance.pose;
    const double obj_vel_norm = std::hypot(
      predicted_object.kinematics.initial_twist_with_covariance.twist.linear.x,
      predicted_object.kinematics.initial_twist_with_covariance.twist.linear.y);
    const auto prev_object = getObstacleFromUuid(prev_objects, obj_uuid);
    const auto obj_path = *std::max_element(
      predicted_object.kinematics.predicted_paths.begin(),
      predicted_object.kinematics.predicted_paths.end(),
      [](const PredictedPath & a, const PredictedPath & b) { return a.confidence < b.confidence; });

    // 1.a. check label
    const bool is_label_target_obstacle =
      isLabelTargetObstacle(predicted_object.classification.front().label);
    if (!is_label_target_obstacle) {
      continue;
    }

    // 1.b. check if velocity is large enough
    const auto [obj_tangent_vel, obj_normal_vel] =
      projectObstacleVelocityToTrajectory(prev_module_path->points, predicted_object);
    if (std::abs(obj_tangent_vel) < parameters_->min_obstacle_vel) {
      continue;
    }

    // 1.c. check if object is not crossing ego's path
    const double obj_angle = calcDiffAngleBetweenPaths(prev_module_path->points, obj_path);
    const double max_crossing_object_angle = 0.0 <= obj_tangent_vel
                                               ? parameters_->max_overtaking_crossing_object_angle
                                               : parameters_->max_oncoming_crossing_object_angle;
    const bool is_obstacle_crossing_path = max_crossing_object_angle < std::abs(obj_angle) &&
                                           max_crossing_object_angle < M_PI - std::abs(obj_angle);
    const double min_crossing_object_vel = 0.0 <= obj_tangent_vel
                                             ? parameters_->min_overtaking_crossing_object_vel
                                             : parameters_->min_oncoming_crossing_object_vel;
    const bool is_crossing_object_to_ignore =
      min_crossing_object_vel < obj_vel_norm && is_obstacle_crossing_path;
    if (is_crossing_object_to_ignore) {
      RCLCPP_INFO_EXPRESSION(
        getLogger(), parameters_->enable_debug_info,
        "[DynamicAvoidance] Ignore obstacle (%s) since it crosses the ego's path.",
        obj_uuid.c_str());
      continue;
    }

    // 1.e. check if object lateral offset to ego's path is small enough
    const double obj_dist_to_path = calcDistanceToPath(prev_module_path->points, obj_pose.position);
    const bool is_object_far_from_path = isObjectFarFromPath(predicted_object, obj_dist_to_path);
    if (is_object_far_from_path) {
      RCLCPP_INFO_EXPRESSION(
        getLogger(), parameters_->enable_debug_info,
        "[DynamicAvoidance] Ignore obstacle (%s) since lateral offset is large.", obj_uuid.c_str());
      continue;
    }

    // 1.f. calculate the object is on ego's path or not
    const bool is_object_on_ego_path =
      obj_dist_to_path <
      planner_data_->parameters.vehicle_width / 2.0 + parameters_->min_obj_lat_offset_to_ego_path;

    // 1.g. calculate latest time inside ego's path
    const auto latest_time_inside_ego_path = [&]() -> std::optional<rclcpp::Time> {
      if (!prev_object || !prev_object->latest_time_inside_ego_path) {
        if (is_object_on_ego_path) {
          return clock_->now();
        }
        return std::nullopt;
      }
      if (is_object_on_ego_path) {
        return clock_->now();
      }
      return *prev_object->latest_time_inside_ego_path;
    }();

    const auto target_object = DynamicAvoidanceObject(
      predicted_object, obj_tangent_vel, obj_normal_vel, is_object_on_ego_path,
      latest_time_inside_ego_path);
    target_objects_manager_.updateObject(obj_uuid, target_object);
  }
  target_objects_manager_.finalize();

  // 2. Precise filtering of target objects and check if they should be avoided
  for (const auto & object : target_objects_manager_.getValidObjects()) {
    PolygonGenerationMethod polygon_generation_method{PolygonGenerationMethod::EGO_PATH_BASE};
    const auto obj_uuid = object.uuid;
    const auto prev_object = getObstacleFromUuid(prev_objects, obj_uuid);
    const auto obj_path = *std::max_element(
      object.predicted_paths.begin(), object.predicted_paths.end(),
      [](const PredictedPath & a, const PredictedPath & b) { return a.confidence < b.confidence; });

    // 2.a. check if object is not to be followed by ego
    const double obj_angle = calcDiffAngleAgainstPath(prev_module_path->points, object.pose);
    const bool is_object_aligned_to_path =
      std::abs(obj_angle) < parameters_->max_front_object_angle ||
      M_PI - parameters_->max_front_object_angle < std::abs(obj_angle);
    if (object.is_object_on_ego_path && is_object_aligned_to_path) {
      RCLCPP_INFO_EXPRESSION(
        getLogger(), parameters_->enable_debug_info,
        "[DynamicAvoidance] Ignore obstacle (%s) since it is to be followed.", obj_uuid.c_str());
      continue;
    }

    // 2.b. calculate which side object exists against ego's path
    const bool is_object_left = isLeft(prev_module_path->points, object.pose.position);
    const auto lat_lon_offset =
      getLateralLongitudinalOffset(prev_module_path->points, object.pose, object.shape);

    // 2.c. check if object will not cut in
    const bool will_object_cut_in = willObjectCutIn(
      prev_module_path->points, obj_path, object.vel, lat_lon_offset, polygon_generation_method);
    if (will_object_cut_in) {
      RCLCPP_INFO_EXPRESSION(
        getLogger(), parameters_->enable_debug_info,
        "[DynamicAvoidance] Ignore obstacle (%s) since it will cut in.", obj_uuid.c_str());
      continue;
    }

    // 2.d. check if object will not cut out
    const auto will_object_cut_out =
      willObjectCutOut(object.vel, object.lat_vel, is_object_left, prev_object);
    if (will_object_cut_out.decision) {
      printIgnoreReason(obj_uuid.c_str(), will_object_cut_out.reason);
      continue;
    }

    // 2.e. check time to collision
    const double time_to_collision =
      calcTimeToCollision(prev_module_path->points, object.pose, object.vel, lat_lon_offset);
    if (
      (0 <= object.vel &&
       parameters_->max_time_to_collision_overtaking_object < time_to_collision) ||
      (object.vel <= 0 && parameters_->max_time_to_collision_oncoming_object < time_to_collision)) {
      RCLCPP_INFO_EXPRESSION(
        getLogger(), parameters_->enable_debug_info,
        "[DynamicAvoidance] Ignore obstacle (%s) since time to collision (%f) is large.",
        obj_uuid.c_str(), time_to_collision);
      continue;
    }
    if (time_to_collision < -parameters_->duration_to_hold_avoidance_overtaking_object) {
      RCLCPP_INFO_EXPRESSION(
        getLogger(), parameters_->enable_debug_info,
        "[DynamicAvoidance] Ignore obstacle (%s) since time to collision (%f) is a small negative "
        "value.",
        obj_uuid.c_str(), time_to_collision);
      continue;
    }

    // 2.f. calculate which side object will be against ego's path
    const auto future_obj_pose =
      object_recognition_utils::calcInterpolatedPose(obj_path, time_to_collision);
    const bool is_collision_left = future_obj_pose
                                     ? isLeft(prev_module_path->points, future_obj_pose->position)
                                     : is_object_left;

    // 2.g. check if the ego is not ahead of the object.
    const double signed_dist_ego_to_obj = [&]() {
      const size_t ego_seg_idx = planner_data_->findEgoSegmentIndex(prev_module_path->points);
      const double lon_offset_ego_to_obj = motion_utils::calcSignedArcLength(
        prev_module_path->points, getEgoPose().position, ego_seg_idx, lat_lon_offset.nearest_idx);
      if (0 < lon_offset_ego_to_obj) {
        return std::max(
          0.0, lon_offset_ego_to_obj - planner_data_->parameters.front_overhang +
                 lat_lon_offset.min_lon_offset);
      }
      return std::min(
        0.0, lon_offset_ego_to_obj + planner_data_->parameters.rear_overhang +
               lat_lon_offset.max_lon_offset);
    }();
    if (signed_dist_ego_to_obj < 0) {
      RCLCPP_INFO_EXPRESSION(
        getLogger(), parameters_->enable_debug_info,
        "[DynamicAvoidance] Ignore obstacle (%s) since distance from ego to object (%f) is less "
        "than 0.",
        obj_uuid.c_str(), signed_dist_ego_to_obj);
      continue;
    }

    // 2.h. calculate longitudinal and lateral offset to avoid to generate object polygon by
    // "object_path_base"
    const auto obj_points = tier4_autoware_utils::toPolygon2d(object.pose, object.shape);
    const auto lon_offset_to_avoid = calcMinMaxLongitudinalOffsetToAvoid(
      path_points_for_object_polygon, object.pose, obj_points, object.vel, time_to_collision);
    const auto lat_offset_to_avoid = calcMinMaxLateralOffsetToAvoid(
      path_points_for_object_polygon, obj_points, is_collision_left, object.lat_vel, prev_object);

    const bool should_be_avoided = true;
    target_objects_manager_.updateObject(
      obj_uuid, lon_offset_to_avoid, lat_offset_to_avoid, is_collision_left, should_be_avoided,
      polygon_generation_method);
  }
}

[[maybe_unused]] std::optional<std::pair<size_t, size_t>>
DynamicAvoidanceModule::calcCollisionSection(
  const std::vector<PathPointWithLaneId> & ego_path, const PredictedPath & obj_path) const
{
  const size_t ego_idx = planner_data_->findEgoIndex(ego_path);
  const double ego_vel = getEgoSpeed();

  std::optional<size_t> collision_start_idx{std::nullopt};
  double lon_dist = 0.0;
  for (size_t i = ego_idx; i < ego_path.size() - 1; ++i) {
    lon_dist += tier4_autoware_utils::calcDistance2d(ego_path.at(i), ego_path.at(i + 1));
    const double elapsed_time = lon_dist / ego_vel;

    const auto future_ego_pose = ego_path.at(i);
    const auto future_obj_pose =
      object_recognition_utils::calcInterpolatedPose(obj_path, elapsed_time);

    if (future_obj_pose) {
      const double dist_ego_to_obj =
        tier4_autoware_utils::calcDistance2d(future_ego_pose, *future_obj_pose);
      if (dist_ego_to_obj < 1.0) {
        if (!collision_start_idx) {
          collision_start_idx = i;
        }
        continue;
      }
    } else {
      if (!collision_start_idx) {
        continue;
      }
    }

    return std::make_pair(*collision_start_idx, i - 1);
  }

  return std::make_pair(*collision_start_idx, ego_path.size() - 1);
}

double DynamicAvoidanceModule::calcTimeToCollision(
  const std::vector<PathPointWithLaneId> & ego_path, const geometry_msgs::msg::Pose & obj_pose,
  const double obj_tangent_vel, const LatLonOffset & lat_lon_offset) const
{
  // Set maximum time-to-collision 0 if the object longitudinally overlaps ego.
  // NOTE: This is to avoid objects running right beside ego even if time-to-collision is negative.
  const size_t ego_seg_idx = planner_data_->findEgoSegmentIndex(ego_path);
  const double lon_offset_ego_to_obj =
    motion_utils::calcSignedArcLength(
      ego_path, getEgoPose().position, ego_seg_idx, lat_lon_offset.nearest_idx) +
    lat_lon_offset.max_lon_offset;
  const double maximum_time_to_collision =
    (0.0 < lon_offset_ego_to_obj) ? 0.0 : -std::numeric_limits<double>::max();

  const double relative_velocity = getEgoSpeed() - obj_tangent_vel;
  const size_t obj_seg_idx = motion_utils::findNearestSegmentIndex(ego_path, obj_pose.position);
  const double signed_lon_length = motion_utils::calcSignedArcLength(
    ego_path, getEgoPosition(), ego_seg_idx, obj_pose.position, obj_seg_idx);
  const double positive_relative_velocity = std::max(relative_velocity, 1.0);
  return std::max(maximum_time_to_collision, signed_lon_length / positive_relative_velocity);
}

bool DynamicAvoidanceModule::isObjectFarFromPath(
  const PredictedObject & predicted_object, const double obj_dist_to_path) const
{
  const double obj_max_length = calcObstacleMaxLength(predicted_object.shape);
  const double min_obj_dist_to_path = std::max(
    0.0, obj_dist_to_path - planner_data_->parameters.vehicle_width / 2.0 - obj_max_length);

  return parameters_->max_obj_lat_offset_to_ego_path < min_obj_dist_to_path;
}

bool DynamicAvoidanceModule::willObjectCutIn(
  const std::vector<PathPointWithLaneId> & ego_path, const PredictedPath & predicted_path,
  const double obj_tangent_vel, const LatLonOffset & lat_lon_offset,
  PolygonGenerationMethod & polygon_generation_method) const
{
  // Check if ego's path and object's path are close.
  const bool will_object_cut_in = [&]() {
    for (const auto & predicted_path_point : predicted_path.path) {
      const double paths_lat_diff =
        motion_utils::calcLateralOffset(ego_path, predicted_path_point.position);
      if (std::abs(paths_lat_diff) < planner_data_->parameters.vehicle_width / 2.0) {
        return true;
      }
    }
    return false;
  }();
  if (!will_object_cut_in) {
    // The object's path will not cut in
    return false;
  }

  // Ignore object longitudinally close to the ego
  const size_t ego_seg_idx = planner_data_->findEgoSegmentIndex(ego_path);
  const double relative_velocity = getEgoSpeed() - obj_tangent_vel;
  const double lon_offset_ego_to_obj =
    motion_utils::calcSignedArcLength(
      ego_path, getEgoPose().position, ego_seg_idx, lat_lon_offset.nearest_idx) +
    lat_lon_offset.min_lon_offset;
  if (
    lon_offset_ego_to_obj < std::max(
                              parameters_->min_lon_offset_ego_to_cut_in_object,
                              relative_velocity * parameters_->min_time_to_start_cut_in)) {
    polygon_generation_method = PolygonGenerationMethod::EGO_PATH_BASE;
    return false;
  }

  return true;
}

DynamicAvoidanceModule::DecisionWithReason DynamicAvoidanceModule::willObjectCutOut(
  const double obj_tangent_vel, const double obj_normal_vel, const bool is_object_left,
  const std::optional<DynamicAvoidanceObject> & prev_object) const
{
  // Ignore oncoming object
  if (obj_tangent_vel < 0) {
    return DecisionWithReason{false};
  }

  // Check if previous object is memorized
  if (!prev_object || !prev_object->latest_time_inside_ego_path) {
    return DecisionWithReason{false};
  }
  if (
    parameters_->max_time_from_outside_ego_path_for_cut_out <
    (clock_->now() - *prev_object->latest_time_inside_ego_path).seconds()) {
    return DecisionWithReason{false};
  }

  // Check object's lateral velocity
  std::stringstream reason;
  reason << "since latest time inside ego's path is small enough ("
         << (clock_->now() - *prev_object->latest_time_inside_ego_path).seconds() << "<"
         << parameters_->max_time_from_outside_ego_path_for_cut_out << ")";
  if (is_object_left) {
    if (parameters_->min_cut_out_object_lat_vel < obj_normal_vel) {
      reason << ", and lateral velocity is large enough ("
             << parameters_->min_cut_out_object_lat_vel << "<abs(" << obj_normal_vel << ")";
      return DecisionWithReason{true, reason.str()};
    }
  } else {
    if (obj_normal_vel < -parameters_->min_cut_out_object_lat_vel) {
      reason << ", and lateral velocity is large enough ("
             << parameters_->min_cut_out_object_lat_vel << "<abs(" << obj_normal_vel << ")";
      return DecisionWithReason{true, reason.str()};
    }
  }

  return DecisionWithReason{false};
}

std::pair<lanelet::ConstLanelets, lanelet::ConstLanelets> DynamicAvoidanceModule::getAdjacentLanes(
  const double forward_distance, const double backward_distance) const
{
  const auto & rh = planner_data_->route_handler;

  lanelet::ConstLanelet current_lane;
  if (!rh->getClosestLaneletWithinRoute(getEgoPose(), &current_lane)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("behavior_path_planner").get_child("dynamic_avoidance"),
      "failed to find closest lanelet within route!!!");
    return {};
  }

  const auto ego_succeeding_lanes =
    rh->getLaneletSequence(current_lane, getEgoPose(), backward_distance, forward_distance);

  lanelet::ConstLanelets right_lanes;
  lanelet::ConstLanelets left_lanes;
  for (const auto & lane : ego_succeeding_lanes) {
    // left lane
    const auto opt_left_lane = rh->getLeftLanelet(lane);
    if (opt_left_lane) {
      left_lanes.push_back(opt_left_lane.get());
    }

    // right lane
    const auto opt_right_lane = rh->getRightLanelet(lane);
    if (opt_right_lane) {
      right_lanes.push_back(opt_right_lane.get());
    }

    const auto right_opposite_lanes = rh->getRightOppositeLanelets(lane);
    if (!right_opposite_lanes.empty()) {
      right_lanes.push_back(right_opposite_lanes.front());
    }
  }

  return std::make_pair(right_lanes, left_lanes);
}

DynamicAvoidanceModule::LatLonOffset DynamicAvoidanceModule::getLateralLongitudinalOffset(
  const std::vector<PathPointWithLaneId> & ego_path, const geometry_msgs::msg::Pose & obj_pose,
  const autoware_auto_perception_msgs::msg::Shape & obj_shape) const
{
  const size_t obj_seg_idx = motion_utils::findNearestSegmentIndex(ego_path, obj_pose.position);
  const auto obj_points = tier4_autoware_utils::toPolygon2d(obj_pose, obj_shape);

  // TODO(murooka) calculation is not so accurate.
  std::vector<double> obj_lat_offset_vec;
  std::vector<double> obj_lon_offset_vec;
  for (size_t i = 0; i < obj_points.outer().size(); ++i) {
    const auto geom_obj_point = toGeometryPoint(obj_points.outer().at(i));
    const size_t obj_point_seg_idx =
      motion_utils::findNearestSegmentIndex(ego_path, geom_obj_point);

    // calculate lateral offset
    const double obj_point_lat_offset =
      motion_utils::calcLateralOffset(ego_path, geom_obj_point, obj_point_seg_idx);
    obj_lat_offset_vec.push_back(obj_point_lat_offset);

    // calculate longitudinal offset
    const double lon_offset =
      motion_utils::calcLongitudinalOffsetToSegment(ego_path, obj_seg_idx, geom_obj_point);
    obj_lon_offset_vec.push_back(lon_offset);
  }

  const auto obj_lat_min_max_offset = getMinMaxValues(obj_lat_offset_vec);
  const auto obj_lon_min_max_offset = getMinMaxValues(obj_lon_offset_vec);

  return LatLonOffset{
    obj_seg_idx, obj_lat_min_max_offset.max_value, obj_lat_min_max_offset.min_value,
    obj_lon_min_max_offset.max_value, obj_lon_min_max_offset.min_value};
}

MinMaxValue DynamicAvoidanceModule::calcMinMaxLongitudinalOffsetToAvoid(
  const std::vector<PathPointWithLaneId> & path_points_for_object_polygon,
  const geometry_msgs::msg::Pose & obj_pose, const Polygon2d & obj_points, const double obj_vel,
  const double time_to_collision) const
{
  const size_t obj_seg_idx =
    motion_utils::findNearestSegmentIndex(path_points_for_object_polygon, obj_pose.position);

  // calculate min/max longitudinal offset from object to path
  const auto obj_lon_offset = [&]() {
    std::vector<double> obj_lon_offset_vec;
    for (size_t i = 0; i < obj_points.outer().size(); ++i) {
      const auto geom_obj_point = toGeometryPoint(obj_points.outer().at(i));
      const double lon_offset = motion_utils::calcLongitudinalOffsetToSegment(
        path_points_for_object_polygon, obj_seg_idx, geom_obj_point);
      obj_lon_offset_vec.push_back(lon_offset);
    }

    const auto raw_obj_lon_offset = getMinMaxValues(obj_lon_offset_vec);

    if (obj_vel < 0) {
      return MinMaxValue{
        raw_obj_lon_offset.min_value + obj_vel * time_to_collision, raw_obj_lon_offset.max_value};
    }

    // NOTE: If time to collision is considered here, the ego is close to the object when starting
    // avoidance.
    //       The ego should start avoidance before overtaking.
    return raw_obj_lon_offset;
  }();

  // calculate bound start and end index
  const bool is_object_overtaking = (0.0 <= obj_vel);
  // TODO(murooka) use getEgoSpeed() instead of obj_vel
  const double start_length_to_avoid =
    std::abs(obj_vel) * (is_object_overtaking
                           ? parameters_->start_duration_to_avoid_overtaking_object
                           : parameters_->start_duration_to_avoid_oncoming_object);
  const double end_length_to_avoid =
    std::abs(obj_vel) * (is_object_overtaking ? parameters_->end_duration_to_avoid_overtaking_object
                                              : parameters_->end_duration_to_avoid_oncoming_object);

  return MinMaxValue{
    obj_lon_offset.min_value - start_length_to_avoid,
    obj_lon_offset.max_value + end_length_to_avoid};
}

MinMaxValue DynamicAvoidanceModule::calcMinMaxLateralOffsetToAvoid(
  const std::vector<PathPointWithLaneId> & path_points_for_object_polygon,
  const Polygon2d & obj_points, const bool is_collision_left, const double obj_normal_vel,
  const std::optional<DynamicAvoidanceObject> & prev_object) const
{
  // calculate min/max lateral offset from object to path
  const auto obj_lat_abs_offset = [&]() {
    std::vector<double> obj_lat_abs_offset_vec;
    for (size_t i = 0; i < obj_points.outer().size(); ++i) {
      const auto geom_obj_point = toGeometryPoint(obj_points.outer().at(i));
      const size_t obj_point_seg_idx =
        motion_utils::findNearestSegmentIndex(path_points_for_object_polygon, geom_obj_point);
      const double obj_point_lat_offset = motion_utils::calcLateralOffset(
        path_points_for_object_polygon, geom_obj_point, obj_point_seg_idx);
      obj_lat_abs_offset_vec.push_back(std::abs(obj_point_lat_offset));
    }
    return getMinMaxValues(obj_lat_abs_offset_vec);
  }();
  const double min_obj_lat_abs_offset = obj_lat_abs_offset.min_value;
  const double max_obj_lat_abs_offset = obj_lat_abs_offset.max_value;

  // calculate bound min and max lateral offset
  const double min_bound_lat_offset = [&]() {
    const double lat_abs_offset_to_shift =
      std::max(0.0, obj_normal_vel * (is_collision_left ? -1.0 : 1.0)) *
      parameters_->max_time_for_lat_shift;
    const double raw_min_bound_lat_offset =
      min_obj_lat_abs_offset - parameters_->lat_offset_from_obstacle - lat_abs_offset_to_shift;
    const double min_bound_lat_abs_offset_limit =
      planner_data_->parameters.vehicle_width / 2.0 - parameters_->max_lat_offset_to_avoid;
    return std::max(raw_min_bound_lat_offset, min_bound_lat_abs_offset_limit) *
           (is_collision_left ? 1.0 : -1.0);
  }();
  const double max_bound_lat_offset =
    (max_obj_lat_abs_offset + parameters_->lat_offset_from_obstacle) *
    (is_collision_left ? 1.0 : -1.0);

  // filter min_bound_lat_offset
  const auto prev_min_lat_avoid_to_offset = [&]() -> std::optional<double> {
    if (!prev_object || !prev_object->lat_offset_to_avoid) {
      return std::nullopt;
    }
    return prev_object->lat_offset_to_avoid->min_value;
  }();
  const double filtered_min_bound_lat_offset =
    prev_min_lat_avoid_to_offset ? signal_processing::lowpassFilter(
                                     min_bound_lat_offset, *prev_min_lat_avoid_to_offset,
                                     parameters_->lpf_gain_for_lat_avoid_to_offset)
                                 : min_bound_lat_offset;

  return MinMaxValue{filtered_min_bound_lat_offset, max_bound_lat_offset};
}

// NOTE: object does not have const only to update min_bound_lat_offset.
std::optional<tier4_autoware_utils::Polygon2d>
DynamicAvoidanceModule::calcEgoPathBasedDynamicObstaclePolygon(
  const DynamicAvoidanceObject & object) const
{
  if (!object.lon_offset_to_avoid || !object.lat_offset_to_avoid) {
    return std::nullopt;
  }

  auto path_points_for_object_polygon = getPreviousModuleOutput().reference_path->points;

  const size_t obj_seg_idx =
    motion_utils::findNearestSegmentIndex(path_points_for_object_polygon, object.pose.position);
  const auto obj_points = tier4_autoware_utils::toPolygon2d(object.pose, object.shape);

  const auto lon_bound_start_idx_opt = motion_utils::insertTargetPoint(
    obj_seg_idx, object.lon_offset_to_avoid->min_value, path_points_for_object_polygon);
  const size_t updated_obj_seg_idx =
    (lon_bound_start_idx_opt && lon_bound_start_idx_opt.value() <= obj_seg_idx) ? obj_seg_idx + 1
                                                                                : obj_seg_idx;
  const auto lon_bound_end_idx_opt = motion_utils::insertTargetPoint(
    updated_obj_seg_idx, object.lon_offset_to_avoid->max_value, path_points_for_object_polygon);

  if (!lon_bound_start_idx_opt && !lon_bound_end_idx_opt) {
    // NOTE: The obstacle is longitudinally out of the ego's trajectory.
    return std::nullopt;
  }
  const size_t lon_bound_start_idx =
    lon_bound_start_idx_opt ? lon_bound_start_idx_opt.value() : static_cast<size_t>(0);
  const size_t lon_bound_end_idx =
    lon_bound_end_idx_opt ? lon_bound_end_idx_opt.value()
                          : static_cast<size_t>(path_points_for_object_polygon.size() - 1);

  // create inner/outer bound points
  std::vector<geometry_msgs::msg::Point> obj_inner_bound_points;
  std::vector<geometry_msgs::msg::Point> obj_outer_bound_points;
  for (size_t i = lon_bound_start_idx; i <= lon_bound_end_idx; ++i) {
    obj_inner_bound_points.push_back(tier4_autoware_utils::calcOffsetPose(
                                       path_points_for_object_polygon.at(i).point.pose, 0.0,
                                       object.lat_offset_to_avoid->min_value, 0.0)
                                       .position);
    obj_outer_bound_points.push_back(tier4_autoware_utils::calcOffsetPose(
                                       path_points_for_object_polygon.at(i).point.pose, 0.0,
                                       object.lat_offset_to_avoid->max_value, 0.0)
                                       .position);
  }

  // create obj_polygon from inner/outer bound points
  tier4_autoware_utils::Polygon2d obj_poly;
  for (const auto & bound_point : obj_inner_bound_points) {
    const auto obj_poly_point = tier4_autoware_utils::Point2d(bound_point.x, bound_point.y);
    obj_poly.outer().push_back(obj_poly_point);
  }
  std::reverse(obj_outer_bound_points.begin(), obj_outer_bound_points.end());
  for (const auto & bound_point : obj_outer_bound_points) {
    const auto obj_poly_point = tier4_autoware_utils::Point2d(bound_point.x, bound_point.y);
    obj_poly.outer().push_back(obj_poly_point);
  }

  boost::geometry::correct(obj_poly);
  return obj_poly;
}

std::optional<tier4_autoware_utils::Polygon2d>
DynamicAvoidanceModule::calcObjectPathBasedDynamicObstaclePolygon(
  const DynamicAvoidanceObject & object) const
{
  const auto obj_path = *std::max_element(
    object.predicted_paths.begin(), object.predicted_paths.end(),
    [](const PredictedPath & a, const PredictedPath & b) { return a.confidence < b.confidence; });

  // calculate left and right bound
  std::vector<geometry_msgs::msg::Point> obj_left_bound_points;
  std::vector<geometry_msgs::msg::Point> obj_right_bound_points;
  const double obj_path_length = motion_utils::calcArcLength(obj_path.path);
  for (size_t i = 0; i < obj_path.path.size(); ++i) {
    const double lon_offset = [&]() {
      if (i == 0)
        return -object.shape.dimensions.x / 2.0 -
               std::max(
                 parameters_->min_obj_path_based_lon_polygon_margin,
                 parameters_->lat_offset_from_obstacle);
      if (i == obj_path.path.size() - 1)
        return object.shape.dimensions.x / 2.0 +
               std::max(
                 parameters_->min_obj_path_based_lon_polygon_margin - obj_path_length,
                 parameters_->lat_offset_from_obstacle);
      return 0.0;
    }();

    const auto & obj_pose = obj_path.path.at(i);
    obj_left_bound_points.push_back(
      tier4_autoware_utils::calcOffsetPose(
        obj_pose, lon_offset,
        object.shape.dimensions.y / 2.0 + parameters_->lat_offset_from_obstacle, 0.0)
        .position);
    obj_right_bound_points.push_back(
      tier4_autoware_utils::calcOffsetPose(
        obj_pose, lon_offset,
        -object.shape.dimensions.y / 2.0 - parameters_->lat_offset_from_obstacle, 0.0)
        .position);
  }

  // create obj_polygon from inner/outer bound points
  tier4_autoware_utils::Polygon2d obj_poly;
  for (const auto & bound_point : obj_right_bound_points) {
    const auto obj_poly_point = tier4_autoware_utils::Point2d(bound_point.x, bound_point.y);
    obj_poly.outer().push_back(obj_poly_point);
  }
  std::reverse(obj_left_bound_points.begin(), obj_left_bound_points.end());
  for (const auto & bound_point : obj_left_bound_points) {
    const auto obj_poly_point = tier4_autoware_utils::Point2d(bound_point.x, bound_point.y);
    obj_poly.outer().push_back(obj_poly_point);
  }

  boost::geometry::correct(obj_poly);
  return obj_poly;
}
}  // namespace behavior_path_planner
