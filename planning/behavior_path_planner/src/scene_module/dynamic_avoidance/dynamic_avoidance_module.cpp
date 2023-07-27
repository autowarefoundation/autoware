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

std::pair<double, double> getMinMaxValues(const std::vector<double> & vec)
{
  const size_t min_idx = std::distance(vec.begin(), std::min_element(vec.begin(), vec.end()));

  const size_t max_idx = std::distance(vec.begin(), std::max_element(vec.begin(), vec.end()));

  return std::make_pair(vec.at(min_idx), vec.at(max_idx));
}

void appendObjectMarker(MarkerArray & marker_array, const geometry_msgs::msg::Pose & obj_pose)
{
  auto marker = tier4_autoware_utils::createDefaultMarker(
    "map", rclcpp::Clock{RCL_ROS_TIME}.now(), "dynamic_objects_to_avoid",
    marker_array.markers.size(), visualization_msgs::msg::Marker::CUBE,
    tier4_autoware_utils::createMarkerScale(3.0, 1.0, 1.0),
    tier4_autoware_utils::createMarkerColor(0.7, 0.15, 0.9, 0.8));
  marker.pose = obj_pose;

  marker_array.markers.push_back(marker);
}

void appendExtractedPolygonMarker(
  MarkerArray & marker_array, const tier4_autoware_utils::Polygon2d & obj_poly)
{
  auto marker = tier4_autoware_utils::createDefaultMarker(
    "map", rclcpp::Clock{RCL_ROS_TIME}.now(), "extracted_polygons", marker_array.markers.size(),
    visualization_msgs::msg::Marker::LINE_STRIP,
    tier4_autoware_utils::createMarkerScale(0.1, 0.0, 0.0),
    tier4_autoware_utils::createMarkerColor(0.7, 0.15, 0.9, 0.8));

  // NOTE: obj_poly.outer() has already duplicated points to close the polygon.
  for (size_t i = 0; i < obj_poly.outer().size(); ++i) {
    const auto & bound_point = obj_poly.outer().at(i);

    geometry_msgs::msg::Point bound_geom_point;
    bound_geom_point.x = bound_point.x();
    bound_geom_point.y = bound_point.y();
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
  const double obj_vel = object.kinematics.initial_twist_with_covariance.twist.linear.x;

  const size_t obj_idx = motion_utils::findNearestIndex(path_points, obj_pose.position);

  const double obj_yaw = tf2::getYaw(obj_pose.orientation);
  const double path_yaw = tf2::getYaw(path_points.at(obj_idx).point.pose.orientation);

  return std::make_pair(
    obj_vel * std::cos(obj_yaw - path_yaw), obj_vel * std::sin(obj_yaw - path_yaw));
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
}  // namespace

DynamicAvoidanceModule::DynamicAvoidanceModule(
  const std::string & name, rclcpp::Node & node,
  std::shared_ptr<DynamicAvoidanceParameters> parameters,
  const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> & rtc_interface_ptr_map)
: SceneModuleInterface{name, node, rtc_interface_ptr_map}, parameters_{std::move(parameters)}
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
  // calculate target objects candidate
  const auto target_objects_candidate = calcTargetObjectsCandidate();

  // calculate target objects considering flickering suppress
  target_objects_.clear();
  for (const auto & target_object_candidate : target_objects_candidate) {
    if (
      parameters_->successive_num_to_entry_dynamic_avoidance_condition <=
      target_object_candidate.alive_counter) {
      target_objects_.push_back(target_object_candidate.object);
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
  const auto drivable_lanes = getPreviousModuleOutput().drivable_area_info.drivable_lanes;

  // create obstacles to avoid (= extract from the drivable area)
  std::vector<DrivableAreaInfo::Obstacle> obstacles_for_drivable_area;
  prev_objects_min_bound_lat_offset_.resetCurrentUuids();
  for (const auto & object : target_objects_) {
    const auto obstacle_poly = calcDynamicObstaclePolygon(object);
    if (obstacle_poly) {
      obstacles_for_drivable_area.push_back({object.pose, obstacle_poly.value(), object.is_left});

      appendObjectMarker(info_marker_, object.pose);
      appendExtractedPolygonMarker(debug_marker_, obstacle_poly.value());

      prev_objects_min_bound_lat_offset_.addCurrentUuid(object.uuid);
    }
  }
  prev_objects_min_bound_lat_offset_.removeCounterUnlessUpdated();

  BehaviorModuleOutput output;
  output.path = prev_module_path;
  output.reference_path = getPreviousModuleOutput().reference_path;
  output.drivable_area_info.drivable_lanes = drivable_lanes;
  output.drivable_area_info.obstacles = obstacles_for_drivable_area;
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

std::vector<DynamicAvoidanceModule::DynamicAvoidanceObjectCandidate>
DynamicAvoidanceModule::calcTargetObjectsCandidate()
{
  const auto prev_module_path = getPreviousModuleOutput().path;
  const auto & predicted_objects = planner_data_->dynamic_object->objects;

  // convert predicted objects to dynamic avoidance objects
  std::vector<DynamicAvoidanceObjectCandidate> output_objects_candidate;
  for (const auto & predicted_object : predicted_objects) {
    const auto obj_uuid = tier4_autoware_utils::toHexString(predicted_object.object_id);
    const auto & obj_pose = predicted_object.kinematics.initial_pose_with_covariance.pose;
    const double obj_vel = predicted_object.kinematics.initial_twist_with_covariance.twist.linear.x;
    const auto obj_path = *std::max_element(
      predicted_object.kinematics.predicted_paths.begin(),
      predicted_object.kinematics.predicted_paths.end(),
      [](const PredictedPath & a, const PredictedPath & b) { return a.confidence < b.confidence; });

    // 1. check label
    const bool is_label_target_obstacle =
      isLabelTargetObstacle(predicted_object.classification.front().label);
    if (!is_label_target_obstacle) {
      continue;
    }

    // 2. check if velocity is large enough
    const auto [obj_tangent_vel, obj_normal_vel] =
      projectObstacleVelocityToTrajectory(prev_module_path->points, predicted_object);
    if (std::abs(obj_tangent_vel) < parameters_->min_obstacle_vel) {
      continue;
    }

    // 3. check if object is not crossing ego's path
    const double obj_angle = calcDiffAngleAgainstPath(prev_module_path->points, obj_pose);
    const bool is_obstacle_crossing_path =
      parameters_->max_crossing_object_angle < std::abs(obj_angle) &&
      parameters_->max_crossing_object_angle < M_PI - std::abs(obj_angle);
    const bool is_crossing_object_to_ignore =
      parameters_->min_crossing_object_vel < std::abs(obj_vel) && is_obstacle_crossing_path;
    if (is_crossing_object_to_ignore) {
      RCLCPP_INFO_EXPRESSION(
        getLogger(), parameters_->enable_debug_info,
        "[DynamicAvoidance] Ignore obstacle (%s) since it crosses the ego's path.",
        obj_uuid.c_str());
      continue;
    }

    // 4. check if object is not to be followed by ego
    const double obj_dist_to_path = calcDistanceToPath(prev_module_path->points, obj_pose.position);
    const bool is_object_on_ego_path =
      obj_dist_to_path <
      planner_data_->parameters.vehicle_width / 2.0 + parameters_->min_obj_lat_offset_to_ego_path;
    const bool is_object_aligned_to_path =
      std::abs(obj_angle) < parameters_->max_front_object_angle ||
      M_PI - parameters_->max_front_object_angle < std::abs(obj_angle);
    if (is_object_on_ego_path && is_object_aligned_to_path) {
      RCLCPP_INFO_EXPRESSION(
        getLogger(), parameters_->enable_debug_info,
        "[DynamicAvoidance] Ignore obstacle (%s) since it is to be followed.", obj_uuid.c_str());
      continue;
    }

    // 5. check if object lateral offset to ego's path is large enough
    const bool is_object_far_from_path = isObjectFarFromPath(predicted_object, obj_dist_to_path);
    if (is_object_far_from_path) {
      RCLCPP_INFO_EXPRESSION(
        getLogger(), parameters_->enable_debug_info,
        "[DynamicAvoidance] Ignore obstacle (%s) since lateral offset is large.", obj_uuid.c_str());
      continue;
    }

    // 6. calculate which side object exists against ego's path
    // TODO(murooka) use obj_path.back() instead of obj_pose for the case where the object crosses
    // the ego's path
    const bool is_left = isLeft(prev_module_path->points, obj_path.path.back().position);

    // 6. check if object will not cut in or cut out
    const bool will_object_cut_in =
      willObjectCutIn(prev_module_path->points, obj_path, obj_tangent_vel);
    const bool will_object_cut_out = willObjectCutOut(obj_tangent_vel, obj_normal_vel, is_left);
    if (will_object_cut_in) {
      RCLCPP_INFO_EXPRESSION(
        getLogger(), parameters_->enable_debug_info,
        "[DynamicAvoidance] Ignore obstacle (%s) since it will cut in.", obj_uuid.c_str());
      continue;
    }
    if (will_object_cut_out) {
      RCLCPP_INFO_EXPRESSION(
        getLogger(), parameters_->enable_debug_info,
        "[DynamicAvoidance] Ignore obstacle (%s) since it will cut out.", obj_uuid.c_str());
      continue;
    }

    // 7. check if time to collision
    const double time_to_collision =
      calcTimeToCollision(prev_module_path->points, obj_pose, obj_tangent_vel);
    if (
      (0 <= obj_tangent_vel &&
       parameters_->max_time_to_collision_overtaking_object < time_to_collision) ||
      (obj_tangent_vel <= 0 &&
       parameters_->max_time_to_collision_oncoming_object < time_to_collision)) {
      RCLCPP_INFO_EXPRESSION(
        getLogger(), parameters_->enable_debug_info,
        "[DynamicAvoidance] Ignore obstacle (%s) since time to collision is large.",
        obj_uuid.c_str());
      continue;
    }

    // 8. calculate alive counter for filtering objects
    const auto prev_target_object_candidate =
      DynamicAvoidanceObjectCandidate::getObjectFromUuid(prev_target_objects_candidate_, obj_uuid);
    const int alive_counter =
      prev_target_object_candidate
        ? std::min(
            parameters_->successive_num_to_entry_dynamic_avoidance_condition,
            prev_target_object_candidate->alive_counter + 1)
        : 0;

    const auto target_object = DynamicAvoidanceObject(
      predicted_object, obj_tangent_vel, obj_normal_vel, is_left, time_to_collision);
    const auto target_object_candidate =
      DynamicAvoidanceObjectCandidate{target_object, alive_counter};
    output_objects_candidate.push_back(target_object_candidate);
  }

  prev_target_objects_candidate_ = output_objects_candidate;
  return output_objects_candidate;
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
  const double obj_tangent_vel) const
{
  const double relative_velocity = getEgoSpeed() - obj_tangent_vel;
  const size_t ego_seg_idx = planner_data_->findEgoSegmentIndex(ego_path);
  const size_t obj_seg_idx = motion_utils::findNearestSegmentIndex(ego_path, obj_pose.position);
  const double signed_lon_length = motion_utils::calcSignedArcLength(
    ego_path, getEgoPosition(), ego_seg_idx, obj_pose.position, obj_seg_idx);
  const double positive_relative_velocity = std::max(relative_velocity, 1.0);
  return signed_lon_length / positive_relative_velocity;
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
  const double obj_tangent_vel) const
{
  constexpr double epsilon_path_lat_diff = 0.3;

  // Ignore oncoming object
  if (obj_tangent_vel < 0) {
    return false;
  }

  for (const auto & predicted_path_point : predicted_path.path) {
    const double paths_lat_diff =
      motion_utils::calcLateralOffset(ego_path, predicted_path_point.position);
    if (std::abs(paths_lat_diff) < epsilon_path_lat_diff) {
      return true;
    }
  }
  return false;
}

bool DynamicAvoidanceModule::willObjectCutOut(
  const double obj_tangent_vel, const double obj_normal_vel, const bool is_left) const
{
  // Ignore oncoming object
  if (obj_tangent_vel < 0) {
    return false;
  }

  constexpr double object_lat_vel_thresh = 0.3;
  if (is_left) {
    if (object_lat_vel_thresh < obj_normal_vel) {
      return true;
    }
  } else {
    if (obj_normal_vel < -object_lat_vel_thresh) {
      return true;
    }
  }
  return false;
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

// NOTE: object does not have const only to update min_bound_lat_offset.
std::optional<tier4_autoware_utils::Polygon2d> DynamicAvoidanceModule::calcDynamicObstaclePolygon(
  const DynamicAvoidanceObject & object) const
{
  const auto ego_pose = getEgoPose();
  const auto & rh = planner_data_->route_handler;

  // get path with backward margin
  lanelet::ConstLanelet current_lane;
  if (!rh->getClosestLaneletWithinRoute(ego_pose, &current_lane)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("behavior_path_planner").get_child("dynamic_avoidance"),
      "failed to find closest lanelet within route!!!");
    return std::nullopt;
  }

  auto path_with_backward_margin = [&]() {
    constexpr double forward_length = 100.0;
    const double backward_length = 50.0;
    const auto current_lanes =
      rh->getLaneletSequence(current_lane, ego_pose, backward_length, forward_length);
    return utils::getCenterLinePath(
      *rh, current_lanes, ego_pose, backward_length, forward_length, planner_data_->parameters);
  }();

  const size_t obj_seg_idx =
    motion_utils::findNearestSegmentIndex(path_with_backward_margin.points, object.pose.position);
  const auto obj_points = tier4_autoware_utils::toPolygon2d(object.pose, object.shape);

  // calculate min/max lateral offset from object to path
  const auto [min_obj_lat_abs_offset, max_obj_lat_abs_offset] = [&]() {
    std::vector<double> obj_lat_abs_offset_vec;
    for (size_t i = 0; i < obj_points.outer().size(); ++i) {
      const auto geom_obj_point = toGeometryPoint(obj_points.outer().at(i));
      const size_t obj_point_seg_idx =
        motion_utils::findNearestSegmentIndex(path_with_backward_margin.points, geom_obj_point);
      const double obj_point_lat_offset = motion_utils::calcLateralOffset(
        path_with_backward_margin.points, geom_obj_point, obj_point_seg_idx);
      obj_lat_abs_offset_vec.push_back(std::abs(obj_point_lat_offset));
    }
    return getMinMaxValues(obj_lat_abs_offset_vec);
  }();
  const double min_obj_lat_offset = min_obj_lat_abs_offset * (object.is_left ? 1.0 : -1.0);
  const double max_obj_lat_offset = max_obj_lat_abs_offset * (object.is_left ? 1.0 : -1.0);

  // calculate min/max longitudinal offset from object to path
  const auto obj_lon_offset = [&]() -> std::optional<std::pair<double, double>> {
    std::vector<double> obj_lon_offset_vec;
    for (size_t i = 0; i < obj_points.outer().size(); ++i) {
      const auto geom_obj_point = toGeometryPoint(obj_points.outer().at(i));
      const double lon_offset = motion_utils::calcLongitudinalOffsetToSegment(
        path_with_backward_margin.points, obj_seg_idx, geom_obj_point);
      obj_lon_offset_vec.push_back(lon_offset);
    }

    const auto [raw_min_obj_lon_offset, raw_max_obj_lon_offset] =
      getMinMaxValues(obj_lon_offset_vec);
    if (object.time_to_collision < -parameters_->duration_to_hold_avoidance_overtaking_object) {
      return std::nullopt;
    }

    return std::make_pair(
      raw_min_obj_lon_offset + object.vel * object.time_to_collision,
      raw_max_obj_lon_offset + object.vel * object.time_to_collision);
  }();

  if (!obj_lon_offset) {
    return std::nullopt;
  }
  const double min_obj_lon_offset = obj_lon_offset->first;
  const double max_obj_lon_offset = obj_lon_offset->second;

  // calculate bound start and end index
  const bool is_object_overtaking = (0.0 <= object.vel);
  // TODO(murooka) use getEgoSpeed() instead of object.vel
  const double start_length_to_avoid =
    std::abs(object.vel) * (is_object_overtaking
                              ? parameters_->start_duration_to_avoid_overtaking_object
                              : parameters_->start_duration_to_avoid_oncoming_object);
  const double end_length_to_avoid =
    std::abs(object.vel) * (is_object_overtaking
                              ? parameters_->end_duration_to_avoid_overtaking_object
                              : parameters_->end_duration_to_avoid_oncoming_object);
  const auto lon_bound_start_idx_opt = motion_utils::insertTargetPoint(
    obj_seg_idx, min_obj_lon_offset - start_length_to_avoid, path_with_backward_margin.points);
  const size_t updated_obj_seg_idx =
    (lon_bound_start_idx_opt && lon_bound_start_idx_opt.value() <= obj_seg_idx) ? obj_seg_idx + 1
                                                                                : obj_seg_idx;
  const auto lon_bound_end_idx_opt = motion_utils::insertTargetPoint(
    updated_obj_seg_idx, max_obj_lon_offset + end_length_to_avoid,
    path_with_backward_margin.points);

  if (!lon_bound_start_idx_opt && !lon_bound_end_idx_opt) {
    // NOTE: The obstacle is longitudinally out of the ego's trajectory.
    return std::nullopt;
  }
  const size_t lon_bound_start_idx =
    lon_bound_start_idx_opt ? lon_bound_start_idx_opt.value() : static_cast<size_t>(0);
  const size_t lon_bound_end_idx =
    lon_bound_end_idx_opt ? lon_bound_end_idx_opt.value()
                          : static_cast<size_t>(path_with_backward_margin.points.size() - 1);

  // calculate bound min and max lateral offset
  const double min_bound_lat_offset = [&]() {
    const double raw_min_bound_lat_offset =
      min_obj_lat_offset - parameters_->lat_offset_from_obstacle * (object.is_left ? 1.0 : -1.0);
    const double min_bound_lat_abs_offset_limit =
      planner_data_->parameters.vehicle_width / 2.0 - parameters_->max_lat_offset_to_avoid;
    if (object.is_left) {
      if (raw_min_bound_lat_offset < min_bound_lat_abs_offset_limit) {
        return min_bound_lat_abs_offset_limit;
      }
    } else {
      if (-min_bound_lat_abs_offset_limit < raw_min_bound_lat_offset) {
        return -min_bound_lat_abs_offset_limit;
      }
    }
    return raw_min_bound_lat_offset;
  }();
  const double max_bound_lat_offset =
    max_obj_lat_offset + parameters_->lat_offset_from_obstacle * (object.is_left ? 1.0 : -1.0);

  // filter min_bound_lat_offset
  const auto prev_min_bound_lat_offset = prev_objects_min_bound_lat_offset_.get(object.uuid);
  const double filtered_min_bound_lat_offset =
    prev_min_bound_lat_offset
      ? signal_processing::lowpassFilter(min_bound_lat_offset, *prev_min_bound_lat_offset, 0.3)
      : min_bound_lat_offset;
  prev_objects_min_bound_lat_offset_.update(object.uuid, filtered_min_bound_lat_offset);

  // create inner/outer bound points
  std::vector<geometry_msgs::msg::Point> obj_inner_bound_points;
  std::vector<geometry_msgs::msg::Point> obj_outer_bound_points;
  for (size_t i = lon_bound_start_idx; i <= lon_bound_end_idx; ++i) {
    obj_inner_bound_points.push_back(
      tier4_autoware_utils::calcOffsetPose(
        path_with_backward_margin.points.at(i).point.pose, 0.0, filtered_min_bound_lat_offset, 0.0)
        .position);
    obj_outer_bound_points.push_back(
      tier4_autoware_utils::calcOffsetPose(
        path_with_backward_margin.points.at(i).point.pose, 0.0, max_bound_lat_offset, 0.0)
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
}  // namespace behavior_path_planner
