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
bool isCentroidWithinLanelets(
  const geometry_msgs::msg::Point & obj_pos, const lanelet::ConstLanelets & target_lanelets)
{
  if (target_lanelets.empty()) {
    return false;
  }

  lanelet::BasicPoint2d object_centroid(obj_pos.x, obj_pos.y);

  for (const auto & llt : target_lanelets) {
    if (boost::geometry::within(object_centroid, llt.polygon2d().basicPolygon())) {
      return true;
    }
  }

  return false;
}

std::vector<DynamicAvoidanceModule::DynamicAvoidanceObject> getObjectsInLanes(
  const std::vector<DynamicAvoidanceModule::DynamicAvoidanceObject> & objects,
  const lanelet::ConstLanelets & target_lanes)
{
  std::vector<DynamicAvoidanceModule::DynamicAvoidanceObject> target_objects;
  for (const auto & object : objects) {
    if (isCentroidWithinLanelets(object.pose.position, target_lanes)) {
      target_objects.push_back(object);
    }
  }

  return target_objects;
}

geometry_msgs::msg::Point toGeometryPoint(const tier4_autoware_utils::Point2d & point)
{
  geometry_msgs::msg::Point geom_obj_point;
  geom_obj_point.x = point.x();
  geom_obj_point.y = point.y();
  return geom_obj_point;
}

double calcObstacleProjectedVelocity(
  const std::vector<PathPointWithLaneId> & path_points, const PredictedObject & object)
{
  const auto & obj_pose = object.kinematics.initial_pose_with_covariance.pose;
  const double obj_vel = object.kinematics.initial_twist_with_covariance.twist.linear.x;

  const size_t obj_idx = motion_utils::findNearestIndex(path_points, obj_pose.position);

  const double obj_yaw = tf2::getYaw(obj_pose.orientation);
  const double path_yaw = tf2::getYaw(path_points.at(obj_idx).point.pose.orientation);

  return obj_vel * std::cos(obj_yaw - path_yaw);
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
}  // namespace

#ifdef USE_OLD_ARCHITECTURE
DynamicAvoidanceModule::DynamicAvoidanceModule(
  const std::string & name, rclcpp::Node & node,
  std::shared_ptr<DynamicAvoidanceParameters> parameters)
: SceneModuleInterface{name, node, createRTCInterfaceMap(node, name, {""})},
  parameters_{std::move(parameters)}
#else
DynamicAvoidanceModule::DynamicAvoidanceModule(
  const std::string & name, rclcpp::Node & node,
  std::shared_ptr<DynamicAvoidanceParameters> parameters,
  const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> & rtc_interface_ptr_map)
: SceneModuleInterface{name, node, rtc_interface_ptr_map}, parameters_{std::move(parameters)}
#endif
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
  target_objects_ = calcTargetObjects();
}

ModuleStatus DynamicAvoidanceModule::updateState()
{
  const bool has_avoidance_target = !target_objects_.empty();

  if (!has_avoidance_target) {
    return ModuleStatus::SUCCESS;
  }

#ifndef USE_OLD_ARCHITECTURE
  if (!isActivated() || isWaitingApproval()) {
    return ModuleStatus::IDLE;
  }
#endif

  return ModuleStatus::RUNNING;
}

BehaviorModuleOutput DynamicAvoidanceModule::plan()
{
  info_marker_.markers.clear();
  debug_marker_.markers.clear();

  // 1. get reference path from previous module
  const auto prev_module_path = getPreviousModuleOutput().path;

  // 2. get drivable lanes from previous module
  const auto drivable_lanes = getPreviousModuleOutput().drivable_area_info.drivable_lanes;

  // 3. create obstacles to avoid (= extract from the drivable area)
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
  // for new architecture
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

std::vector<DynamicAvoidanceModule::DynamicAvoidanceObject>
DynamicAvoidanceModule::calcTargetObjects() const
{
  const auto prev_module_path = getPreviousModuleOutput().path;
  const auto & predicted_objects = planner_data_->dynamic_object->objects;

  // 1. convert predicted objects to dynamic avoidance objects
  std::vector<DynamicAvoidanceObject> input_objects;
  for (const auto & predicted_object : predicted_objects) {
    // check label
    const bool is_label_target_obstacle =
      isLabelTargetObstacle(predicted_object.classification.front().label);
    if (!is_label_target_obstacle) {
      continue;
    }

    const double path_projected_vel =
      calcObstacleProjectedVelocity(prev_module_path->points, predicted_object);
    // check if velocity is high enough
    if (std::abs(path_projected_vel) < parameters_->min_obstacle_vel) {
      continue;
    }

    input_objects.push_back(DynamicAvoidanceObject(predicted_object, path_projected_vel));
  }

  // 2. calculate target lanes to filter obstacles
  const auto [right_lanes, left_lanes] = getAdjacentLanes(100.0, 50.0);

  // 3. filter obstacles for dynamic avoidance
  const auto objects_in_right_lanes = getObjectsInLanes(input_objects, right_lanes);
  const auto objects_in_left_lanes = getObjectsInLanes(input_objects, left_lanes);

  // 4. check if object will cut into the ego lane.
  // NOTE: The oncoming object will be ignored.
  constexpr double epsilon_path_lat_diff = 0.3;
  std::vector<DynamicAvoidanceObject> output_objects;
  for (const bool is_left : {true, false}) {
    for (const auto & object : (is_left ? objects_in_left_lanes : objects_in_right_lanes)) {
      const auto reliable_predicted_path = std::max_element(
        object.predicted_paths.begin(), object.predicted_paths.end(),
        [](const PredictedPath & a, const PredictedPath & b) {
          return a.confidence < b.confidence;
        });

      // Ignore object that will cut into the ego lane
      const bool will_object_cut_in = [&]() {
        if (object.path_projected_vel < 0) {
          // Ignore oncoming object
          return false;
        }

        for (const auto & predicted_path_point : reliable_predicted_path->path) {
          const double paths_lat_diff = motion_utils::calcLateralOffset(
            prev_module_path->points, predicted_path_point.position);
          if (std::abs(paths_lat_diff) < epsilon_path_lat_diff) {
            return true;
          }
        }
        return false;
      }();
      if (will_object_cut_in) {
        continue;
      }

      auto target_object = object;
      target_object.is_left = is_left;
      output_objects.push_back(target_object);
    }
  }

  return output_objects;
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

    // calculate time to collision and apply it to drivable area extraction
    const double relative_velocity = getEgoSpeed() - object.path_projected_vel;
    const double time_to_collision = [&]() {
      const auto prev_module_path = getPreviousModuleOutput().path;
      const size_t ego_seg_idx = planner_data_->findEgoSegmentIndex(prev_module_path->points);
      const size_t obj_seg_idx =
        motion_utils::findNearestSegmentIndex(prev_module_path->points, object.pose.position);
      const double signed_lon_length = motion_utils::calcSignedArcLength(
        prev_module_path->points, getEgoPosition(), ego_seg_idx, object.pose.position, obj_seg_idx);
      const double positive_relative_velocity = std::max(relative_velocity, 1.0);
      return signed_lon_length / positive_relative_velocity;
    }();

    if (time_to_collision < -parameters_->duration_to_hold_avoidance_overtaking_object) {
      return std::nullopt;
    }

    if (0 <= object.path_projected_vel) {
      const double limited_time_to_collision =
        std::min(parameters_->max_time_to_collision_overtaking_object, time_to_collision);
      return std::make_pair(
        raw_min_obj_lon_offset + object.path_projected_vel * limited_time_to_collision,
        raw_max_obj_lon_offset + object.path_projected_vel * limited_time_to_collision);
    }

    const double limited_time_to_collision =
      std::min(parameters_->max_time_to_collision_oncoming_object, time_to_collision);
    return std::make_pair(
      raw_min_obj_lon_offset + object.path_projected_vel * limited_time_to_collision,
      raw_max_obj_lon_offset);
  }();

  if (!obj_lon_offset) {
    return std::nullopt;
  }
  const double min_obj_lon_offset = obj_lon_offset->first;
  const double max_obj_lon_offset = obj_lon_offset->second;

  // calculate bound start and end index
  const bool is_object_overtaking = (0.0 <= object.path_projected_vel);
  const double start_length_to_avoid =
    std::abs(object.path_projected_vel) *
    (is_object_overtaking ? parameters_->start_duration_to_avoid_overtaking_object
                          : parameters_->start_duration_to_avoid_oncoming_object);
  const double end_length_to_avoid =
    std::abs(object.path_projected_vel) * (is_object_overtaking
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
