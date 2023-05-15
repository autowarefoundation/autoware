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
  const PredictedObject & object, const lanelet::ConstLanelets & target_lanelets)
{
  if (target_lanelets.empty()) {
    return false;
  }

  const auto & object_pos = object.kinematics.initial_pose_with_covariance.pose.position;
  lanelet::BasicPoint2d object_centroid(object_pos.x, object_pos.y);

  for (const auto & llt : target_lanelets) {
    if (boost::geometry::within(object_centroid, llt.polygon2d().basicPolygon())) {
      return true;
    }
  }

  return false;
}

std::vector<PredictedObject> getObjectsInLanes(
  const std::vector<PredictedObject> & objects, const lanelet::ConstLanelets & target_lanes)
{
  std::vector<PredictedObject> target_objects;
  for (const auto & object : objects) {
    if (isCentroidWithinLanelets(object, target_lanes)) {
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
    tier4_autoware_utils::createMarkerColor(1.0, 0.5, 0.6, 0.8));
  marker.pose = obj_pose;

  marker_array.markers.push_back(marker);
}

void appendExtractedPolygonMarker(
  MarkerArray & marker_array, const tier4_autoware_utils::Polygon2d & obj_poly)
{
  auto marker = tier4_autoware_utils::createDefaultMarker(
    "map", rclcpp::Clock{RCL_ROS_TIME}.now(), "extracted_polygons", marker_array.markers.size(),
    visualization_msgs::msg::Marker::LINE_STRIP,
    tier4_autoware_utils::createMarkerScale(0.05, 0.0, 0.0),
    tier4_autoware_utils::createMarkerColor(1.0, 0.5, 0.6, 0.8));

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
  const std::unordered_map<std::string, std::shared_ptr<RTCInterface> > & rtc_interface_ptr_map)
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
    current_state_ = ModuleStatus::SUCCESS;
  } else {
    current_state_ = ModuleStatus::RUNNING;
  }

  return current_state_;
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
  for (const auto & object : target_objects_) {
    const auto obstacle_poly = calcDynamicObstaclePolygon(*prev_module_path, object);
    if (obstacle_poly) {
      obstacles_for_drivable_area.push_back({object.pose, obstacle_poly.value()});

      appendObjectMarker(info_marker_, object.pose);
      appendExtractedPolygonMarker(debug_marker_, obstacle_poly.value());
    }
  }

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

std::vector<DynamicAvoidanceModule::DynamicAvoidanceObject>
DynamicAvoidanceModule::calcTargetObjects() const
{
  const auto prev_module_path = getPreviousModuleOutput().path;

  // 1. calculate target lanes to filter obstacles
  const auto target_lanes = getAdjacentLanes(100.0, 50.0);

  // 2. filter obstacles for dynamic avoidance
  const auto & predicted_objects = planner_data_->dynamic_object->objects;
  const auto predicted_objects_in_target_lanes = getObjectsInLanes(predicted_objects, target_lanes);

  // check if object will cut into the ego lane.
  std::vector<PredictedObject> target_predicted_objects;
  constexpr double epsilon_path_lat_diff = 0.3;
  for (const auto & object : predicted_objects_in_target_lanes) {
    const auto reliable_predicted_path = std::max_element(
      object.kinematics.predicted_paths.begin(), object.kinematics.predicted_paths.end(),
      [](const PredictedPath & a, const PredictedPath & b) { return a.confidence < b.confidence; });

    // Ignore object since it will cut into the ego lane
    const bool will_object_cut_in = [&]() {
      for (const auto & predicted_path_point : reliable_predicted_path->path) {
        const double paths_lat_diff =
          motion_utils::calcLateralOffset(prev_module_path->points, predicted_path_point.position);
        if (std::abs(paths_lat_diff) < epsilon_path_lat_diff) {
          return true;
        }
      }
      return false;
    }();
    if (will_object_cut_in) {
      continue;
    }

    target_predicted_objects.push_back(object);
  }

  // 3. convert predicted objects to dynamic avoidance objects
  std::vector<DynamicAvoidanceObject> target_avoidance_objects;
  for (const auto & predicted_object : target_predicted_objects) {
    const double path_projected_vel =
      calcObstacleProjectedVelocity(prev_module_path->points, predicted_object);
    // check if velocity is high enough
    if (std::abs(path_projected_vel) < parameters_->min_obstacle_vel) {
      continue;
    }

    target_avoidance_objects.push_back(
      DynamicAvoidanceObject(predicted_object, path_projected_vel));
  }

  return target_avoidance_objects;
}

lanelet::ConstLanelets DynamicAvoidanceModule::getAdjacentLanes(
  const double forward_distance, const double backward_distance) const
{
  const auto & rh = planner_data_->route_handler;

  lanelet::ConstLanelet current_lane;
  if (!rh->getClosestLaneletWithinRoute(getEgoPose(), &current_lane)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("behavior_path_planner").get_child("avoidance"),
      "failed to find closest lanelet within route!!!");
    return {};
  }

  const auto ego_succeeding_lanes =
    rh->getLaneletSequence(current_lane, getEgoPose(), backward_distance, forward_distance);

  lanelet::ConstLanelets target_lanes;
  for (const auto & lane : ego_succeeding_lanes) {
    const auto opt_left_lane = rh->getLeftLanelet(lane);
    if (opt_left_lane) {
      target_lanes.push_back(opt_left_lane.get());
    }

    const auto opt_right_lane = rh->getRightLanelet(lane);
    if (opt_right_lane) {
      target_lanes.push_back(opt_right_lane.get());
    }

    const auto right_opposite_lanes = rh->getRightOppositeLanelets(lane);
    if (!right_opposite_lanes.empty()) {
      target_lanes.push_back(right_opposite_lanes.front());
    }
  }

  return target_lanes;
}

std::optional<tier4_autoware_utils::Polygon2d> DynamicAvoidanceModule::calcDynamicObstaclePolygon(
  const PathWithLaneId & path, const DynamicAvoidanceObject & object) const
{
  auto path_for_bound = path;

  const size_t obj_seg_idx =
    motion_utils::findNearestSegmentIndex(path_for_bound.points, object.pose.position);
  const double obj_lat_offset =
    motion_utils::calcLateralOffset(path_for_bound.points, object.pose.position, obj_seg_idx);
  const bool is_left = 0.0 < obj_lat_offset;
  const auto obj_points = tier4_autoware_utils::toPolygon2d(object.pose, object.shape);

  // calculate min/max lateral offset from object to path
  const auto [min_obj_lat_abs_offset, max_obj_lat_abs_offset] = [&]() {
    std::vector<double> obj_lat_abs_offset_vec;
    for (size_t i = 0; i < obj_points.outer().size(); ++i) {
      const auto geom_obj_point = toGeometryPoint(obj_points.outer().at(i));
      const size_t obj_point_seg_idx =
        motion_utils::findNearestSegmentIndex(path_for_bound.points, geom_obj_point);
      const double obj_point_lat_offset =
        motion_utils::calcLateralOffset(path_for_bound.points, geom_obj_point, obj_point_seg_idx);
      obj_lat_abs_offset_vec.push_back(std::abs(obj_point_lat_offset));
    }
    return getMinMaxValues(obj_lat_abs_offset_vec);
  }();
  const double min_obj_lat_offset = min_obj_lat_abs_offset * (is_left ? 1.0 : -1.0);
  const double max_obj_lat_offset = max_obj_lat_abs_offset * (is_left ? 1.0 : -1.0);

  // calculate min/max longitudinal offset from object to path
  const auto [min_obj_lon_offset, max_obj_lon_offset] = [&]() {
    std::vector<double> obj_lon_offset_vec;
    for (size_t i = 0; i < obj_points.outer().size(); ++i) {
      const auto geom_obj_point = toGeometryPoint(obj_points.outer().at(i));
      const double lon_offset = motion_utils::calcLongitudinalOffsetToSegment(
        path_for_bound.points, obj_seg_idx, geom_obj_point);
      obj_lon_offset_vec.push_back(lon_offset);
    }
    return getMinMaxValues(obj_lon_offset_vec);
  }();

  // calculate bound start and end index
  const double length_to_avoid = [&]() {
    if (0.0 <= object.path_projected_vel) {
      return object.path_projected_vel * parameters_->time_to_avoid_same_directional_object;
    }
    return object.path_projected_vel * parameters_->time_to_avoid_opposite_directional_object;
  }();
  const auto lon_bound_start_idx_opt = motion_utils::insertTargetPoint(
    obj_seg_idx, min_obj_lon_offset + (length_to_avoid < 0 ? length_to_avoid : 0.0),
    path_for_bound.points);
  const auto lon_bound_end_idx_opt = motion_utils::insertTargetPoint(
    obj_seg_idx, max_obj_lon_offset + (0 <= length_to_avoid ? length_to_avoid : 0.0),
    path_for_bound.points);
  if (!lon_bound_start_idx_opt && !lon_bound_end_idx_opt) {
    // NOTE: The obstacle is longitudinally out of the ego's trajectory.
    return std::nullopt;
  }
  const size_t lon_bound_start_idx =
    lon_bound_start_idx_opt ? lon_bound_start_idx_opt.value() : static_cast<size_t>(0);
  const size_t lon_bound_end_idx = lon_bound_end_idx_opt
                                     ? lon_bound_end_idx_opt.value()
                                     : static_cast<size_t>(path_for_bound.points.size() - 1);

  // calculate bound min and max lateral offset
  const double min_bound_lat_offset = [&]() {
    const double raw_min_bound_lat_offset =
      min_obj_lat_offset - parameters_->lat_offset_from_obstacle * (is_left ? 1.0 : -1.0);
    const double min_bound_lat_abs_offset_limit =
      planner_data_->parameters.vehicle_width / 2.0 - parameters_->max_lat_offset_to_avoid;
    if (is_left) {
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
    max_obj_lat_offset + parameters_->lat_offset_from_obstacle * (is_left ? 1.0 : -1.0);

  // create inner/outer bound points
  std::vector<geometry_msgs::msg::Point> obj_inner_bound_points;
  std::vector<geometry_msgs::msg::Point> obj_outer_bound_points;
  for (size_t i = lon_bound_start_idx; i <= lon_bound_end_idx; ++i) {
    obj_inner_bound_points.push_back(
      tier4_autoware_utils::calcOffsetPose(
        path_for_bound.points.at(i).point.pose, 0.0, min_bound_lat_offset, 0.0)
        .position);
    obj_outer_bound_points.push_back(
      tier4_autoware_utils::calcOffsetPose(
        path_for_bound.points.at(i).point.pose, 0.0, max_bound_lat_offset, 0.0)
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
