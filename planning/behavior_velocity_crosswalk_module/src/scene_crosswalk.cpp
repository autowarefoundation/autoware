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

#include "scene_crosswalk.hpp"

#include <autoware_auto_tf2/tf2_autoware_auto_msgs.hpp>
#include <behavior_velocity_planner_common/utilization/path_utilization.hpp>
#include <behavior_velocity_planner_common/utilization/util.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <vector>

namespace behavior_velocity_planner
{
namespace bg = boost::geometry;
using motion_utils::calcArcLength;
using motion_utils::calcLateralOffset;
using motion_utils::calcLongitudinalOffsetPoint;
using motion_utils::calcLongitudinalOffsetPose;
using motion_utils::calcSignedArcLength;
using motion_utils::findNearestSegmentIndex;
using motion_utils::insertTargetPoint;
using motion_utils::resamplePath;
using tier4_autoware_utils::createPoint;
using tier4_autoware_utils::getPoint;
using tier4_autoware_utils::getPose;
using tier4_autoware_utils::Point2d;
using tier4_autoware_utils::Polygon2d;
using tier4_autoware_utils::pose2transform;
using tier4_autoware_utils::toHexString;

namespace
{
geometry_msgs::msg::Point32 createPoint32(const double x, const double y, const double z)
{
  geometry_msgs::msg::Point32 p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

std::vector<geometry_msgs::msg::Point> toGeometryPointVector(
  const Polygon2d & polygon, const double z)
{
  std::vector<geometry_msgs::msg::Point> points;
  for (const auto & p : polygon.outer()) {
    points.push_back(createPoint(p.x(), p.y(), z));
  }
  return points;
}

Polygon2d createOneStepPolygon(
  const geometry_msgs::msg::Pose & p_front, const geometry_msgs::msg::Pose & p_back,
  const geometry_msgs::msg::Polygon & base_polygon)
{
  Polygon2d one_step_polygon{};

  {
    geometry_msgs::msg::Polygon out_polygon{};
    geometry_msgs::msg::TransformStamped geometry_tf{};
    geometry_tf.transform = pose2transform(p_front);
    tf2::doTransform(base_polygon, out_polygon, geometry_tf);

    for (const auto & p : out_polygon.points) {
      one_step_polygon.outer().push_back(Point2d(p.x, p.y));
    }
  }

  {
    geometry_msgs::msg::Polygon out_polygon{};
    geometry_msgs::msg::TransformStamped geometry_tf{};
    geometry_tf.transform = pose2transform(p_back);
    tf2::doTransform(base_polygon, out_polygon, geometry_tf);

    for (const auto & p : out_polygon.points) {
      one_step_polygon.outer().push_back(Point2d(p.x, p.y));
    }
  }

  Polygon2d hull_polygon{};
  bg::convex_hull(one_step_polygon, hull_polygon);
  bg::correct(hull_polygon);

  return hull_polygon;
}

void sortCrosswalksByDistance(
  const PathWithLaneId & ego_path, const geometry_msgs::msg::Point & ego_pos,
  lanelet::ConstLanelets & crosswalks)
{
  const auto compare = [&](const lanelet::ConstLanelet & l1, const lanelet::ConstLanelet & l2) {
    const auto l1_intersects =
      getPolygonIntersects(ego_path, l1.polygon2d().basicPolygon(), ego_pos, 2);
    const auto l2_intersects =
      getPolygonIntersects(ego_path, l2.polygon2d().basicPolygon(), ego_pos, 2);

    if (l1_intersects.empty() || l2_intersects.empty()) {
      return true;
    }

    const auto dist_l1 = calcSignedArcLength(ego_path.points, size_t(0), l1_intersects.front());

    const auto dist_l2 = calcSignedArcLength(ego_path.points, size_t(0), l2_intersects.front());

    return dist_l1 < dist_l2;
  };

  std::sort(crosswalks.begin(), crosswalks.end(), compare);
}

std::vector<Point2d> calcOverlappingPoints(const Polygon2d & polygon1, const Polygon2d & polygon2)
{
  // NOTE: If one polygon is fully inside the other polygon, the result is empty.
  std::vector<Point2d> intersection{};
  bg::intersection(polygon1, polygon2, intersection);

  if (bg::within(polygon1, polygon2)) {
    for (const auto & p : polygon1.outer()) {
      intersection.push_back(Point2d{p.x(), p.y()});
    }
  }
  if (bg::within(polygon2, polygon1)) {
    for (const auto & p : polygon2.outer()) {
      intersection.push_back(Point2d{p.x(), p.y()});
    }
  }

  return intersection;
}
}  // namespace

CrosswalkModule::CrosswalkModule(
  const int64_t module_id, const lanelet::LaneletMapPtr & lanelet_map_ptr,
  const PlannerParam & planner_param, const bool use_regulatory_element,
  const rclcpp::Logger & logger, const rclcpp::Clock::SharedPtr clock)
: SceneModuleInterface(module_id, logger, clock),
  module_id_(module_id),
  planner_param_(planner_param),
  use_regulatory_element_(use_regulatory_element)
{
  velocity_factor_.init(VelocityFactor::CROSSWALK);
  passed_safety_slow_point_ = false;

  if (use_regulatory_element_) {
    const auto reg_elem_ptr = std::dynamic_pointer_cast<const lanelet::autoware::Crosswalk>(
      lanelet_map_ptr->regulatoryElementLayer.get(module_id));
    stop_lines_ = reg_elem_ptr->stopLines();
    crosswalk_ = reg_elem_ptr->crosswalkLanelet();
  } else {
    const auto stop_line = getStopLineFromMap(module_id_, lanelet_map_ptr, "crosswalk_id");
    if (!!stop_line) {
      stop_lines_.push_back(stop_line.get());
    }
    crosswalk_ = lanelet_map_ptr->laneletLayer.get(module_id);
  }
}

bool CrosswalkModule::modifyPathVelocity(PathWithLaneId * path, StopReason * stop_reason)
{
  stop_watch_.tic("total_processing_time");
  RCLCPP_INFO_EXPRESSION(
    logger_, planner_param_.show_processing_time,
    "=========== module_id: %ld ===========", module_id_);

  *stop_reason = planning_utils::initializeStopReason(StopReason::CROSSWALK);
  const auto & ego_pos = planner_data_->current_odometry->pose.position;

  // Initialize debug data
  debug_data_ = DebugData(planner_data_);
  for (const auto & p : crosswalk_.polygon2d().basicPolygon()) {
    debug_data_.crosswalk_polygon.push_back(createPoint(p.x(), p.y(), ego_pos.z));
  }
  recordTime(1);

  // Calculate intersection between path and crosswalks
  const auto path_intersects =
    getPolygonIntersects(*path, crosswalk_.polygon2d().basicPolygon(), ego_pos, 2);

  // Apply safety slow down speed if defined in Lanelet2 map
  if (crosswalk_.hasAttribute("safety_slow_down_speed")) {
    applySafetySlowDownSpeed(*path, path_intersects);
  }
  recordTime(2);

  // Calculate stop point
  const auto default_stop_factor = findDefaultStopFactor(*path, path_intersects);
  const auto nearest_stop_factor = findNearestStopFactor(*path, path_intersects);

  // Generate stop factor with type (NEAREST or DEFAULT)
  const auto stop_factor_info = generateStopFactorInfo(nearest_stop_factor, default_stop_factor);
  recordTime(3);

  // Set safe or unsafe
  setSafe(!stop_factor_info || stop_factor_info->type != StopFactorInfo::Type::NEAREST);

  // plan Go/Stop
  const bool result = [&]() {
    if (isActivated()) {
      planGo(stop_factor_info, *path);
      // TODO(murooka) call setDistance here
      return true;
    }
    return planStop(stop_factor_info, *path, stop_reason);
  }();
  recordTime(4);

  return result;
}

boost::optional<std::pair<double, geometry_msgs::msg::Point>> CrosswalkModule::getStopLine(
  const PathWithLaneId & ego_path, bool & exist_stopline_in_map,
  const std::vector<geometry_msgs::msg::Point> & path_intersects) const
{
  const auto & ego_pos = planner_data_->current_odometry->pose.position;
  for (const auto & stop_line : stop_lines_) {
    const auto p_stop_lines = getLinestringIntersects(
      ego_path, lanelet::utils::to2D(stop_line).basicLineString(), ego_pos, 2);
    if (p_stop_lines.empty()) {
      continue;
    }

    exist_stopline_in_map = true;

    const auto dist_ego_to_stop =
      calcSignedArcLength(ego_path.points, ego_pos, p_stop_lines.front());
    return std::make_pair(dist_ego_to_stop, p_stop_lines.front());
  }

  {
    exist_stopline_in_map = false;

    if (!path_intersects.empty()) {
      const auto p_stop_line = path_intersects.front();
      const auto dist_ego_to_stop = calcSignedArcLength(ego_path.points, ego_pos, p_stop_line) -
                                    planner_param_.stop_line_distance;
      return std::make_pair(dist_ego_to_stop, p_stop_line);
    }
  }

  return {};
}

boost::optional<StopFactor> CrosswalkModule::findDefaultStopFactor(
  const PathWithLaneId & ego_path, const std::vector<geometry_msgs::msg::Point> & path_intersects)
{
  const auto & base_link2front = planner_data_->vehicle_info_.max_longitudinal_offset_m;

  bool exist_stopline_in_map;
  const auto p_stop_line = getStopLine(ego_path, exist_stopline_in_map, path_intersects);
  if (!p_stop_line) {
    return {};
  }

  const auto & p_stop = p_stop_line.get().second;
  const auto stop_line_distance = exist_stopline_in_map ? 0.0 : planner_param_.stop_line_distance;
  const auto margin = stop_line_distance + base_link2front;
  const auto stop_pose = calcLongitudinalOffsetPose(ego_path.points, p_stop, -margin);

  if (!stop_pose) {
    return {};
  }

  StopFactor stop_factor;
  stop_factor.stop_pose = stop_pose.get();
  return stop_factor;
}

boost::optional<StopFactor> CrosswalkModule::findNearestStopFactor(
  const PathWithLaneId & ego_path, const std::vector<geometry_msgs::msg::Point> & path_intersects)
{
  if (ego_path.points.size() < 2) {
    RCLCPP_DEBUG(logger_, "Do not interpolate because path size is 1.");
    return {};
  }

  const auto & ego_pos = planner_data_->current_odometry->pose.position;
  const auto & objects_ptr = planner_data_->predicted_objects;
  const auto & base_link2front = planner_data_->vehicle_info_.max_longitudinal_offset_m;

  // Resample path sparsely for less computation cost
  constexpr double resample_interval = 4.0;
  const auto sparse_resample_path =
    resamplePath(ego_path, resample_interval, false, true, true, false);

  // Calculate attention range for crosswalk
  const auto crosswalk_attention_range = getAttentionRange(sparse_resample_path, path_intersects);

  // Calculate stop line
  bool exist_stopline_in_map;
  const auto p_stop_line =
    getStopLine(sparse_resample_path, exist_stopline_in_map, path_intersects);
  if (!p_stop_line) {
    return {};
  }

  // Get attention area, which is ego's footprints on the crosswalk
  const auto attention_area = getAttentionArea(sparse_resample_path, crosswalk_attention_range);

  // Check stuck vehicle
  const bool found_stuck_vehicle =
    isStuckVehicle(sparse_resample_path, objects_ptr->objects, path_intersects);

  // Check if ego is yielding
  const bool is_ego_yielding = [&]() {
    const auto has_reached_stop_point =
      p_stop_line.get().first - base_link2front < planner_param_.stop_position_threshold;

    return planner_data_->isVehicleStopped(planner_param_.ego_yield_query_stop_duration) &&
           has_reached_stop_point;
  }();

  // Update object state
  object_info_manager_.init();
  for (const auto & object : objects_ptr->objects) {
    const auto obj_uuid = toHexString(object.object_id);
    const auto & obj_vel = object.kinematics.initial_twist_with_covariance.twist.linear;
    object_info_manager_.update(
      obj_uuid, std::hypot(obj_vel.x, obj_vel.y), clock_->now(), is_ego_yielding, planner_param_);
  }
  object_info_manager_.finalize();

  // Check pedestrian for stop
  std::optional<std::pair<geometry_msgs::msg::Point, double>>
    nearest_stop_info;  // first stop point and minimum stop distance
  StopFactor stop_factor;
  const auto ignore_crosswalk = debug_data_.ignore_crosswalk = isRedSignalForPedestrians();
  for (const auto & object : objects_ptr->objects) {
    const auto & obj_pos = object.kinematics.initial_pose_with_covariance.pose.position;
    const auto obj_uuid = toHexString(object.object_id);

    if (!isCrosswalkUserType(object)) {
      continue;
    }

    if (ignore_crosswalk) {
      continue;
    }

    // NOTE: Collision points are calculated on each predicted path
    const auto collision_point =
      getCollisionPoints(sparse_resample_path, object, attention_area, crosswalk_attention_range);

    for (const auto & cp : collision_point) {
      const auto dist_ego2cp =
        calcSignedArcLength(sparse_resample_path.points, ego_pos, cp.collision_point) -
        planner_param_.stop_margin;

      const auto collision_state =
        getCollisionState(obj_uuid, cp.time_to_collision, cp.time_to_vehicle);
      debug_data_.collision_points.push_back(std::make_pair(cp, collision_state));

      if (collision_state != CollisionState::YIELD) {
        continue;
      }

      stop_factor.stop_factor_points.push_back(obj_pos);

      if (!nearest_stop_info || dist_ego2cp < nearest_stop_info->second) {
        nearest_stop_info = std::make_pair(cp.collision_point, dist_ego2cp);
      }
    }
  }

  // Check if stop is required
  const bool found_pedestrians = static_cast<bool>(nearest_stop_info);
  const auto need_to_stop = found_pedestrians || found_stuck_vehicle;
  if (!need_to_stop) {
    return {};
  }

  if (!nearest_stop_info) {
    nearest_stop_info = std::make_pair(p_stop_line.get().second, p_stop_line.get().first);
  }

  const auto within_stop_line_margin =
    p_stop_line.get().first < nearest_stop_info->second &&
    nearest_stop_info->second < p_stop_line.get().first + planner_param_.stop_line_margin;

  const auto stop_at_stop_line = !found_pedestrians || within_stop_line_margin;

  const auto & p_stop = stop_at_stop_line ? p_stop_line.get().second : nearest_stop_info->first;
  const auto stop_line_distance = exist_stopline_in_map ? 0.0 : planner_param_.stop_line_distance;
  const auto margin = stop_at_stop_line ? stop_line_distance + base_link2front
                                        : planner_param_.stop_margin + base_link2front;
  const auto stop_pose = calcLongitudinalOffsetPose(ego_path.points, p_stop, -margin);

  if (!stop_pose) {
    return {};
  }

  stop_factor.stop_pose = stop_pose.get();
  return stop_factor;
}

std::pair<double, double> CrosswalkModule::getAttentionRange(
  const PathWithLaneId & ego_path, const std::vector<geometry_msgs::msg::Point> & path_intersects)
{
  stop_watch_.tic(__func__);

  if (path_intersects.size() < 2) {
    return std::make_pair(0.0, 0.0);
  }

  const auto & ego_pos = planner_data_->current_odometry->pose.position;
  const auto near_attention_range =
    calcSignedArcLength(ego_path.points, ego_pos, path_intersects.front()) -
    planner_param_.crosswalk_attention_range;
  const auto far_attention_range =
    calcSignedArcLength(ego_path.points, ego_pos, path_intersects.back()) +
    planner_param_.crosswalk_attention_range;

  const auto [clamped_near_attention_range, clamped_far_attention_range] =
    clampAttentionRangeByNeighborCrosswalks(ego_path, near_attention_range, far_attention_range);

  RCLCPP_INFO_EXPRESSION(
    logger_, planner_param_.show_processing_time, "%s : %f ms", __func__,
    stop_watch_.toc(__func__, true));

  return std::make_pair(
    std::max(0.0, clamped_near_attention_range), std::max(0.0, clamped_far_attention_range));
}

void CrosswalkModule::insertDecelPointWithDebugInfo(
  const geometry_msgs::msg::Point & stop_point, const float target_velocity,
  PathWithLaneId & output)
{
  const auto stop_pose = planning_utils::insertDecelPoint(stop_point, output, target_velocity);
  if (!stop_pose) {
    return;
  }
  const auto & ego_pos = planner_data_->current_odometry->pose.position;

  setDistance(calcSignedArcLength(output.points, ego_pos, stop_pose->position));

  debug_data_.first_stop_pose = getPose(*stop_pose);

  if (std::abs(target_velocity) < 1e-3) {
    debug_data_.stop_poses.push_back(*stop_pose);
  } else {
    debug_data_.slow_poses.push_back(*stop_pose);
  }
}

float CrosswalkModule::calcTargetVelocity(
  const geometry_msgs::msg::Point & stop_point, const PathWithLaneId & ego_path) const
{
  const auto max_jerk = planner_param_.max_slow_down_jerk;
  const auto max_accel = planner_param_.max_slow_down_accel;
  const auto & ego_pos = planner_data_->current_odometry->pose.position;
  const auto ego_vel = planner_data_->current_velocity->twist.linear.x;

  if (ego_vel < planner_param_.no_relax_velocity) {
    return 0.0;
  }

  const auto ego_acc = planner_data_->current_acceleration->accel.accel.linear.x;
  const auto dist_deceleration = calcSignedArcLength(ego_path.points, ego_pos, stop_point);
  const auto feasible_velocity = planning_utils::calcDecelerationVelocityFromDistanceToTarget(
    max_jerk, max_accel, ego_acc, ego_vel, dist_deceleration);

  constexpr double margin_velocity = 0.5;  // 1.8 km/h
  return margin_velocity < feasible_velocity ? feasible_velocity : 0.0;
}

std::pair<double, double> CrosswalkModule::clampAttentionRangeByNeighborCrosswalks(
  const PathWithLaneId & ego_path, const double near_attention_range,
  const double far_attention_range)
{
  stop_watch_.tic(__func__);

  const auto & ego_pos = planner_data_->current_odometry->pose.position;

  const auto p_near = calcLongitudinalOffsetPoint(ego_path.points, ego_pos, near_attention_range);
  const auto p_far = calcLongitudinalOffsetPoint(ego_path.points, ego_pos, far_attention_range);

  if (!p_near || !p_far) {
    return std::make_pair(near_attention_range, far_attention_range);
  }

  const auto near_idx = findNearestSegmentIndex(ego_path.points, p_near.get());
  const auto far_idx = findNearestSegmentIndex(ego_path.points, p_far.get()) + 1;

  std::set<int64_t> lane_ids;
  for (size_t i = near_idx; i < far_idx; ++i) {
    for (const auto & id : ego_path.points.at(i).lane_ids) {
      lane_ids.insert(id);
    }
  }

  lanelet::ConstLanelets crosswalks{};
  constexpr int PEDESTRIAN_GRAPH_ID = 1;
  const auto lanelet_map_ptr = planner_data_->route_handler_->getLaneletMapPtr();
  const auto overall_graphs_ptr = planner_data_->route_handler_->getOverallGraphPtr();

  for (const auto & id : lane_ids) {
    const auto ll = lanelet_map_ptr->laneletLayer.get(id);
    const auto conflicting_crosswalks =
      overall_graphs_ptr->conflictingInGraph(ll, PEDESTRIAN_GRAPH_ID);
    for (const auto & crosswalk : conflicting_crosswalks) {
      const auto itr = std::find_if(
        crosswalks.begin(), crosswalks.end(),
        [&](const lanelet::ConstLanelet & ll) { return ll.id() == crosswalk.id(); });
      if (itr == crosswalks.end()) {
        crosswalks.push_back(crosswalk);
      }
    }
  }

  sortCrosswalksByDistance(ego_path, ego_pos, crosswalks);

  boost::optional<lanelet::ConstLanelet> prev_crosswalk{boost::none};
  boost::optional<lanelet::ConstLanelet> next_crosswalk{boost::none};

  if (!crosswalks.empty()) {
    for (size_t i = 0; i < crosswalks.size() - 1; ++i) {
      const auto ll_front = crosswalks.at(i);
      const auto ll_back = crosswalks.at(i + 1);

      if (ll_front.id() == module_id_ && ll_back.id() != module_id_) {
        next_crosswalk = ll_back;
      }

      if (ll_front.id() != module_id_ && ll_back.id() == module_id_) {
        prev_crosswalk = ll_front;
      }
    }
  }

  const double clamped_near_attention_range = [&]() {
    if (!prev_crosswalk) {
      return near_attention_range;
    }
    auto reverse_ego_path = ego_path;
    std::reverse(reverse_ego_path.points.begin(), reverse_ego_path.points.end());

    const auto prev_crosswalk_intersects = getPolygonIntersects(
      reverse_ego_path, prev_crosswalk.get().polygon2d().basicPolygon(), ego_pos, 2);
    if (prev_crosswalk_intersects.empty()) {
      return near_attention_range;
    }

    const auto dist_to_prev_crosswalk =
      calcSignedArcLength(ego_path.points, ego_pos, prev_crosswalk_intersects.front());
    return std::max(near_attention_range, dist_to_prev_crosswalk);
  }();

  const double clamped_far_attention_range = [&]() {
    if (!next_crosswalk) {
      return far_attention_range;
    }
    const auto next_crosswalk_intersects =
      getPolygonIntersects(ego_path, next_crosswalk.get().polygon2d().basicPolygon(), ego_pos, 2);

    if (next_crosswalk_intersects.empty()) {
      return far_attention_range;
    }

    const auto dist_to_next_crosswalk =
      calcSignedArcLength(ego_path.points, ego_pos, next_crosswalk_intersects.front());
    return std::min(far_attention_range, dist_to_next_crosswalk);
  }();

  const auto update_p_near =
    calcLongitudinalOffsetPoint(ego_path.points, ego_pos, near_attention_range);
  const auto update_p_far =
    calcLongitudinalOffsetPoint(ego_path.points, ego_pos, far_attention_range);

  if (update_p_near && update_p_far) {
    debug_data_.range_near_point = update_p_near.get();
    debug_data_.range_far_point = update_p_far.get();
  }

  RCLCPP_INFO_EXPRESSION(
    logger_, planner_param_.show_processing_time, "%s : %f ms", __func__,
    stop_watch_.toc(__func__, true));

  return std::make_pair(clamped_near_attention_range, clamped_far_attention_range);
}

std::vector<CollisionPoint> CrosswalkModule::getCollisionPoints(
  const PathWithLaneId & ego_path, const PredictedObject & object, const Polygon2d & attention_area,
  const std::pair<double, double> & crosswalk_attention_range)
{
  stop_watch_.tic(__func__);

  std::vector<CollisionPoint> ret{};

  const auto & obj_vel = object.kinematics.initial_twist_with_covariance.twist.linear;

  const auto & ego_pos = planner_data_->current_odometry->pose.position;
  const auto & ego_vel = planner_data_->current_velocity->twist.linear;

  const auto obj_polygon =
    createObjectPolygon(object.shape.dimensions.x, object.shape.dimensions.y);

  for (const auto & obj_path : object.kinematics.predicted_paths) {
    for (size_t i = 0; i < obj_path.path.size() - 1; ++i) {
      const auto & p_obj_front = obj_path.path.at(i);
      const auto & p_obj_back = obj_path.path.at(i + 1);
      const auto obj_one_step_polygon = createOneStepPolygon(p_obj_front, p_obj_back, obj_polygon);

      // Calculate intersection points between object and attention area
      const auto tmp_intersection = calcOverlappingPoints(obj_one_step_polygon, attention_area);
      if (tmp_intersection.empty()) {
        continue;
      }

      // Calculate nearest collision point
      double minimum_stop_dist = std::numeric_limits<double>::max();
      geometry_msgs::msg::Point nearest_collision_point{};
      for (const auto & p : tmp_intersection) {
        const auto cp = createPoint(p.x(), p.y(), ego_pos.z);
        const auto dist_ego2cp = calcSignedArcLength(ego_path.points, ego_pos, cp);

        if (dist_ego2cp < minimum_stop_dist) {
          minimum_stop_dist = dist_ego2cp;
          nearest_collision_point = cp;
        }
      }

      const auto dist_ego2cp =
        calcSignedArcLength(ego_path.points, ego_pos, nearest_collision_point);
      constexpr double eps = 1e-3;
      const auto dist_obj2cp =
        calcArcLength(obj_path.path) < eps
          ? 0.0
          : calcSignedArcLength(obj_path.path, size_t(0), nearest_collision_point);

      if (
        dist_ego2cp < crosswalk_attention_range.first ||
        crosswalk_attention_range.second < dist_ego2cp) {
        continue;
      }

      ret.push_back(
        createCollisionPoint(nearest_collision_point, dist_ego2cp, dist_obj2cp, ego_vel, obj_vel));

      debug_data_.obj_polygons.push_back(toGeometryPointVector(obj_one_step_polygon, ego_pos.z));

      break;
    }
  }

  RCLCPP_INFO_EXPRESSION(
    logger_, planner_param_.show_processing_time, "%s : %f ms", __func__,
    stop_watch_.toc(__func__, true));

  return ret;
}

CollisionPoint CrosswalkModule::createCollisionPoint(
  const geometry_msgs::msg::Point & nearest_collision_point, const double dist_ego2cp,
  const double dist_obj2cp, const geometry_msgs::msg::Vector3 & ego_vel,
  const geometry_msgs::msg::Vector3 & obj_vel) const
{
  constexpr double min_ego_velocity = 1.38;  // [m/s]

  const auto & base_link2front = planner_data_->vehicle_info_.max_longitudinal_offset_m;

  const auto estimated_velocity = std::hypot(obj_vel.x, obj_vel.y);
  const auto velocity = std::max(planner_param_.min_object_velocity, estimated_velocity);

  CollisionPoint collision_point{};
  collision_point.collision_point = nearest_collision_point;
  collision_point.time_to_collision =
    std::max(0.0, dist_ego2cp - planner_param_.stop_margin - base_link2front) /
    std::max(ego_vel.x, min_ego_velocity);
  collision_point.time_to_vehicle = std::max(0.0, dist_obj2cp) / velocity;

  return collision_point;
}

CollisionState CrosswalkModule::getCollisionState(
  const std::string & obj_uuid, const double ttc, const double ttv) const
{
  // First, check if the object can be ignored
  const auto obj_state = object_info_manager_.getState(obj_uuid);
  if (obj_state == ObjectInfo::State::FULLY_STOPPED) {
    return CollisionState::IGNORE;
  }

  // Compare time to collision and vehicle
  if (ttc + planner_param_.ego_pass_first_margin < ttv) {
    return CollisionState::EGO_PASS_FIRST;
  }

  if (ttv + planner_param_.ego_pass_later_margin < ttc) {
    return CollisionState::EGO_PASS_LATER;
  }
  return CollisionState::YIELD;
}

void CrosswalkModule::applySafetySlowDownSpeed(
  PathWithLaneId & output, const std::vector<geometry_msgs::msg::Point> & path_intersects)
{
  if (path_intersects.empty()) {
    return;
  }

  const auto & ego_pos = planner_data_->current_odometry->pose.position;
  const auto ego_path = output;

  // Safety slow down speed in [m/s]
  const auto & safety_slow_down_speed =
    static_cast<float>(crosswalk_.attribute("safety_slow_down_speed").asDouble().get());

  if (!passed_safety_slow_point_) {
    // Safety slow down distance [m]
    const double safety_slow_down_distance =
      crosswalk_.attributeOr("safety_slow_down_distance", 2.0);

    // the range until to the point where ego will have a const safety slow down speed
    const double safety_slow_margin =
      planner_data_->vehicle_info_.max_longitudinal_offset_m + safety_slow_down_distance;
    const double safety_slow_point_range =
      calcSignedArcLength(ego_path.points, ego_pos, path_intersects.front()) - safety_slow_margin;

    const auto & p_safety_slow =
      calcLongitudinalOffsetPoint(ego_path.points, ego_pos, safety_slow_point_range);

    insertDecelPointWithDebugInfo(p_safety_slow.get(), safety_slow_down_speed, output);

    if (safety_slow_point_range < 0.0) {
      passed_safety_slow_point_ = true;
    }
  } else {
    // the range until to the point where ego will start accelerate
    const double safety_slow_end_point_range =
      calcSignedArcLength(ego_path.points, ego_pos, path_intersects.back());

    if (0.0 < safety_slow_end_point_range) {
      // insert constant ego speed until the end of the crosswalk
      for (auto & p : output.points) {
        const float original_velocity = p.point.longitudinal_velocity_mps;
        p.point.longitudinal_velocity_mps = std::min(original_velocity, safety_slow_down_speed);
      }
    }
  }
}

Polygon2d CrosswalkModule::getAttentionArea(
  const PathWithLaneId & sparse_resample_path,
  const std::pair<double, double> & crosswalk_attention_range) const
{
  const auto & ego_pos = planner_data_->current_odometry->pose.position;
  const auto ego_polygon = createVehiclePolygon(planner_data_->vehicle_info_);

  Polygon2d attention_area;
  for (size_t j = 0; j < sparse_resample_path.points.size() - 1; ++j) {
    const auto & p_ego_front = sparse_resample_path.points.at(j).point.pose;
    const auto & p_ego_back = sparse_resample_path.points.at(j + 1).point.pose;
    const auto front_length =
      calcSignedArcLength(sparse_resample_path.points, ego_pos, p_ego_front.position);
    const auto back_length =
      calcSignedArcLength(sparse_resample_path.points, ego_pos, p_ego_back.position);

    if (back_length < crosswalk_attention_range.first) {
      continue;
    }

    if (crosswalk_attention_range.second < front_length) {
      break;
    }

    const auto ego_one_step_polygon = createOneStepPolygon(p_ego_front, p_ego_back, ego_polygon);

    debug_data_.ego_polygons.push_back(toGeometryPointVector(ego_one_step_polygon, ego_pos.z));

    std::vector<Polygon2d> unions;
    bg::union_(attention_area, ego_one_step_polygon, unions);
    if (!unions.empty()) {
      attention_area = unions.front();
      bg::correct(attention_area);
    }
  }

  return attention_area;
}

bool CrosswalkModule::isStuckVehicle(
  const PathWithLaneId & ego_path, const std::vector<PredictedObject> & objects,
  const std::vector<geometry_msgs::msg::Point> & path_intersects) const
{
  if (path_intersects.size() < 2) {
    return false;
  }

  for (const auto & object : objects) {
    if (!isVehicle(object)) {
      continue;
    }

    const auto & obj_vel = object.kinematics.initial_twist_with_covariance.twist.linear;
    if (planner_param_.stuck_vehicle_velocity < std::hypot(obj_vel.x, obj_vel.y)) {
      continue;
    }

    const auto & obj_pos = object.kinematics.initial_pose_with_covariance.pose.position;
    const auto lateral_offset = calcLateralOffset(ego_path.points, obj_pos);
    if (planner_param_.max_lateral_offset < std::abs(lateral_offset)) {
      continue;
    }

    const auto & ego_pos = planner_data_->current_odometry->pose.position;
    const double near_attention_range =
      calcSignedArcLength(ego_path.points, ego_pos, path_intersects.back());
    const double far_attention_range =
      near_attention_range + planner_param_.stuck_vehicle_attention_range;

    const auto dist_ego2obj = calcSignedArcLength(ego_path.points, ego_pos, obj_pos);

    if (near_attention_range < dist_ego2obj && dist_ego2obj < far_attention_range) {
      return true;
    }
  }

  return false;
}

bool CrosswalkModule::isRedSignalForPedestrians() const
{
  const auto traffic_lights_reg_elems =
    crosswalk_.regulatoryElementsAs<const lanelet::TrafficLight>();

  for (const auto & traffic_lights_reg_elem : traffic_lights_reg_elems) {
    lanelet::ConstLineStringsOrPolygons3d traffic_lights = traffic_lights_reg_elem->trafficLights();
    for (const auto & traffic_light : traffic_lights) {
      const auto ll_traffic_light = static_cast<lanelet::ConstLineString3d>(traffic_light);
      const auto traffic_signal_stamped = planner_data_->getTrafficSignal(ll_traffic_light.id());
      if (!traffic_signal_stamped) {
        continue;
      }

      if (
        planner_param_.tl_state_timeout <
        (clock_->now() - traffic_signal_stamped->header.stamp).seconds()) {
        continue;
      }

      const auto & lights = traffic_signal_stamped->signal.lights;
      if (lights.empty()) {
        continue;
      }

      if (lights.front().color == TrafficLight::RED) {
        return true;
      }
    }
  }

  return false;
}

bool CrosswalkModule::isVehicle(const PredictedObject & object)
{
  if (object.classification.empty()) {
    return false;
  }

  const auto & label = object.classification.front().label;

  if (label == ObjectClassification::CAR) {
    return true;
  }

  if (label == ObjectClassification::TRUCK) {
    return true;
  }

  if (label == ObjectClassification::BUS) {
    return true;
  }

  if (label == ObjectClassification::TRAILER) {
    return true;
  }

  if (label == ObjectClassification::MOTORCYCLE) {
    return true;
  }

  return false;
}

bool CrosswalkModule::isCrosswalkUserType(const PredictedObject & object) const
{
  if (object.classification.empty()) {
    return false;
  }

  const auto & label = object.classification.front().label;

  if (label == ObjectClassification::UNKNOWN && planner_param_.look_unknown) {
    return true;
  }

  if (label == ObjectClassification::BICYCLE && planner_param_.look_bicycle) {
    return true;
  }

  if (label == ObjectClassification::MOTORCYCLE && planner_param_.look_motorcycle) {
    return true;
  }

  if (label == ObjectClassification::PEDESTRIAN && planner_param_.look_pedestrian) {
    return true;
  }

  return false;
}

geometry_msgs::msg::Polygon CrosswalkModule::createObjectPolygon(
  const double width_m, const double length_m)
{
  geometry_msgs::msg::Polygon polygon{};

  polygon.points.push_back(createPoint32(length_m / 2.0, -width_m / 2.0, 0.0));
  polygon.points.push_back(createPoint32(length_m / 2.0, width_m / 2.0, 0.0));
  polygon.points.push_back(createPoint32(-length_m / 2.0, width_m / 2.0, 0.0));
  polygon.points.push_back(createPoint32(-length_m / 2.0, -width_m / 2.0, 0.0));

  return polygon;
}

geometry_msgs::msg::Polygon CrosswalkModule::createVehiclePolygon(
  const vehicle_info_util::VehicleInfo & vehicle_info)
{
  const auto & i = vehicle_info;
  const auto & front_m = i.max_longitudinal_offset_m;
  const auto & width_m = i.vehicle_width_m / 2.0;
  const auto & back_m = i.rear_overhang_m;

  geometry_msgs::msg::Polygon polygon{};

  polygon.points.push_back(createPoint32(front_m, -width_m, 0.0));
  polygon.points.push_back(createPoint32(front_m, width_m, 0.0));
  polygon.points.push_back(createPoint32(-back_m, width_m, 0.0));
  polygon.points.push_back(createPoint32(-back_m, -width_m, 0.0));

  return polygon;
}

boost::optional<StopFactorInfo> CrosswalkModule::generateStopFactorInfo(
  const boost::optional<StopFactor> & nearest_stop_factor,
  const boost::optional<StopFactor> & default_stop_factor) const
{
  if (nearest_stop_factor) {
    return StopFactorInfo{*nearest_stop_factor, StopFactorInfo::Type::NEAREST};
  }
  if (default_stop_factor) {
    return StopFactorInfo{*default_stop_factor, StopFactorInfo::Type::DEFAULT};
  }
  return {};
}

void CrosswalkModule::planGo(
  const boost::optional<StopFactorInfo> & stop_factor_info, PathWithLaneId & ego_path)
{
  const auto & ego_pos = planner_data_->current_odometry->pose.position;

  if (!stop_factor_info) {
    setDistance(std::numeric_limits<double>::lowest());
    return;
  }

  if (stop_factor_info->type == StopFactorInfo::Type::NEAREST) {
    // Plan slow down
    const auto target_velocity =
      calcTargetVelocity(stop_factor_info->stop_factor.stop_pose.position, ego_path);
    insertDecelPointWithDebugInfo(
      stop_factor_info->stop_factor.stop_pose.position,
      std::max(planner_param_.min_slow_down_velocity, target_velocity), ego_path);
    return;
  }

  // NOTE: if (stop_factor_info->type == StopFactorInfo::Type::DEFAULT)
  const auto crosswalk_distance = calcSignedArcLength(
    ego_path.points, ego_pos, getPoint(stop_factor_info->stop_factor.stop_pose.position));
  setDistance(crosswalk_distance);
}

bool CrosswalkModule::planStop(
  const boost::optional<StopFactorInfo> & stop_factor_info, PathWithLaneId & ego_path,
  StopReason * stop_reason)
{
  if (!stop_factor_info) {
    return false;
  }

  insertDecelPointWithDebugInfo(stop_factor_info->stop_factor.stop_pose.position, 0.0, ego_path);
  planning_utils::appendStopReason(stop_factor_info->stop_factor, stop_reason);
  velocity_factor_.set(
    ego_path.points, planner_data_->current_odometry->pose, stop_factor_info->stop_factor.stop_pose,
    VelocityFactor::UNKNOWN);
  return true;
}
}  // namespace behavior_velocity_planner
