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

#include "scene.hpp"

#include <behavior_velocity_planner_common/utilization/boost_geometry_helper.hpp>
#include <behavior_velocity_planner_common/utilization/path_utilization.hpp>
#include <behavior_velocity_planner_common/utilization/util.hpp>
#include <lanelet2_extension/regulatory_elements/road_marking.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <motion_utils/trajectory/trajectory.hpp>

#include <boost/geometry/algorithms/area.hpp>
#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/algorithms/length.hpp>
#include <boost/geometry/algorithms/union.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace behavior_velocity_planner
{
namespace bg = boost::geometry;

namespace
{
[[maybe_unused]] geometry_msgs::msg::Polygon toGeomPoly(const lanelet::CompoundPolygon3d & poly)
{
  geometry_msgs::msg::Polygon geom_poly;

  for (const auto & p : poly) {
    geometry_msgs::msg::Point32 geom_point;
    geom_point.x = p.x();
    geom_point.y = p.y();
    geom_point.z = p.z();
    geom_poly.points.push_back(geom_point);
  }

  return geom_poly;
}
}  // namespace

BlindSpotModule::BlindSpotModule(
  const int64_t module_id, const int64_t lane_id, std::shared_ptr<const PlannerData> planner_data,
  const PlannerParam & planner_param, const rclcpp::Logger logger,
  const rclcpp::Clock::SharedPtr clock)
: SceneModuleInterface(module_id, logger, clock),
  lane_id_(lane_id),
  turn_direction_(TurnDirection::INVALID),
  is_over_pass_judge_line_(false)
{
  velocity_factor_.init(PlanningBehavior::REAR_CHECK);
  planner_param_ = planner_param;

  const auto & assigned_lanelet =
    planner_data->route_handler_->getLaneletMapPtr()->laneletLayer.get(lane_id);
  const std::string turn_direction = assigned_lanelet.attributeOr("turn_direction", "else");
  if (!turn_direction.compare("left")) {
    turn_direction_ = TurnDirection::LEFT;
  } else if (!turn_direction.compare("right")) {
    turn_direction_ = TurnDirection::RIGHT;
  }
}

bool BlindSpotModule::modifyPathVelocity(PathWithLaneId * path, StopReason * stop_reason)
{
  debug_data_ = DebugData();
  *stop_reason = planning_utils::initializeStopReason(StopReason::BLIND_SPOT);

  const auto input_path = *path;

  StateMachine::State current_state = state_machine_.getState();
  RCLCPP_DEBUG(
    logger_, "lane_id = %ld, state = %s", lane_id_, StateMachine::toString(current_state).c_str());

  /* get current pose */
  const auto & current_pose = planner_data_->current_odometry->pose;

  /* get lanelet map */
  const auto lanelet_map_ptr = planner_data_->route_handler_->getLaneletMapPtr();
  const auto routing_graph_ptr = planner_data_->route_handler_->getRoutingGraphPtr();

  /* set stop-line and stop-judgement-line for base_link */
  const auto interpolated_path_info_opt = generateInterpolatedPathInfo(input_path);
  if (!interpolated_path_info_opt) {
    RCLCPP_DEBUG(logger_, "[BlindSpotModule::run] failed to interpolate path");
    return false;
  }
  const auto & interpolated_path_info = interpolated_path_info_opt.value();

  if (!first_conflicting_lanelet_) {
    first_conflicting_lanelet_ = getFirstConflictingLanelet(interpolated_path_info);
  }
  if (!first_conflicting_lanelet_) {
    RCLCPP_DEBUG(logger_, "[BlindSpotModule::run] failed to find first conflicting lanelet");
    return false;
  }

  const auto stoplines_idx_opt = generateStopLine(interpolated_path_info, path);
  if (!stoplines_idx_opt) {
    RCLCPP_DEBUG(logger_, "[BlindSpotModule::run] setStopLineIdx fail");
    return false;
  }

  const auto [default_stopline_idx, critical_stopline_idx] = stoplines_idx_opt.value();

  if (default_stopline_idx <= 0) {
    RCLCPP_DEBUG(
      logger_, "[Blind Spot] stop line or pass judge line is at path[0], ignore planning.");
    *path = input_path;  // reset path
    setSafe(true);
    setDistance(std::numeric_limits<double>::lowest());
    return true;
  }

  /* calc closest index */
  const auto closest_idx = motion_utils::findFirstNearestIndexWithSoftConstraints(
    input_path.points, current_pose, planner_data_->ego_nearest_dist_threshold,
    planner_data_->ego_nearest_yaw_threshold);
  const auto stop_line_idx =
    closest_idx > default_stopline_idx ? critical_stopline_idx : default_stopline_idx;

  /* set judge line dist */
  const double current_vel = planner_data_->current_velocity->twist.linear.x;
  const double max_acc = planner_data_->max_stop_acceleration_threshold;
  const double delay_response_time = planner_data_->delay_response_time;
  const double pass_judge_line_dist =
    planning_utils::calcJudgeLineDistWithAccLimit(current_vel, max_acc, delay_response_time);
  const auto stop_point_pose = path->points.at(stop_line_idx).point.pose;
  const auto ego_segment_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
    input_path.points, current_pose, planner_data_->ego_nearest_dist_threshold,
    planner_data_->ego_nearest_yaw_threshold);
  const size_t stop_point_segment_idx =
    motion_utils::findNearestSegmentIndex(input_path.points, stop_point_pose.position);
  const auto distance_until_stop = motion_utils::calcSignedArcLength(
    input_path.points, current_pose.position, ego_segment_idx, stop_point_pose.position,
    stop_point_segment_idx);

  /* get debug info */
  const auto stop_line_pose = planning_utils::getAheadPose(
    stop_line_idx, planner_data_->vehicle_info_.max_longitudinal_offset_m, *path);
  debug_data_.virtual_wall_pose = stop_line_pose;
  const auto stop_pose = path->points.at(stop_line_idx).point.pose;
  debug_data_.stop_point_pose = stop_pose;

  /* if current_state = GO, and current_pose is over judge_line, ignore planning. */
  if (planner_param_.use_pass_judge_line) {
    const double eps = 1e-1;  // to prevent hunting
    if (
      current_state == StateMachine::State::GO &&
      distance_until_stop + eps < pass_judge_line_dist) {
      RCLCPP_DEBUG(logger_, "over the pass judge line. no plan needed.");
      *path = input_path;  // reset path
      setSafe(true);
      setDistance(std::numeric_limits<double>::lowest());
      return true;  // no plan needed.
    }
  }

  /* get dynamic object */
  const auto objects_ptr = planner_data_->predicted_objects;

  /* calculate dynamic collision around detection area */
  bool has_obstacle = checkObstacleInBlindSpot(
    lanelet_map_ptr, routing_graph_ptr, *path, objects_ptr, closest_idx, stop_line_pose);
  state_machine_.setStateWithMarginTime(
    has_obstacle ? StateMachine::State::STOP : StateMachine::State::GO,
    logger_.get_child("state_machine"), *clock_);

  /* set stop speed */
  setSafe(state_machine_.getState() != StateMachine::State::STOP);
  setDistance(motion_utils::calcSignedArcLength(
    path->points, current_pose.position, path->points.at(stop_line_idx).point.pose.position));
  if (!isActivated()) {
    constexpr double stop_vel = 0.0;
    planning_utils::setVelocityFromIndex(stop_line_idx, stop_vel, path);

    /* get stop point and stop factor */
    tier4_planning_msgs::msg::StopFactor stop_factor;
    stop_factor.stop_pose = debug_data_.stop_point_pose;
    stop_factor.stop_factor_points = planning_utils::toRosPoints(debug_data_.conflicting_targets);
    planning_utils::appendStopReason(stop_factor, stop_reason);
    velocity_factor_.set(
      path->points, planner_data_->current_odometry->pose, stop_pose, VelocityFactor::UNKNOWN);
  } else {
    *path = input_path;  // reset path
  }
  return true;
}

static bool hasLaneIds(
  const autoware_auto_planning_msgs::msg::PathPointWithLaneId & p, const lanelet::Id id)
{
  for (const auto & pid : p.lane_ids) {
    if (pid == id) {
      return true;
    }
  }
  return false;
}

static std::optional<std::pair<size_t, size_t>> findLaneIdInterval(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & p, const lanelet::Id id)
{
  bool found = false;
  size_t start = 0;
  size_t end = p.points.size() > 0 ? p.points.size() - 1 : 0;
  if (start == end) {
    // there is only one point in the path
    return std::nullopt;
  }
  for (size_t i = 0; i < p.points.size(); ++i) {
    if (hasLaneIds(p.points.at(i), id)) {
      if (!found) {
        // found interval for the first time
        found = true;
        start = i;
      }
    } else if (found) {
      // prior point was in the interval. interval ended
      end = i;
      break;
    }
  }
  start = start > 0 ? start - 1 : 0;  // the idx of last point before the interval
  return found ? std::make_optional(std::make_pair(start, end)) : std::nullopt;
}

std::optional<InterpolatedPathInfo> BlindSpotModule::generateInterpolatedPathInfo(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & input_path) const
{
  constexpr double ds = 0.2;
  InterpolatedPathInfo interpolated_path_info;
  if (!splineInterpolate(input_path, ds, interpolated_path_info.path, logger_)) {
    return std::nullopt;
  }
  interpolated_path_info.ds = ds;
  interpolated_path_info.lane_id = lane_id_;
  interpolated_path_info.lane_id_interval =
    findLaneIdInterval(interpolated_path_info.path, lane_id_);
  return interpolated_path_info;
}

std::optional<lanelet::ConstLanelet> BlindSpotModule::getFirstConflictingLanelet(
  const InterpolatedPathInfo & interpolated_path_info) const
{
  if (!interpolated_path_info.lane_id_interval) {
    return std::nullopt;
  }

  const auto lanelet_map_ptr = planner_data_->route_handler_->getLaneletMapPtr();
  const auto routing_graph_ptr = planner_data_->route_handler_->getRoutingGraphPtr();

  const auto assigned_lane = lanelet_map_ptr->laneletLayer.get(lane_id_);
  const auto conflicting_lanes =
    lanelet::utils::getConflictingLanelets(routing_graph_ptr, assigned_lane);
  lanelet::ConstLanelets conflicting_ex_sibling_lanes{};
  lanelet::ConstLanelets sibling_lanes{};
  for (const auto & prev : routing_graph_ptr->previous(assigned_lane)) {
    for (const auto & following : routing_graph_ptr->following(prev)) {
      if (!lanelet::utils::contains(sibling_lanes, following)) {
        sibling_lanes.push_back(following);
      }
    }
  }
  for (const auto & conflicting : conflicting_lanes) {
    if (!lanelet::utils::contains(sibling_lanes, conflicting)) {
      conflicting_ex_sibling_lanes.push_back(conflicting);
    }
  }

  const auto [lane_start, lane_end] = interpolated_path_info.lane_id_interval.value();
  const size_t vehicle_length_idx = static_cast<size_t>(
    planner_data_->vehicle_info_.max_longitudinal_offset_m / interpolated_path_info.ds);
  const size_t start =
    static_cast<size_t>(std::max<int>(0, static_cast<int>(lane_start) - vehicle_length_idx));
  const auto local_footprint = planner_data_->vehicle_info_.createFootprint(0.0, 0.0);
  for (size_t i = start; i <= lane_end; ++i) {
    const auto & pose = interpolated_path_info.path.points.at(i).point.pose;
    const auto path_footprint = tier4_autoware_utils::transformVector(
      local_footprint, tier4_autoware_utils::pose2transform(pose));
    for (const auto & conflicting : conflicting_ex_sibling_lanes) {
      const auto area2d = conflicting.polygon2d().basicPolygon();
      const bool is_in_polygon = bg::intersects(area2d, path_footprint);
      if (is_in_polygon) {
        return std::make_optional<lanelet::ConstLanelet>(conflicting);
      }
    }
  }
  return std::nullopt;
}

static std::optional<size_t> getFirstPointInsidePolygonByFootprint(
  const lanelet::CompoundPolygon2d & polygon, const InterpolatedPathInfo & interpolated_path_info,
  const tier4_autoware_utils::LinearRing2d & footprint, const double vehicle_length)
{
  const auto & path_ip = interpolated_path_info.path;
  const auto [lane_start, lane_end] = interpolated_path_info.lane_id_interval.value();
  const size_t vehicle_length_idx = static_cast<size_t>(vehicle_length / interpolated_path_info.ds);
  const size_t start =
    static_cast<size_t>(std::max<int>(0, static_cast<int>(lane_start) - vehicle_length_idx));
  const auto area_2d = polygon.basicPolygon();
  for (auto i = start; i <= lane_end; ++i) {
    const auto & base_pose = path_ip.points.at(i).point.pose;
    const auto path_footprint = tier4_autoware_utils::transformVector(
      footprint, tier4_autoware_utils::pose2transform(base_pose));
    if (bg::intersects(path_footprint, area_2d)) {
      return std::make_optional<size_t>(i);
    }
  }
  return std::nullopt;
}

static std::optional<size_t> getDuplicatedPointIdx(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
  const geometry_msgs::msg::Point & point)
{
  for (size_t i = 0; i < path.points.size(); i++) {
    const auto & p = path.points.at(i).point.pose.position;

    constexpr double min_dist = 0.001;
    if (tier4_autoware_utils::calcDistance2d(p, point) < min_dist) {
      return i;
    }
  }

  return std::nullopt;
}

static std::optional<size_t> insertPointIndex(
  const geometry_msgs::msg::Pose & in_pose,
  autoware_auto_planning_msgs::msg::PathWithLaneId * inout_path,
  const double ego_nearest_dist_threshold, const double ego_nearest_yaw_threshold)
{
  const auto duplicate_idx_opt = getDuplicatedPointIdx(*inout_path, in_pose.position);
  if (duplicate_idx_opt) {
    return duplicate_idx_opt.value();
  }

  const size_t closest_idx = motion_utils::findFirstNearestIndexWithSoftConstraints(
    inout_path->points, in_pose, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
  // vector.insert(i) inserts element on the left side of v[i]
  // the velocity need to be zero order hold(from prior point)
  int insert_idx = closest_idx;
  autoware_auto_planning_msgs::msg::PathPointWithLaneId inserted_point =
    inout_path->points.at(closest_idx);
  if (planning_utils::isAheadOf(in_pose, inout_path->points.at(closest_idx).point.pose)) {
    ++insert_idx;
  } else {
    // copy with velocity from prior point
    const size_t prior_ind = closest_idx > 0 ? closest_idx - 1 : 0;
    inserted_point.point.longitudinal_velocity_mps =
      inout_path->points.at(prior_ind).point.longitudinal_velocity_mps;
  }
  inserted_point.point.pose = in_pose;

  auto it = inout_path->points.begin() + insert_idx;
  inout_path->points.insert(it, inserted_point);

  return insert_idx;
}

std::optional<std::pair<size_t, size_t>> BlindSpotModule::generateStopLine(
  const InterpolatedPathInfo & interpolated_path_info,
  autoware_auto_planning_msgs::msg::PathWithLaneId * path) const
{
  const int margin_idx_dist =
    std::ceil(planner_param_.stop_line_margin / interpolated_path_info.ds);

  const auto first_conflict_idx_ip_opt = getFirstPointInsidePolygonByFootprint(
    first_conflicting_lanelet_.value().polygon2d(), interpolated_path_info,
    planner_data_->vehicle_info_.createFootprint(0.0, 0.0),
    planner_data_->vehicle_info_.max_longitudinal_offset_m);
  if (!first_conflict_idx_ip_opt) {
    return std::nullopt;
  }
  const int first_conflict_idx_ip = static_cast<int>(first_conflict_idx_ip_opt.value());

  const size_t stop_idx_default_ip =
    static_cast<size_t>(std::max(first_conflict_idx_ip - margin_idx_dist, 0));
  const size_t stop_idx_critical_ip = static_cast<size_t>(first_conflict_idx_ip);

  /* insert stop_point to use interpolated path*/
  const auto & path_ip = interpolated_path_info.path;
  const auto stopline_idx_default_opt = insertPointIndex(
    path_ip.points.at(stop_idx_default_ip).point.pose, path,
    planner_data_->ego_nearest_dist_threshold, planner_data_->ego_nearest_yaw_threshold);
  const auto stopline_idx_critical_opt = insertPointIndex(
    path_ip.points.at(stop_idx_critical_ip).point.pose, path,
    planner_data_->ego_nearest_dist_threshold, planner_data_->ego_nearest_yaw_threshold);

  if (!stopline_idx_default_opt || !stopline_idx_critical_opt) {
    return std::nullopt;
  }
  return std::make_pair(stopline_idx_default_opt.value(), stopline_idx_critical_opt.value());
}

void BlindSpotModule::cutPredictPathWithDuration(
  autoware_auto_perception_msgs::msg::PredictedObjects * objects_ptr, const double time_thr) const
{
  const rclcpp::Time current_time = clock_->now();

  for (auto & object : objects_ptr->objects) {                         // each objects
    for (auto & predicted_path : object.kinematics.predicted_paths) {  // each predicted paths
      const auto origin_path = predicted_path;
      predicted_path.path.clear();

      for (size_t k = 0; k < origin_path.path.size(); ++k) {  // each path points
        const auto & predicted_pose = origin_path.path.at(k);
        const auto predicted_time =
          rclcpp::Time(objects_ptr->header.stamp) +
          rclcpp::Duration(origin_path.time_step) * static_cast<double>(k);
        if ((predicted_time - current_time).seconds() < time_thr) {
          predicted_path.path.push_back(predicted_pose);
        }
      }
    }
  }
}

bool BlindSpotModule::checkObstacleInBlindSpot(
  lanelet::LaneletMapConstPtr lanelet_map_ptr, lanelet::routing::RoutingGraphPtr routing_graph_ptr,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
  const autoware_auto_perception_msgs::msg::PredictedObjects::ConstSharedPtr objects_ptr,
  const int closest_idx, const geometry_msgs::msg::Pose & stop_line_pose)
{
  /* get detection area */
  if (turn_direction_ == TurnDirection::INVALID) {
    RCLCPP_WARN(logger_, "blind spot detector is running, turn_direction_ = not right or left.");
    return false;
  }

  const auto areas_opt = generateBlindSpotPolygons(
    lanelet_map_ptr, routing_graph_ptr, path, closest_idx, stop_line_pose);
  if (areas_opt) {
    const auto & detection_areas = areas_opt.value().detection_areas;
    const auto & conflict_areas = areas_opt.value().conflict_areas;
    const auto & opposite_detection_areas = areas_opt.value().opposite_detection_areas;
    const auto & opposite_conflict_areas = areas_opt.value().opposite_conflict_areas;
    debug_data_.detection_areas = detection_areas;
    debug_data_.conflict_areas = conflict_areas;
    debug_data_.detection_areas.insert(
      debug_data_.detection_areas.end(), opposite_detection_areas.begin(),
      opposite_detection_areas.end());
    debug_data_.conflict_areas.insert(
      debug_data_.conflict_areas.end(), opposite_conflict_areas.begin(),
      opposite_conflict_areas.end());

    autoware_auto_perception_msgs::msg::PredictedObjects objects = *objects_ptr;
    cutPredictPathWithDuration(&objects, planner_param_.max_future_movement_time);

    // check objects in blind spot areas
    for (const auto & object : objects.objects) {
      if (!isTargetObjectType(object)) {
        continue;
      }

      // right direction
      const bool exist_in_right_detection_area =
        std::any_of(detection_areas.begin(), detection_areas.end(), [&object](const auto & area) {
          return bg::within(
            to_bg2d(object.kinematics.initial_pose_with_covariance.pose.position),
            lanelet::utils::to2D(area));
        });
      // opposite direction
      const bool exist_in_opposite_detection_area = std::any_of(
        opposite_detection_areas.begin(), opposite_detection_areas.end(),
        [&object](const auto & area) {
          return bg::within(
            to_bg2d(object.kinematics.initial_pose_with_covariance.pose.position),
            lanelet::utils::to2D(area));
        });
      const bool exist_in_detection_area =
        exist_in_right_detection_area || exist_in_opposite_detection_area;
      if (!exist_in_detection_area) {
        continue;
      }
      const bool exist_in_right_conflict_area =
        isPredictedPathInArea(object, conflict_areas, planner_data_->current_odometry->pose);
      const bool exist_in_opposite_conflict_area = isPredictedPathInArea(
        object, opposite_conflict_areas, planner_data_->current_odometry->pose);
      const bool exist_in_conflict_area =
        exist_in_right_conflict_area || exist_in_opposite_conflict_area;
      if (exist_in_detection_area && exist_in_conflict_area) {
        debug_data_.conflicting_targets.objects.push_back(object);
        setObjectsOfInterestData(
          object.kinematics.initial_pose_with_covariance.pose, object.shape, ColorName::RED);
        return true;
      }
    }
    return false;
  } else {
    return false;
  }
}

bool BlindSpotModule::isPredictedPathInArea(
  const autoware_auto_perception_msgs::msg::PredictedObject & object,
  const std::vector<lanelet::CompoundPolygon3d> & areas, geometry_msgs::msg::Pose ego_pose) const
{
  const auto ego_yaw = tf2::getYaw(ego_pose.orientation);
  const auto threshold_yaw_diff = planner_param_.threshold_yaw_diff;
  // NOTE: iterating all paths including those of low confidence
  return std::any_of(
    areas.begin(), areas.end(), [&object, &ego_yaw, &threshold_yaw_diff](const auto & area) {
      const auto area_2d = lanelet::utils::to2D(area);
      return std::any_of(
        object.kinematics.predicted_paths.begin(), object.kinematics.predicted_paths.end(),
        [&area_2d, &ego_yaw, &threshold_yaw_diff](const auto & path) {
          return std::any_of(
            path.path.begin(), path.path.end(),
            [&area_2d, &ego_yaw, &threshold_yaw_diff](const auto & point) {
              const auto is_in_area = bg::within(to_bg2d(point.position), area_2d);
              const auto match_yaw =
                std::fabs(ego_yaw - tf2::getYaw(point.orientation)) < threshold_yaw_diff;
              return is_in_area && match_yaw;
            });
        });
    });
}

lanelet::ConstLanelet BlindSpotModule::generateHalfLanelet(
  const lanelet::ConstLanelet lanelet) const
{
  lanelet::Points3d lefts, rights;

  const double offset = (turn_direction_ == TurnDirection::LEFT)
                          ? planner_param_.ignore_width_from_center_line
                          : -planner_param_.ignore_width_from_center_line;
  const auto offset_centerline = lanelet::utils::getCenterlineWithOffset(lanelet, offset);

  const auto original_left_bound =
    (turn_direction_ == TurnDirection::LEFT) ? lanelet.leftBound() : offset_centerline;
  const auto original_right_bound =
    (turn_direction_ == TurnDirection::LEFT) ? offset_centerline : lanelet.rightBound();

  for (const auto & pt : original_left_bound) {
    lefts.push_back(lanelet::Point3d(pt));
  }
  for (const auto & pt : original_right_bound) {
    rights.push_back(lanelet::Point3d(pt));
  }
  const auto left_bound = lanelet::LineString3d(lanelet::InvalId, std::move(lefts));
  const auto right_bound = lanelet::LineString3d(lanelet::InvalId, std::move(rights));
  auto half_lanelet = lanelet::Lanelet(lanelet::InvalId, left_bound, right_bound);
  const auto centerline = lanelet::utils::generateFineCenterline(half_lanelet, 5.0);
  half_lanelet.setCenterline(centerline);
  return half_lanelet;
}

lanelet::ConstLanelet BlindSpotModule::generateExtendedAdjacentLanelet(
  const lanelet::ConstLanelet lanelet, const TurnDirection direction) const
{
  const auto centerline = lanelet.centerline2d();
  const auto width =
    boost::geometry::area(lanelet.polygon2d().basicPolygon()) / boost::geometry::length(centerline);
  const double extend_width = std::min<double>(planner_param_.adjacent_extend_width, width);
  const auto left_bound_ =
    direction == TurnDirection::LEFT
      ? lanelet::utils::getCenterlineWithOffset(lanelet, -width / 2 + extend_width)
      : lanelet.leftBound();
  const auto right_bound_ =
    direction == TurnDirection::RIGHT
      ? lanelet::utils::getCenterlineWithOffset(lanelet, width / 2 - extend_width)
      : lanelet.rightBound();
  lanelet::Points3d lefts, rights;
  for (const auto & pt : left_bound_) {
    lefts.push_back(lanelet::Point3d(pt));
  }
  for (const auto & pt : right_bound_) {
    rights.push_back(lanelet::Point3d(pt));
  }
  const auto left_bound = lanelet::LineString3d(lanelet::InvalId, std::move(lefts));
  const auto right_bound = lanelet::LineString3d(lanelet::InvalId, std::move(rights));
  auto new_lanelet = lanelet::Lanelet(lanelet::InvalId, left_bound, right_bound);
  const auto new_centerline = lanelet::utils::generateFineCenterline(new_lanelet, 5.0);
  new_lanelet.setCenterline(new_centerline);
  return new_lanelet;
}

lanelet::ConstLanelet BlindSpotModule::generateExtendedOppositeAdjacentLanelet(
  const lanelet::ConstLanelet lanelet, const TurnDirection direction) const
{
  const auto centerline = lanelet.centerline2d();
  const auto width =
    boost::geometry::area(lanelet.polygon2d().basicPolygon()) / boost::geometry::length(centerline);
  const double extend_width =
    std::min<double>(planner_param_.opposite_adjacent_extend_width, width);
  const auto left_bound_ =
    direction == TurnDirection::RIGHT
      ? lanelet.rightBound().invert()
      : lanelet::utils::getCenterlineWithOffset(lanelet.invert(), -width / 2 + extend_width);
  const auto right_bound_ =
    direction == TurnDirection::RIGHT
      ? lanelet::utils::getCenterlineWithOffset(lanelet.invert(), width / 2 - extend_width)
      : lanelet.leftBound().invert();
  lanelet::Points3d lefts, rights;
  for (const auto & pt : left_bound_) {
    lefts.push_back(lanelet::Point3d(pt));
  }
  for (const auto & pt : right_bound_) {
    rights.push_back(lanelet::Point3d(pt));
  }
  const auto left_bound = lanelet::LineString3d(lanelet::InvalId, std::move(lefts));
  const auto right_bound = lanelet::LineString3d(lanelet::InvalId, std::move(rights));
  auto new_lanelet = lanelet::Lanelet(lanelet::InvalId, left_bound, right_bound);
  const auto new_centerline = lanelet::utils::generateFineCenterline(new_lanelet, 5.0);
  new_lanelet.setCenterline(new_centerline);
  return new_lanelet;
}

std::optional<BlindSpotPolygons> BlindSpotModule::generateBlindSpotPolygons(
  lanelet::LaneletMapConstPtr lanelet_map_ptr,
  [[maybe_unused]] lanelet::routing::RoutingGraphPtr routing_graph_ptr,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const int closest_idx,
  const geometry_msgs::msg::Pose & stop_line_pose) const
{
  std::vector<int64_t> lane_ids;
  lanelet::ConstLanelets blind_spot_lanelets;
  lanelet::ConstLanelets blind_spot_opposite_lanelets;
  /* get lane ids until intersection */
  for (const auto & point : path.points) {
    bool found_intersection_lane = false;
    for (const auto lane_id : point.lane_ids) {
      if (lane_id == lane_id_) {
        found_intersection_lane = true;
        lane_ids.push_back(lane_id);
        break;
      }
      // make lane_ids unique
      if (std::find(lane_ids.begin(), lane_ids.end(), lane_id) == lane_ids.end()) {
        lane_ids.push_back(lane_id);
      }
    }
    if (found_intersection_lane) break;
  }

  for (size_t i = 0; i < lane_ids.size(); ++i) {
    const auto half_lanelet =
      generateHalfLanelet(lanelet_map_ptr->laneletLayer.get(lane_ids.at(i)));
    blind_spot_lanelets.push_back(half_lanelet);
  }

  // additional detection area on left/right side
  lanelet::ConstLanelets adjacent_lanelets;
  lanelet::ConstLanelets opposite_adjacent_lanelets;
  for (const auto i : lane_ids) {
    const auto lane = lanelet_map_ptr->laneletLayer.get(i);
    const auto adj =
      turn_direction_ == TurnDirection::LEFT
        ? (routing_graph_ptr->adjacentLeft(lane))
        : (turn_direction_ == TurnDirection::RIGHT ? (routing_graph_ptr->adjacentRight(lane))
                                                   : boost::none);
    const std::optional<lanelet::ConstLanelet> opposite_adj =
      [&]() -> std::optional<lanelet::ConstLanelet> {
      if (!!adj) {
        return std::nullopt;
      }
      if (turn_direction_ == TurnDirection::LEFT) {
        // this should exist in right-hand traffic
        const auto adjacent_lanes =
          lanelet_map_ptr->laneletLayer.findUsages(lane.leftBound().invert());
        if (adjacent_lanes.empty()) {
          return std::nullopt;
        }
        return adjacent_lanes.front();
      }
      if (turn_direction_ == TurnDirection::RIGHT) {
        // this should exist in left-hand traffic
        const auto adjacent_lanes =
          lanelet_map_ptr->laneletLayer.findUsages(lane.rightBound().invert());
        if (adjacent_lanes.empty()) {
          return std::nullopt;
        }
        return adjacent_lanes.front();
      } else {
        return std::nullopt;
      }
    }();
    if (adj) {
      const auto half_lanelet = generateExtendedAdjacentLanelet(adj.value(), turn_direction_);
      adjacent_lanelets.push_back(half_lanelet);
    }
    if (opposite_adj) {
      const auto half_lanelet =
        generateExtendedOppositeAdjacentLanelet(opposite_adj.value(), turn_direction_);
      opposite_adjacent_lanelets.push_back(half_lanelet);
    }
  }

  const auto current_arc_ego =
    lanelet::utils::getArcCoordinates(blind_spot_lanelets, path.points[closest_idx].point.pose)
      .length;
  const auto stop_line_arc_ego =
    lanelet::utils::getArcCoordinates(blind_spot_lanelets, stop_line_pose).length;
  const auto detection_area_start_length_ego = stop_line_arc_ego - planner_param_.backward_length;
  if (detection_area_start_length_ego < current_arc_ego && current_arc_ego < stop_line_arc_ego) {
    BlindSpotPolygons blind_spot_polygons;
    auto conflict_area_ego = lanelet::utils::getPolygonFromArcLength(
      blind_spot_lanelets, current_arc_ego, stop_line_arc_ego);
    auto detection_area_ego = lanelet::utils::getPolygonFromArcLength(
      blind_spot_lanelets, detection_area_start_length_ego, stop_line_arc_ego);
    blind_spot_polygons.conflict_areas.emplace_back(std::move(conflict_area_ego));
    blind_spot_polygons.detection_areas.emplace_back(std::move(detection_area_ego));
    // additional detection area on left/right side
    if (!adjacent_lanelets.empty()) {
      const auto stop_line_arc_adj = lanelet::utils::getLaneletLength3d(adjacent_lanelets);
      const auto current_arc_adj = stop_line_arc_adj - (stop_line_arc_ego - current_arc_ego);
      const auto detection_area_start_length_adj =
        stop_line_arc_adj - planner_param_.backward_length;
      auto conflicting_area_adj = lanelet::utils::getPolygonFromArcLength(
        adjacent_lanelets, current_arc_adj, stop_line_arc_adj);
      auto detection_area_adj = lanelet::utils::getPolygonFromArcLength(
        adjacent_lanelets, detection_area_start_length_adj, stop_line_arc_adj);
      blind_spot_polygons.conflict_areas.emplace_back(std::move(conflicting_area_adj));
      blind_spot_polygons.detection_areas.emplace_back(std::move(detection_area_adj));
    }
    // additional detection area on left/right opposite lane side
    if (!opposite_adjacent_lanelets.empty()) {
      const auto stop_line_arc_opposite_adj =
        lanelet::utils::getLaneletLength3d(opposite_adjacent_lanelets);
      const auto current_arc_opposite_adj =
        stop_line_arc_opposite_adj - (stop_line_arc_ego - current_arc_ego);
      const auto detection_area_start_length_opposite_adj =
        stop_line_arc_opposite_adj - planner_param_.backward_length;
      auto conflicting_area_opposite_adj = lanelet::utils::getPolygonFromArcLength(
        opposite_adjacent_lanelets, current_arc_opposite_adj, stop_line_arc_opposite_adj);
      auto detection_area_opposite_adj = lanelet::utils::getPolygonFromArcLength(
        opposite_adjacent_lanelets, detection_area_start_length_opposite_adj,
        stop_line_arc_opposite_adj);
      blind_spot_polygons.opposite_conflict_areas.emplace_back(
        std::move(conflicting_area_opposite_adj));
      blind_spot_polygons.opposite_detection_areas.emplace_back(
        std::move(detection_area_opposite_adj));
    }
    return blind_spot_polygons;
  } else {
    return std::nullopt;
  }
}

bool BlindSpotModule::isTargetObjectType(
  const autoware_auto_perception_msgs::msg::PredictedObject & object) const
{
  if (
    object.classification.at(0).label ==
      autoware_auto_perception_msgs::msg::ObjectClassification::BICYCLE ||
    object.classification.at(0).label ==
      autoware_auto_perception_msgs::msg::ObjectClassification::PEDESTRIAN ||
    object.classification.at(0).label ==
      autoware_auto_perception_msgs::msg::ObjectClassification::MOTORCYCLE) {
    return true;
  }
  return false;
}

}  // namespace behavior_velocity_planner
