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

#include <lanelet2_extension/regulatory_elements/road_marking.hpp>
#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <scene_module/intersection/scene_intersection.hpp>
#include <scene_module/intersection/util.hpp>
#include <utilization/boost_geometry_helper.hpp>
#include <utilization/path_utilization.hpp>
#include <utilization/trajectory_utils.hpp>
#include <utilization/util.hpp>

#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>

#include <algorithm>
#include <memory>
#include <vector>

namespace behavior_velocity_planner
{
namespace bg = boost::geometry;

static geometry_msgs::msg::Pose getObjectPoseWithVelocityDirection(
  const autoware_auto_perception_msgs::msg::PredictedObjectKinematics & obj_state)
{
  if (obj_state.initial_twist_with_covariance.twist.linear.x >= 0) {
    return obj_state.initial_pose_with_covariance.pose;
  }

  // When the object velocity is negative, invert orientation (yaw)
  auto obj_pose = obj_state.initial_pose_with_covariance.pose;
  double yaw, pitch, roll;
  tf2::getEulerYPR(obj_pose.orientation, yaw, pitch, roll);
  tf2::Quaternion inv_q;
  inv_q.setRPY(roll, pitch, yaw + M_PI);
  obj_pose.orientation = tf2::toMsg(inv_q);
  return obj_pose;
}

IntersectionModule::IntersectionModule(
  const int64_t module_id, const int64_t lane_id, std::shared_ptr<const PlannerData> planner_data,
  const PlannerParam & planner_param, const rclcpp::Logger logger,
  const rclcpp::Clock::SharedPtr clock)
: SceneModuleInterface(module_id, logger, clock), lane_id_(lane_id), is_go_out_(false)
{
  planner_param_ = planner_param;

  const auto & assigned_lanelet =
    planner_data->route_handler_->getLaneletMapPtr()->laneletLayer.get(lane_id);
  turn_direction_ = assigned_lanelet.attributeOr("turn_direction", "else");
  has_traffic_light_ =
    !(assigned_lanelet.regulatoryElementsAs<const lanelet::TrafficLight>().empty());
  state_machine_.setMarginTime(planner_param_.state_transit_margin_time);
}

bool IntersectionModule::modifyPathVelocity(PathWithLaneId * path, StopReason * stop_reason)
{
  RCLCPP_DEBUG(logger_, "===== plan start =====");

  const bool external_go = isTargetExternalInputStatus(tier4_api_msgs::msg::IntersectionStatus::GO);
  const bool external_stop =
    isTargetExternalInputStatus(tier4_api_msgs::msg::IntersectionStatus::STOP);
  const StateMachine::State current_state = state_machine_.getState();

  debug_data_ = DebugData();
  debug_data_.path_raw = *path;

  *stop_reason = planning_utils::initializeStopReason(StopReason::INTERSECTION);

  RCLCPP_DEBUG(
    logger_, "lane_id = %ld, state = %s", lane_id_, StateMachine::toString(current_state).c_str());

  /* get current pose */
  const geometry_msgs::msg::PoseStamped current_pose = planner_data_->current_pose;
  const double current_vel = planner_data_->current_velocity->twist.linear.x;

  /* get lanelet map */
  const auto lanelet_map_ptr = planner_data_->route_handler_->getLaneletMapPtr();
  const auto routing_graph_ptr = planner_data_->route_handler_->getRoutingGraphPtr();
  const auto & assigned_lanelet =
    planner_data_->route_handler_->getLaneletMapPtr()->laneletLayer.get(lane_id_);
  const std::string turn_direction = assigned_lanelet.attributeOr("turn_direction", "else");

  /* get detection area*/
  /* dynamically change detection area based on tl_arrow_solid_on */
  [[maybe_unused]] const bool has_tl = util::hasAssociatedTrafficLight(assigned_lanelet);
  const bool tl_arrow_solid_on =
    util::isTrafficLightArrowActivated(assigned_lanelet, planner_data_->traffic_light_id_map);
  auto && [detection_lanelets, conflicting_lanelets] = util::getObjectiveLanelets(
    lanelet_map_ptr, routing_graph_ptr, lane_id_, planner_param_.detection_area_length,
    tl_arrow_solid_on);
  const std::vector<lanelet::CompoundPolygon3d> detection_area =
    util::getPolygon3dFromLanelets(detection_lanelets, planner_param_.detection_area_length);
  const std::vector<lanelet::CompoundPolygon3d> conflicting_area =
    util::getPolygon3dFromLanelets(conflicting_lanelets);
  debug_data_.detection_area = detection_area;

  /* get intersection area */
  const auto intersection_area = util::getIntersectionArea(assigned_lanelet, lanelet_map_ptr);
  if (intersection_area) {
    const auto intersection_area_2d = intersection_area.value();
    debug_data_.intersection_area = toGeomPoly(intersection_area_2d);
  }

  /* get adjacent lanelets */
  const auto adjacent_lanelets =
    util::extendedAdjacentDirectionLanes(lanelet_map_ptr, routing_graph_ptr, assigned_lanelet);
  debug_data_.adjacent_area = util::getPolygon3dFromLanelets(adjacent_lanelets);

  /* set stop lines for base_link */
  const auto [stuck_line_idx_opt, stop_lines_idx_opt] = util::generateStopLine(
    lane_id_, detection_area, conflicting_area, planner_data_, planner_param_.stop_line_margin,
    planner_param_.keep_detection_line_margin, planner_param_.use_stuck_stopline, path, *path,
    logger_.get_child("util"), clock_);
  if (!stuck_line_idx_opt.has_value()) {
    // returns here if path is not intersecting with conflicting areas
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      logger_, *clock_, 1000 /* ms */, "setStopLineIdx for stuck line fail");
    RCLCPP_DEBUG(logger_, "===== plan end =====");
    setSafe(true);
    setDistance(std::numeric_limits<double>::lowest());
    return false;
  }
  const auto stuck_line_idx = stuck_line_idx_opt.value();

  /* calc closest index */
  const auto closest_idx_opt =
    motion_utils::findNearestIndex(path->points, current_pose.pose, 3.0, M_PI_4);
  if (!closest_idx_opt) {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      logger_, *clock_, 1000 /* ms */, "motion_utils::findNearestIndex fail");
    RCLCPP_DEBUG(logger_, "===== plan end =====");
    setSafe(true);
    setDistance(std::numeric_limits<double>::lowest());
    return false;
  }
  const size_t closest_idx = closest_idx_opt.get();

  if (stop_lines_idx_opt.has_value()) {
    const auto & stop_lines = stop_lines_idx_opt.value();
    const size_t stop_line_idx = stop_lines.stop_line;
    const size_t pass_judge_line_idx = stop_lines.pass_judge_line;
    const size_t keep_detection_line_idx = stop_lines.keep_detection_line;

    const bool is_over_pass_judge_line =
      util::isOverTargetIndex(*path, closest_idx, current_pose.pose, pass_judge_line_idx);
    const bool is_before_keep_detection_line =
      util::isBeforeTargetIndex(*path, closest_idx, current_pose.pose, keep_detection_line_idx);
    const bool keep_detection = is_before_keep_detection_line &&
                                std::fabs(current_vel) < planner_param_.keep_detection_vel_thr;

    if (is_over_pass_judge_line && keep_detection) {
      // in case ego could not stop exactly before the stop line, but with some overshoot,
      // keep detection within some margin under low velocity threshold
      RCLCPP_DEBUG(
        logger_,
        "over the pass judge line, but before keep detection line and low speed, "
        "continue planning");
      // no return here, keep planning
    } else if (is_over_pass_judge_line && is_go_out_ && !external_stop) {
      RCLCPP_DEBUG(logger_, "over the keep_detection line and not low speed. no plan needed.");
      RCLCPP_DEBUG(logger_, "===== plan end =====");
      setSafe(true);
      setDistance(motion_utils::calcSignedArcLength(
        path->points, planner_data_->current_pose.pose.position,
        path->points.at(stop_line_idx).point.pose.position));
      // no plan needed.
      return true;
    }
  }

  /* get dynamic object */
  const auto objects_ptr = planner_data_->predicted_objects;

  /* calculate final stop lines */
  bool is_entry_prohibited = false;
  const double detect_length =
    planner_param_.stuck_vehicle_detect_dist + planner_data_->vehicle_info_.vehicle_length_m;
  const auto stuck_vehicle_detect_area = generateEgoIntersectionLanePolygon(
    lanelet_map_ptr, *path, closest_idx, stuck_line_idx, detect_length,
    planner_param_.stuck_vehicle_detect_dist);
  const bool is_stuck = checkStuckVehicleInIntersection(objects_ptr, stuck_vehicle_detect_area);
  int stop_line_idx_final = stuck_line_idx;
  int pass_judge_line_idx_final = stuck_line_idx;
  if (external_go) {
    is_entry_prohibited = false;
  } else if (external_stop) {
    is_entry_prohibited = true;
  } else if (is_stuck) {
    is_entry_prohibited = true;
    stop_line_idx_final = stuck_line_idx;
    pass_judge_line_idx_final = stuck_line_idx;
  } else {
    /* calculate dynamic collision around detection area */
    const bool has_collision = checkCollision(
      lanelet_map_ptr, *path, detection_lanelets, adjacent_lanelets, intersection_area, objects_ptr,
      closest_idx, stuck_vehicle_detect_area);
    is_entry_prohibited = has_collision;
    if (stop_lines_idx_opt.has_value()) {
      const auto & stop_lines_idx = stop_lines_idx_opt.value();
      stop_line_idx_final = stop_lines_idx.stop_line;
      pass_judge_line_idx_final = stop_lines_idx.pass_judge_line;
    } else {
      if (has_collision) {
        RCLCPP_ERROR(logger_, "generateStopLine() failed for detected objects");
        RCLCPP_DEBUG(logger_, "===== plan end =====");
        setSafe(true);
        setDistance(std::numeric_limits<double>::lowest());
        return false;
      } else {
        RCLCPP_DEBUG(logger_, "no need to stop");
        RCLCPP_DEBUG(logger_, "===== plan end =====");
        setSafe(true);
        setDistance(std::numeric_limits<double>::lowest());
        return true;
      }
    }
  }

  state_machine_.setStateWithMarginTime(
    is_entry_prohibited ? StateMachine::State::STOP : StateMachine::State::GO,
    logger_.get_child("state_machine"), *clock_);

  setSafe(state_machine_.getState() == StateMachine::State::GO);
  if (is_entry_prohibited) {
    setDistance(motion_utils::calcSignedArcLength(
      path->points, planner_data_->current_pose.pose.position,
      path->points.at(stop_line_idx_final).point.pose.position));
  } else {
    setDistance(std::numeric_limits<double>::lowest());
  }

  if (!isActivated()) {
    // if RTC says intersection entry is 'dangerous', insert stop_line(v == 0.0) in this block
    is_go_out_ = false;

    constexpr double v = 0.0;
    planning_utils::setVelocityFromIndex(stop_line_idx_final, v, path);
    debug_data_.stop_required = true;
    const double base_link2front = planner_data_->vehicle_info_.max_longitudinal_offset_m;
    debug_data_.stop_wall_pose =
      planning_utils::getAheadPose(stop_line_idx_final, base_link2front, *path);
    debug_data_.stop_point_pose = path->points.at(stop_line_idx_final).point.pose;
    debug_data_.judge_point_pose = path->points.at(pass_judge_line_idx_final).point.pose;

    /* get stop point and stop factor */
    tier4_planning_msgs::msg::StopFactor stop_factor;
    stop_factor.stop_pose = debug_data_.stop_point_pose;
    const auto stop_factor_conflict = planning_utils::toRosPoints(debug_data_.conflicting_targets);
    const auto stop_factor_stuck = planning_utils::toRosPoints(debug_data_.stuck_targets);
    stop_factor.stop_factor_points =
      planning_utils::concatVector(stop_factor_conflict, stop_factor_stuck);
    planning_utils::appendStopReason(stop_factor, stop_reason);

    RCLCPP_DEBUG(logger_, "not activated. stop at the line.");
    RCLCPP_DEBUG(logger_, "===== plan end =====");
    return true;
  }

  is_go_out_ = true;
  RCLCPP_DEBUG(logger_, "===== plan end =====");
  return true;
}

void IntersectionModule::cutPredictPathWithDuration(
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

bool IntersectionModule::checkCollision(
  lanelet::LaneletMapConstPtr lanelet_map_ptr,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
  const lanelet::ConstLanelets & detection_area_lanelets,
  const lanelet::ConstLanelets & adjacent_lanelets,
  const std::optional<Polygon2d> & intersection_area,
  const autoware_auto_perception_msgs::msg::PredictedObjects::ConstSharedPtr objects_ptr,
  const int closest_idx, const Polygon2d & stuck_vehicle_detect_area)
{
  using lanelet::utils::getArcCoordinates;
  using lanelet::utils::getPolygonFromArcLength;

  /* generate ego-lane polygon */
  const auto ego_poly =
    generateEgoIntersectionLanePolygon(lanelet_map_ptr, path, closest_idx, closest_idx, 0.0, 0.0);
  lanelet::ConstLanelets ego_lane_with_next_lane = getEgoLaneWithNextLane(lanelet_map_ptr, path);
  lanelet::ConstLanelet closest_lanelet;
  lanelet::utils::query::getClosestLanelet(
    ego_lane_with_next_lane, tier4_autoware_utils::getPose(path.points.at(closest_idx).point),
    &closest_lanelet);

  debug_data_.ego_lane_polygon = toGeomPoly(ego_poly);

  /* extract target objects */
  autoware_auto_perception_msgs::msg::PredictedObjects target_objects;
  target_objects.header = objects_ptr->header;
  for (const auto & object : objects_ptr->objects) {
    // ignore non-vehicle type objects, such as pedestrian.
    if (!isTargetCollisionVehicleType(object)) {
      continue;
    }

    // ignore vehicle in ego-lane && behind ego
    const auto object_pose = object.kinematics.initial_pose_with_covariance.pose;
    const bool is_in_ego_lane = bg::within(to_bg2d(object_pose.position), ego_poly);
    if (is_in_ego_lane) {
      if (!planning_utils::isAheadOf(object_pose, planner_data_->current_pose.pose)) {
        continue;
      }
      if (
        planner_param_.enable_front_car_decel_prediction &&
        checkFrontVehicleDeceleration(
          ego_lane_with_next_lane, closest_lanelet, stuck_vehicle_detect_area, object))
        return true;
    }

    // check direction of objects
    const auto object_direction = getObjectPoseWithVelocityDirection(object.kinematics);
    if (intersection_area) {
      const auto obj_poly = tier4_autoware_utils::toPolygon2d(object);
      const auto intersection_area_2d = intersection_area.value();
      const auto is_in_intersection_area = bg::within(obj_poly, intersection_area_2d);
      const auto is_in_adjacent_lanelets = checkAngleForTargetLanelets(
        object_direction, adjacent_lanelets, planner_param_.detection_area_margin);
      if (is_in_adjacent_lanelets) continue;
      if (is_in_intersection_area) {
        target_objects.objects.push_back(object);
      } else if (checkAngleForTargetLanelets(
                   object_direction, detection_area_lanelets,
                   planner_param_.detection_area_margin)) {
        target_objects.objects.push_back(object);
      }
    } else if (checkAngleForTargetLanelets(
                 object_direction, detection_area_lanelets, planner_param_.detection_area_margin)) {
      // intersection_area is not available, use detection_area_with_margin as before
      target_objects.objects.push_back(object);
    }
  }

  /* check collision between target_objects predicted path and ego lane */

  // cut the predicted path at passing_time
  const auto time_distance_array = calcIntersectionPassingTime(path, closest_idx, lane_id_);
  const double passing_time = time_distance_array.back().first;
  cutPredictPathWithDuration(&target_objects, passing_time);

  const auto closest_arc_coords = getArcCoordinates(
    ego_lane_with_next_lane, tier4_autoware_utils::getPose(path.points.at(closest_idx).point));
  const double distance_until_intersection =
    calcDistanceUntilIntersectionLanelet(lanelet_map_ptr, path, closest_idx);
  const double base_link2front = planner_data_->vehicle_info_.max_longitudinal_offset_m;

  // check collision between predicted_path and ego_area
  bool collision_detected = false;
  for (const auto & object : target_objects.objects) {
    bool has_collision = false;
    for (const auto & predicted_path : object.kinematics.predicted_paths) {
      if (predicted_path.confidence < planner_param_.min_predicted_path_confidence) {
        // ignore the predicted path with too low confidence
        continue;
      }

      std::vector<geometry_msgs::msg::Pose> predicted_poses;
      for (const auto & pose : predicted_path.path) {
        predicted_poses.push_back(pose);
      }
      has_collision = bg::intersects(ego_poly, to_bg2d(predicted_poses));
      if (has_collision) {
        const auto first_itr = std::adjacent_find(
          predicted_path.path.cbegin(), predicted_path.path.cend(),
          [&ego_poly](const auto & a, const auto & b) {
            return bg::intersects(ego_poly, LineString2d{to_bg2d(a), to_bg2d(b)});
          });
        const auto last_itr = std::adjacent_find(
          predicted_path.path.crbegin(), predicted_path.path.crend(),
          [&ego_poly](const auto & a, const auto & b) {
            return bg::intersects(ego_poly, LineString2d{to_bg2d(a), to_bg2d(b)});
          });
        const double ref_object_enter_time =
          static_cast<double>(first_itr - predicted_path.path.begin()) *
          rclcpp::Duration(predicted_path.time_step).seconds();
        auto start_time_distance_itr = time_distance_array.begin();
        if (ref_object_enter_time - planner_param_.collision_start_margin_time > 0) {
          start_time_distance_itr = std::lower_bound(
            time_distance_array.begin(), time_distance_array.end(),
            ref_object_enter_time - planner_param_.collision_start_margin_time,
            [](const auto & a, const double b) { return a.first < b; });
          if (start_time_distance_itr == time_distance_array.end()) {
            continue;
          }
        }
        const double ref_object_exit_time =
          static_cast<double>(last_itr.base() - predicted_path.path.begin()) *
          rclcpp::Duration(predicted_path.time_step).seconds();
        auto end_time_distance_itr = std::lower_bound(
          time_distance_array.begin(), time_distance_array.end(),
          ref_object_exit_time + planner_param_.collision_end_margin_time,
          [](const auto & a, const double b) { return a.first < b; });
        if (end_time_distance_itr == time_distance_array.end()) {
          end_time_distance_itr = time_distance_array.end() - 1;
        }
        const double start_arc_length = std::max(
          0.0, closest_arc_coords.length + (*start_time_distance_itr).second -
                 distance_until_intersection);
        const double end_arc_length = std::max(
          0.0, closest_arc_coords.length + (*end_time_distance_itr).second + base_link2front -
                 distance_until_intersection);
        const auto trimmed_ego_polygon =
          getPolygonFromArcLength(ego_lane_with_next_lane, start_arc_length, end_arc_length);

        Polygon2d polygon{};
        for (const auto & p : trimmed_ego_polygon) {
          polygon.outer().emplace_back(p.x(), p.y());
        }

        polygon.outer().emplace_back(polygon.outer().front());

        bg::correct(polygon);

        debug_data_.candidate_collision_ego_lane_polygon = toGeomPoly(polygon);

        for (auto itr = first_itr; itr != last_itr.base(); ++itr) {
          const auto footprint_polygon = tier4_autoware_utils::toPolygon2d(*itr, object.shape);
          debug_data_.candidate_collision_object_polygons.emplace_back(
            toGeomPoly(footprint_polygon));
          if (bg::intersects(polygon, footprint_polygon)) {
            collision_detected = true;
            break;
          }
        }
        if (collision_detected) {
          debug_data_.conflicting_targets.objects.push_back(object);
          break;
        }
      }
    }
  }

  return collision_detected;
}

Polygon2d IntersectionModule::generateEgoIntersectionLanePolygon(
  lanelet::LaneletMapConstPtr lanelet_map_ptr,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const int closest_idx,
  const int start_idx, const double extra_dist, const double ignore_dist) const
{
  using lanelet::utils::getArcCoordinates;
  using lanelet::utils::getLaneletLength3d;
  using lanelet::utils::getPolygonFromArcLength;
  using lanelet::utils::to2D;

  lanelet::ConstLanelets ego_lane_with_next_lane = getEgoLaneWithNextLane(lanelet_map_ptr, path);

  const auto start_arc_coords = getArcCoordinates(
    ego_lane_with_next_lane, tier4_autoware_utils::getPose(path.points.at(start_idx).point));

  const auto closest_arc_coords = getArcCoordinates(
    ego_lane_with_next_lane, tier4_autoware_utils::getPose(path.points.at(closest_idx).point));

  const double start_arc_length = start_arc_coords.length + ignore_dist < closest_arc_coords.length
                                    ? closest_arc_coords.length
                                    : start_arc_coords.length + ignore_dist;

  const double end_arc_length = getLaneletLength3d(ego_lane_with_next_lane.front()) + extra_dist;

  const auto target_polygon =
    to2D(getPolygonFromArcLength(ego_lane_with_next_lane, start_arc_length, end_arc_length))
      .basicPolygon();

  Polygon2d polygon{};

  if (target_polygon.empty()) {
    return polygon;
  }

  for (const auto & p : target_polygon) {
    polygon.outer().emplace_back(p.x(), p.y());
  }

  polygon.outer().emplace_back(polygon.outer().front());
  bg::correct(polygon);

  return polygon;
}

TimeDistanceArray IntersectionModule::calcIntersectionPassingTime(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const int closest_idx,
  const int objective_lane_id) const
{
  static constexpr double k_minimum_velocity = 1e-01;

  double dist_sum = 0.0;
  int assigned_lane_found = false;

  // crop intersection part of the path, and set the reference velocity to intersection_velocity for
  // ego's ttc
  PathWithLaneId reference_path;
  for (size_t i = closest_idx; i < path.points.size(); ++i) {
    auto reference_point = path.points.at(i);
    reference_point.point.longitudinal_velocity_mps = planner_param_.intersection_velocity;
    reference_path.points.push_back(reference_point);
    bool has_objective_lane_id = util::hasLaneId(path.points.at(i), objective_lane_id);
    if (assigned_lane_found && !has_objective_lane_id) {
      break;
    }
    assigned_lane_found = has_objective_lane_id;
  }
  if (!assigned_lane_found) {
    return {{0.0, 0.0}};  // has already passed the intersection.
  }

  // apply smoother to reference velocity
  PathWithLaneId smoothed_reference_path = reference_path;
  if (!smoothPath(reference_path, smoothed_reference_path, planner_data_)) {
    RCLCPP_WARN(logger_, "smoothPath failed");
  }

  // calculate when ego is going to reach each (interpolated) points on the path
  TimeDistanceArray time_distance_array{};
  dist_sum = 0.0;
  double passing_time = 0.0;
  time_distance_array.emplace_back(passing_time, dist_sum);
  for (size_t i = 1; i < smoothed_reference_path.points.size(); ++i) {
    const auto & p1 = smoothed_reference_path.points.at(i - 1);
    const auto & p2 = smoothed_reference_path.points.at(i);

    const double dist = tier4_autoware_utils::calcDistance2d(p1, p2);
    dist_sum += dist;

    // use average velocity between p1 and p2
    const double average_velocity =
      (p1.point.longitudinal_velocity_mps + p2.point.longitudinal_velocity_mps) / 2.0;
    passing_time +=
      (dist / std::max<double>(k_minimum_velocity, average_velocity));  // to avoid zero-division

    time_distance_array.emplace_back(passing_time, dist_sum);
  }
  RCLCPP_DEBUG(logger_, "intersection dist = %f, passing_time = %f", dist_sum, passing_time);

  return time_distance_array;
}

bool IntersectionModule::checkStuckVehicleInIntersection(
  const autoware_auto_perception_msgs::msg::PredictedObjects::ConstSharedPtr objects_ptr,
  const Polygon2d & stuck_vehicle_detect_area) const
{
  debug_data_.stuck_vehicle_detect_area = toGeomPoly(stuck_vehicle_detect_area);

  for (const auto & object : objects_ptr->objects) {
    if (!isTargetStuckVehicleType(object)) {
      continue;  // not target vehicle type
    }
    const auto obj_v = std::fabs(object.kinematics.initial_twist_with_covariance.twist.linear.x);
    if (obj_v > planner_param_.stuck_vehicle_vel_thr) {
      continue;  // not stop vehicle
    }

    // check if the footprint is in the stuck detect area
    const auto obj_footprint = tier4_autoware_utils::toPolygon2d(object);
    const bool is_in_stuck_area = !bg::disjoint(obj_footprint, stuck_vehicle_detect_area);
    if (is_in_stuck_area) {
      RCLCPP_DEBUG(logger_, "stuck vehicle found.");
      debug_data_.stuck_targets.objects.push_back(object);
      return true;
    }
  }
  return false;
}

bool IntersectionModule::isTargetCollisionVehicleType(
  const autoware_auto_perception_msgs::msg::PredictedObject & object) const
{
  if (
    object.classification.at(0).label ==
      autoware_auto_perception_msgs::msg::ObjectClassification::CAR ||
    object.classification.at(0).label ==
      autoware_auto_perception_msgs::msg::ObjectClassification::BUS ||
    object.classification.at(0).label ==
      autoware_auto_perception_msgs::msg::ObjectClassification::TRUCK ||
    object.classification.at(0).label ==
      autoware_auto_perception_msgs::msg::ObjectClassification::TRAILER ||
    object.classification.at(0).label ==
      autoware_auto_perception_msgs::msg::ObjectClassification::MOTORCYCLE ||
    object.classification.at(0).label ==
      autoware_auto_perception_msgs::msg::ObjectClassification::BICYCLE) {
    return true;
  }
  return false;
}

bool IntersectionModule::isTargetStuckVehicleType(
  const autoware_auto_perception_msgs::msg::PredictedObject & object) const
{
  if (
    object.classification.at(0).label ==
      autoware_auto_perception_msgs::msg::ObjectClassification::CAR ||
    object.classification.at(0).label ==
      autoware_auto_perception_msgs::msg::ObjectClassification::BUS ||
    object.classification.at(0).label ==
      autoware_auto_perception_msgs::msg::ObjectClassification::TRUCK ||
    object.classification.at(0).label ==
      autoware_auto_perception_msgs::msg::ObjectClassification::TRAILER ||
    object.classification.at(0).label ==
      autoware_auto_perception_msgs::msg::ObjectClassification::MOTORCYCLE) {
    return true;
  }
  return false;
}

bool IntersectionModule::isTargetExternalInputStatus(const int target_status)
{
  return planner_data_->external_intersection_status_input &&
         planner_data_->external_intersection_status_input.get().status == target_status &&
         (clock_->now() - planner_data_->external_intersection_status_input.get().header.stamp)
             .seconds() < planner_param_.external_input_timeout;
}

bool IntersectionModule::checkAngleForTargetLanelets(
  const geometry_msgs::msg::Pose & pose, const lanelet::ConstLanelets & target_lanelets,
  const double margin)
{
  for (const auto & ll : target_lanelets) {
    if (!lanelet::utils::isInLanelet(pose, ll, margin)) {
      continue;
    }
    const double ll_angle = lanelet::utils::getLaneletAngle(ll, pose.position);
    const double pose_angle = tf2::getYaw(pose.orientation);
    const double angle_diff = tier4_autoware_utils::normalizeRadian(ll_angle - pose_angle);
    if (std::fabs(angle_diff) < planner_param_.detection_area_angle_thr) {
      return true;
    }
  }
  return false;
}

lanelet::ConstLanelets IntersectionModule::getEgoLaneWithNextLane(
  lanelet::LaneletMapConstPtr lanelet_map_ptr,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path) const
{
  const auto & assigned_lanelet = lanelet_map_ptr->laneletLayer.get(lane_id_);
  const auto last_itr =
    std::find_if(path.points.crbegin(), path.points.crend(), [this](const auto & p) {
      return std::find(p.lane_ids.begin(), p.lane_ids.end(), lane_id_) != p.lane_ids.end();
    });
  lanelet::ConstLanelets ego_lane_with_next_lane;
  if (last_itr.base() != path.points.end()) {
    const auto & next_lanelet =
      lanelet_map_ptr->laneletLayer.get((*last_itr.base()).lane_ids.front());
    ego_lane_with_next_lane = {assigned_lanelet, next_lanelet};
  } else {
    ego_lane_with_next_lane = {assigned_lanelet};
  }
  return ego_lane_with_next_lane;
}

double IntersectionModule::calcDistanceUntilIntersectionLanelet(
  lanelet::LaneletMapConstPtr lanelet_map_ptr,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const size_t closest_idx) const
{
  const auto & assigned_lanelet = lanelet_map_ptr->laneletLayer.get(lane_id_);
  const auto intersection_first_itr =
    std::find_if(path.points.cbegin(), path.points.cend(), [this](const auto & p) {
      return std::find(p.lane_ids.begin(), p.lane_ids.end(), lane_id_) != p.lane_ids.end();
    });
  if (
    intersection_first_itr == path.points.begin() || intersection_first_itr == path.points.end()) {
    return 0.0;
  }
  const auto dst_idx = std::distance(path.points.begin(), intersection_first_itr) - 1;

  if (closest_idx > static_cast<size_t>(dst_idx)) {
    return 0.0;
  }

  double distance = std::abs(motion_utils::calcSignedArcLength(path.points, closest_idx, dst_idx));
  const auto & lane_first_point = assigned_lanelet.centerline2d().front();
  distance += std::hypot(
    path.points.at(dst_idx).point.pose.position.x - lane_first_point.x(),
    path.points.at(dst_idx).point.pose.position.y - lane_first_point.y());
  return distance;
}

bool IntersectionModule::checkFrontVehicleDeceleration(
  lanelet::ConstLanelets & ego_lane_with_next_lane, lanelet::ConstLanelet & closest_lanelet,
  const Polygon2d & stuck_vehicle_detect_area,
  const autoware_auto_perception_msgs::msg::PredictedObject & object) const
{
  const auto & object_pose = object.kinematics.initial_pose_with_covariance.pose;
  // consider vehicle in ego-lane && in front of ego
  const auto lon_vel = object.kinematics.initial_twist_with_covariance.twist.linear.x;
  const double object_decel = planner_param_.assumed_front_car_decel;  // NOTE: this is positive
  const double stopping_distance = lon_vel * lon_vel / (2 * object_decel);

  std::vector<geometry_msgs::msg::Point> center_points;
  for (auto && p : ego_lane_with_next_lane[0].centerline())
    center_points.push_back(std::move(lanelet::utils::conversion::toGeomMsgPt(p)));
  for (auto && p : ego_lane_with_next_lane[1].centerline())
    center_points.push_back(std::move(lanelet::utils::conversion::toGeomMsgPt(p)));
  const double lat_offset =
    std::fabs(motion_utils::calcLateralOffset(center_points, object_pose.position));
  // get the nearest centerpoint to object
  std::vector<double> dist_obj_center_points;
  for (const auto & p : center_points)
    dist_obj_center_points.push_back(tier4_autoware_utils::calcDistance2d(object_pose.position, p));
  const int obj_closest_centerpoint_idx = std::distance(
    dist_obj_center_points.begin(),
    std::min_element(dist_obj_center_points.begin(), dist_obj_center_points.end()));
  // find two center_points whose distances from `closest_centerpoint` cross stopping_distance
  double acc_dist_prev = 0.0, acc_dist = 0.0;
  auto p1 = center_points[obj_closest_centerpoint_idx];
  auto p2 = center_points[obj_closest_centerpoint_idx];
  for (unsigned i = obj_closest_centerpoint_idx; i < center_points.size() - 1; ++i) {
    p1 = center_points[i];
    p2 = center_points[i + 1];
    acc_dist_prev = acc_dist;
    const auto arc_position_p1 =
      lanelet::utils::getArcCoordinates(ego_lane_with_next_lane, util::toPose(p1));
    const auto arc_position_p2 =
      lanelet::utils::getArcCoordinates(ego_lane_with_next_lane, util::toPose(p2));
    const double delta = arc_position_p2.length - arc_position_p1.length;
    acc_dist += delta;
    if (acc_dist > stopping_distance) {
      break;
    }
  }
  // if stopping_distance >= center_points, stopping_point is center_points[end]
  const double ratio = (acc_dist <= stopping_distance)
                         ? 0.0
                         : (acc_dist - stopping_distance) / (stopping_distance - acc_dist_prev);
  // linear interpolation
  geometry_msgs::msg::Point stopping_point;
  stopping_point.x = (p1.x * ratio + p2.x) / (1 + ratio);
  stopping_point.y = (p1.y * ratio + p2.y) / (1 + ratio);
  stopping_point.z = (p1.z * ratio + p2.z) / (1 + ratio);
  const double lane_yaw = lanelet::utils::getLaneletAngle(closest_lanelet, stopping_point);
  stopping_point.x += lat_offset * std::cos(lane_yaw + M_PI / 2.0);
  stopping_point.y += lat_offset * std::sin(lane_yaw + M_PI / 2.0);

  // calculate footprint of predicted stopping pose
  autoware_auto_perception_msgs::msg::PredictedObject predicted_object = object;
  predicted_object.kinematics.initial_pose_with_covariance.pose.position = stopping_point;
  predicted_object.kinematics.initial_pose_with_covariance.pose.orientation =
    tier4_autoware_utils::createQuaternionFromRPY(0, 0, lane_yaw);
  auto predicted_obj_footprint = tier4_autoware_utils::toPolygon2d(predicted_object);
  const bool is_in_stuck_area = !bg::disjoint(predicted_obj_footprint, stuck_vehicle_detect_area);
  debug_data_.predicted_obj_pose.position = stopping_point;
  debug_data_.predicted_obj_pose.orientation =
    tier4_autoware_utils::createQuaternionFromRPY(0, 0, lane_yaw);

  if (is_in_stuck_area) {
    return true;
  }
  return false;
}

}  // namespace behavior_velocity_planner
