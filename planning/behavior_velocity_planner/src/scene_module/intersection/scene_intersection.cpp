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

bool IntersectionModule::modifyPathVelocity(
  autoware_auto_planning_msgs::msg::PathWithLaneId * path,
  tier4_planning_msgs::msg::StopReason * stop_reason)
{
  const bool external_go = isTargetExternalInputStatus(tier4_api_msgs::msg::IntersectionStatus::GO);
  const bool external_stop =
    isTargetExternalInputStatus(tier4_api_msgs::msg::IntersectionStatus::STOP);
  RCLCPP_DEBUG(logger_, "===== plan start =====");
  debug_data_ = DebugData();
  *stop_reason =
    planning_utils::initializeStopReason(tier4_planning_msgs::msg::StopReason::INTERSECTION);

  debug_data_.path_raw = *path;

  State current_state = state_machine_.getState();
  RCLCPP_DEBUG(logger_, "lane_id = %ld, state = %s", lane_id_, toString(current_state).c_str());

  /* get current pose */
  geometry_msgs::msg::PoseStamped current_pose = planner_data_->current_pose;

  /* get lanelet map */
  const auto lanelet_map_ptr = planner_data_->route_handler_->getLaneletMapPtr();
  const auto routing_graph_ptr = planner_data_->route_handler_->getRoutingGraphPtr();

  /* get detection area and conflicting area */
  lanelet::ConstLanelets detection_area_lanelets;
  std::vector<lanelet::ConstLanelets> conflicting_area_lanelets;
  std::vector<lanelet::ConstLanelets> detection_area_lanelets_with_margin;

  util::getObjectiveLanelets(
    lanelet_map_ptr, routing_graph_ptr, lane_id_, planner_param_.detection_area_length,
    planner_param_.detection_area_right_margin, planner_param_.detection_area_left_margin,
    &conflicting_area_lanelets, &detection_area_lanelets, &detection_area_lanelets_with_margin,
    logger_);
  std::vector<lanelet::CompoundPolygon3d> conflicting_areas = util::getPolygon3dFromLaneletsVec(
    conflicting_area_lanelets, planner_param_.detection_area_length);
  std::vector<lanelet::CompoundPolygon3d> detection_areas =
    util::getPolygon3dFromLanelets(detection_area_lanelets, planner_param_.detection_area_length);
  std::vector<lanelet::CompoundPolygon3d> detection_areas_with_margin =
    util::getPolygon3dFromLaneletsVec(
      detection_area_lanelets_with_margin, planner_param_.detection_area_length);
  std::vector<int> conflicting_area_lanelet_ids =
    util::getLaneletIdsFromLaneletsVec(conflicting_area_lanelets);
  std::vector<int> detection_area_lanelet_ids =
    util::getLaneletIdsFromLaneletsVec({detection_area_lanelets});

  if (detection_areas.empty()) {
    RCLCPP_DEBUG(logger_, "no detection area. skip computation.");
    setSafe(true);
    setDistance(std::numeric_limits<double>::lowest());
    return true;
  }
  debug_data_.detection_area = detection_areas;
  debug_data_.detection_area_with_margin = detection_areas_with_margin;

  /* set stop-line and stop-judgement-line for base_link */
  util::StopLineIdx stop_line_idxs;
  if (!util::generateStopLine(
        lane_id_, conflicting_areas, planner_data_, planner_param_.stop_line_margin,
        planner_param_.keep_detection_line_margin, path, *path, &stop_line_idxs,
        logger_.get_child("util"))) {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(logger_, *clock_, 1000 /* ms */, "setStopLineIdx fail");
    RCLCPP_DEBUG(logger_, "===== plan end =====");
    return false;
  }

  const int stop_line_idx = stop_line_idxs.stop_line_idx;
  const int pass_judge_line_idx = stop_line_idxs.pass_judge_line_idx;
  const int keep_detection_line_idx = stop_line_idxs.keep_detection_line_idx;
  if (stop_line_idx <= 0) {
    RCLCPP_DEBUG(logger_, "stop line line is at path[0], ignore planning.");
    RCLCPP_DEBUG(logger_, "===== plan end =====");
    setSafe(true);
    setDistance(std::numeric_limits<double>::lowest());
    return true;
  }

  /* calc closest index */
  const auto closest_idx_opt =
    motion_utils::findNearestIndex(path->points, current_pose.pose, 3.0, M_PI_4);
  if (!closest_idx_opt) {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      logger_, *clock_, 1000 /* ms */, "motion_utils::findNearestIndex fail");
    RCLCPP_DEBUG(logger_, "===== plan end =====");
    return false;
  }
  const size_t closest_idx = closest_idx_opt.get();

  const bool is_over_pass_judge_line =
    util::isOverTargetIndex(*path, closest_idx, current_pose.pose, pass_judge_line_idx);
  if (is_over_pass_judge_line) {
    /*
      in case ego could not stop exactly before the stop line, but with some overshoot, keep
      detection within some margin and low velocity threshold
     */
    const bool is_before_keep_detection_line =
      util::isBeforeTargetIndex(*path, closest_idx, current_pose.pose, keep_detection_line_idx);
    if (
      is_before_keep_detection_line && std::fabs(planner_data_->current_velocity->twist.linear.x) <
                                         planner_param_.keep_detection_vel_thr) {
      RCLCPP_DEBUG(
        logger_,
        "over the pass judge line, but before keep detection line and low speed, "
        "continue planning");
      // no return here, keep planning
    } else if (is_go_out_ && !external_stop) {
      RCLCPP_DEBUG(logger_, "over the keep_detection line and not low speed. no plan needed.");
      RCLCPP_DEBUG(logger_, "===== plan end =====");
      setSafe(true);
      setDistance(motion_utils::calcSignedArcLength(
        path->points, planner_data_->current_pose.pose.position,
        path->points.at(stop_line_idx).point.pose.position));
      return true;  // no plan needed.
    }
  }

  /* get dynamic object */
  const auto objects_ptr = planner_data_->predicted_objects;

  /* calculate dynamic collision around detection area */
  const double detect_length =
    planner_param_.stuck_vehicle_detect_dist + planner_data_->vehicle_info_.vehicle_length_m;
  const auto stuck_vehicle_detect_area = generateEgoIntersectionLanePolygon(
    lanelet_map_ptr, *path, closest_idx, stop_line_idx, detect_length,
    planner_param_.stuck_vehicle_detect_dist);
  bool is_stuck = checkStuckVehicleInIntersection(objects_ptr, stuck_vehicle_detect_area);
  bool has_collision = checkCollision(
    lanelet_map_ptr, *path, detection_area_lanelet_ids, objects_ptr, closest_idx,
    stuck_vehicle_detect_area);
  bool is_entry_prohibited = (has_collision || is_stuck);
  if (external_go) {
    is_entry_prohibited = false;
  } else if (external_stop) {
    is_entry_prohibited = true;
  }
  state_machine_.setStateWithMarginTime(
    is_entry_prohibited ? State::STOP : State::GO, logger_.get_child("state_machine"), *clock_);

  const double base_link2front = planner_data_->vehicle_info_.max_longitudinal_offset_m;

  setSafe(state_machine_.getState() == State::GO);
  setDistance(motion_utils::calcSignedArcLength(
    path->points, planner_data_->current_pose.pose.position,
    path->points.at(stop_line_idx).point.pose.position));

  if (!isActivated()) {
    constexpr double v = 0.0;
    is_go_out_ = false;
    int stop_line_idx_stop = stop_line_idx;
    int pass_judge_line_idx_stop = pass_judge_line_idx;
    if (planner_param_.use_stuck_stopline && is_stuck) {
      int stuck_stop_line_idx = -1;
      int stuck_pass_judge_line_idx = -1;
      if (util::generateStopLineBeforeIntersection(
            lane_id_, lanelet_map_ptr, planner_data_, *path, path, &stuck_stop_line_idx,
            &stuck_pass_judge_line_idx, logger_.get_child("util"))) {
        stop_line_idx_stop = stuck_stop_line_idx;
        pass_judge_line_idx_stop = stuck_pass_judge_line_idx;
      }
    }
    planning_utils::setVelocityFromIndex(stop_line_idx_stop, v, path);
    debug_data_.stop_required = true;
    debug_data_.stop_wall_pose =
      planning_utils::getAheadPose(stop_line_idx_stop, base_link2front, *path);
    debug_data_.stop_point_pose = path->points.at(stop_line_idx_stop).point.pose;
    debug_data_.judge_point_pose = path->points.at(pass_judge_line_idx_stop).point.pose;

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
  const std::vector<int> & detection_area_lanelet_ids,
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

  debug_data_.ego_lane_polygon = toGeomMsg(ego_poly);

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
    if (checkAngleForTargetLanelets(object_direction, detection_area_lanelet_ids)) {
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

        debug_data_.candidate_collision_ego_lane_polygon = toGeomMsg(polygon);

        for (auto itr = first_itr; itr != last_itr.base(); ++itr) {
          const auto footprint_polygon = toPredictedFootprintPolygon(object, *itr);
          debug_data_.candidate_collision_object_polygons.emplace_back(
            toGeomMsg(footprint_polygon));
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

  return polygon;
}

TimeDistanceArray IntersectionModule::calcIntersectionPassingTime(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const int closest_idx,
  const int objective_lane_id) const
{
  double closest_vel =
    (std::max(1e-01, std::fabs(planner_data_->current_velocity->twist.linear.x)));
  double dist_sum = 0.0;
  int assigned_lane_found = false;

  PathWithLaneId reference_path;
  reference_path.points.push_back(path.points.at(closest_idx));
  reference_path.points.at(0).point.longitudinal_velocity_mps = closest_vel;
  for (size_t i = closest_idx + 1; i < path.points.size(); ++i) {
    const double dist =
      tier4_autoware_utils::calcDistance2d(path.points.at(i - 1), path.points.at(i));
    dist_sum += dist;
    // calc vel in idx i+1 (v_{i+1}^2 - v_{i}^2 = 2ax)
    const double next_vel = std::min(
      std::sqrt(std::pow(closest_vel, 2.0) + 2.0 * planner_param_.intersection_max_acc * dist),
      planner_param_.intersection_velocity);
    // calc average vel in idx i~i+1
    const double average_vel =
      std::min((closest_vel + next_vel) / 2.0, planner_param_.intersection_velocity);
    // passing_time += dist / average_vel;
    // time_distance_array.emplace_back(passing_time, dist_sum);
    auto reference_point = path.points[i];
    reference_point.point.longitudinal_velocity_mps = average_vel;
    reference_path.points.push_back(reference_point);

    closest_vel = next_vel;

    bool has_objective_lane_id = util::hasLaneId(path.points.at(i), objective_lane_id);

    if (assigned_lane_found && !has_objective_lane_id) {
      break;
    }
    assigned_lane_found = has_objective_lane_id;
  }
  if (!assigned_lane_found) {
    return {{0.0, 0.0}};  // has already passed the intersection.
  }

  PathWithLaneId smoothed_reference_path = reference_path;
  if (!smoothPath(reference_path, smoothed_reference_path, planner_data_)) {
    RCLCPP_WARN(logger_, "smoothPath failed");
  }

  TimeDistanceArray time_distance_array{};
  dist_sum = 0.0;
  double passing_time = 0.0;
  time_distance_array.emplace_back(passing_time, dist_sum);
  for (size_t i = 1; i < smoothed_reference_path.points.size(); ++i) {
    const double dist = tier4_autoware_utils::calcDistance2d(
      smoothed_reference_path.points.at(i - 1), smoothed_reference_path.points.at(i));
    dist_sum += dist;
    // to avoid zero division
    passing_time +=
      (dist / std::max<double>(
                0.01, smoothed_reference_path.points.at(i - 1).point.longitudinal_velocity_mps));
    time_distance_array.emplace_back(passing_time, dist_sum);
  }
  RCLCPP_DEBUG(logger_, "intersection dist = %f, passing_time = %f", dist_sum, passing_time);

  return time_distance_array;
}

bool IntersectionModule::checkStuckVehicleInIntersection(
  const autoware_auto_perception_msgs::msg::PredictedObjects::ConstSharedPtr objects_ptr,
  const Polygon2d & stuck_vehicle_detect_area) const
{
  debug_data_.stuck_vehicle_detect_area = toGeomMsg(stuck_vehicle_detect_area);

  for (const auto & object : objects_ptr->objects) {
    if (!isTargetStuckVehicleType(object)) {
      continue;  // not target vehicle type
    }
    const auto obj_v = std::fabs(object.kinematics.initial_twist_with_covariance.twist.linear.x);
    if (obj_v > planner_param_.stuck_vehicle_vel_thr) {
      continue;  // not stop vehicle
    }

    // check if the footprint is in the stuck detect area
    const Polygon2d obj_footprint = toFootprintPolygon(object);
    const bool is_in_stuck_area = !bg::disjoint(obj_footprint, stuck_vehicle_detect_area);
    if (is_in_stuck_area) {
      RCLCPP_DEBUG(logger_, "stuck vehicle found.");
      debug_data_.stuck_targets.objects.push_back(object);
      return true;
    }
  }
  return false;
}

Polygon2d IntersectionModule::toFootprintPolygon(
  const autoware_auto_perception_msgs::msg::PredictedObject & object) const
{
  Polygon2d obj_footprint;
  if (object.shape.type == autoware_auto_perception_msgs::msg::Shape::POLYGON) {
    obj_footprint = toBoostPoly(object.shape.footprint);
  } else {
    // cylinder type is treated as square-polygon
    obj_footprint =
      obj2polygon(object.kinematics.initial_pose_with_covariance.pose, object.shape.dimensions);
  }
  return obj_footprint;
}

Polygon2d IntersectionModule::toPredictedFootprintPolygon(
  const autoware_auto_perception_msgs::msg::PredictedObject & object,
  const geometry_msgs::msg::Pose & predicted_pose) const
{
  return obj2polygon(predicted_pose, object.shape.dimensions);
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
void IntersectionModule::StateMachine::setStateWithMarginTime(
  State state, rclcpp::Logger logger, rclcpp::Clock & clock)
{
  /* same state request */
  if (state_ == state) {
    start_time_ = nullptr;  // reset timer
    return;
  }

  /* GO -> STOP */
  if (state == State::STOP) {
    state_ = State::STOP;
    start_time_ = nullptr;  // reset timer
    return;
  }

  /* STOP -> GO */
  if (state == State::GO) {
    if (start_time_ == nullptr) {
      start_time_ = std::make_shared<rclcpp::Time>(clock.now());
    } else {
      const double duration = (clock.now() - *start_time_).seconds();
      if (duration > margin_time_) {
        state_ = State::GO;
        start_time_ = nullptr;  // reset timer
      }
    }
    return;
  }

  RCLCPP_ERROR(logger, "Unsuitable state. ignore request.");
}

void IntersectionModule::StateMachine::setState(State state) { state_ = state; }

void IntersectionModule::StateMachine::setMarginTime(const double t) { margin_time_ = t; }

IntersectionModule::State IntersectionModule::StateMachine::getState() { return state_; }

bool IntersectionModule::isTargetExternalInputStatus(const int target_status)
{
  return planner_data_->external_intersection_status_input &&
         planner_data_->external_intersection_status_input.get().status == target_status &&
         (clock_->now() - planner_data_->external_intersection_status_input.get().header.stamp)
             .seconds() < planner_param_.external_input_timeout;
}

bool IntersectionModule::checkAngleForTargetLanelets(
  const geometry_msgs::msg::Pose & pose, const std::vector<int> & target_lanelet_ids)
{
  for (const int lanelet_id : target_lanelet_ids) {
    const auto ll = planner_data_->route_handler_->getLaneletMapPtr()->laneletLayer.get(lanelet_id);
    if (!lanelet::utils::isInLanelet(pose, ll, planner_param_.detection_area_margin)) {
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
  Polygon2d predicted_obj_footprint = toFootprintPolygon(predicted_object);
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
