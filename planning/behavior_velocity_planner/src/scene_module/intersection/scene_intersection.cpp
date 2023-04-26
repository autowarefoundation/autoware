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

#include <grid_map_cv/grid_map_cv.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <lanelet2_extension/regulatory_elements/road_marking.hpp>
#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>

#include <cv_bridge/cv_bridge.h>
// #include <sensor_msgs/image_encodings.h>
// #include <opencv2/highgui/highgui.hpp>
#include <magic_enum.hpp>
#include <opencv2/imgproc.hpp>
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
  const PlannerParam & planner_param, const std::set<int> & assoc_ids,
  const bool enable_occlusion_detection, rclcpp::Node & node, const rclcpp::Logger logger,
  const rclcpp::Clock::SharedPtr clock)
: SceneModuleInterface(module_id, logger, clock),
  node_(node),
  lane_id_(lane_id),
  assoc_ids_(assoc_ids),
  enable_occlusion_detection_(enable_occlusion_detection),
  detection_divisions_(std::nullopt),
  occlusion_uuid_(tier4_autoware_utils::generateUUID()),
  occlusion_first_stop_uuid_(tier4_autoware_utils::generateUUID())
{
  velocity_factor_.init(VelocityFactor::INTERSECTION);
  planner_param_ = planner_param;

  const auto & assigned_lanelet =
    planner_data->route_handler_->getLaneletMapPtr()->laneletLayer.get(lane_id);
  turn_direction_ = assigned_lanelet.attributeOr("turn_direction", "else");
  collision_state_machine_.setMarginTime(
    planner_param_.collision_detection.state_transit_margin_time);
  before_creep_state_machine_.setMarginTime(planner_param_.occlusion.before_creep_stop_time);
  // TODO(Mamoru Sobue): maybe optional is better
  before_creep_state_machine_.setState(StateMachine::State::STOP);
  if (enable_occlusion_detection) {
    occlusion_grid_pub_ = node_.create_publisher<grid_map_msgs::msg::GridMap>(
      "~/debug/intersection/occlusion_grid", rclcpp::QoS(1).transient_local());
  }
}

bool IntersectionModule::modifyPathVelocity(PathWithLaneId * path, StopReason * stop_reason)
{
  RCLCPP_DEBUG(logger_, "===== plan start =====");

  debug_data_ = DebugData();
  *stop_reason = planning_utils::initializeStopReason(StopReason::INTERSECTION);

  /* set default RTC */
  // safe_, distance_
  setSafe(true);
  setDistance(std::numeric_limits<double>::lowest());
  // occlusion
  occlusion_safety_ = true;
  occlusion_stop_distance_ = std::numeric_limits<double>::lowest();
  occlusion_first_stop_safety_ = true;
  occlusion_first_stop_distance_ = std::numeric_limits<double>::lowest();

  /* get current pose */
  const geometry_msgs::msg::Pose current_pose = planner_data_->current_odometry->pose;

  /* get lanelet map */
  const auto lanelet_map_ptr = planner_data_->route_handler_->getLaneletMapPtr();
  const auto routing_graph_ptr = planner_data_->route_handler_->getRoutingGraphPtr();
  const auto & assigned_lanelet =
    planner_data_->route_handler_->getLaneletMapPtr()->laneletLayer.get(lane_id_);
  const std::string turn_direction = assigned_lanelet.attributeOr("turn_direction", "else");

  /* spline interpolation */
  constexpr double interval = 0.2;
  autoware_auto_planning_msgs::msg::PathWithLaneId path_ip;
  if (!splineInterpolate(*path, interval, path_ip, logger_)) {
    RCLCPP_DEBUG_SKIPFIRST_THROTTLE(logger_, *clock_, 1000 /* ms */, "splineInterpolate failed");
    RCLCPP_DEBUG(logger_, "===== plan end =====");
    return false;
  }
  const auto lane_interval_ip_opt = util::findLaneIdsInterval(path_ip, assoc_ids_);
  if (!lane_interval_ip_opt) {
    RCLCPP_WARN(logger_, "Path has no interval on intersection lane %ld", lane_id_);
    RCLCPP_DEBUG(logger_, "===== plan end =====");
    return false;
  }
  const auto lane_interval_ip = lane_interval_ip_opt.value();

  /* get detection area*/
  /* dynamically change detection area based on tl_arrow_solid_on */
  const bool tl_arrow_solid_on =
    util::isTrafficLightArrowActivated(assigned_lanelet, planner_data_->traffic_light_id_map);
  if (
    !intersection_lanelets_ ||
    intersection_lanelets_.value().tl_arrow_solid_on != tl_arrow_solid_on) {
    intersection_lanelets_ = util::getObjectiveLanelets(
      lanelet_map_ptr, routing_graph_ptr, lane_id_, assoc_ids_, path_ip, lane_interval_ip,
      planner_param_.common.detection_area_length, tl_arrow_solid_on);
  }
  const auto & detection_lanelets = intersection_lanelets_.value().attention;
  const auto & adjacent_lanelets = intersection_lanelets_.value().adjacent;
  const auto & occlusion_attention_lanelets = intersection_lanelets_.value().occlusion_attention;
  const auto & detection_area = intersection_lanelets_.value().attention_area;
  const auto & occlusion_attention_area = intersection_lanelets_.value().occlusion_attention_area;
  const auto & first_conflicting_area = intersection_lanelets_.value().first_conflicting_area;
  const auto & first_detection_area = intersection_lanelets_.value().first_detection_area;
  const auto & conflicting_lanelets = intersection_lanelets_.value().conflicting;
  debug_data_.detection_area = detection_area;
  debug_data_.adjacent_area = intersection_lanelets_.value().adjacent_area;

  /* get intersection area */
  const auto intersection_area = util::getIntersectionArea(assigned_lanelet, lanelet_map_ptr);
  if (intersection_area) {
    const auto intersection_area_2d = intersection_area.value();
    debug_data_.intersection_area = toGeomPoly(intersection_area_2d);
  }

  if (conflicting_lanelets.empty()) {
    RCLCPP_DEBUG(logger_, "conflicting area is empty");
    return false;
  }

  if (!detection_divisions_.has_value()) {
    detection_divisions_ = util::generateDetectionLaneDivisions(
      occlusion_attention_lanelets, routing_graph_ptr,
      planner_data_->occupancy_grid->info.resolution / std::sqrt(2.0));
  }

  /* calc closest index */
  const auto closest_idx_opt =
    motion_utils::findNearestIndex(path->points, current_pose, 3.0, M_PI_4);
  if (!closest_idx_opt) {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      logger_, *clock_, 1000 /* ms */, "motion_utils::findNearestIndex fail");
    RCLCPP_DEBUG(logger_, "===== plan end =====");
    return false;
  }
  const size_t closest_idx = closest_idx_opt.get();

  const auto static_pass_judge_line_opt =
    first_detection_area
      ? util::generateStaticPassJudgeLine(
          first_detection_area.value(), path, path_ip, interval, lane_interval_ip, planner_data_)
      : std::nullopt;

  if (static_pass_judge_line_opt) {
    const auto pass_judge_line_idx = static_pass_judge_line_opt.value();
    const bool is_over_pass_judge_line =
      util::isOverTargetIndex(*path, closest_idx, current_pose, pass_judge_line_idx);
    const double vel = std::fabs(planner_data_->current_velocity->twist.linear.x);
    const bool keep_detection = (vel < planner_param_.collision_detection.keep_detection_vel_thr);
    // if ego is over the pass judge line and not stopped
    if (is_over_pass_judge_line && is_go_out_ && !keep_detection) {
      RCLCPP_DEBUG(logger_, "over the pass judge line. no plan needed.");
      RCLCPP_DEBUG(logger_, "===== plan end =====");
      return true;
    }
  }

  /* considering lane change in the intersection, these lanelets are generated from the path */
  const auto ego_lane_with_next_lane =
    getEgoLaneWithNextLane(*path, planner_data_->vehicle_info_.vehicle_width_m);
  const auto ego_lane = ego_lane_with_next_lane.front();
  debug_data_.ego_lane = ego_lane.polygon3d();

  /* get dynamic object */
  // TODO(Mamoru Sobue): filter objects on detection area here
  const auto objects_ptr = planner_data_->predicted_objects;

  /* check stuck vehicle */
  const auto stuck_vehicle_detect_area =
    generateStuckVehicleDetectAreaPolygon(*path, ego_lane_with_next_lane, closest_idx);
  const bool is_stuck = checkStuckVehicleInIntersection(objects_ptr, stuck_vehicle_detect_area);
  debug_data_.stuck_vehicle_detect_area = toGeomPoly(stuck_vehicle_detect_area);
  const std::optional<size_t> stuck_line_idx_opt =
    first_conflicting_area
      ? util::generateStuckStopLine(
          first_conflicting_area.value(), planner_data_, planner_param_.common.stop_line_margin,
          planner_param_.stuck_vehicle.use_stuck_stopline, path, path_ip, interval,
          lane_interval_ip, logger_.get_child("util"))
      : std::nullopt;

  /* calculate dynamic collision around detection area */
  /* set stop lines for base_link */
  const auto default_stop_line_idx_opt =
    first_detection_area ? util::generateCollisionStopLine(
                             lane_id_, first_detection_area.value(), planner_data_,
                             planner_param_.common.stop_line_margin, path, path_ip, interval,
                             lane_interval_ip, logger_.get_child("util"))
                         : std::nullopt;
  const double time_delay = is_go_out_
                              ? 0.0
                              : (planner_param_.collision_detection.state_transit_margin_time -
                                 collision_state_machine_.getDuration());
  const bool has_collision = checkCollision(
    lanelet_map_ptr, *path, detection_lanelets, adjacent_lanelets, intersection_area, ego_lane,
    ego_lane_with_next_lane, objects_ptr, closest_idx, time_delay);

  /* check occlusion on detection lane */
  const double occlusion_dist_thr = std::fabs(
    std::pow(planner_param_.occlusion.max_vehicle_velocity_for_rss, 2) /
    (2 * planner_param_.occlusion.min_vehicle_brake_for_rss));
  const bool is_occlusion_cleared =
    (enable_occlusion_detection_ && first_detection_area && !occlusion_attention_lanelets.empty())
      ? isOcclusionCleared(
          *planner_data_->occupancy_grid, occlusion_attention_area, first_detection_area.value(),
          path_ip, lane_interval_ip, detection_divisions_.value(), occlusion_dist_thr)
      : true;
  const auto occlusion_peeking_line_idx_opt =
    first_detection_area
      ? util::generatePeekingLimitLine(
          first_detection_area.value(), path, path_ip, interval, lane_interval_ip, planner_data_,
          planner_param_.occlusion.peeking_offset)
      : std::nullopt;

  /* calculate final stop lines */
  std::optional<size_t> stop_line_idx = default_stop_line_idx_opt;
  std::optional<size_t> occlusion_peeking_line_idx =
    occlusion_peeking_line_idx_opt;  // TODO(Mamoru Sobue): different position depending on the flag
  std::optional<size_t> occlusion_first_stop_line_idx = default_stop_line_idx_opt;
  std::optional<std::pair<size_t, size_t>> insert_creep_during_occlusion = std::nullopt;

  /* set RTC distance */
  const double dist_1st_stopline =
    default_stop_line_idx_opt
      ? motion_utils::calcSignedArcLength(
          path_ip.points, current_pose.position,
          path->points.at(default_stop_line_idx_opt.value()).point.pose.position)
      : std::numeric_limits<double>::lowest();
  const double dist_2nd_stopline =
    occlusion_peeking_line_idx
      ? motion_utils::calcSignedArcLength(
          path_ip.points, current_pose.position,
          path->points.at(occlusion_peeking_line_idx.value()).point.pose.position)
      : std::numeric_limits<double>::lowest();

  bool stuck_stop_required = false;
  bool collision_stop_required = false;
  bool first_phase_stop_required = false;
  bool occlusion_stop_required = false;

  /* check safety */
  const bool ext_occlusion_requested = (is_occlusion_cleared && !occlusion_activated_);
  is_actually_occluded_ = !is_occlusion_cleared;
  is_forcefully_occluded_ = ext_occlusion_requested;
  if (!is_occlusion_cleared || ext_occlusion_requested) {
    if (!default_stop_line_idx_opt) {
      RCLCPP_DEBUG(logger_, "occlusion is detected but default stop line is not set or generated");
      RCLCPP_DEBUG(logger_, "===== plan end =====");
      return true;
    }
    if (
      before_creep_state_machine_.getState() == StateMachine::State::GO &&
      !ext_occlusion_requested) {
      occlusion_stop_required = true;
      stop_line_idx = occlusion_peeking_line_idx = occlusion_peeking_line_idx_opt;
      // clear first stop line
      // insert creep velocity [closest_idx, occlusion_stop_line)
      insert_creep_during_occlusion =
        std::make_pair(closest_idx, occlusion_peeking_line_idx_opt.value());
      occlusion_state_ = OcclusionState::CREEP_SECOND_STOP_LINE;
    } else {
      const bool approached_stop_line =
        (dist_1st_stopline < planner_param_.common.stop_overshoot_margin);
      const bool is_stopped = planner_data_->isVehicleStopped();
      if (is_stopped && approached_stop_line) {
        // start waiting at the first stop line
        before_creep_state_machine_.setStateWithMarginTime(
          StateMachine::State::GO, logger_.get_child("occlusion state_machine"), *clock_);
        occlusion_state_ = OcclusionState::WAIT_FIRST_STOP_LINE;
      }
      first_phase_stop_required = true;
      occlusion_stop_required = true;
      occlusion_peeking_line_idx = occlusion_peeking_line_idx_opt;
      stop_line_idx = occlusion_first_stop_line_idx;
      // insert creep velocity [default_stop_line, occlusion_stop_line)
      insert_creep_during_occlusion =
        std::make_pair(default_stop_line_idx_opt.value(), occlusion_peeking_line_idx_opt.value());
      occlusion_state_ = OcclusionState::BEFORE_FIRST_STOP_LINE;
    }
  } else if (occlusion_state_ != OcclusionState::CLEARED) {
    // previously occlusion existed, but now it is clear
    if (
      default_stop_line_idx_opt &&
      !util::isOverTargetIndex(
        *path, closest_idx, current_pose, default_stop_line_idx_opt.value())) {
      stop_line_idx = default_stop_line_idx_opt.value();
    } else if (
      static_pass_judge_line_opt &&
      !util::isOverTargetIndex(
        *path, closest_idx, current_pose, static_pass_judge_line_opt.value())) {
      stop_line_idx = static_pass_judge_line_opt;
    }
    occlusion_state_ = OcclusionState::CLEARED;
    if (stop_line_idx && has_collision) {
      // do collision checking at previous occlusion stop line
      collision_stop_required = true;
    } else {
      collision_stop_required = false;
    }
    if (is_stuck && stuck_line_idx_opt) {
      stuck_stop_required = true;
      stop_line_idx = static_pass_judge_line_opt;
    }
  } else if (is_stuck && stuck_line_idx_opt) {
    stuck_stop_required = true;
    const size_t stuck_line_idx = stuck_line_idx_opt.value();
    const double dist_stuck_stopline = motion_utils::calcSignedArcLength(
      path->points, path->points.at(stuck_line_idx).point.pose.position,
      path->points.at(closest_idx).point.pose.position);
    const bool is_over_stuck_stopline =
      util::isOverTargetIndex(*path, closest_idx, current_pose, stuck_line_idx) &&
      (dist_stuck_stopline > planner_param_.common.stop_overshoot_margin);
    if (!is_over_stuck_stopline) {
      stop_line_idx = stuck_line_idx;
    } else if (is_over_stuck_stopline && default_stop_line_idx_opt) {
      stop_line_idx = default_stop_line_idx_opt.value();
    }
  } else if (has_collision) {
    collision_stop_required = true;
    stop_line_idx = default_stop_line_idx_opt;
  }

  if (!stop_line_idx) {
    RCLCPP_DEBUG(logger_, "detection_area is empty");
    RCLCPP_DEBUG(logger_, "===== plan end =====");
    return false;
  }

  const std::string occlusion_state = std::string(magic_enum::enum_name(occlusion_state_));
  RCLCPP_DEBUG(logger_, "occlusion state: OcclusionState::%s", occlusion_state.c_str());

  /* for collision and stuck state */
  collision_state_machine_.setStateWithMarginTime(
    (collision_stop_required || stuck_stop_required) ? StateMachine::State::STOP
                                                     : StateMachine::State::GO,
    logger_.get_child("collision state_machine"), *clock_);

  /* set RTC safety respectively */
  if (occlusion_stop_required) {
    if (first_phase_stop_required) {
      occlusion_first_stop_safety_ = false;
      occlusion_first_stop_distance_ = dist_1st_stopline;
    }
    occlusion_safety_ = false;
    occlusion_stop_distance_ = dist_2nd_stopline;
  } else {
    /* collision */
    setSafe(collision_state_machine_.getState() == StateMachine::State::GO);
    setDistance(dist_1st_stopline);
  }

  /* make decision */
  const double baselink2front = planner_data_->vehicle_info_.max_longitudinal_offset_m;
  if (!occlusion_activated_) {
    is_go_out_ = false;

    /* in case of creeping */
    if (insert_creep_during_occlusion && planner_param_.occlusion.enable_creeping) {
      const auto [start, end] = insert_creep_during_occlusion.value();
      for (size_t i = start; i < end; ++i) {
        planning_utils::setVelocityFromIndex(
          i, planner_param_.occlusion.occlusion_creep_velocity /* [m/s] */, path);
      }
    }

    if (!occlusion_first_stop_activated_ && occlusion_first_stop_line_idx) {
      planning_utils::setVelocityFromIndex(
        occlusion_first_stop_line_idx.value(), 0.0 /* [m/s] */, path);
      debug_data_.occlusion_first_stop_wall_pose =
        planning_utils::getAheadPose(occlusion_first_stop_line_idx.value(), baselink2front, *path);
    }

    if (occlusion_peeking_line_idx) {
      planning_utils::setVelocityFromIndex(
        occlusion_peeking_line_idx.value(), 0.0 /* [m/s] */, path);
      debug_data_.occlusion_stop_wall_pose =
        planning_utils::getAheadPose(occlusion_peeking_line_idx.value(), baselink2front, *path);
    }

    RCLCPP_DEBUG(logger_, "not activated. stop at the line.");
    RCLCPP_DEBUG(logger_, "===== plan end =====");
    return true;
  }

  if (!isActivated() /* collision*/) {
    is_go_out_ = false;

    planning_utils::setVelocityFromIndex(stop_line_idx.value(), 0.0 /* [m/s] */, path);
    debug_data_.collision_stop_wall_pose =
      planning_utils::getAheadPose(stop_line_idx.value(), baselink2front, *path);

    // Get stop point and stop factor
    {
      tier4_planning_msgs::msg::StopFactor stop_factor;
      stop_factor.stop_pose = path->points.at(stop_line_idx.value()).point.pose;
      const auto stop_factor_conflict =
        planning_utils::toRosPoints(debug_data_.conflicting_targets);
      const auto stop_factor_stuck = planning_utils::toRosPoints(debug_data_.stuck_targets);
      stop_factor.stop_factor_points =
        planning_utils::concatVector(stop_factor_conflict, stop_factor_stuck);
      planning_utils::appendStopReason(stop_factor, stop_reason);

      const auto & stop_pose = path->points.at(stop_line_idx.value()).point.pose;
      velocity_factor_.set(
        path->points, planner_data_->current_odometry->pose, stop_pose, VelocityFactor::UNKNOWN);
    }
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
  const std::optional<Polygon2d> & intersection_area, const lanelet::ConstLanelet & ego_lane,
  const lanelet::ConstLanelets & ego_lane_with_next_lane,
  const autoware_auto_perception_msgs::msg::PredictedObjects::ConstSharedPtr objects_ptr,
  const int closest_idx, const double time_delay)
{
  using lanelet::utils::getArcCoordinates;
  using lanelet::utils::getPolygonFromArcLength;

  /* extract target objects */
  autoware_auto_perception_msgs::msg::PredictedObjects target_objects;
  target_objects.header = objects_ptr->header;
  for (const auto & object : objects_ptr->objects) {
    // ignore non-vehicle type objects, such as pedestrian.
    if (!isTargetCollisionVehicleType(object)) {
      continue;
    }

    // check direction of objects
    const auto object_direction = getObjectPoseWithVelocityDirection(object.kinematics);
    const auto is_in_adjacent_lanelets = checkAngleForTargetLanelets(
      object_direction, adjacent_lanelets, planner_param_.common.detection_area_margin);
    if (is_in_adjacent_lanelets) {
      continue;
    }

    if (intersection_area) {
      const auto obj_poly = tier4_autoware_utils::toPolygon2d(object);
      const auto intersection_area_2d = intersection_area.value();
      const auto is_in_intersection_area = bg::within(obj_poly, intersection_area_2d);
      if (is_in_intersection_area) {
        target_objects.objects.push_back(object);
      } else if (checkAngleForTargetLanelets(
                   object_direction, detection_area_lanelets,
                   planner_param_.common.detection_area_margin)) {
        target_objects.objects.push_back(object);
      }
    } else if (checkAngleForTargetLanelets(
                 object_direction, detection_area_lanelets,
                 planner_param_.common.detection_area_margin)) {
      // intersection_area is not available, use detection_area_with_margin as before
      target_objects.objects.push_back(object);
    }
  }

  /* check collision between target_objects predicted path and ego lane */

  // cut the predicted path at passing_time
  const auto time_distance_array = calcIntersectionPassingTime(path, closest_idx, time_delay);
  const double passing_time = time_distance_array.back().first;
  cutPredictPathWithDuration(&target_objects, passing_time);

  const auto closest_arc_coords = getArcCoordinates(
    ego_lane_with_next_lane, tier4_autoware_utils::getPose(path.points.at(closest_idx).point));
  const double distance_until_intersection =
    calcDistanceUntilIntersectionLanelet(lanelet_map_ptr, path, closest_idx);
  const double base_link2front = planner_data_->vehicle_info_.max_longitudinal_offset_m;

  const auto ego_poly = ego_lane.polygon2d().basicPolygon();
  // check collision between predicted_path and ego_area
  bool collision_detected = false;
  for (const auto & object : target_objects.objects) {
    bool has_collision = false;
    for (const auto & predicted_path : object.kinematics.predicted_paths) {
      if (
        predicted_path.confidence <
        planner_param_.collision_detection.min_predicted_path_confidence) {
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
        if (
          ref_object_enter_time - planner_param_.collision_detection.collision_start_margin_time >
          0) {
          start_time_distance_itr = std::lower_bound(
            time_distance_array.begin(), time_distance_array.end(),
            ref_object_enter_time - planner_param_.collision_detection.collision_start_margin_time,
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
          ref_object_exit_time + planner_param_.collision_detection.collision_end_margin_time,
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

Polygon2d IntersectionModule::generateStuckVehicleDetectAreaPolygon(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
  const lanelet::ConstLanelets & ego_lane_with_next_lane, const int closest_idx) const
{
  using lanelet::utils::getArcCoordinates;
  using lanelet::utils::getLaneletLength3d;
  using lanelet::utils::getPolygonFromArcLength;
  using lanelet::utils::to2D;

  const double extra_dist = planner_param_.stuck_vehicle.stuck_vehicle_detect_dist +
                            planner_data_->vehicle_info_.vehicle_length_m;
  const double ignore_dist = planner_param_.stuck_vehicle.stuck_vehicle_ignore_dist +
                             planner_data_->vehicle_info_.vehicle_length_m;

  const double intersection_exit_length = getLaneletLength3d(ego_lane_with_next_lane.front());

  const auto closest_arc_coords = getArcCoordinates(
    ego_lane_with_next_lane, tier4_autoware_utils::getPose(path.points.at(closest_idx).point));

  const double start_arc_length = intersection_exit_length - ignore_dist > closest_arc_coords.length
                                    ? intersection_exit_length - ignore_dist
                                    : closest_arc_coords.length;

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
  const double time_delay) const
{
  double dist_sum = 0.0;
  int assigned_lane_found = false;

  // crop intersection part of the path, and set the reference velocity to intersection_velocity for
  // ego's ttc
  PathWithLaneId reference_path;
  for (size_t i = closest_idx; i < path.points.size(); ++i) {
    auto reference_point = path.points.at(i);
    reference_point.point.longitudinal_velocity_mps = planner_param_.common.intersection_velocity;
    reference_path.points.push_back(reference_point);
    bool has_objective_lane_id = util::hasLaneIds(path.points.at(i), assoc_ids_);
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
  double passing_time = time_delay;
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
      (dist / std::max<double>(
                planner_param_.collision_detection.minimum_ego_predicted_velocity,
                average_velocity));  // to avoid zero-division

    time_distance_array.emplace_back(passing_time, dist_sum);
  }
  RCLCPP_DEBUG(logger_, "intersection dist = %f, passing_time = %f", dist_sum, passing_time);

  return time_distance_array;
}

bool IntersectionModule::checkStuckVehicleInIntersection(
  const autoware_auto_perception_msgs::msg::PredictedObjects::ConstSharedPtr objects_ptr,
  const Polygon2d & stuck_vehicle_detect_area) const
{
  for (const auto & object : objects_ptr->objects) {
    if (!isTargetStuckVehicleType(object)) {
      continue;  // not target vehicle type
    }
    const auto obj_v = std::fabs(object.kinematics.initial_twist_with_covariance.twist.linear.x);
    if (obj_v > planner_param_.stuck_vehicle.stuck_vehicle_vel_thr) {
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
    if (std::fabs(angle_diff) < planner_param_.common.detection_area_angle_thr) {
      return true;
    }
  }
  return false;
}

lanelet::ConstLanelets IntersectionModule::getEgoLaneWithNextLane(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const double width) const
{
  // NOTE: findLaneIdsInterval returns (start, end) of assoc_ids
  const auto ego_lane_interval_opt = util::findLaneIdsInterval(path, assoc_ids_);
  if (!ego_lane_interval_opt) {
    return lanelet::ConstLanelets({});
  }
  const auto [ego_start, ego_end] = ego_lane_interval_opt.value();
  if (ego_end < path.points.size() - 1) {
    const int next_id = path.points.at(ego_end).lane_ids.at(0);
    const auto next_lane_interval_opt = util::findLaneIdsInterval(path, {next_id});
    if (next_lane_interval_opt) {
      const auto [next_start, next_end] = next_lane_interval_opt.value();
      return {
        planning_utils::generatePathLanelet(path, ego_start, next_start + 1, width),
        planning_utils::generatePathLanelet(path, next_start + 1, next_end, width)};
    }
  }
  return {planning_utils::generatePathLanelet(path, ego_start, ego_end, width)};
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
  const double object_decel =
    planner_param_.stuck_vehicle.assumed_front_car_decel;  // NOTE: this is positive
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

bool IntersectionModule::isOcclusionCleared(
  const nav_msgs::msg::OccupancyGrid & occ_grid,
  const std::vector<lanelet::CompoundPolygon3d> & detection_areas,
  const lanelet::CompoundPolygon3d & first_detection_area,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path_ip,
  const std::pair<size_t, size_t> & lane_interval_ip,
  const std::vector<util::DetectionLaneDivision> & lane_divisions,
  const double occlusion_dist_thr) const
{
  const auto first_detection_area_idx =
    util::getFirstPointInsidePolygon(path_ip, lane_interval_ip, first_detection_area);
  if (!first_detection_area_idx) {
    return false;
  }

  const auto first_inside_detection_idx_ip_opt =
    util::getFirstPointInsidePolygon(path_ip, lane_interval_ip, first_detection_area);
  const std::pair<size_t, size_t> lane_detection_interval_ip =
    first_inside_detection_idx_ip_opt
      ? std::make_pair(first_inside_detection_idx_ip_opt.value(), std::get<1>(lane_interval_ip))
      : lane_interval_ip;

  const int width = occ_grid.info.width;
  const int height = occ_grid.info.height;
  const double reso = occ_grid.info.resolution;
  const auto & origin = occ_grid.info.origin.position;

  // NOTE: interesting area is set to 0 for later masking
  cv::Mat detection_mask(width, height, CV_8UC1, cv::Scalar(0));
  cv::Mat unknown_mask(width, height, CV_8UC1, cv::Scalar(0));

  // (1) prepare detection area mask
  Polygon2d grid_poly;
  grid_poly.outer().emplace_back(origin.x, origin.y);
  grid_poly.outer().emplace_back(origin.x + (width - 1) * reso, origin.y);
  grid_poly.outer().emplace_back(origin.x + (width - 1) * reso, origin.y + (height - 1) * reso);
  grid_poly.outer().emplace_back(origin.x, origin.y + (height - 1) * reso);
  grid_poly.outer().emplace_back(origin.x, origin.y);
  bg::correct(grid_poly);

  std::vector<std::vector<cv::Point>> detection_area_cv_polygons;
  for (const auto & detection_area : detection_areas) {
    const auto area2d = lanelet::utils::to2D(detection_area);
    Polygon2d area2d_poly;
    for (const auto & p : area2d) {
      area2d_poly.outer().emplace_back(p.x(), p.y());
    }
    area2d_poly.outer().push_back(area2d_poly.outer().front());
    bg::correct(area2d_poly);
    std::vector<Polygon2d> common_areas;
    bg::intersection(area2d_poly, grid_poly, common_areas);
    if (common_areas.empty()) {
      continue;
    }
    for (size_t i = 0; i < common_areas.size(); ++i) {
      common_areas[i].outer().push_back(common_areas[i].outer().front());
      bg::correct(common_areas[i]);
    }
    for (const auto & common_area : common_areas) {
      std::vector<cv::Point> detection_area_cv_polygon;
      for (const auto & p : common_area.outer()) {
        const int idx_x = static_cast<int>((p.x() - origin.x) / reso);
        const int idx_y = static_cast<int>((p.y() - origin.y) / reso);
        detection_area_cv_polygon.emplace_back(idx_x, height - 1 - idx_y);
      }
      detection_area_cv_polygons.push_back(detection_area_cv_polygon);
    }
  }
  for (const auto & poly : detection_area_cv_polygons) {
    cv::fillPoly(detection_mask, poly, cv::Scalar(255), cv::LINE_AA);
  }

  // (2) prepare unknown mask
  // In OpenCV the pixel at (X=x, Y=y) (with left-upper origin) is accesed by img[y, x]
  for (int x = 0; x < width; x++) {
    for (int y = 0; y < height; y++) {
      const int idx = y * width + x;
      const unsigned char intensity = occ_grid.data.at(idx);
      if (
        planner_param_.occlusion.free_space_max <= intensity &&
        intensity < planner_param_.occlusion.occupied_min) {
        unknown_mask.at<unsigned char>(height - 1 - y, x) = 255;
      }
    }
  }

  // (3) occlusion mask
  cv::Mat occlusion_mask(width, height, CV_8UC1, cv::Scalar(0));
  cv::bitwise_and(detection_mask, unknown_mask, occlusion_mask);

  // (4) create distance grid
  // value: 0 - 254: signed distance representing [distamce_min, distance_max]
  // 255: undefined value
  const double distance_max = std::hypot(width * reso / 2, height * reso / 2);
  const double distance_min = -distance_max;
  const int undef_pixel = 255;
  const int max_cost_pixel = 254;

  auto dist2pixel = [=](const double dist) {
    return std::min(
      max_cost_pixel,
      static_cast<int>((dist - distance_min) / (distance_max - distance_min) * max_cost_pixel));
  };
  auto pixel2dist = [=](const int pixel) {
    return pixel * 1.0 / max_cost_pixel * (distance_max - distance_min) + distance_min;
  };

  const int zero_dist_pixel = dist2pixel(0.0);

  auto coord2index = [&](const double x, const double y) {
    const int idx_x = (x - origin.x) / reso;
    const int idx_y = (y - origin.y) / reso;
    if (idx_x < 0 || idx_x >= width) return std::make_tuple(false, -1, -1);
    if (idx_y < 0 || idx_y >= height) return std::make_tuple(false, -1, -1);
    return std::make_tuple(true, idx_x, idx_y);
  };

  cv::Mat distance_grid(width, height, CV_8UC1, cv::Scalar(undef_pixel));
  cv::Mat projection_ind_grid(width, height, CV_32S, cv::Scalar(-1));

  const auto [lane_start, lane_end] = lane_detection_interval_ip;
  for (int i = static_cast<int>(lane_end); i >= static_cast<int>(lane_start); i--) {
    const auto & path_pos = path_ip.points.at(i).point.pose.position;
    const int idx_x = (path_pos.x - origin.x) / reso;
    const int idx_y = (path_pos.y - origin.y) / reso;
    if (idx_x < 0 || idx_x >= width) continue;
    if (idx_y < 0 || idx_y >= height) continue;
    distance_grid.at<unsigned char>(height - 1 - idx_y, idx_x) = zero_dist_pixel;
    projection_ind_grid.at<int>(height - 1 - idx_y, idx_x) = i;
  }

  for (const auto & lane_division : lane_divisions) {
    const auto & divisions = lane_division.divisions;
    for (const auto & division : divisions) {
      bool is_in_grid = false;
      bool zero_dist_cell_found = false;
      int projection_ind = -1;
      std::optional<std::tuple<double, double, double, int>> cost_prev_checkpoint =
        std::nullopt;  // cost, x, y, projection_ind
      for (const auto & point : division) {
        const auto [valid, idx_x, idx_y] = coord2index(point.x(), point.y());
        // exited grid just now
        if (is_in_grid && !valid) break;

        // still not entering grid
        if (!is_in_grid && !valid) continue;

        // From here, "valid"
        const int pixel = distance_grid.at<unsigned char>(height - 1 - idx_y, idx_x);

        // entered grid for 1st time
        if (!is_in_grid) {
          assert(pixel == undef_pixel || pixel == zero_dist_pixel);
          is_in_grid = true;
          if (pixel == undef_pixel) {
            continue;
          }
        }

        if (pixel == zero_dist_pixel) {
          zero_dist_cell_found = true;
          projection_ind = projection_ind_grid.at<int>(height - 1 - idx_y, idx_x);
          assert(projection_ind >= 0);
          cost_prev_checkpoint = std::make_optional<std::tuple<double, double, double, int>>(
            0.0, point.x(), point.y(), projection_ind);
          continue;
        }

        if (zero_dist_cell_found) {
          // finally traversed to defined cell (first half)
          const auto [prev_cost, prev_checkpoint_x, prev_checkpoint_y, prev_projection_ind] =
            cost_prev_checkpoint.value();
          const double dy = point.y() - prev_checkpoint_y, dx = point.x() - prev_checkpoint_x;
          double new_dist = prev_cost + std::hypot(dy, dx);
          const int new_projection_ind = projection_ind_grid.at<int>(height - 1 - idx_y, idx_x);
          const double cur_dist = pixel2dist(pixel);
          if (planner_param_.occlusion.do_dp && cur_dist < new_dist) {
            new_dist = cur_dist;
            if (new_projection_ind > 0) {
              projection_ind = std::min<int>(prev_projection_ind, new_projection_ind);
            }
          }
          projection_ind_grid.at<int>(height - 1 - idx_y, idx_x) = projection_ind;
          distance_grid.at<unsigned char>(height - 1 - idx_y, idx_x) = dist2pixel(new_dist);
          cost_prev_checkpoint = std::make_optional<std::tuple<double, double, double, int>>(
            new_dist, point.x(), point.y(), projection_ind);
        }
      }
    }
  }

  // clean-up and find nearest risk
  const int min_cost_thr = dist2pixel(occlusion_dist_thr);
  int min_cost = undef_pixel - 1;
  int max_cost = 0;
  std::optional<int> min_cost_projection_ind = std::nullopt;
  geometry_msgs::msg::Point nearest_occlusion_point;
  for (int i = 0; i < width; ++i) {
    for (int j = 0; j < height; ++j) {
      const int pixel = static_cast<int>(distance_grid.at<unsigned char>(height - 1 - j, i));
      const bool occluded = (occlusion_mask.at<unsigned char>(height - 1 - j, i) == 255);
      if (pixel == undef_pixel || !occluded) {
        // ignore outside of cropped
        // some cell maybe undef still
        distance_grid.at<unsigned char>(height - 1 - j, i) = 0;
        continue;
      }
      if (max_cost < pixel) {
        max_cost = pixel;
      }
      const int projection_ind = projection_ind_grid.at<int>(height - 1 - j, i);
      if (pixel < min_cost) {
        assert(projection_ind >= 0);
        min_cost = pixel;
        min_cost_projection_ind = projection_ind;
        nearest_occlusion_point.x = origin.x + i * reso;
        nearest_occlusion_point.y = origin.y + j * reso;
        nearest_occlusion_point.z = origin.z + distance_max * pixel / max_cost_pixel;
      }
    }
  }
  debug_data_.nearest_occlusion_point = nearest_occlusion_point;

  cv::Mat distance_grid_heatmap;
  cv::applyColorMap(distance_grid, distance_grid_heatmap, cv::COLORMAP_JET);
  /*
  cv::namedWindow("distance_grid_viz" + std::to_string(lane_id_), cv::WINDOW_NORMAL);
  cv::imshow("distance_grid_viz" + std::to_string(lane_id_), distance_grid_heatmap);
  cv::waitKey(1);
  */
  grid_map::GridMap occlusion_grid({"elevation"});
  occlusion_grid.setFrameId("map");
  occlusion_grid.setGeometry(
    grid_map::Length(width * reso, height * reso), reso,
    grid_map::Position(origin.x + width * reso / 2, origin.y + height * reso / 2));
  cv::rotate(distance_grid, distance_grid, cv::ROTATE_90_COUNTERCLOCKWISE);
  cv::rotate(distance_grid_heatmap, distance_grid_heatmap, cv::ROTATE_90_COUNTERCLOCKWISE);
  grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 1>(
    distance_grid, "elevation", occlusion_grid, origin.z /* elevation for 0 */,
    origin.z + distance_max /* elevation for 255 */);
  grid_map::GridMapCvConverter::addColorLayerFromImage<unsigned char, 3>(
    distance_grid_heatmap, "color", occlusion_grid);
  occlusion_grid_pub_->publish(grid_map::GridMapRosConverter::toMessage(occlusion_grid));
  if (min_cost > min_cost_thr || !min_cost_projection_ind.has_value()) {
    return true;
  } else {
    return false;
  }
}

}  // namespace behavior_velocity_planner
