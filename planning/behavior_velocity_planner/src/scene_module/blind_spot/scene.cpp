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
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <scene_module/blind_spot/scene.hpp>
#include <tier4_autoware_utils/geometry/path_with_lane_id_geometry.hpp>
#include <utilization/boost_geometry_helper.hpp>
#include <utilization/path_utilization.hpp>
#include <utilization/util.hpp>

#include <boost/geometry/algorithms/distance.hpp>

#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace behavior_velocity_planner
{
namespace bg = boost::geometry;

namespace
{
geometry_msgs::msg::Polygon toGeomPoly(const lanelet::CompoundPolygon3d & poly)
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
  planner_param_ = planner_param;

  const auto & assigned_lanelet =
    planner_data->route_handler_->getLaneletMapPtr()->laneletLayer.get(lane_id);
  const std::string turn_direction = assigned_lanelet.attributeOr("turn_direction", "else");
  if (!turn_direction.compare("left")) {
    turn_direction_ = TurnDirection::LEFT;
  } else if (!turn_direction.compare("right")) {
    turn_direction_ = TurnDirection::RIGHT;
  }
  has_traffic_light_ =
    !(assigned_lanelet.regulatoryElementsAs<const lanelet::TrafficLight>().empty());
}

bool BlindSpotModule::modifyPathVelocity(PathWithLaneId * path, StopReason * stop_reason)
{
  debug_data_ = DebugData();
  *stop_reason = planning_utils::initializeStopReason(StopReason::BLIND_SPOT);

  const auto input_path = *path;
  debug_data_.path_raw = input_path;

  StateMachine::State current_state = state_machine_.getState();
  RCLCPP_DEBUG(
    logger_, "lane_id = %ld, state = %s", lane_id_, StateMachine::toString(current_state).c_str());

  /* get current pose */
  geometry_msgs::msg::PoseStamped current_pose = planner_data_->current_pose;

  /* get lanelet map */
  const auto lanelet_map_ptr = planner_data_->route_handler_->getLaneletMapPtr();
  const auto routing_graph_ptr = planner_data_->route_handler_->getRoutingGraphPtr();

  /* set stop-line and stop-judgement-line for base_link */
  int stop_line_idx = -1;
  const auto straight_lanelets = getStraightLanelets(lanelet_map_ptr, routing_graph_ptr, lane_id_);
  if (!generateStopLine(straight_lanelets, path, &stop_line_idx)) {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      logger_, *clock_, 1000 /* ms */, "[BlindSpotModule::run] setStopLineIdx fail");
    *path = input_path;  // reset path
    return false;
  }

  if (stop_line_idx <= 0) {
    RCLCPP_DEBUG(
      logger_, "[Blind Spot] stop line or pass judge line is at path[0], ignore planning.");
    *path = input_path;  // reset path
    setSafe(true);
    setDistance(std::numeric_limits<double>::lowest());
    return true;
  }

  /* calc closest index */
  const auto closest_idx_opt =
    motion_utils::findNearestIndex(input_path.points, current_pose.pose, 3.0, M_PI_4);
  if (!closest_idx_opt) {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      logger_, *clock_, 1000 /* ms */, "[Blind Spot] motion_utils::findNearestIndex fail");
    *path = input_path;  // reset path
    return false;
  }
  const size_t closest_idx = closest_idx_opt.get();

  /* set judge line dist */
  const double current_vel = planner_data_->current_velocity->twist.linear.x;
  const double max_acc = planner_data_->max_stop_acceleration_threshold;
  const double delay_response_time = planner_data_->delay_response_time;
  const double pass_judge_line_dist =
    planning_utils::calcJudgeLineDistWithAccLimit(current_vel, max_acc, delay_response_time);
  const auto stop_point_pose = path->points.at(stop_line_idx).point.pose;
  const auto distance_until_stop = motion_utils::calcSignedArcLength(
    input_path.points, current_pose.pose, stop_point_pose.position);
  if (distance_until_stop == boost::none) return true;

  /* get debug info */
  const auto stop_line_pose = planning_utils::getAheadPose(
    stop_line_idx, planner_data_->vehicle_info_.max_longitudinal_offset_m, *path);
  debug_data_.virtual_wall_pose = stop_line_pose;
  const auto stop_pose = path->points.at(stop_line_idx).point.pose;
  debug_data_.stop_point_pose = stop_pose;
  auto offset_pose = motion_utils::calcLongitudinalOffsetPose(
    path->points, stop_pose.position, -pass_judge_line_dist);
  if (offset_pose) debug_data_.judge_point_pose = *offset_pose;

  /* if current_state = GO, and current_pose is over judge_line, ignore planning. */
  if (planner_param_.use_pass_judge_line) {
    const double eps = 1e-1;  // to prevent hunting
    if (
      current_state == StateMachine::State::GO &&
      *distance_until_stop + eps < pass_judge_line_dist) {
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
    path->points, current_pose.pose.position, path->points.at(stop_line_idx).point.pose.position));
  if (!isActivated()) {
    constexpr double stop_vel = 0.0;
    planning_utils::setVelocityFromIndex(stop_line_idx, stop_vel, path);

    /* get stop point and stop factor */
    tier4_planning_msgs::msg::StopFactor stop_factor;
    stop_factor.stop_pose = debug_data_.stop_point_pose;
    stop_factor.stop_factor_points = planning_utils::toRosPoints(debug_data_.conflicting_targets);
    planning_utils::appendStopReason(stop_factor, stop_reason);
  } else {
    *path = input_path;  // reset path
  }
  return true;
}

boost::optional<int> BlindSpotModule::getFirstPointConflictingLanelets(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
  const lanelet::ConstLanelets & lanelets) const
{
  using lanelet::utils::to2D;
  using lanelet::utils::toHybrid;

  int first_idx_conflicting_lanelets = path.points.size() - 1;
  bool is_conflict = false;
  for (const auto & ll : lanelets) {
    const auto line = (turn_direction_ == TurnDirection::LEFT) ? ll.leftBound() : ll.rightBound();
    for (size_t i = 0; i < path.points.size(); ++i) {
      const auto vehicle_edge = getVehicleEdge(
        path.points.at(i).point.pose, planner_data_->vehicle_info_.vehicle_width_m,
        planner_data_->vehicle_info_.max_longitudinal_offset_m);
      if (bg::intersects(toHybrid(to2D(line)), toHybrid(vehicle_edge))) {
        first_idx_conflicting_lanelets =
          std::min(first_idx_conflicting_lanelets, static_cast<int>(i));
        is_conflict = true;
        break;
      }
    }
  }
  if (is_conflict) {
    return first_idx_conflicting_lanelets;
  } else {
    return boost::none;
  }
}

bool BlindSpotModule::generateStopLine(
  const lanelet::ConstLanelets straight_lanelets,
  autoware_auto_planning_msgs::msg::PathWithLaneId * path, int * stop_line_idx) const
{
  /* set parameters */
  constexpr double interval = 0.2;
  const int margin_idx_dist = std::ceil(planner_param_.stop_line_margin / interval);
  const int base2front_idx_dist =
    std::ceil(planner_data_->vehicle_info_.max_longitudinal_offset_m / interval);

  /* spline interpolation */
  autoware_auto_planning_msgs::msg::PathWithLaneId path_ip;
  if (!splineInterpolate(*path, interval, path_ip, logger_)) {
    return false;
  }
  debug_data_.spline_path = path_ip;

  /* generate stop point */
  int stop_idx_ip = 0;  // stop point index for interpolated path.
  if (straight_lanelets.size() > 0) {
    boost::optional<int> first_idx_conflicting_lane_opt =
      getFirstPointConflictingLanelets(path_ip, straight_lanelets);
    if (!first_idx_conflicting_lane_opt) {
      RCLCPP_DEBUG(logger_, "No conflicting line found.");
      return false;
    }
    stop_idx_ip =
      std::max(first_idx_conflicting_lane_opt.get() - 1 - margin_idx_dist - base2front_idx_dist, 0);
  } else {
    boost::optional<geometry_msgs::msg::Pose> intersection_enter_point_opt =
      getStartPointFromLaneLet(lane_id_);
    if (!intersection_enter_point_opt) {
      RCLCPP_DEBUG(logger_, "No intersection enter point found.");
      return false;
    }

    geometry_msgs::msg::Pose intersection_enter_pose;
    intersection_enter_pose = intersection_enter_point_opt.get();
    const auto stop_idx_ip_opt =
      motion_utils::findNearestIndex(path_ip.points, intersection_enter_pose, 10.0, M_PI_4);
    if (stop_idx_ip_opt) {
      stop_idx_ip = stop_idx_ip_opt.get();
    }

    stop_idx_ip = std::max(stop_idx_ip - base2front_idx_dist, 0);
  }

  /* insert stop_point to use interpolated path*/
  *stop_line_idx = insertPoint(stop_idx_ip, path_ip, path);

  /* if another stop point exist before intersection stop_line, disable judge_line. */
  bool has_prior_stopline = false;
  for (int i = 0; i < *stop_line_idx; ++i) {
    if (std::fabs(path->points.at(i).point.longitudinal_velocity_mps) < 0.1) {
      has_prior_stopline = true;
      break;
    }
  }

  RCLCPP_DEBUG(
    logger_, "generateStopLine() : stop_idx = %d, stop_idx_ip = %d, has_prior_stopline = %d",
    *stop_line_idx, stop_idx_ip, has_prior_stopline);

  return true;
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

int BlindSpotModule::insertPoint(
  const int insert_idx_ip, const autoware_auto_planning_msgs::msg::PathWithLaneId path_ip,
  autoware_auto_planning_msgs::msg::PathWithLaneId * inout_path) const
{
  double insert_point_s = 0.0;
  for (int i = 1; i <= insert_idx_ip; i++) {
    insert_point_s += tier4_autoware_utils::calcDistance2d(
      path_ip.points[i].point.pose.position, path_ip.points[i - 1].point.pose.position);
  }
  int insert_idx = -1;
  // initialize with epsilon so that comparison with insert_point_s = 0.0 would work
  constexpr double eps = 1e-2;
  double accum_s = eps + std::numeric_limits<double>::epsilon();
  for (size_t i = 1; i < inout_path->points.size(); i++) {
    accum_s += tier4_autoware_utils::calcDistance2d(
      inout_path->points[i].point.pose.position, inout_path->points[i - 1].point.pose.position);
    if (accum_s > insert_point_s) {
      insert_idx = i;
      break;
    }
  }
  if (insert_idx >= 0) {
    const auto it = inout_path->points.begin() + insert_idx;
    autoware_auto_planning_msgs::msg::PathPointWithLaneId inserted_point;
    // copy from previous point
    inserted_point = inout_path->points.at(std::max(insert_idx - 1, 0));
    inserted_point.point.pose = path_ip.points[insert_idx_ip].point.pose;
    constexpr double min_dist = eps;  // to make sure path point is forward insert index
    //! avoid to insert duplicated point
    if (
      tier4_autoware_utils::calcDistance2d(
        inserted_point, inout_path->points.at(insert_idx).point) < min_dist) {
      inout_path->points.at(insert_idx).point.longitudinal_velocity_mps = 0.0;
      return insert_idx;
    } else if (
      insert_idx != 0 &&
      tier4_autoware_utils::calcDistance2d(
        inserted_point, inout_path->points.at(static_cast<size_t>(insert_idx - 1)).point) <
        min_dist) {
      inout_path->points.at(insert_idx - 1).point.longitudinal_velocity_mps = 0.0;
      insert_idx--;
      return insert_idx;
    }
    inout_path->points.insert(it, inserted_point);
  }
  return insert_idx;
}

bool BlindSpotModule::checkObstacleInBlindSpot(
  lanelet::LaneletMapConstPtr lanelet_map_ptr, lanelet::routing::RoutingGraphPtr routing_graph_ptr,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
  const autoware_auto_perception_msgs::msg::PredictedObjects::ConstSharedPtr objects_ptr,
  const int closest_idx, const geometry_msgs::msg::Pose & stop_line_pose) const
{
  /* get detection area */
  if (turn_direction_ == TurnDirection::INVALID) {
    RCLCPP_WARN(logger_, "blind spot detector is running, turn_direction_ = not right or left.");
    return false;
  }

  const auto areas_opt = generateBlindSpotPolygons(
    lanelet_map_ptr, routing_graph_ptr, path, closest_idx, stop_line_pose);
  if (!!areas_opt) {
    debug_data_.detection_area_for_blind_spot = toGeomPoly(areas_opt.get().detection_area);
    debug_data_.conflict_area_for_blind_spot = toGeomPoly(areas_opt.get().conflict_area);

    autoware_auto_perception_msgs::msg::PredictedObjects objects = *objects_ptr;
    cutPredictPathWithDuration(&objects, planner_param_.max_future_movement_time);

    // check objects in blind spot areas
    bool obstacle_detected = false;
    for (const auto & object : objects.objects) {
      if (!isTargetObjectType(object)) {
        continue;
      }

      bool exist_in_detection_area = bg::within(
        to_bg2d(object.kinematics.initial_pose_with_covariance.pose.position),
        lanelet::utils::to2D(areas_opt.get().detection_area));
      bool exist_in_conflict_area = isPredictedPathInArea(object, areas_opt.get().conflict_area);
      if (exist_in_detection_area || exist_in_conflict_area) {
        obstacle_detected = true;
        debug_data_.conflicting_targets.objects.push_back(object);
      }
    }
    return obstacle_detected;
  } else {
    return false;
  }
}

bool BlindSpotModule::isPredictedPathInArea(
  const autoware_auto_perception_msgs::msg::PredictedObject & object,
  const lanelet::CompoundPolygon3d & area) const
{
  const auto area_2d = lanelet::utils::to2D(area);
  // NOTE: iterating all paths including those of low confidence
  return std::any_of(
    object.kinematics.predicted_paths.begin(), object.kinematics.predicted_paths.end(),
    [&area_2d](const auto & path) {
      return std::any_of(path.path.begin(), path.path.end(), [&area_2d](const auto & point) {
        return bg::within(to_bg2d(point.position), area_2d);
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
  const auto left_bound = lanelet::LineString3d(lanelet::InvalId, lefts);
  const auto right_bound = lanelet::LineString3d(lanelet::InvalId, rights);
  auto half_lanelet = lanelet::Lanelet(lanelet::InvalId, left_bound, right_bound);
  const auto centerline = lanelet::utils::generateFineCenterline(half_lanelet, 5.0);
  half_lanelet.setCenterline(centerline);
  return std::move(half_lanelet);
}

boost::optional<BlindSpotPolygons> BlindSpotModule::generateBlindSpotPolygons(
  lanelet::LaneletMapConstPtr lanelet_map_ptr,
  [[maybe_unused]] lanelet::routing::RoutingGraphPtr routing_graph_ptr,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const int closest_idx,
  const geometry_msgs::msg::Pose & stop_line_pose) const
{
  std::vector<int64_t> lane_ids;
  lanelet::ConstLanelets blind_spot_lanelets;
  /* get lane ids until intersection */
  for (const auto & point : path.points) {
    bool found_intersection_lane = false;
    for (const auto lane_id : point.lane_ids) {
      // make lane_ids unique
      if (std::find(lane_ids.begin(), lane_ids.end(), lane_id) == lane_ids.end()) {
        lane_ids.push_back(lane_id);
      }

      if (lane_id == lane_id_) {
        found_intersection_lane = true;
        break;
      }
    }
    if (found_intersection_lane) break;
  }

  for (size_t i = 0; i < lane_ids.size(); ++i) {
    const auto half_lanelet =
      generateHalfLanelet(lanelet_map_ptr->laneletLayer.get(lane_ids.at(i)));
    blind_spot_lanelets.push_back(half_lanelet);
  }

  const auto current_arc =
    lanelet::utils::getArcCoordinates(blind_spot_lanelets, path.points[closest_idx].point.pose);
  const auto stop_line_arc = lanelet::utils::getArcCoordinates(blind_spot_lanelets, stop_line_pose);
  const auto detection_area_start_length = stop_line_arc.length - planner_param_.backward_length;
  if (
    detection_area_start_length < current_arc.length && current_arc.length < stop_line_arc.length) {
    const auto conflict_area = lanelet::utils::getPolygonFromArcLength(
      blind_spot_lanelets, current_arc.length, stop_line_arc.length);
    const auto detection_area = lanelet::utils::getPolygonFromArcLength(
      blind_spot_lanelets, detection_area_start_length, stop_line_arc.length);

    BlindSpotPolygons blind_spot_polygons;
    blind_spot_polygons.conflict_area = conflict_area;
    blind_spot_polygons.detection_area = detection_area;

    return blind_spot_polygons;
  } else {
    return boost::none;
  }
}

lanelet::LineString2d BlindSpotModule::getVehicleEdge(
  const geometry_msgs::msg::Pose & vehicle_pose, const double vehicle_width,
  const double base_link2front) const
{
  lanelet::LineString2d vehicle_edge;
  tf2::Vector3 front_left, front_right, rear_left, rear_right;
  front_left.setValue(base_link2front, vehicle_width / 2, 0);
  front_right.setValue(base_link2front, -vehicle_width / 2, 0);
  rear_left.setValue(0, vehicle_width / 2, 0);
  rear_right.setValue(0, -vehicle_width / 2, 0);

  tf2::Transform tf;
  tf2::fromMsg(vehicle_pose, tf);
  const auto front_left_transformed = tf * front_left;
  const auto front_right_transformed = tf * front_right;
  const auto rear_left_transformed = tf * rear_left;
  const auto rear_right_transformed = tf * rear_right;

  if (turn_direction_ == TurnDirection::LEFT) {
    vehicle_edge.push_back(
      lanelet::Point2d(0, front_left_transformed.x(), front_left_transformed.y()));
    vehicle_edge.push_back(
      lanelet::Point2d(0, rear_left_transformed.x(), rear_left_transformed.y()));
  } else if (turn_direction_ == TurnDirection::RIGHT) {
    vehicle_edge.push_back(
      lanelet::Point2d(0, front_right_transformed.x(), front_right_transformed.y()));
    vehicle_edge.push_back(
      lanelet::Point2d(0, rear_right_transformed.x(), rear_right_transformed.y()));
  }
  return vehicle_edge;
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

boost::optional<geometry_msgs::msg::Pose> BlindSpotModule::getStartPointFromLaneLet(
  const int lane_id) const
{
  lanelet::ConstLanelet lanelet =
    planner_data_->route_handler_->getLaneletMapPtr()->laneletLayer.get(lane_id);
  if (lanelet.centerline().empty()) {
    return boost::none;
  }
  const auto p = lanelet.centerline().front();
  geometry_msgs::msg::Point start_point;
  start_point.x = p.x();
  start_point.y = p.y();
  start_point.z = p.z();
  const double yaw = lanelet::utils::getLaneletAngle(lanelet, start_point);
  geometry_msgs::msg::Pose start_pose;
  start_pose.position = start_point;
  tf2::Quaternion quat;
  quat.setRPY(0, 0, yaw);
  start_pose.orientation = tf2::toMsg(quat);

  return start_pose;
}

lanelet::ConstLanelets BlindSpotModule::getStraightLanelets(
  lanelet::LaneletMapConstPtr lanelet_map_ptr, lanelet::routing::RoutingGraphPtr routing_graph_ptr,
  const int lane_id)
{
  lanelet::ConstLanelets straight_lanelets;
  const auto intersection_lanelet = lanelet_map_ptr->laneletLayer.get(lane_id);
  const auto prev_intersection_lanelets = routing_graph_ptr->previous(intersection_lanelet);
  if (prev_intersection_lanelets.empty()) {
    return straight_lanelets;
  }

  const auto next_lanelets = routing_graph_ptr->following(prev_intersection_lanelets.front());
  for (const auto & ll : next_lanelets) {
    const std::string turn_direction = ll.attributeOr("turn_direction", "else");
    if (!turn_direction.compare("straight")) {
      straight_lanelets.push_back(ll);
    }
  }
  return straight_lanelets;
}
}  // namespace behavior_velocity_planner
