// Copyright 2022 TIER IV, Inc.
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

#include "behavior_velocity_crosswalk_module/util.hpp"
#include "path_utils.hpp"

#include <behavior_velocity_planner_common/utilization/trajectory_utils.hpp>
#include <behavior_velocity_planner_common/utilization/util.hpp>
#include <motion_utils/distance/distance.hpp>
#include <motion_utils/trajectory/trajectory.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>

#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/algorithms/within.hpp>

#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_core/geometry/Polygon.h>

#include <algorithm>
#include <limits>
#include <utility>

namespace behavior_velocity_planner
{
namespace bg = boost::geometry;

RunOutModule::RunOutModule(
  const int64_t module_id, const std::shared_ptr<const PlannerData> & planner_data,
  const PlannerParam & planner_param, const rclcpp::Logger logger,
  std::unique_ptr<DynamicObstacleCreator> dynamic_obstacle_creator,
  const std::shared_ptr<RunOutDebug> & debug_ptr, const rclcpp::Clock::SharedPtr clock)
: SceneModuleInterface(module_id, logger, clock),
  planner_param_(planner_param),
  dynamic_obstacle_creator_(std::move(dynamic_obstacle_creator)),
  debug_ptr_(debug_ptr),
  state_machine_(std::make_unique<run_out_utils::StateMachine>(planner_param.state_param))
{
  velocity_factor_.init(PlanningBehavior::UNKNOWN);

  if (planner_param.run_out.use_partition_lanelet) {
    const lanelet::LaneletMapConstPtr & ll = planner_data->route_handler_->getLaneletMapPtr();
    planning_utils::getAllPartitionLanelets(ll, partition_lanelets_);
  }
}

void RunOutModule::setPlannerParam(const PlannerParam & planner_param)
{
  planner_param_ = planner_param;
}

bool RunOutModule::modifyPathVelocity(
  PathWithLaneId * path, [[maybe_unused]] StopReason * stop_reason)
{
  // timer starts
  const auto t_start = std::chrono::system_clock::now();

  // set planner data
  const auto current_vel = planner_data_->current_velocity->twist.linear.x;
  const auto current_acc = planner_data_->current_acceleration->accel.accel.linear.x;
  const auto & current_pose = planner_data_->current_odometry->pose;

  // set height of debug data
  debug_ptr_->setHeight(current_pose.position.z);

  // extend path to consider obstacles after the goal
  const auto extended_path =
    run_out_utils::extendPath(*path, planner_param_.vehicle_param.base_to_front);

  // trim path ahead of the base_link to make calculation easier
  const double trim_distance = planner_param_.run_out.detection_distance;
  const auto trim_path =
    run_out_utils::trimPathFromSelfPose(extended_path, current_pose, trim_distance);

  // smooth velocity of the path to calculate time to collision accurately
  PathWithLaneId extended_smoothed_path;
  if (!smoothPath(trim_path, extended_smoothed_path, planner_data_)) {
    return true;
  }

  // record time for path processing
  const auto t_path_processing = std::chrono::system_clock::now();
  const auto elapsed_path_processing =
    std::chrono::duration_cast<std::chrono::microseconds>(t_path_processing - t_start);
  debug_ptr_->setDebugValues(
    DebugValues::TYPE::CALCULATION_TIME_PATH_PROCESSING, elapsed_path_processing.count() / 1000.0);

  // create abstracted dynamic obstacles from objects or points
  dynamic_obstacle_creator_->setData(*planner_data_, planner_param_, *path, extended_smoothed_path);
  const auto dynamic_obstacles = dynamic_obstacle_creator_->createDynamicObstacles();
  debug_ptr_->setDebugValues(DebugValues::TYPE::NUM_OBSTACLES, dynamic_obstacles.size());

  // extract obstacles using lanelet information
  const auto partition_excluded_obstacles =
    excludeObstaclesOutSideOfPartition(dynamic_obstacles, extended_smoothed_path, current_pose);

  // record time for obstacle creation
  const auto t_obstacle_creation = std::chrono::system_clock::now();
  const auto elapsed_obstacle_creation =
    std::chrono::duration_cast<std::chrono::microseconds>(t_obstacle_creation - t_path_processing);
  debug_ptr_->setDebugValues(
    DebugValues::TYPE::CALCULATION_TIME_OBSTACLE_CREATION,
    elapsed_obstacle_creation.count() / 1000.0);

  // detect collision with dynamic obstacles using velocity planning of ego
  const auto crosswalk_lanelets = planner_param_.run_out.suppress_on_crosswalk
                                    ? getCrosswalksOnPath(
                                        planner_data_->current_odometry->pose, *path,
                                        planner_data_->route_handler_->getLaneletMapPtr(),
                                        planner_data_->route_handler_->getOverallGraphPtr())
                                    : std::vector<std::pair<int64_t, lanelet::ConstLanelet>>();
  const auto dynamic_obstacle =
    detectCollision(partition_excluded_obstacles, extended_smoothed_path, crosswalk_lanelets);

  // record time for collision check
  const auto t_collision_check = std::chrono::system_clock::now();
  const auto elapsed_collision_check =
    std::chrono::duration_cast<std::chrono::microseconds>(t_collision_check - t_obstacle_creation);
  debug_ptr_->setDebugValues(
    DebugValues::TYPE::CALCULATION_TIME_COLLISION_CHECK, elapsed_collision_check.count() / 1000.0);

  // insert stop point for the detected obstacle
  if (planner_param_.approaching.enable) {
    // after a certain amount of time has elapsed since the ego stopped,
    // approach the obstacle with slow velocity
    insertVelocityForState(
      dynamic_obstacle, *planner_data_, planner_param_, extended_smoothed_path, *path);
  } else {
    // just insert zero velocity for stopping
    insertStoppingVelocity(dynamic_obstacle, current_pose, current_vel, current_acc, *path);
  }

  // apply max jerk limit if the ego can't stop with specified max jerk and acc
  if (planner_param_.slow_down_limit.enable) {
    applyMaxJerkLimit(current_pose, current_vel, current_acc, *path);
  }

  publishDebugValue(
    extended_smoothed_path, partition_excluded_obstacles, dynamic_obstacle, current_pose);

  // record time for collision check
  const auto t_path_planning = std::chrono::system_clock::now();
  const auto elapsed_path_planning =
    std::chrono::duration_cast<std::chrono::microseconds>(t_path_planning - t_collision_check);
  debug_ptr_->setDebugValues(
    DebugValues::TYPE::CALCULATION_TIME_PATH_PLANNING, elapsed_path_planning.count() / 1000.0);

  // record time for the function
  const auto t_end = std::chrono::system_clock::now();
  const auto elapsed_all = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start);
  debug_ptr_->setDebugValues(DebugValues::TYPE::CALCULATION_TIME, elapsed_all.count() / 1000.0);

  return true;
}

std::optional<DynamicObstacle> RunOutModule::detectCollision(
  const std::vector<DynamicObstacle> & dynamic_obstacles, const PathWithLaneId & path,
  const std::vector<std::pair<int64_t, lanelet::ConstLanelet>> & crosswalk_lanelets)
{
  if (path.points.size() < 2) {
    RCLCPP_WARN_STREAM(logger_, "path doesn't have enough points.");
    return {};
  }

  // detect collision with obstacles from the nearest path point to the end
  // ignore the travel time from current pose to nearest path point?
  float travel_time = 0.0;
  float dist_sum = 0.0;
  for (size_t idx = 1; idx < path.points.size(); idx++) {
    const auto & p1 = path.points.at(idx - 1).point;
    const auto & p2 = path.points.at(idx).point;
    const float prev_vel =
      std::max(p1.longitudinal_velocity_mps, planner_param_.run_out.min_vel_ego_kmph / 3.6f);
    const float ds = tier4_autoware_utils::calcDistance2d(p1, p2);

    // calculate travel time from nearest point to p2
    travel_time += ds / prev_vel;
    dist_sum += ds;

    // skip collision detection to reduce calculation time
    if (idx != 1 && dist_sum < planner_param_.run_out.detection_span) {
      continue;
    }
    dist_sum = 0.0;

    const auto vehicle_poly = createVehiclePolygon(p2.pose);

    debug_ptr_->pushPredictedVehiclePolygons(vehicle_poly);
    debug_ptr_->pushTravelTimeTexts(travel_time, p2.pose, /* lateral_offset */ 3.0);

    auto obstacles_collision =
      checkCollisionWithObstacles(dynamic_obstacles, vehicle_poly, travel_time, crosswalk_lanelets);
    if (obstacles_collision.empty()) {
      continue;
    }

    const auto obstacle_selected = findNearestCollisionObstacle(path, p2.pose, obstacles_collision);
    if (!obstacle_selected) {
      continue;
    }

    // ignore momentary obstacle detection to avoid sudden stopping by false positive
    if (isMomentaryDetection()) {
      return {};
    }

    debug_ptr_->pushCollisionPoints(obstacle_selected->collision_points);
    debug_ptr_->pushNearestCollisionPoint(obstacle_selected->nearest_collision_point);

    return obstacle_selected;
  }

  // no collision detected
  first_detected_time_.reset();
  return {};
}

std::optional<DynamicObstacle> RunOutModule::findNearestCollisionObstacle(
  const PathWithLaneId & path, const geometry_msgs::msg::Pose & base_pose,
  std::vector<DynamicObstacle> & dynamic_obstacles) const
{
  // sort obstacles with distance from ego
  std::sort(
    dynamic_obstacles.begin(), dynamic_obstacles.end(),
    [&path, &base_pose](const auto & lhs, const auto & rhs) -> bool {
      const auto dist_lhs =
        motion_utils::calcSignedArcLength(path.points, base_pose.position, lhs.pose.position);
      const auto dist_rhs =
        motion_utils::calcSignedArcLength(path.points, base_pose.position, rhs.pose.position);

      return dist_lhs < dist_rhs;
    });

  // select obstacle to decelerate from the nearest obstacle
  DynamicObstacle obstacle_collision;
  for (const auto & obstacle : dynamic_obstacles) {
    const auto obstacle_same_side_points = run_out_utils::findLateralSameSidePoints(
      obstacle.collision_points, base_pose, obstacle.pose.position);

    const auto nearest_collision_point = run_out_utils::findLongitudinalNearestPoint(
      path.points, base_pose.position, obstacle_same_side_points);

    const auto collision_position_from_ego_front =
      calcCollisionPositionOfVehicleSide(nearest_collision_point, base_pose);

    // if position of collision on ego side is less than passing margin,
    // which is considered to be collision
    // TODO(Tomohito Ando): calculate collision position more precisely
    if (collision_position_from_ego_front < planner_param_.run_out.passing_margin) {
      debug_ptr_->setDebugValues(
        DebugValues::TYPE::COLLISION_POS_FROM_EGO_FRONT, collision_position_from_ego_front);

      obstacle_collision = obstacle;
      obstacle_collision.nearest_collision_point = nearest_collision_point;
      return obstacle_collision;
    }

    // the obstacle is considered to be able to pass
    debug_ptr_->setAccelReason(RunOutDebug::AccelReason::PASS);
  }

  // no collision points
  return {};
}

// calculate longitudinal offset of collision point from vehicle front
float RunOutModule::calcCollisionPositionOfVehicleSide(
  const geometry_msgs::msg::Point & point, const geometry_msgs::msg::Pose & base_pose) const
{
  const auto vehicle_front_pose = tier4_autoware_utils::calcOffsetPose(
    base_pose, planner_param_.vehicle_param.base_to_front, 0, 0);
  const auto longitudinal_offset_from_front =
    std::abs(tier4_autoware_utils::calcLongitudinalDeviation(vehicle_front_pose, point));

  return longitudinal_offset_from_front;
}

/**
 *    p4               p1
 *    +----------------+
 *    |  ↑-> (base)   |
 *    +----------------+
 *    p3               p2
 */
std::vector<geometry_msgs::msg::Point> RunOutModule::createVehiclePolygon(
  const geometry_msgs::msg::Pose & base_pose) const
{
  const float base_to_rear = planner_param_.vehicle_param.base_to_rear;
  const float base_to_front = planner_param_.vehicle_param.base_to_front;
  const double base_to_right =
    (planner_param_.vehicle_param.wheel_tread / 2.0) + planner_param_.vehicle_param.right_overhang;
  const double base_to_left =
    (planner_param_.vehicle_param.wheel_tread / 2.0) + planner_param_.vehicle_param.left_overhang;

  using tier4_autoware_utils::calcOffsetPose;
  const auto p1 = calcOffsetPose(base_pose, base_to_front, base_to_left, 0.0);
  const auto p2 = calcOffsetPose(base_pose, base_to_front, -base_to_right, 0.0);
  const auto p3 = calcOffsetPose(base_pose, -base_to_rear, -base_to_right, 0.0);
  const auto p4 = calcOffsetPose(base_pose, -base_to_rear, base_to_left, 0.0);

  std::vector<geometry_msgs::msg::Point> vehicle_poly;
  vehicle_poly.push_back(p1.position);
  vehicle_poly.push_back(p2.position);
  vehicle_poly.push_back(p3.position);
  vehicle_poly.push_back(p4.position);

  return vehicle_poly;
}

std::vector<DynamicObstacle> RunOutModule::checkCollisionWithObstacles(
  const std::vector<DynamicObstacle> & dynamic_obstacles,
  std::vector<geometry_msgs::msg::Point> poly, const float travel_time,
  const std::vector<std::pair<int64_t, lanelet::ConstLanelet>> & crosswalk_lanelets) const
{
  const auto bg_poly_vehicle = run_out_utils::createBoostPolyFromMsg(poly);

  // check collision for each objects
  std::vector<DynamicObstacle> obstacles_collision;
  for (const auto & obstacle : dynamic_obstacles) {
    // get classification that has highest probability
    const auto classification = run_out_utils::getHighestProbLabel(obstacle.classifications);

    // detect only pedestrian and bicycle
    if (
      classification != ObjectClassification::PEDESTRIAN &&
      classification != ObjectClassification::BICYCLE) {
      continue;
    }

    // calculate predicted obstacle pose for min velocity and max velocity
    const auto predicted_obstacle_pose_min_vel =
      calcPredictedObstaclePose(obstacle.predicted_paths, travel_time, obstacle.min_velocity_mps);
    const auto predicted_obstacle_pose_max_vel =
      calcPredictedObstaclePose(obstacle.predicted_paths, travel_time, obstacle.max_velocity_mps);
    if (!predicted_obstacle_pose_min_vel || !predicted_obstacle_pose_max_vel) {
      continue;
    }
    const PoseWithRange pose_with_range = {
      *predicted_obstacle_pose_min_vel, *predicted_obstacle_pose_max_vel};

    std::vector<geometry_msgs::msg::Point> collision_points;
    const bool collision_detected = checkCollisionWithShape(
      bg_poly_vehicle, pose_with_range, obstacle.shape, crosswalk_lanelets, collision_points);

    if (!collision_detected) {
      continue;
    }

    DynamicObstacle obstacle_collision = obstacle;
    obstacle_collision.collision_points = collision_points;
    obstacles_collision.emplace_back(obstacle_collision);
  }

  return obstacles_collision;
}

// calculate the predicted pose of the obstacle on the predicted path with given travel time
// assume that the obstacle moves with constant velocity
std::optional<geometry_msgs::msg::Pose> RunOutModule::calcPredictedObstaclePose(
  const std::vector<PredictedPath> & predicted_paths, const float travel_time,
  const float velocity_mps) const
{
  // use the path that has highest confidence for now
  const auto predicted_path = run_out_utils::getHighestConfidencePath(predicted_paths);

  if (predicted_path.size() < 2) {
    RCLCPP_WARN_STREAM(logger_, "predicted path doesn't have enough points");
    return {};
  }

  if (
    travel_time < std::numeric_limits<float>::epsilon() ||
    velocity_mps < std::numeric_limits<float>::epsilon()) {
    return predicted_path.at(0);
  }

  // calculate predicted pose
  float time_sum = 0.0;
  for (size_t i = 1; i < predicted_path.size(); i++) {
    const auto & p1 = predicted_path.at(i - 1);
    const auto & p2 = predicted_path.at(i);

    const float ds = tier4_autoware_utils::calcDistance2d(p1, p2);
    const float dt = ds / velocity_mps;

    // apply linear interpolation between the predicted path points
    if (time_sum + dt > travel_time) {
      const float time_remaining = travel_time - time_sum;
      const float ratio = time_remaining / dt;
      return run_out_utils::lerpByPose(p1, p2, ratio);
    }

    time_sum += dt;
  }

  // reach the end of the predicted path
  return predicted_path.back();
}

bool RunOutModule::checkCollisionWithShape(
  const Polygon2d & vehicle_polygon, const PoseWithRange pose_with_range, const Shape & shape,
  const std::vector<std::pair<int64_t, lanelet::ConstLanelet>> & crosswalk_lanelets,
  std::vector<geometry_msgs::msg::Point> & collision_points) const
{
  bool collision_detected = false;
  switch (shape.type) {
    case Shape::CYLINDER:
      collision_detected = checkCollisionWithCylinder(
        vehicle_polygon, pose_with_range, shape.dimensions.x / 2.0, collision_points);
      break;

    case Shape::BOUNDING_BOX:
      collision_detected = checkCollisionWithBoundingBox(
        vehicle_polygon, pose_with_range, shape.dimensions, collision_points);
      break;

    case Shape::POLYGON:
      collision_detected = checkCollisionWithPolygon();
      break;

    default:
      break;
  }

  if (!collision_points.empty()) {
    for (const auto & crosswalk : crosswalk_lanelets) {
      const auto poly = crosswalk.second.polygon2d().basicPolygon();
      for (auto it = collision_points.begin(); it != collision_points.end();) {
        if (boost::geometry::within(lanelet::BasicPoint2d(it->x, it->y), poly)) {
          it = collision_points.erase(it);
        } else {
          ++it;
        }
      }
      if (collision_points.empty()) {
        break;
      }
    }
    collision_detected = !collision_points.empty();
  }

  return collision_detected;
}

bool RunOutModule::checkCollisionWithCylinder(
  const Polygon2d & vehicle_polygon, const PoseWithRange pose_with_range, const float radius,
  std::vector<geometry_msgs::msg::Point> & collision_points) const
{
  // create bounding box for min and max velocity point
  const auto bounding_box_for_points =
    createBoundingBoxForRangedPoints(pose_with_range, radius, radius);
  const auto bg_bounding_box_for_points =
    run_out_utils::createBoostPolyFromMsg(bounding_box_for_points);

  // check collision with 2d polygon
  std::vector<tier4_autoware_utils::Point2d> collision_points_bg;
  bg::intersection(vehicle_polygon, bg_bounding_box_for_points, collision_points_bg);

  // no collision detected
  if (collision_points_bg.empty()) {
    debug_ptr_->pushPredictedObstaclePolygons(bounding_box_for_points);
    return false;
  }

  // detected collision
  debug_ptr_->pushCollisionObstaclePolygons(bounding_box_for_points);
  for (const auto & p : collision_points_bg) {
    const auto p_msg =
      tier4_autoware_utils::createPoint(p.x(), p.y(), pose_with_range.pose_min.position.z);
    collision_points.emplace_back(p_msg);
  }

  return true;
}

// create 2D bounding box for two points
// Box is better to reduce calculation cost?
std::vector<geometry_msgs::msg::Point> RunOutModule::createBoundingBoxForRangedPoints(
  const PoseWithRange & pose_with_range, const float x_offset, const float y_offset) const
{
  const auto dist_p1_p2 =
    tier4_autoware_utils::calcDistance2d(pose_with_range.pose_min, pose_with_range.pose_max);

  geometry_msgs::msg::Pose p_min_to_p_max;
  if (dist_p1_p2 < std::numeric_limits<double>::epsilon()) {
    // can't calculate the angle if two points are the same
    p_min_to_p_max = pose_with_range.pose_min;
  } else {
    const auto azimuth_angle = tier4_autoware_utils::calcAzimuthAngle(
      pose_with_range.pose_min.position, pose_with_range.pose_max.position);
    p_min_to_p_max.position = pose_with_range.pose_min.position;
    p_min_to_p_max.orientation = tier4_autoware_utils::createQuaternionFromYaw(azimuth_angle);
  }

  std::vector<geometry_msgs::msg::Point> poly;
  poly.emplace_back(
    tier4_autoware_utils::calcOffsetPose(p_min_to_p_max, dist_p1_p2 + x_offset, y_offset, 0.0)
      .position);
  poly.emplace_back(
    tier4_autoware_utils::calcOffsetPose(p_min_to_p_max, dist_p1_p2 + x_offset, -y_offset, 0.0)
      .position);
  poly.emplace_back(
    tier4_autoware_utils::calcOffsetPose(p_min_to_p_max, -x_offset, -y_offset, 0.0).position);
  poly.emplace_back(
    tier4_autoware_utils::calcOffsetPose(p_min_to_p_max, -x_offset, y_offset, 0.0).position);

  return poly;
}

bool RunOutModule::checkCollisionWithBoundingBox(
  const Polygon2d & vehicle_polygon, const PoseWithRange pose_with_range,
  const geometry_msgs::msg::Vector3 & dimension,
  std::vector<geometry_msgs::msg::Point> & collision_points) const
{
  // create bounding box for min and max velocity point
  const auto bounding_box =
    createBoundingBoxForRangedPoints(pose_with_range, dimension.x / 2.0, dimension.y / 2.0);
  const auto bg_bounding_box = run_out_utils::createBoostPolyFromMsg(bounding_box);

  // check collision with 2d polygon
  std::vector<tier4_autoware_utils::Point2d> collision_points_bg;
  bg::intersection(vehicle_polygon, bg_bounding_box, collision_points_bg);

  // no collision detected
  if (collision_points_bg.empty()) {
    debug_ptr_->pushPredictedObstaclePolygons(bounding_box);
    return false;
  }

  // detected collision
  debug_ptr_->pushCollisionObstaclePolygons(bounding_box);
  for (const auto & p : collision_points_bg) {
    const auto p_msg =
      tier4_autoware_utils::createPoint(p.x(), p.y(), pose_with_range.pose_min.position.z);
    collision_points.emplace_back(p_msg);
  }

  return true;
}

bool RunOutModule::checkCollisionWithPolygon() const
{
  RCLCPP_WARN_STREAM(logger_, "detection for POLYGON type is not implemented yet.");

  return false;
}

std::optional<geometry_msgs::msg::Pose> RunOutModule::calcStopPoint(
  const std::optional<DynamicObstacle> & dynamic_obstacle, const PathWithLaneId & path,
  const geometry_msgs::msg::Pose & current_pose, const float current_vel,
  const float current_acc) const
{
  // no obstacles
  if (!dynamic_obstacle) {
    return {};
  }

  // calculate distance to collision with the obstacle
  const float dist_to_collision_point = motion_utils::calcSignedArcLength(
    path.points, current_pose.position, dynamic_obstacle->nearest_collision_point);
  const float dist_to_collision =
    dist_to_collision_point - planner_param_.vehicle_param.base_to_front;

  // insert the stop point without considering the distance from the obstacle
  // smoother will calculate appropriate jerk for deceleration
  if (!planner_param_.run_out.specify_decel_jerk) {
    // calculate the stop point for base link
    const float base_to_collision_point =
      planner_param_.run_out.stop_margin + planner_param_.vehicle_param.base_to_front;
    const auto stop_point = motion_utils::calcLongitudinalOffsetPose(
      path.points, dynamic_obstacle->nearest_collision_point, -base_to_collision_point, false);
    if (!stop_point) {
      RCLCPP_WARN_STREAM(logger_, "failed to calculate stop point.");
      return {};
    }

    // debug
    debug_ptr_->setAccelReason(RunOutDebug::AccelReason::STOP);
    debug_ptr_->pushStopPose(tier4_autoware_utils::calcOffsetPose(
      *stop_point, planner_param_.vehicle_param.base_to_front, 0, 0));

    return stop_point;
  }

  // calculate distance needed to stop with jerk and acc constraints
  const float target_vel = 0.0;
  const float jerk_dec = planner_param_.run_out.deceleration_jerk;
  const float jerk_acc = std::abs(jerk_dec);
  const float planning_dec = jerk_dec < planner_param_.common.normal_min_jerk
                               ? planner_param_.common.limit_min_acc
                               : planner_param_.common.normal_min_acc;
  auto stop_dist = motion_utils::calcDecelDistWithJerkAndAccConstraints(
    current_vel, target_vel, current_acc, planning_dec, jerk_acc, jerk_dec);
  if (!stop_dist) {
    RCLCPP_WARN_STREAM(logger_, "failed to calculate stop distance.");

    // force to insert zero velocity
    stop_dist = std::make_optional<double>(dist_to_collision);
  }

  debug_ptr_->setDebugValues(DebugValues::TYPE::STOP_DISTANCE, *stop_dist);

  // vehicle have to decelerate if there is not enough distance with deceleration_jerk
  const bool deceleration_needed =
    *stop_dist > dist_to_collision - planner_param_.run_out.stop_margin;
  const auto & detection_method = planner_param_.run_out.detection_method;

  if (!deceleration_needed && detection_method == "Object") {
    debug_ptr_->setAccelReason(RunOutDebug::AccelReason::LOW_JERK);
    return {};
  }

  // If the detection method assumes running out, avoid acceleration when the ego is decelerating.
  // TODO(Tomohito Ando): replace with more appropriate way
  if (!deceleration_needed && detection_method != "Object") {
    constexpr float epsilon = 1.0e-2;
    constexpr float stopping_vel_mps = 2.5 / 3.6;
    const bool is_stopping = current_vel < stopping_vel_mps && current_acc < epsilon;
    if (!is_stopping) {
      debug_ptr_->setAccelReason(RunOutDebug::AccelReason::LOW_JERK);
      return {};
    }
  }

  // calculate the stop point for base link
  const float base_to_collision_point =
    planner_param_.run_out.stop_margin + planner_param_.vehicle_param.base_to_front;
  const auto stop_point = motion_utils::calcLongitudinalOffsetPose(
    path.points, dynamic_obstacle->nearest_collision_point, -base_to_collision_point, false);
  if (!stop_point) {
    RCLCPP_WARN_STREAM(logger_, "failed to calculate stop point.");
    return {};
  }

  // debug
  debug_ptr_->setAccelReason(RunOutDebug::AccelReason::STOP);
  debug_ptr_->pushStopPose(tier4_autoware_utils::calcOffsetPose(
    *stop_point, planner_param_.vehicle_param.base_to_front, 0, 0));

  return stop_point;
}

void RunOutModule::insertStopPoint(
  const std::optional<geometry_msgs::msg::Pose> stop_point,
  autoware_auto_planning_msgs::msg::PathWithLaneId & path)
{
  // no stop point
  if (!stop_point) {
    RCLCPP_DEBUG_STREAM(logger_, "already has same point");
    return;
  }

  // find nearest point index behind the stop point
  const auto nearest_seg_idx =
    motion_utils::findNearestSegmentIndex(path.points, stop_point->position);
  auto insert_idx = nearest_seg_idx + 1;

  // if stop point is ahead of the end of the path, don't insert
  if (
    insert_idx == path.points.size() - 1 &&
    planning_utils::isAheadOf(*stop_point, path.points.at(insert_idx).point.pose)) {
    return;
  }

  // to PathPointWithLaneId
  autoware_auto_planning_msgs::msg::PathPointWithLaneId stop_point_with_lane_id;
  stop_point_with_lane_id = path.points.at(nearest_seg_idx);
  stop_point_with_lane_id.point.pose = *stop_point;

  planning_utils::insertVelocity(path, stop_point_with_lane_id, 0.0, insert_idx);
}

void RunOutModule::insertVelocityForState(
  const std::optional<DynamicObstacle> & dynamic_obstacle, const PlannerData planner_data,
  const PlannerParam & planner_param, const PathWithLaneId & smoothed_path,
  PathWithLaneId & output_path)
{
  using State = run_out_utils::StateMachine::State;

  const auto & current_pose = planner_data.current_odometry->pose;
  const auto & current_vel = planner_data.current_velocity->twist.linear.x;
  const auto & current_acc = planner_data.current_acceleration->accel.accel.linear.x;

  // set data to judge the state
  run_out_utils::StateMachine::StateInput state_input;
  state_input.current_velocity = current_vel;
  state_input.current_obstacle = dynamic_obstacle;
  state_input.dist_to_collision =
    motion_utils::calcSignedArcLength(
      smoothed_path.points, current_pose.position, dynamic_obstacle->nearest_collision_point) -
    planner_param.vehicle_param.base_to_front;

  // update state
  state_machine_->updateState(state_input, *clock_);

  // get updated state and target obstacle to decelerate
  const auto state = state_machine_->getCurrentState();
  const auto target_obstacle = state_machine_->getTargetObstacle();

  // no obstacles to decelerate
  if (!target_obstacle) {
    return;
  }

  // insert velocity for each state
  switch (state) {
    case State::GO: {
      insertStoppingVelocity(target_obstacle, current_pose, current_vel, current_acc, output_path);
      break;
    }

    case State::STOP: {
      insertStoppingVelocity(target_obstacle, current_pose, current_vel, current_acc, output_path);
      break;
    }

    case State::APPROACH: {
      insertApproachingVelocity(
        *target_obstacle, current_pose, planner_param.approaching.limit_vel_kmph / 3.6,
        planner_param.approaching.margin, output_path);
      debug_ptr_->setAccelReason(RunOutDebug::AccelReason::STOP);
      break;
    }

    default: {
      RCLCPP_WARN_STREAM(logger_, "invalid state");
      break;
    }
  }
}

void RunOutModule::insertStoppingVelocity(
  const std::optional<DynamicObstacle> & dynamic_obstacle,
  const geometry_msgs::msg::Pose & current_pose, const float current_vel, const float current_acc,
  PathWithLaneId & output_path)
{
  const auto stop_point =
    calcStopPoint(dynamic_obstacle, output_path, current_pose, current_vel, current_acc);
  insertStopPoint(stop_point, output_path);
}

void RunOutModule::insertApproachingVelocity(
  const DynamicObstacle & dynamic_obstacle, const geometry_msgs::msg::Pose & current_pose,
  const float approaching_vel, const float approach_margin, PathWithLaneId & output_path)
{
  // insert slow down velocity from nearest segment point
  const auto nearest_seg_idx =
    motion_utils::findNearestSegmentIndex(output_path.points, current_pose.position);
  run_out_utils::insertPathVelocityFromIndexLimited(
    nearest_seg_idx, approaching_vel, output_path.points);

  // calculate stop point to insert 0 velocity
  const float base_to_collision_point =
    approach_margin + planner_param_.vehicle_param.base_to_front;
  const auto stop_point = motion_utils::calcLongitudinalOffsetPose(
    output_path.points, dynamic_obstacle.nearest_collision_point, -base_to_collision_point, false);
  if (!stop_point) {
    RCLCPP_WARN_STREAM(logger_, "failed to calculate stop point.");
    return;
  }

  // debug
  debug_ptr_->pushStopPose(tier4_autoware_utils::calcOffsetPose(
    *stop_point, planner_param_.vehicle_param.base_to_front, 0, 0));

  const auto nearest_seg_idx_stop =
    motion_utils::findNearestSegmentIndex(output_path.points, stop_point->position);
  auto insert_idx_stop = nearest_seg_idx_stop + 1;

  // to PathPointWithLaneId
  // use lane id of point behind inserted point
  autoware_auto_planning_msgs::msg::PathPointWithLaneId stop_point_with_lane_id;
  stop_point_with_lane_id = output_path.points.at(nearest_seg_idx_stop);
  stop_point_with_lane_id.point.pose = *stop_point;

  planning_utils::insertVelocity(output_path, stop_point_with_lane_id, 0.0, insert_idx_stop);
}

void RunOutModule::applyMaxJerkLimit(
  const geometry_msgs::msg::Pose & current_pose, const float current_vel, const float current_acc,
  PathWithLaneId & path) const
{
  const auto stop_point_idx = run_out_utils::findFirstStopPointIdx(path.points);
  if (!stop_point_idx) {
    return;
  }

  const auto stop_point = path.points.at(stop_point_idx.value()).point.pose.position;
  const auto dist_to_stop_point =
    motion_utils::calcSignedArcLength(path.points, current_pose.position, stop_point);

  // calculate desired velocity with limited jerk
  const auto jerk_limited_vel = planning_utils::calcDecelerationVelocityFromDistanceToTarget(
    planner_param_.slow_down_limit.max_jerk, planner_param_.slow_down_limit.max_acc, current_acc,
    current_vel, dist_to_stop_point);

  // overwrite velocity with limited velocity
  run_out_utils::insertPathVelocityFromIndex(stop_point_idx.value(), jerk_limited_vel, path.points);
}

std::vector<DynamicObstacle> RunOutModule::excludeObstaclesOutSideOfPartition(
  const std::vector<DynamicObstacle> & dynamic_obstacles, const PathWithLaneId & path,
  const geometry_msgs::msg::Pose & current_pose) const
{
  if (!planner_param_.run_out.use_partition_lanelet || partition_lanelets_.empty()) {
    return dynamic_obstacles;
  }

  // extract partitions within detection distance
  BasicPolygons2d close_partitions;
  planning_utils::extractClosePartition(
    current_pose.position, partition_lanelets_, close_partitions,
    planner_param_.run_out.detection_distance);

  // decimate trajectory to reduce calculation time
  constexpr float decimate_step = 1.0;
  const auto decimate_path_points = run_out_utils::decimatePathPoints(path.points, decimate_step);

  // exclude obstacles outside of partition
  std::vector<DynamicObstacle> extracted_obstacles = dynamic_obstacles;
  for (const auto & partition : close_partitions) {
    extracted_obstacles = run_out_utils::excludeObstaclesOutSideOfLine(
      extracted_obstacles, decimate_path_points, partition);
  }

  return extracted_obstacles;
}

void RunOutModule::publishDebugValue(
  const PathWithLaneId & path, const std::vector<DynamicObstacle> extracted_obstacles,
  const std::optional<DynamicObstacle> & dynamic_obstacle,
  const geometry_msgs::msg::Pose & current_pose) const
{
  if (dynamic_obstacle) {
    const auto lateral_dist =
      std::abs(motion_utils::calcLateralOffset(path.points, dynamic_obstacle->pose.position)) -
      planner_param_.vehicle_param.width / 2.0;
    const auto longitudinal_dist_to_obstacle =
      motion_utils::calcSignedArcLength(
        path.points, current_pose.position, dynamic_obstacle->pose.position) -
      planner_param_.vehicle_param.base_to_front;

    const float dist_to_collision_point = motion_utils::calcSignedArcLength(
      path.points, current_pose.position, dynamic_obstacle->nearest_collision_point);
    const auto dist_to_collision =
      dist_to_collision_point - planner_param_.vehicle_param.base_to_front;

    debug_ptr_->setDebugValues(DebugValues::TYPE::LONGITUDINAL_DIST_COLLISION, dist_to_collision);
    debug_ptr_->setDebugValues(DebugValues::TYPE::LATERAL_DIST, lateral_dist);
    debug_ptr_->setDebugValues(
      DebugValues::TYPE::LONGITUDINAL_DIST_OBSTACLE, longitudinal_dist_to_obstacle);
  } else {
    // max value
    constexpr float max_val = 50.0;
    debug_ptr_->setDebugValues(DebugValues::TYPE::LATERAL_DIST, max_val);
    debug_ptr_->setDebugValues(DebugValues::TYPE::LONGITUDINAL_DIST_COLLISION, max_val);
    debug_ptr_->setDebugValues(DebugValues::TYPE::LONGITUDINAL_DIST_OBSTACLE, max_val);
  }

  if (extracted_obstacles.empty()) {
    debug_ptr_->setAccelReason(RunOutDebug::AccelReason::NO_OBSTACLE);
  }

  debug_ptr_->publishDebugValue();
}

bool RunOutModule::isMomentaryDetection()
{
  if (!planner_param_.ignore_momentary_detection.enable) {
    return false;
  }

  if (!first_detected_time_) {
    first_detected_time_ = std::make_shared<rclcpp::Time>(clock_->now());
    return true;
  }

  const auto now = clock_->now();
  const double elapsed_time_since_detection = (now - *first_detected_time_).seconds();
  RCLCPP_DEBUG_STREAM(logger_, "elapsed_time_since_detection: " << elapsed_time_since_detection);

  return elapsed_time_since_detection < planner_param_.ignore_momentary_detection.time_threshold;
}
}  // namespace behavior_velocity_planner
