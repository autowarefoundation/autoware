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

#include "utils.hpp"

#include <motion_utils/distance/distance.hpp>
#include <motion_utils/trajectory/path_with_lane_id.hpp>

#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/intersects.hpp>
#include <boost/geometry/algorithms/make.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <limits>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif
#include <motion_utils/trajectory/trajectory.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>
namespace behavior_velocity_planner
{
namespace run_out_utils
{
Polygon2d createBoostPolyFromMsg(const std::vector<geometry_msgs::msg::Point> & input_poly)
{
  Polygon2d bg_poly;
  for (const auto & p : input_poly) {
    bg_poly.outer().emplace_back(bg::make<Point2d>(p.x, p.y));
  }

  // one more point to close polygon
  const auto & first_point = input_poly.front();
  bg_poly.outer().emplace_back(bg::make<Point2d>(first_point.x, first_point.y));

  bg::correct(bg_poly);
  return bg_poly;
}

std::uint8_t getHighestProbLabel(const std::vector<ObjectClassification> & classification)
{
  std::uint8_t label = autoware_auto_perception_msgs::msg::ObjectClassification::UNKNOWN;
  float highest_prob = 0.0;
  for (const auto & _class : classification) {
    if (highest_prob < _class.probability) {
      highest_prob = _class.probability;
      label = _class.label;
    }
  }
  return label;
}

std::vector<geometry_msgs::msg::Pose> getHighestConfidencePath(
  const std::vector<PredictedPath> & predicted_paths)
{
  std::vector<geometry_msgs::msg::Pose> predicted_path{};
  float highest_confidence = 0.0;
  for (const auto & path : predicted_paths) {
    if (path.confidence > highest_confidence) {
      highest_confidence = path.confidence;
      predicted_path = path.path;
    }
  }

  return predicted_path;
}

// apply linear interpolation to position
geometry_msgs::msg::Pose lerpByPose(
  const geometry_msgs::msg::Pose & p1, const geometry_msgs::msg::Pose & p2, const float t)
{
  tf2::Transform tf_transform1, tf_transform2;
  tf2::fromMsg(p1, tf_transform1);
  tf2::fromMsg(p2, tf_transform2);
  const auto & tf_point = tf2::lerp(tf_transform1.getOrigin(), tf_transform2.getOrigin(), t);

  geometry_msgs::msg::Pose pose;
  pose.position.x = tf_point.getX();
  pose.position.y = tf_point.getY();
  pose.position.z = tf_point.getZ();
  pose.orientation = p1.orientation;
  return pose;
}

std::vector<geometry_msgs::msg::Point> findLateralSameSidePoints(
  const std::vector<geometry_msgs::msg::Point> & points, const geometry_msgs::msg::Pose & base_pose,
  const geometry_msgs::msg::Point & target_point)
{
  const auto signed_deviation = tier4_autoware_utils::calcLateralDeviation(base_pose, target_point);
  RCLCPP_DEBUG_STREAM(
    rclcpp::get_logger("findLateralSameSidePoints"), "signed dev of target: " << signed_deviation);

  std::vector<geometry_msgs::msg::Point> same_side_points;
  for (const auto & p : points) {
    const auto signed_deviation_of_point = tier4_autoware_utils::calcLateralDeviation(base_pose, p);
    RCLCPP_DEBUG_STREAM(
      rclcpp::get_logger("findLateralSameSidePoints"),
      "signed dev of point: " << signed_deviation_of_point);
    if (signed_deviation * signed_deviation_of_point > 0) {
      same_side_points.emplace_back(p);
    }
  }

  if (same_side_points.empty()) {
    return points;
  }

  return same_side_points;
}

bool isSamePoint(const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2)
{
  if (tier4_autoware_utils::calcDistance2d(p1, p2) < std::numeric_limits<float>::epsilon()) {
    return true;
  }

  return false;
}

// if path points have the same point as target_point, return the index
std::optional<size_t> haveSamePoint(
  const PathPointsWithLaneId & path_points, const geometry_msgs::msg::Point & target_point)
{
  for (size_t i = 0; i < path_points.size(); i++) {
    const auto & path_point = path_points.at(i).point.pose.position;
    if (isSamePoint(path_point, target_point)) {
      return i;
    }
  }

  return {};
}

// insert path velocity which doesn't exceed original velocity
void insertPathVelocityFromIndexLimited(
  const size_t & start_idx, const float velocity_mps, PathPointsWithLaneId & path_points)
{
  for (size_t i = start_idx; i < path_points.size(); i++) {
    const auto current_path_vel = path_points.at(i).point.longitudinal_velocity_mps;
    path_points.at(i).point.longitudinal_velocity_mps = std::min(velocity_mps, current_path_vel);
  }
}

void insertPathVelocityFromIndex(
  const size_t & start_idx, const float velocity_mps, PathPointsWithLaneId & path_points)
{
  for (size_t i = start_idx; i < path_points.size(); i++) {
    path_points.at(i).point.longitudinal_velocity_mps = velocity_mps;
  }
}

std::optional<size_t> findFirstStopPointIdx(PathPointsWithLaneId & path_points)
{
  for (size_t i = 0; i < path_points.size(); i++) {
    const auto vel = path_points.at(i).point.longitudinal_velocity_mps;
    if (vel < std::numeric_limits<float>::epsilon()) {
      return i;
    }
  }

  return {};
}

LineString2d createLineString2d(const lanelet::BasicPolygon2d & poly)
{
  LineString2d line_string;
  for (const auto & p : poly) {
    Point2d bg_point{p.x(), p.y()};
    line_string.push_back(bg_point);
  }

  return line_string;
}

std::vector<DynamicObstacle> excludeObstaclesOutSideOfLine(
  const std::vector<DynamicObstacle> & dynamic_obstacles, const PathPointsWithLaneId & path_points,
  const lanelet::BasicPolygon2d & partition)
{
  std::vector<DynamicObstacle> extracted_dynamic_obstacle;
  for (const auto & obstacle : dynamic_obstacles) {
    const auto obstacle_nearest_idx =
      motion_utils::findNearestIndex(path_points, obstacle.pose.position);
    const auto & obstacle_nearest_path_point =
      path_points.at(obstacle_nearest_idx).point.pose.position;

    // create linestring from traj point to obstacle
    const LineString2d path_point_to_obstacle{
      {obstacle_nearest_path_point.x, obstacle_nearest_path_point.y},
      {obstacle.pose.position.x, obstacle.pose.position.y}};

    // create linestring for partition
    const LineString2d partition_bg = createLineString2d(partition);

    // ignore obstacle outside of partition
    if (bg::intersects(path_point_to_obstacle, partition_bg)) {
      continue;
    }
    extracted_dynamic_obstacle.emplace_back(obstacle);
  }

  return extracted_dynamic_obstacle;
}

PathPointsWithLaneId decimatePathPoints(
  const PathPointsWithLaneId & input_path_points, const float step)
{
  if (input_path_points.empty()) {
    return PathPointsWithLaneId();
  }

  float dist_sum = 0.0;
  PathPointsWithLaneId decimate_path_points;
  // push first point
  decimate_path_points.emplace_back(input_path_points.front());

  for (size_t i = 1; i < input_path_points.size(); i++) {
    const auto p1 = input_path_points.at(i - 1);
    const auto p2 = input_path_points.at(i);
    const auto dist = tier4_autoware_utils::calcDistance2d(p1, p2);
    dist_sum += dist;

    if (dist_sum > step) {
      decimate_path_points.emplace_back(p2);
      dist_sum = 0.0;
    }
  }

  return decimate_path_points;
}

// trim path from self_pose to trim_distance
PathWithLaneId trimPathFromSelfPose(
  const PathWithLaneId & input, const geometry_msgs::msg::Pose & self_pose,
  const double trim_distance)
{
  const size_t nearest_idx = motion_utils::findNearestIndex(input.points, self_pose.position);

  PathWithLaneId output{};
  output.header = input.header;
  output.left_bound = input.left_bound;
  output.right_bound = input.right_bound;
  double dist_sum = 0;
  for (size_t i = nearest_idx; i < input.points.size(); ++i) {
    output.points.push_back(input.points.at(i));

    if (i != nearest_idx) {
      dist_sum += tier4_autoware_utils::calcDistance2d(input.points.at(i - 1), input.points.at(i));
    }

    if (dist_sum > trim_distance) {
      break;
    }
  }

  return output;
}

// create polygon for passing lines and deceleration line calculated by stopping jerk
// note that this polygon is not closed
std::optional<std::vector<geometry_msgs::msg::Point>> createDetectionAreaPolygon(
  const std::vector<std::vector<geometry_msgs::msg::Point>> & passing_lines,
  const size_t deceleration_line_idx)
{
  if (passing_lines.size() != 2) {
    return {};
  }

  std::vector<geometry_msgs::msg::Point> detection_area_polygon;
  const auto & line1 = passing_lines.at(0);
  const int poly_corner_idx = std::min(deceleration_line_idx, line1.size() - 1);
  for (int i = 0; i <= poly_corner_idx; i++) {
    const auto & p = line1.at(i);
    detection_area_polygon.push_back(p);
  }

  // push points from the end to create the polygon
  const auto & line2 = passing_lines.at(1);
  for (int i = poly_corner_idx; i >= 0; i--) {
    const auto & p = line2.at(i);
    detection_area_polygon.push_back(p);
  }

  return detection_area_polygon;
}

PathPointWithLaneId createExtendPathPoint(
  const double extend_distance, const PathPointWithLaneId & goal_point)
{
  PathPointWithLaneId extend_path_point = goal_point;
  extend_path_point.point.pose =
    tier4_autoware_utils::calcOffsetPose(goal_point.point.pose, extend_distance, 0.0, 0.0);
  return extend_path_point;
}

PathWithLaneId extendPath(const PathWithLaneId & input, const double extend_distance)
{
  PathWithLaneId output = input;
  if (extend_distance < std::numeric_limits<double>::epsilon() || input.points.empty()) {
    return output;
  }

  const auto goal_point = input.points.back();
  constexpr double interpolation_interval = 0.1;
  double extend_sum = interpolation_interval;
  while (extend_sum < extend_distance) {
    const auto extend_path_point = createExtendPathPoint(extend_sum, goal_point);
    output.points.push_back(extend_path_point);
    extend_sum += interpolation_interval;
  }

  return output;
}

DetectionMethod toEnum(const std::string & detection_method)
{
  if (detection_method == "Object") {
    return DetectionMethod::Object;
  } else if (detection_method == "ObjectWithoutPath") {
    return DetectionMethod::ObjectWithoutPath;
  } else if (detection_method == "Points") {
    return DetectionMethod::Points;
  } else {
    return DetectionMethod::Unknown;
  }
}

Polygons2d createDetectionAreaPolygon(
  const PathWithLaneId & path, const PlannerData & planner_data, const PlannerParam & planner_param)
{
  const auto & pd = planner_data;
  const auto & pp = planner_param;

  // calculate distance needed to stop with jerk and acc constraints
  const float initial_vel = pd.current_velocity->twist.linear.x;
  const float initial_acc = pd.current_acceleration->accel.accel.linear.x;
  const float target_vel = 0.0;
  const float jerk_dec_max = pp.smoother.start_jerk;
  const float jerk_dec =
    pp.run_out.specify_decel_jerk ? pp.run_out.deceleration_jerk : jerk_dec_max;
  const float jerk_acc = std::abs(jerk_dec);
  const float planning_dec =
    jerk_dec < pp.common.normal_min_jerk ? pp.common.limit_min_acc : pp.common.normal_min_acc;
  auto stop_dist = motion_utils::calcDecelDistWithJerkAndAccConstraints(
    initial_vel, target_vel, initial_acc, planning_dec, jerk_acc, jerk_dec);

  if (!stop_dist) {
    stop_dist = std::make_optional<double>(0.0);
  }

  // create detection area polygon
  DetectionRange da_range;
  const double obstacle_vel_mps = pp.dynamic_obstacle.max_vel_kmph / 3.6;
  da_range.interval = pp.run_out.detection_distance;
  da_range.min_longitudinal_distance =
    pp.vehicle_param.base_to_front - pp.detection_area.margin_behind;
  da_range.max_longitudinal_distance =
    *stop_dist + pp.run_out.stop_margin + pp.detection_area.margin_ahead;
  da_range.wheel_tread = pp.vehicle_param.wheel_tread;
  da_range.right_overhang = pp.vehicle_param.right_overhang;
  da_range.left_overhang = pp.vehicle_param.left_overhang;
  da_range.max_lateral_distance = obstacle_vel_mps * pp.dynamic_obstacle.max_prediction_time;
  Polygons2d detection_area_poly;
  const size_t ego_seg_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
    path.points, pd.current_odometry->pose, pd.ego_nearest_dist_threshold,
    pd.ego_nearest_yaw_threshold);
  planning_utils::createDetectionAreaPolygons(
    detection_area_poly, path, pd.current_odometry->pose, ego_seg_idx, da_range,
    pp.dynamic_obstacle.max_vel_kmph / 3.6);

  return detection_area_poly;
}

Polygons2d createMandatoryDetectionAreaPolygon(
  const PathWithLaneId & path, const PlannerData & planner_data, const PlannerParam & planner_param)
{
  const auto & pd = planner_data;
  const auto & pp = planner_param;

  // calculate distance needed to stop with jerk and acc constraints
  const float initial_vel = pd.current_velocity->twist.linear.x;
  const float initial_acc = pd.current_acceleration->accel.accel.linear.x;
  const float target_vel = 0.0;
  const float jerk_dec = pp.mandatory_area.decel_jerk;
  const float jerk_acc = std::abs(jerk_dec);
  const float planning_dec =
    jerk_dec < pp.common.normal_min_jerk ? pp.common.limit_min_acc : pp.common.normal_min_acc;
  auto stop_dist = motion_utils::calcDecelDistWithJerkAndAccConstraints(
    initial_vel, target_vel, initial_acc, planning_dec, jerk_acc, jerk_dec);

  if (!stop_dist) {
    stop_dist = std::make_optional<double>(0.0);
  }

  // create detection area polygon
  DetectionRange da_range;
  const double obstacle_vel_mps = pp.dynamic_obstacle.max_vel_kmph / 3.6;
  da_range.interval = pp.run_out.detection_distance;
  da_range.min_longitudinal_distance = pp.vehicle_param.base_to_front;
  da_range.max_longitudinal_distance = *stop_dist + pp.run_out.stop_margin;
  da_range.wheel_tread = pp.vehicle_param.wheel_tread;
  da_range.right_overhang = pp.vehicle_param.right_overhang;
  da_range.left_overhang = pp.vehicle_param.left_overhang;
  da_range.max_lateral_distance = obstacle_vel_mps * pp.dynamic_obstacle.max_prediction_time;
  Polygons2d detection_area_poly;
  const size_t ego_seg_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
    path.points, pd.current_odometry->pose, pd.ego_nearest_dist_threshold,
    pd.ego_nearest_yaw_threshold);
  planning_utils::createDetectionAreaPolygons(
    detection_area_poly, path, pd.current_odometry->pose, ego_seg_idx, da_range,
    pp.dynamic_obstacle.max_vel_kmph / 3.6);

  return detection_area_poly;
}

}  // namespace run_out_utils
}  // namespace behavior_velocity_planner
