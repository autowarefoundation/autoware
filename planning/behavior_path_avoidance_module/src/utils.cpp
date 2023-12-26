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

#include "behavior_path_planner_common/utils/utils.hpp"

#include "behavior_path_avoidance_module/data_structs.hpp"
#include "behavior_path_avoidance_module/utils.hpp"
#include "behavior_path_planner_common/utils/create_vehicle_footprint.hpp"
#include "behavior_path_planner_common/utils/path_safety_checker/objects_filtering.hpp"
#include "behavior_path_planner_common/utils/path_utils.hpp"
#include "behavior_path_planner_common/utils/traffic_light_utils.hpp"
#include "tier4_autoware_utils/geometry/boost_polygon_utils.hpp"

#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/ros/uuid_helper.hpp>

#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <tier4_planning_msgs/msg/detail/avoidance_debug_factor__struct.hpp>

#include <boost/geometry/algorithms/buffer.hpp>
#include <boost/geometry/algorithms/convex_hull.hpp>
#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/union.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

#include <lanelet2_routing/RoutingGraphContainer.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace behavior_path_planner::utils::avoidance
{

using autoware_perception_msgs::msg::TrafficSignalElement;
using behavior_path_planner::utils::traffic_light::calcDistanceToRedTrafficLight;
using behavior_path_planner::utils::traffic_light::getDistanceToNextTrafficLight;
using geometry_msgs::msg::TransformStamped;
using motion_utils::calcLongitudinalOffsetPoint;
using motion_utils::calcSignedArcLength;
using motion_utils::findNearestIndex;
using motion_utils::findNearestSegmentIndex;
using motion_utils::insertTargetPoint;
using tier4_autoware_utils::calcDistance2d;
using tier4_autoware_utils::calcLateralDeviation;
using tier4_autoware_utils::calcOffsetPose;
using tier4_autoware_utils::calcYawDeviation;
using tier4_autoware_utils::createQuaternionFromRPY;
using tier4_autoware_utils::getPose;
using tier4_autoware_utils::pose2transform;
using tier4_planning_msgs::msg::AvoidanceDebugFactor;
using tier4_planning_msgs::msg::AvoidanceDebugMsg;

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

geometry_msgs::msg::Polygon toMsg(const tier4_autoware_utils::Polygon2d & polygon, const double z)
{
  geometry_msgs::msg::Polygon ret;
  for (const auto & p : polygon.outer()) {
    ret.points.push_back(createPoint32(p.x(), p.y(), z));
  }
  return ret;
}

template <class T>
size_t findFirstNearestIndex(const T & points, const geometry_msgs::msg::Point & point)
{
  motion_utils::validateNonEmpty(points);

  double min_dist = std::numeric_limits<double>::max();
  size_t min_idx = 0;
  bool decreasing = false;

  for (size_t i = 0; i < points.size(); ++i) {
    const auto dist = tier4_autoware_utils::calcSquaredDistance2d(points.at(i), point);
    if (dist < min_dist) {
      decreasing = true;
      min_dist = dist;
      min_idx = i;
      continue;
    }

    if (decreasing) {
      return min_idx;
    }
  }

  return min_idx;
}

template <class T>
size_t findFirstNearestSegmentIndex(const T & points, const geometry_msgs::msg::Point & point)
{
  const size_t nearest_idx = findFirstNearestIndex(points, point);

  if (nearest_idx == 0) {
    return 0;
  }
  if (nearest_idx == points.size() - 1) {
    return points.size() - 2;
  }

  const double signed_length =
    motion_utils::calcLongitudinalOffsetToSegment(points, nearest_idx, point);

  if (signed_length <= 0) {
    return nearest_idx - 1;
  }

  return nearest_idx;
}

template <class T>
double calcSignedArcLengthToFirstNearestPoint(
  const T & points, const geometry_msgs::msg::Point & src_point,
  const geometry_msgs::msg::Point & dst_point)
{
  try {
    motion_utils::validateNonEmpty(points);
  } catch (const std::exception & e) {
    std::cerr << e.what() << std::endl;
    return 0.0;
  }

  const size_t src_seg_idx = findFirstNearestSegmentIndex(points, src_point);
  const size_t dst_seg_idx = findFirstNearestSegmentIndex(points, dst_point);

  const double signed_length_on_traj =
    motion_utils::calcSignedArcLength(points, src_seg_idx, dst_seg_idx);
  const double signed_length_src_offset =
    motion_utils::calcLongitudinalOffsetToSegment(points, src_seg_idx, src_point);
  const double signed_length_dst_offset =
    motion_utils::calcLongitudinalOffsetToSegment(points, dst_seg_idx, dst_point);

  return signed_length_on_traj - signed_length_src_offset + signed_length_dst_offset;
}

geometry_msgs::msg::Polygon createVehiclePolygon(
  const vehicle_info_util::VehicleInfo & vehicle_info, const double offset)
{
  const auto & i = vehicle_info;
  const auto & front_m = i.max_longitudinal_offset_m;
  const auto & width_m = i.vehicle_width_m / 2.0 + offset;
  const auto & back_m = i.rear_overhang_m;

  geometry_msgs::msg::Polygon polygon{};

  polygon.points.push_back(createPoint32(front_m, -width_m, 0.0));
  polygon.points.push_back(createPoint32(front_m, width_m, 0.0));
  polygon.points.push_back(createPoint32(-back_m, width_m, 0.0));
  polygon.points.push_back(createPoint32(-back_m, -width_m, 0.0));

  return polygon;
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
  boost::geometry::convex_hull(one_step_polygon, hull_polygon);
  boost::geometry::correct(hull_polygon);

  return hull_polygon;
}

bool isEndPointsConnected(
  const lanelet::ConstLanelet & left_lane, const lanelet::ConstLanelet & right_lane)
{
  const auto & left_back_point_2d = right_lane.leftBound2d().back().basicPoint();
  const auto & right_back_point_2d = left_lane.rightBound2d().back().basicPoint();

  constexpr double epsilon = 1e-5;
  return (right_back_point_2d - left_back_point_2d).norm() < epsilon;
}

template <typename T>
void pushUniqueVector(T & base_vector, const T & additional_vector)
{
  base_vector.insert(base_vector.end(), additional_vector.begin(), additional_vector.end());
}

bool existShiftSideLane(
  const double start_shift_length, const double end_shift_length, const bool no_left_lanes,
  const bool no_right_lanes)
{
  constexpr double THRESHOLD = 0.1;
  const auto relative_shift_length = end_shift_length - start_shift_length;

  const auto avoid_shift =
    std::abs(start_shift_length) < THRESHOLD && std::abs(end_shift_length) > THRESHOLD;
  if (avoid_shift) {
    // Left avoid. But there is no adjacent lane. No need blinker.
    if (relative_shift_length > 0.0 && no_left_lanes) {
      return false;
    }

    // Right avoid. But there is no adjacent lane. No need blinker.
    if (relative_shift_length < 0.0 && no_right_lanes) {
      return false;
    }
  }

  const auto return_shift =
    std::abs(start_shift_length) > THRESHOLD && std::abs(end_shift_length) < THRESHOLD;
  if (return_shift) {
    // Right return. But there is no adjacent lane. No need blinker.
    if (relative_shift_length > 0.0 && no_right_lanes) {
      return false;
    }

    // Left return. But there is no adjacent lane. No need blinker.
    if (relative_shift_length < 0.0 && no_left_lanes) {
      return false;
    }
  }

  const auto left_middle_shift = start_shift_length > THRESHOLD && end_shift_length > THRESHOLD;
  if (left_middle_shift) {
    // Left avoid. But there is no adjacent lane. No need blinker.
    if (relative_shift_length > 0.0 && no_left_lanes) {
      return false;
    }

    // Left return. But there is no adjacent lane. No need blinker.
    if (relative_shift_length < 0.0 && no_left_lanes) {
      return false;
    }
  }

  const auto right_middle_shift = start_shift_length < THRESHOLD && end_shift_length < THRESHOLD;
  if (right_middle_shift) {
    // Right avoid. But there is no adjacent lane. No need blinker.
    if (relative_shift_length < 0.0 && no_right_lanes) {
      return false;
    }

    // Left avoid. But there is no adjacent lane. No need blinker.
    if (relative_shift_length > 0.0 && no_right_lanes) {
      return false;
    }
  }

  return true;
}

bool straddleRoadBound(
  const ShiftedPath & path, const ShiftLine & shift_line, const lanelet::ConstLanelets & lanes,
  const vehicle_info_util::VehicleInfo & vehicle_info)
{
  using boost::geometry::intersects;
  using tier4_autoware_utils::pose2transform;
  using tier4_autoware_utils::transformVector;

  const auto footprint = vehicle_info.createFootprint();

  for (const auto & lane : lanes) {
    for (size_t i = shift_line.start_idx; i < shift_line.end_idx; ++i) {
      const auto transform = pose2transform(path.path.points.at(i).point.pose);
      const auto shifted_vehicle_footprint = transformVector(footprint, transform);

      if (intersects(lane.leftBound2d().basicLineString(), shifted_vehicle_footprint)) {
        return true;
      }

      if (intersects(lane.rightBound2d().basicLineString(), shifted_vehicle_footprint)) {
        return true;
      }
    }
  }

  return false;
}

}  // namespace

namespace filtering_utils
{
bool isAvoidanceTargetObjectType(
  const PredictedObject & object, const std::shared_ptr<AvoidanceParameters> & parameters)
{
  const auto object_type = utils::getHighestProbLabel(object.classification);

  if (parameters->object_parameters.count(object_type) == 0) {
    return false;
  }

  return parameters->object_parameters.at(object_type).is_avoidance_target;
}

bool isSafetyCheckTargetObjectType(
  const PredictedObject & object, const std::shared_ptr<AvoidanceParameters> & parameters)
{
  const auto object_type = utils::getHighestProbLabel(object.classification);

  if (parameters->object_parameters.count(object_type) == 0) {
    return false;
  }

  return parameters->object_parameters.at(object_type).is_safety_check_target;
}

bool isVehicleTypeObject(const ObjectData & object)
{
  const auto object_type = utils::getHighestProbLabel(object.object.classification);

  if (object_type == ObjectClassification::UNKNOWN) {
    return false;
  }

  if (object_type == ObjectClassification::PEDESTRIAN) {
    return false;
  }

  if (object_type == ObjectClassification::BICYCLE) {
    return false;
  }

  return true;
}

bool isWithinCrosswalk(
  const ObjectData & object,
  const std::shared_ptr<const lanelet::routing::RoutingGraphContainer> & overall_graphs)
{
  using Point = boost::geometry::model::d2::point_xy<double>;

  const auto & p = object.object.kinematics.initial_pose_with_covariance.pose.position;
  const Point p_object{p.x, p.y};

  // get conflicting crosswalk crosswalk
  constexpr int PEDESTRIAN_GRAPH_ID = 1;
  const auto conflicts =
    overall_graphs->conflictingInGraph(object.overhang_lanelet, PEDESTRIAN_GRAPH_ID);

  constexpr double THRESHOLD = 2.0;
  for (const auto & crosswalk : conflicts) {
    auto polygon = crosswalk.polygon2d().basicPolygon();

    boost::geometry::correct(polygon);

    // ignore objects around the crosswalk
    if (boost::geometry::distance(p_object, polygon) < THRESHOLD) {
      return true;
    }
  }

  return false;
}

bool isWithinIntersection(
  const ObjectData & object, const std::shared_ptr<RouteHandler> & route_handler)
{
  const std::string id = object.overhang_lanelet.attributeOr("intersection_area", "else");
  if (id == "else") {
    return false;
  }

  const auto object_polygon = tier4_autoware_utils::toPolygon2d(object.object);

  const auto polygon = route_handler->getLaneletMapPtr()->polygonLayer.get(std::atoi(id.c_str()));

  return boost::geometry::within(
    object_polygon, utils::toPolygon2d(lanelet::utils::to2D(polygon.basicPolygon())));
}

bool isParallelToEgoLane(const ObjectData & object, const double threshold)
{
  const auto & object_pose = object.object.kinematics.initial_pose_with_covariance.pose;
  const auto closest_pose =
    lanelet::utils::getClosestCenterPose(object.overhang_lanelet, object_pose.position);
  const auto yaw_deviation = std::abs(calcYawDeviation(closest_pose, object_pose));

  return yaw_deviation < threshold || yaw_deviation > M_PI - threshold;
}

bool isMergingToEgoLane(const ObjectData & object)
{
  const auto & object_pose = object.object.kinematics.initial_pose_with_covariance.pose;
  const auto closest_pose =
    lanelet::utils::getClosestCenterPose(object.overhang_lanelet, object_pose.position);
  const auto yaw_deviation = calcYawDeviation(closest_pose, object_pose);

  if (isOnRight(object)) {
    if (yaw_deviation < 0.0 && -1.0 * M_PI_2 < yaw_deviation) {
      return false;
    }

    if (yaw_deviation > M_PI_2) {
      return false;
    }
  } else {
    if (yaw_deviation > 0.0 && M_PI_2 > yaw_deviation) {
      return false;
    }

    if (yaw_deviation < -1.0 * M_PI_2) {
      return false;
    }
  }

  return true;
}

bool isObjectOnRoadShoulder(
  ObjectData & object, const std::shared_ptr<RouteHandler> & route_handler,
  const std::shared_ptr<AvoidanceParameters> & parameters)
{
  using boost::geometry::return_centroid;
  using boost::geometry::within;
  using lanelet::geometry::distance2d;
  using lanelet::geometry::toArcCoordinates;
  using lanelet::utils::to2D;

  // assume that there are no parked vehicles in intersection.
  std::string turn_direction = object.overhang_lanelet.attributeOr("turn_direction", "else");
  if (turn_direction == "right" || turn_direction == "left" || turn_direction == "straight") {
    return false;
  }

  // ============================================ <- most_left_lanelet.leftBound()
  // y              road shoulder
  // ^ ------------------------------------------
  // |   x                                +
  // +---> --- object closest lanelet --- o ----- <- object_closest_lanelet.centerline()
  //
  // --------------------------------------------
  // +: object position
  // o: nearest point on centerline

  lanelet::BasicPoint2d object_centroid(object.centroid.x(), object.centroid.y());

  const auto & object_pose = object.object.kinematics.initial_pose_with_covariance.pose;
  const auto centerline_pose =
    lanelet::utils::getClosestCenterPose(object.overhang_lanelet, object_pose.position);
  lanelet::BasicPoint3d centerline_point(
    centerline_pose.position.x, centerline_pose.position.y, centerline_pose.position.z);

  bool is_left_side_parked_vehicle = false;
  if (!isOnRight(object)) {
    auto [object_shiftable_distance, sub_type] = [&]() {
      const auto most_left_road_lanelet =
        route_handler->getMostLeftLanelet(object.overhang_lanelet);
      const auto most_left_lanelet_candidates =
        route_handler->getLaneletMapPtr()->laneletLayer.findUsages(
          most_left_road_lanelet.leftBound());

      lanelet::ConstLanelet most_left_lanelet = most_left_road_lanelet;
      const lanelet::Attribute sub_type =
        most_left_lanelet.attribute(lanelet::AttributeName::Subtype);

      for (const auto & ll : most_left_lanelet_candidates) {
        const lanelet::Attribute sub_type = ll.attribute(lanelet::AttributeName::Subtype);
        if (sub_type.value() == "road_shoulder") {
          most_left_lanelet = ll;
        }
      }

      const auto center_to_left_boundary =
        distance2d(to2D(most_left_lanelet.leftBound().basicLineString()), to2D(centerline_point));

      return std::make_pair(
        center_to_left_boundary - 0.5 * object.object.shape.dimensions.y, sub_type);
    }();

    if (sub_type.value() != "road_shoulder") {
      object_shiftable_distance += parameters->object_check_min_road_shoulder_width;
    }

    const auto arc_coordinates = toArcCoordinates(
      to2D(object.overhang_lanelet.centerline().basicLineString()), object_centroid);
    object.shiftable_ratio = arc_coordinates.distance / object_shiftable_distance;

    is_left_side_parked_vehicle = object.shiftable_ratio > parameters->object_check_shiftable_ratio;
  }

  bool is_right_side_parked_vehicle = false;
  if (isOnRight(object)) {
    auto [object_shiftable_distance, sub_type] = [&]() {
      const auto most_right_road_lanelet =
        route_handler->getMostRightLanelet(object.overhang_lanelet);
      const auto most_right_lanelet_candidates =
        route_handler->getLaneletMapPtr()->laneletLayer.findUsages(
          most_right_road_lanelet.rightBound());

      lanelet::ConstLanelet most_right_lanelet = most_right_road_lanelet;
      const lanelet::Attribute sub_type =
        most_right_lanelet.attribute(lanelet::AttributeName::Subtype);

      for (const auto & ll : most_right_lanelet_candidates) {
        const lanelet::Attribute sub_type = ll.attribute(lanelet::AttributeName::Subtype);
        if (sub_type.value() == "road_shoulder") {
          most_right_lanelet = ll;
        }
      }

      const auto center_to_right_boundary =
        distance2d(to2D(most_right_lanelet.rightBound().basicLineString()), to2D(centerline_point));

      return std::make_pair(
        center_to_right_boundary - 0.5 * object.object.shape.dimensions.y, sub_type);
    }();

    if (sub_type.value() != "road_shoulder") {
      object_shiftable_distance += parameters->object_check_min_road_shoulder_width;
    }

    const auto arc_coordinates = toArcCoordinates(
      to2D(object.overhang_lanelet.centerline().basicLineString()), object_centroid);
    object.shiftable_ratio = -1.0 * arc_coordinates.distance / object_shiftable_distance;

    is_right_side_parked_vehicle =
      object.shiftable_ratio > parameters->object_check_shiftable_ratio;
  }

  return is_left_side_parked_vehicle || is_right_side_parked_vehicle;
}

bool isForceAvoidanceTarget(
  ObjectData & object, const AvoidancePlanningData & data,
  const std::shared_ptr<const PlannerData> & planner_data,
  const std::shared_ptr<AvoidanceParameters> & parameters)
{
  if (!parameters->enable_force_avoidance_for_stopped_vehicle) {
    return false;
  }

  const auto stop_time_longer_than_threshold =
    object.stop_time > parameters->threshold_time_force_avoidance_for_stopped_vehicle;

  if (!stop_time_longer_than_threshold) {
    return false;
  }

  const auto & object_pose = object.object.kinematics.initial_pose_with_covariance.pose;
  const auto is_moving_distance_longer_than_threshold =
    tier4_autoware_utils::calcDistance2d(object.init_pose, object_pose) >
    parameters->force_avoidance_distance_threshold;

  if (is_moving_distance_longer_than_threshold) {
    return false;
  }

  if (object.is_within_intersection) {
    RCLCPP_DEBUG(rclcpp::get_logger(__func__), "object is in the intersection area.");
    return false;
  }

  const auto rh = planner_data->route_handler;

  if (
    !!rh->getRoutingGraphPtr()->right(object.overhang_lanelet) &&
    !!rh->getRoutingGraphPtr()->left(object.overhang_lanelet)) {
    RCLCPP_DEBUG(rclcpp::get_logger(__func__), "object isn't on the edge lane.");
    return false;
  }

  const auto & ego_pose = planner_data->self_odometry->pose.pose;

  // force avoidance for stopped vehicle
  bool not_parked_object = true;

  // check traffic light
  const auto to_traffic_light = getDistanceToNextTrafficLight(object_pose, data.extend_lanelets);
  {
    not_parked_object =
      to_traffic_light < parameters->object_ignore_section_traffic_light_in_front_distance;
  }

  // check crosswalk
  const auto to_crosswalk =
    utils::getDistanceToCrosswalk(ego_pose, data.extend_lanelets, *rh->getOverallGraphPtr()) -
    object.longitudinal;
  {
    const auto stop_for_crosswalk =
      to_crosswalk < parameters->object_ignore_section_crosswalk_in_front_distance &&
      to_crosswalk > -1.0 * parameters->object_ignore_section_crosswalk_behind_distance;
    not_parked_object = not_parked_object || stop_for_crosswalk;
  }

  object.to_stop_factor_distance = std::min(to_traffic_light, to_crosswalk);

  return !not_parked_object;
}

bool isSatisfiedWithCommonCondition(
  ObjectData & object, const AvoidancePlanningData & data,
  const std::shared_ptr<const PlannerData> & planner_data,
  const std::shared_ptr<AvoidanceParameters> & parameters)
{
  // Step1. filtered by target object type.
  if (!isAvoidanceTargetObjectType(object.object, parameters)) {
    object.reason = AvoidanceDebugFactor::OBJECT_IS_NOT_TYPE;
    return false;
  }

  // Step2. filtered stopped objects.
  const auto object_type = utils::getHighestProbLabel(object.object.classification);
  const auto object_parameter = parameters->object_parameters.at(object_type);
  if (object.move_time > object_parameter.moving_time_threshold) {
    object.reason = AvoidanceDebugFactor::MOVING_OBJECT;
    return false;
  }

  // Step3. filtered by longitudinal distance.
  const auto & ego_pos = planner_data->self_odometry->pose.pose.position;
  fillLongitudinalAndLengthByClosestEnvelopeFootprint(data.reference_path_rough, ego_pos, object);

  if (object.longitudinal < -parameters->object_check_backward_distance) {
    object.reason = AvoidanceDebugFactor::OBJECT_IS_BEHIND_THRESHOLD;
    return false;
  }

  if (object.longitudinal > parameters->object_check_max_forward_distance) {
    object.reason = AvoidanceDebugFactor::OBJECT_IS_IN_FRONT_THRESHOLD;
    return false;
  }

  // Step4. filtered by distance between object and goal position.
  // TODO(Satoshi OTA): remove following two conditions after it can execute avoidance and goal
  // planner module simultaneously.
  const auto & rh = planner_data->route_handler;
  const auto ego_idx = planner_data->findEgoIndex(data.reference_path_rough.points);
  const auto to_goal_distance =
    rh->isInGoalRouteSection(data.current_lanelets.back())
      ? calcSignedArcLength(
          data.reference_path_rough.points, ego_idx, data.reference_path_rough.points.size() - 1)
      : std::numeric_limits<double>::max();

  if (object.longitudinal > to_goal_distance) {
    object.reason = AvoidanceDebugFactor::OBJECT_BEHIND_PATH_GOAL;
    return false;
  }

  if (
    object.longitudinal + object.length / 2 + parameters->object_check_goal_distance >
    to_goal_distance) {
    object.reason = "TooNearToGoal";
    return false;
  }

  return true;
}

bool isSatisfiedWithNonVehicleCondition(
  ObjectData & object, [[maybe_unused]] const AvoidancePlanningData & data,
  const std::shared_ptr<const PlannerData> & planner_data,
  [[maybe_unused]] const std::shared_ptr<AvoidanceParameters> & parameters)
{
  // avoidance module ignore pedestrian and bicycle around crosswalk
  if (isWithinCrosswalk(object, planner_data->route_handler->getOverallGraphPtr())) {
    object.reason = "CrosswalkUser";
    return false;
  }

  return true;
}

ObjectData::Behavior getObjectBehavior(
  ObjectData & object, const std::shared_ptr<AvoidanceParameters> & parameters)
{
  if (isParallelToEgoLane(object, parameters->object_check_yaw_deviation)) {
    return ObjectData::Behavior::NONE;
  }

  return isMergingToEgoLane(object) ? ObjectData::Behavior::MERGING
                                    : ObjectData::Behavior::DEVIATING;
}

bool isSatisfiedWithVehicleCondition(
  ObjectData & object, const AvoidancePlanningData & data,
  const std::shared_ptr<const PlannerData> & planner_data,
  const std::shared_ptr<AvoidanceParameters> & parameters)
{
  using boost::geometry::within;

  object.behavior = getObjectBehavior(object, parameters);
  object.is_within_intersection = isWithinIntersection(object, planner_data->route_handler);

  // from here condition check for vehicle type objects.
  if (isForceAvoidanceTarget(object, data, planner_data, parameters)) {
    return true;
  }

  // check vehicle shift ratio
  lanelet::BasicPoint2d object_centroid(object.centroid.x(), object.centroid.y());
  const auto on_ego_driving_lane =
    within(object_centroid, object.overhang_lanelet.polygon2d().basicPolygon());
  if (on_ego_driving_lane) {
    if (isObjectOnRoadShoulder(object, planner_data->route_handler, parameters)) {
      return true;
    } else {
      object.reason = AvoidanceDebugFactor::NOT_PARKING_OBJECT;
      return false;
    }
  }

  // Object is on center line -> ignore.
  if (std::abs(object.to_centerline) < parameters->threshold_distance_object_is_on_center) {
    object.reason = AvoidanceDebugFactor::TOO_NEAR_TO_CENTERLINE;
    return false;
  }

  if (object.is_within_intersection) {
    std::string turn_direction = object.overhang_lanelet.attributeOr("turn_direction", "else");
    if (turn_direction == "straight") {
      return true;
    }

    if (object.behavior == ObjectData::Behavior::NONE) {
      object.reason = "ParallelToEgoLane";
      return false;
    }
  }

  if (object.behavior == ObjectData::Behavior::MERGING) {
    object.reason = "MergingToEgoLane";
    return false;
  }

  return true;
}

bool isNoNeedAvoidanceBehavior(
  ObjectData & object, const std::shared_ptr<AvoidanceParameters> & parameters)
{
  if (!object.avoid_margin.has_value()) {
    return false;
  }

  const auto shift_length =
    calcShiftLength(isOnRight(object), object.overhang_dist, object.avoid_margin.value());
  if (!isShiftNecessary(isOnRight(object), shift_length)) {
    object.reason = "NotNeedAvoidance";
    return true;
  }

  if (std::abs(shift_length) < parameters->lateral_execution_threshold) {
    object.reason = "LessThanExecutionThreshold";
    return true;
  }

  return false;
}

std::optional<double> getAvoidMargin(
  const ObjectData & object, const std::shared_ptr<const PlannerData> & planner_data,
  const std::shared_ptr<AvoidanceParameters> & parameters)
{
  const auto & vehicle_width = planner_data->parameters.vehicle_width;
  const auto object_type = utils::getHighestProbLabel(object.object.classification);
  const auto object_parameter = parameters->object_parameters.at(object_type);

  const auto max_avoid_margin = object_parameter.safety_buffer_lateral * object.distance_factor +
                                object_parameter.avoid_margin_lateral + 0.5 * vehicle_width;
  const auto min_avoid_margin = object_parameter.safety_buffer_lateral + 0.5 * vehicle_width;
  const auto soft_lateral_distance_limit =
    object.to_road_shoulder_distance - parameters->soft_road_shoulder_margin - 0.5 * vehicle_width;
  const auto hard_lateral_distance_limit =
    object.to_road_shoulder_distance - parameters->hard_road_shoulder_margin - 0.5 * vehicle_width;

  // Step1. check avoidable or not.
  if (hard_lateral_distance_limit < min_avoid_margin) {
    return std::nullopt;
  }

  // Step2. check if it should expand road shoulder margin.
  if (soft_lateral_distance_limit < min_avoid_margin) {
    return min_avoid_margin;
  }

  // Step3. nominal case. avoid margin is limited by soft constraint.
  return std::min(soft_lateral_distance_limit, max_avoid_margin);
}
}  // namespace filtering_utils

bool isOnRight(const ObjectData & obj)
{
  if (obj.direction == Direction::NONE) {
    throw std::logic_error("object direction is not initialized. something wrong.");
  }

  return obj.direction == Direction::RIGHT;
}

double calcShiftLength(
  const bool & is_object_on_right, const double & overhang_dist, const double & avoid_margin)
{
  const auto shift_length =
    is_object_on_right ? (overhang_dist + avoid_margin) : (overhang_dist - avoid_margin);
  return std::fabs(shift_length) > 1e-3 ? shift_length : 0.0;
}

bool isShiftNecessary(const bool & is_object_on_right, const double & shift_length)
{
  /**
   *   ^
   *   |
   * --+----x-------------------------------x--->
   *   |                 x     x
   *   |                 ==obj==
   */
  if (is_object_on_right && shift_length < 0.0) {
    return false;
  }

  /**
   *   ^                 ==obj==
   *   |                 x     x
   * --+----x-------------------------------x--->
   *   |
   */
  if (!is_object_on_right && shift_length > 0.0) {
    return false;
  }

  return true;
}

bool isSameDirectionShift(const bool & is_object_on_right, const double & shift_length)
{
  return (is_object_on_right == std::signbit(shift_length));
}

ShiftedPath toShiftedPath(const PathWithLaneId & path)
{
  ShiftedPath out;
  out.path = path;
  out.shift_length.resize(path.points.size());
  std::fill(out.shift_length.begin(), out.shift_length.end(), 0.0);
  return out;
}

ShiftLineArray toShiftLineArray(const AvoidLineArray & avoid_points)
{
  ShiftLineArray shift_lines;
  for (const auto & ap : avoid_points) {
    shift_lines.push_back(ap);
  }
  return shift_lines;
}

size_t findPathIndexFromArclength(
  const std::vector<double> & path_arclength_arr, const double target_arc)
{
  if (path_arclength_arr.empty()) {
    return 0;
  }

  for (size_t i = 0; i < path_arclength_arr.size(); ++i) {
    if (path_arclength_arr.at(i) > target_arc) {
      return i;
    }
  }
  return path_arclength_arr.size() - 1;
}

std::vector<UUID> concatParentIds(const std::vector<UUID> & ids1, const std::vector<UUID> & ids2)
{
  std::vector<UUID> ret;

  for (const auto & id : ids1) {
    if (std::any_of(
          ret.begin(), ret.end(), [&id](const auto & exist_id) { return exist_id == id; })) {
      continue;
    }
    ret.push_back(id);
  }

  for (const auto & id : ids2) {
    if (std::any_of(
          ret.begin(), ret.end(), [&id](const auto & exist_id) { return exist_id == id; })) {
      continue;
    }
    ret.push_back(id);
  }

  return ret;
}

std::vector<UUID> calcParentIds(const AvoidLineArray & lines1, const AvoidLine & lines2)
{
  // Get the ID of the original AP whose transition area overlaps with the given AP,
  // and set it to the parent id.
  std::vector<UUID> ret;
  for (const auto & al : lines1) {
    const auto p_s = al.start_longitudinal;
    const auto p_e = al.end_longitudinal;
    const auto has_overlap = !(p_e < lines2.start_longitudinal || lines2.end_longitudinal < p_s);

    if (!has_overlap) {
      continue;
    }

    ret.push_back(al.id);
  }
  return ret;
}

double lerpShiftLengthOnArc(double arc, const AvoidLine & ap)
{
  if (ap.start_longitudinal <= arc && arc < ap.end_longitudinal) {
    if (std::abs(ap.getRelativeLongitudinal()) < 1.0e-5) {
      return ap.end_shift_length;
    }
    const auto start_weight = (ap.end_longitudinal - arc) / ap.getRelativeLongitudinal();
    return start_weight * ap.start_shift_length + (1.0 - start_weight) * ap.end_shift_length;
  }
  return 0.0;
}

void fillLongitudinalAndLengthByClosestEnvelopeFootprint(
  const PathWithLaneId & path, const Point & ego_pos, ObjectData & obj)
{
  double min_distance = std::numeric_limits<double>::max();
  double max_distance = std::numeric_limits<double>::lowest();
  for (const auto & p : obj.envelope_poly.outer()) {
    const auto point = tier4_autoware_utils::createPoint(p.x(), p.y(), 0.0);
    // TODO(someone): search around first position where the ego should avoid the object.
    const double arc_length = calcSignedArcLength(path.points, ego_pos, point);
    min_distance = std::min(min_distance, arc_length);
    max_distance = std::max(max_distance, arc_length);
  }
  obj.longitudinal = min_distance;
  obj.length = max_distance - min_distance;
  return;
}

double calcEnvelopeOverhangDistance(
  const ObjectData & object_data, const PathWithLaneId & path, Point & overhang_pose)
{
  double largest_overhang = isOnRight(object_data) ? -100.0 : 100.0;  // large number

  for (const auto & p : object_data.envelope_poly.outer()) {
    const auto point = tier4_autoware_utils::createPoint(p.x(), p.y(), 0.0);
    // TODO(someone): search around first position where the ego should avoid the object.
    const auto idx = findNearestIndex(path.points, point);
    const auto lateral = calcLateralDeviation(getPose(path.points.at(idx)), point);

    const auto & overhang_pose_on_right = [&overhang_pose, &largest_overhang, &point, &lateral]() {
      if (lateral > largest_overhang) {
        overhang_pose = point;
      }
      return std::max(largest_overhang, lateral);
    };

    const auto & overhang_pose_on_left = [&overhang_pose, &largest_overhang, &point, &lateral]() {
      if (lateral < largest_overhang) {
        overhang_pose = point;
      }
      return std::min(largest_overhang, lateral);
    };

    largest_overhang = isOnRight(object_data) ? overhang_pose_on_right() : overhang_pose_on_left();
  }
  return largest_overhang;
}

void setEndData(
  AvoidLine & ap, const double length, const geometry_msgs::msg::Pose & end, const size_t end_idx,
  const double end_dist)
{
  ap.end_shift_length = length;
  ap.end = end;
  ap.end_idx = end_idx;
  ap.end_longitudinal = end_dist;
}

void setStartData(
  AvoidLine & ap, const double start_shift_length, const geometry_msgs::msg::Pose & start,
  const size_t start_idx, const double start_dist)
{
  ap.start_shift_length = start_shift_length;
  ap.start = start;
  ap.start_idx = start_idx;
  ap.start_longitudinal = start_dist;
}

Polygon2d createEnvelopePolygon(
  const Polygon2d & object_polygon, const Pose & closest_pose, const double envelope_buffer)
{
  namespace bg = boost::geometry;
  using tier4_autoware_utils::expandPolygon;
  using tier4_autoware_utils::Point2d;
  using tier4_autoware_utils::Polygon2d;
  using Box = bg::model::box<Point2d>;

  const auto toPolygon2d = [](const geometry_msgs::msg::Polygon & polygon) {
    Polygon2d ret{};

    for (const auto & p : polygon.points) {
      ret.outer().push_back(Point2d(p.x, p.y));
    }

    return ret;
  };

  Pose pose_2d = closest_pose;
  pose_2d.orientation = createQuaternionFromRPY(0.0, 0.0, tf2::getYaw(closest_pose.orientation));

  TransformStamped geometry_tf{};
  geometry_tf.transform = pose2transform(pose_2d);

  tf2::Transform tf;
  tf2::fromMsg(geometry_tf.transform, tf);
  TransformStamped inverse_geometry_tf{};
  inverse_geometry_tf.transform = tf2::toMsg(tf.inverse());

  geometry_msgs::msg::Polygon out_ros_polygon{};
  tf2::doTransform(
    toMsg(object_polygon, closest_pose.position.z), out_ros_polygon, inverse_geometry_tf);

  const auto envelope_box = bg::return_envelope<Box>(toPolygon2d(out_ros_polygon));

  Polygon2d envelope_poly{};
  bg::convert(envelope_box, envelope_poly);

  geometry_msgs::msg::Polygon envelope_ros_polygon{};
  tf2::doTransform(
    toMsg(envelope_poly, closest_pose.position.z), envelope_ros_polygon, geometry_tf);

  const auto expanded_polygon = expandPolygon(toPolygon2d(envelope_ros_polygon), envelope_buffer);
  return expanded_polygon;
}

Polygon2d createEnvelopePolygon(
  const ObjectData & object_data, const Pose & closest_pose, const double envelope_buffer)
{
  const auto object_polygon = tier4_autoware_utils::toPolygon2d(object_data.object);
  return createEnvelopePolygon(object_polygon, closest_pose, envelope_buffer);
}

std::vector<DrivableAreaInfo::Obstacle> generateObstaclePolygonsForDrivableArea(
  const ObjectDataArray & objects, const std::shared_ptr<AvoidanceParameters> & parameters,
  const double vehicle_width)
{
  std::vector<DrivableAreaInfo::Obstacle> obstacles_for_drivable_area;

  if (objects.empty() || !parameters->enable_bound_clipping) {
    return obstacles_for_drivable_area;
  }

  for (const auto & object : objects) {
    // If avoidance is executed by both behavior and motion, only non-avoidable object will be
    // extracted from the drivable area.
    if (!parameters->disable_path_update) {
      if (object.is_avoidable) {
        continue;
      }
    }

    // check if avoid marin is calculated
    if (!object.avoid_margin.has_value()) {
      continue;
    }

    const auto object_type = utils::getHighestProbLabel(object.object.classification);
    const auto object_parameter = parameters->object_parameters.at(object_type);

    // generate obstacle polygon
    const double diff_poly_buffer =
      object.avoid_margin.value() - object_parameter.envelope_buffer_margin - vehicle_width / 2.0;
    const auto obj_poly =
      tier4_autoware_utils::expandPolygon(object.envelope_poly, diff_poly_buffer);
    obstacles_for_drivable_area.push_back(
      {object.object.kinematics.initial_pose_with_covariance.pose, obj_poly, !isOnRight(object)});
  }
  return obstacles_for_drivable_area;
}

double getLongitudinalVelocity(const Pose & p_ref, const Pose & p_target, const double v)
{
  return v * std::cos(calcYawDeviation(p_ref, p_target));
}

lanelet::ConstLanelets getTargetLanelets(
  const std::shared_ptr<const PlannerData> & planner_data, lanelet::ConstLanelets & route_lanelets,
  const double left_offset, const double right_offset)
{
  const auto & rh = planner_data->route_handler;

  lanelet::ConstLanelets target_lanelets{};
  for (const auto & lane : route_lanelets) {
    auto l_offset = 0.0;
    auto r_offset = 0.0;

    const auto opt_left_lane = rh->getLeftLanelet(lane);
    if (opt_left_lane) {
      target_lanelets.push_back(opt_left_lane.value());
    } else {
      l_offset = left_offset;
    }

    const auto opt_right_lane = rh->getRightLanelet(lane);
    if (opt_right_lane) {
      target_lanelets.push_back(opt_right_lane.value());
    } else {
      r_offset = right_offset;
    }

    const auto right_opposite_lanes = rh->getRightOppositeLanelets(lane);
    if (!right_opposite_lanes.empty()) {
      target_lanelets.push_back(right_opposite_lanes.front());
    }

    const auto expand_lane = lanelet::utils::getExpandedLanelet(lane, l_offset, r_offset);
    target_lanelets.push_back(expand_lane);
  }

  return target_lanelets;
}

lanelet::ConstLanelets getCurrentLanesFromPath(
  const PathWithLaneId & path, const std::shared_ptr<const PlannerData> & planner_data)
{
  if (path.points.empty()) {
    throw std::logic_error("empty path.");
  }

  if (path.points.front().lane_ids.empty()) {
    throw std::logic_error("empty lane ids.");
  }

  const auto start_id = path.points.front().lane_ids.front();
  const auto start_lane = planner_data->route_handler->getLaneletsFromId(start_id);
  const auto & p = planner_data->parameters;

  return planner_data->route_handler->getLaneletSequence(
    start_lane, p.backward_path_length, p.forward_path_length);
}

lanelet::ConstLanelets getExtendLanes(
  const lanelet::ConstLanelets & lanelets, const Pose & ego_pose,
  const std::shared_ptr<const PlannerData> & planner_data)
{
  lanelet::ConstLanelets extend_lanelets = lanelets;

  while (rclcpp::ok()) {
    const double lane_length = lanelet::utils::getLaneletLength2d(extend_lanelets);
    const auto arc_coordinates = lanelet::utils::getArcCoordinates(extend_lanelets, ego_pose);
    const auto forward_length = lane_length - arc_coordinates.length;

    if (forward_length > planner_data->parameters.forward_path_length) {
      break;
    }

    const auto next_lanelets = planner_data->route_handler->getNextLanelets(extend_lanelets.back());

    if (next_lanelets.empty()) {
      break;
    }

    extend_lanelets.push_back(next_lanelets.front());
  }

  return extend_lanelets;
}

void insertDecelPoint(
  const Point & p_src, const double offset, const double velocity, PathWithLaneId & path,
  std::optional<Pose> & p_out)
{
  const auto decel_point = calcLongitudinalOffsetPoint(path.points, p_src, offset);

  if (!decel_point) {
    // TODO(Satoshi OTA)  Think later the process in the case of no decel point found.
    return;
  }

  const auto seg_idx = findNearestSegmentIndex(path.points, decel_point.value());
  const auto insert_idx = insertTargetPoint(seg_idx, decel_point.value(), path.points);

  if (!insert_idx) {
    // TODO(Satoshi OTA)  Think later the process in the case of no decel point found.
    return;
  }

  const auto insertVelocity = [&insert_idx](PathWithLaneId & path, const float v) {
    for (size_t i = insert_idx.value(); i < path.points.size(); ++i) {
      const auto & original_velocity = path.points.at(i).point.longitudinal_velocity_mps;
      path.points.at(i).point.longitudinal_velocity_mps = std::min(original_velocity, v);
    }
  };

  insertVelocity(path, velocity);

  p_out = getPose(path.points.at(insert_idx.value()));
}

void fillObjectEnvelopePolygon(
  ObjectData & object_data, const ObjectDataArray & registered_objects, const Pose & closest_pose,
  const std::shared_ptr<AvoidanceParameters> & parameters)
{
  const auto object_type = utils::getHighestProbLabel(object_data.object.classification);
  const auto object_parameter = parameters->object_parameters.at(object_type);

  const auto & envelope_buffer_margin =
    object_parameter.envelope_buffer_margin * object_data.distance_factor;

  const auto id = object_data.object.object_id;
  const auto same_id_obj = std::find_if(
    registered_objects.begin(), registered_objects.end(),
    [&id](const auto & o) { return o.object.object_id == id; });

  if (same_id_obj == registered_objects.end()) {
    object_data.envelope_poly =
      createEnvelopePolygon(object_data, closest_pose, envelope_buffer_margin);
    return;
  }

  const auto one_shot_envelope_poly =
    createEnvelopePolygon(object_data, closest_pose, envelope_buffer_margin);

  // If the one_shot_envelope_poly is within the registered envelope, use the registered one
  if (boost::geometry::within(one_shot_envelope_poly, same_id_obj->envelope_poly)) {
    object_data.envelope_poly = same_id_obj->envelope_poly;
    return;
  }

  std::vector<Polygon2d> unions;
  boost::geometry::union_(one_shot_envelope_poly, same_id_obj->envelope_poly, unions);

  // If union fails, use the current envelope
  if (unions.empty()) {
    object_data.envelope_poly = one_shot_envelope_poly;
    return;
  }

  boost::geometry::correct(unions.front());

  const auto multi_step_envelope_poly = createEnvelopePolygon(unions.front(), closest_pose, 0.0);

  const auto object_polygon = tier4_autoware_utils::toPolygon2d(object_data.object);
  const auto object_polygon_area = boost::geometry::area(object_polygon);
  const auto envelope_polygon_area = boost::geometry::area(multi_step_envelope_poly);

  // keep multi-step envelope polygon.
  constexpr double THRESHOLD = 5.0;
  if (envelope_polygon_area < object_polygon_area * THRESHOLD) {
    object_data.envelope_poly = multi_step_envelope_poly;
    return;
  }

  // use latest one-shot envelope polygon.
  object_data.envelope_poly = one_shot_envelope_poly;
}

void fillObjectMovingTime(
  ObjectData & object_data, ObjectDataArray & stopped_objects,
  const std::shared_ptr<AvoidanceParameters> & parameters)
{
  const auto object_type = utils::getHighestProbLabel(object_data.object.classification);
  const auto object_parameter = parameters->object_parameters.at(object_type);

  const auto & object_twist = object_data.object.kinematics.initial_twist_with_covariance.twist;
  const auto object_vel_norm = std::hypot(object_twist.linear.x, object_twist.linear.y);
  const auto is_faster_than_threshold = object_vel_norm > object_parameter.moving_speed_threshold;

  const auto id = object_data.object.object_id;
  const auto same_id_obj = std::find_if(
    stopped_objects.begin(), stopped_objects.end(),
    [&id](const auto & o) { return o.object.object_id == id; });

  const auto is_new_object = same_id_obj == stopped_objects.end();
  const rclcpp::Time now = rclcpp::Clock(RCL_ROS_TIME).now();

  if (!is_faster_than_threshold) {
    object_data.last_stop = now;
    object_data.move_time = 0.0;
    if (is_new_object) {
      object_data.stop_time = 0.0;
      object_data.last_move = now;
      stopped_objects.push_back(object_data);
    } else {
      same_id_obj->stop_time = (now - same_id_obj->last_move).seconds();
      same_id_obj->last_stop = now;
      same_id_obj->move_time = 0.0;
      object_data.stop_time = same_id_obj->stop_time;
    }
    return;
  }

  if (is_new_object) {
    object_data.move_time = std::numeric_limits<double>::infinity();
    object_data.stop_time = 0.0;
    object_data.last_move = now;
    return;
  }

  object_data.last_stop = same_id_obj->last_stop;
  object_data.move_time = (now - same_id_obj->last_stop).seconds();
  object_data.stop_time = 0.0;

  if (object_data.move_time > object_parameter.moving_time_threshold) {
    stopped_objects.erase(same_id_obj);
  }
}

void fillAvoidanceNecessity(
  ObjectData & object_data, const ObjectDataArray & registered_objects, const double vehicle_width,
  const std::shared_ptr<AvoidanceParameters> & parameters)
{
  const auto object_type = utils::getHighestProbLabel(object_data.object.classification);
  const auto object_parameter = parameters->object_parameters.at(object_type);
  const auto safety_margin =
    0.5 * vehicle_width + object_parameter.safety_buffer_lateral * object_data.distance_factor;

  const auto check_necessity = [&](const auto hysteresis_factor) {
    return (isOnRight(object_data) &&
            std::abs(object_data.overhang_dist) < safety_margin * hysteresis_factor) ||
           (!isOnRight(object_data) &&
            object_data.overhang_dist < safety_margin * hysteresis_factor);
  };

  const auto id = object_data.object.object_id;
  const auto same_id_obj = std::find_if(
    registered_objects.begin(), registered_objects.end(),
    [&id](const auto & o) { return o.object.object_id == id; });

  // First time
  if (same_id_obj == registered_objects.end()) {
    object_data.avoid_required = check_necessity(1.0);
    return;
  }

  // FALSE -> FALSE or FALSE -> TRUE
  if (!same_id_obj->avoid_required) {
    object_data.avoid_required = check_necessity(1.0);
    return;
  }

  // TRUE -> ? (check with hysteresis factor)
  object_data.avoid_required = check_necessity(parameters->hysteresis_factor_expand_rate);
}

void fillInitialPose(ObjectData & object_data, ObjectDataArray & detected_objects)
{
  const auto id = object_data.object.object_id;
  const auto same_id_obj = std::find_if(
    detected_objects.begin(), detected_objects.end(),
    [&id](const auto & o) { return o.object.object_id == id; });

  if (same_id_obj != detected_objects.end()) {
    object_data.init_pose = same_id_obj->init_pose;
    return;
  }

  object_data.init_pose = object_data.object.kinematics.initial_pose_with_covariance.pose;
  detected_objects.push_back(object_data);
}

void fillObjectStoppableJudge(
  ObjectData & object_data, const ObjectDataArray & registered_objects,
  const double feasible_stop_distance, const std::shared_ptr<AvoidanceParameters> & parameters)
{
  if (parameters->policy_deceleration == "reliable") {
    object_data.is_stoppable = true;
    return;
  }

  if (!object_data.avoid_required) {
    object_data.is_stoppable = false;
    return;
  }

  const auto id = object_data.object.object_id;
  const auto same_id_obj = std::find_if(
    registered_objects.begin(), registered_objects.end(),
    [&id](const auto & o) { return o.object.object_id == id; });

  const auto is_stoppable = object_data.to_stop_line > feasible_stop_distance;
  if (is_stoppable) {
    object_data.is_stoppable = true;
    return;
  }

  if (same_id_obj == registered_objects.end()) {
    object_data.is_stoppable = false;
    return;
  }

  object_data.is_stoppable = same_id_obj->is_stoppable;
}

void updateRegisteredObject(
  ObjectDataArray & registered_objects, const ObjectDataArray & now_objects,
  const std::shared_ptr<AvoidanceParameters> & parameters)
{
  const auto updateIfDetectedNow = [&now_objects](auto & registered_object) {
    const auto & n = now_objects;
    const auto r_id = registered_object.object.object_id;
    const auto same_id_obj = std::find_if(
      n.begin(), n.end(), [&r_id](const auto & o) { return o.object.object_id == r_id; });

    // same id object is detected. update registered.
    if (same_id_obj != n.end()) {
      registered_object = *same_id_obj;
      return true;
    }

    constexpr auto POS_THR = 1.5;
    const auto r_pos = registered_object.object.kinematics.initial_pose_with_covariance.pose;
    const auto similar_pos_obj = std::find_if(n.begin(), n.end(), [&](const auto & o) {
      return calcDistance2d(r_pos, o.object.kinematics.initial_pose_with_covariance.pose) < POS_THR;
    });

    // same id object is not detected, but object is found around registered. update registered.
    if (similar_pos_obj != n.end()) {
      registered_object = *similar_pos_obj;
      return true;
    }

    // Same ID nor similar position object does not found.
    return false;
  };

  const rclcpp::Time now = rclcpp::Clock(RCL_ROS_TIME).now();

  // -- check registered_objects, remove if lost_count exceeds limit. --
  for (int i = static_cast<int>(registered_objects.size()) - 1; i >= 0; --i) {
    auto & r = registered_objects.at(i);

    // registered object is not detected this time. lost count up.
    if (!updateIfDetectedNow(r)) {
      r.lost_time = (now - r.last_seen).seconds();
    } else {
      r.last_seen = now;
      r.lost_time = 0.0;
    }

    // lost count exceeds threshold. remove object from register.
    if (r.lost_time > parameters->object_last_seen_threshold) {
      registered_objects.erase(registered_objects.begin() + i);
    }
  }

  const auto isAlreadyRegistered = [&](const auto & n_id) {
    const auto & r = registered_objects;
    return std::any_of(
      r.begin(), r.end(), [&n_id](const auto & o) { return o.object.object_id == n_id; });
  };

  // -- check now_objects, add it if it has new object id --
  for (const auto & now_obj : now_objects) {
    if (!isAlreadyRegistered(now_obj.object.object_id)) {
      registered_objects.push_back(now_obj);
    }
  }
}

void compensateDetectionLost(
  const ObjectDataArray & registered_objects, ObjectDataArray & now_objects,
  ObjectDataArray & other_objects)
{
  const auto isDetectedNow = [&](const auto & r_id) {
    const auto & n = now_objects;
    return std::any_of(
      n.begin(), n.end(), [&r_id](const auto & o) { return o.object.object_id == r_id; });
  };

  const auto isIgnoreObject = [&](const auto & r_id) {
    const auto & n = other_objects;
    return std::any_of(
      n.begin(), n.end(), [&r_id](const auto & o) { return o.object.object_id == r_id; });
  };

  for (const auto & registered : registered_objects) {
    if (
      !isDetectedNow(registered.object.object_id) && !isIgnoreObject(registered.object.object_id)) {
      now_objects.push_back(registered);
    }
  }
}

double getRoadShoulderDistance(
  ObjectData & object, const AvoidancePlanningData & data,
  const std::shared_ptr<const PlannerData> & planner_data,
  [[maybe_unused]] const std::shared_ptr<AvoidanceParameters> & parameters)
{
  using lanelet::utils::to2D;
  using tier4_autoware_utils::Point2d;

  const auto & object_pose = object.object.kinematics.initial_pose_with_covariance.pose;
  const auto object_closest_index =
    findNearestIndex(data.reference_path.points, object_pose.position);
  const auto object_closest_pose = data.reference_path.points.at(object_closest_index).point.pose;

  const auto rh = planner_data->route_handler;
  if (!rh->getClosestLaneletWithinRoute(object_closest_pose, &object.overhang_lanelet)) {
    return 0.0;
  }

  const auto centerline_pose =
    lanelet::utils::getClosestCenterPose(object.overhang_lanelet, object_pose.position);
  const auto & p1_object = object.overhang_pose.position;
  const auto p_tmp =
    geometry_msgs::build<Pose>().position(p1_object).orientation(centerline_pose.orientation);
  const auto p2_object =
    calcOffsetPose(p_tmp, 0.0, (isOnRight(object) ? 100.0 : -100.0), 0.0).position;

  // TODO(Satoshi OTA): check if the basic point is on right or left of bound.
  const auto bound = isOnRight(object) ? data.left_bound : data.right_bound;

  std::vector<Point> intersects;
  for (size_t i = 1; i < bound.size(); i++) {
    const auto p1_bound =
      geometry_msgs::build<Point>().x(bound[i - 1].x()).y(bound[i - 1].y()).z(bound[i - 1].z());
    const auto p2_bound =
      geometry_msgs::build<Point>().x(bound[i].x()).y(bound[i].y()).z(bound[i].z());

    const auto opt_intersect =
      tier4_autoware_utils::intersect(p1_object, p2_object, p1_bound, p2_bound);

    if (!opt_intersect) {
      continue;
    }

    intersects.push_back(opt_intersect.value());
  }

  if (intersects.empty()) {
    return 0.0;
  }

  std::sort(intersects.begin(), intersects.end(), [&p1_object](const auto & a, const auto & b) {
    return calcDistance2d(p1_object, a) < calcDistance2d(p1_object, b);
  });

  object.nearest_bound_point = intersects.front();

  return calcDistance2d(p1_object, object.nearest_bound_point.value());
}

void filterTargetObjects(
  ObjectDataArray & objects, AvoidancePlanningData & data, [[maybe_unused]] DebugData & debug,
  const std::shared_ptr<const PlannerData> & planner_data,
  const std::shared_ptr<AvoidanceParameters> & parameters)
{
  if (data.current_lanelets.empty()) {
    return;
  }

  const rclcpp::Time now = rclcpp::Clock(RCL_ROS_TIME).now();
  const auto push_target_object = [&data, &now](auto & object) {
    object.last_seen = now;
    data.target_objects.push_back(object);
  };

  for (auto & o : objects) {
    if (!filtering_utils::isSatisfiedWithCommonCondition(o, data, planner_data, parameters)) {
      data.other_objects.push_back(o);
      continue;
    }

    o.to_road_shoulder_distance = getRoadShoulderDistance(o, data, planner_data, parameters);
    o.avoid_margin = filtering_utils::getAvoidMargin(o, planner_data, parameters);

    if (filtering_utils::isNoNeedAvoidanceBehavior(o, parameters)) {
      data.other_objects.push_back(o);
      continue;
    }

    if (filtering_utils::isVehicleTypeObject(o)) {
      if (!filtering_utils::isSatisfiedWithVehicleCondition(o, data, planner_data, parameters)) {
        data.other_objects.push_back(o);
        continue;
      }
    } else {
      if (!filtering_utils::isSatisfiedWithNonVehicleCondition(o, data, planner_data, parameters)) {
        data.other_objects.push_back(o);
        continue;
      }
    }

    push_target_object(o);
  }
}

AvoidLine fillAdditionalInfo(const AvoidancePlanningData & data, const AvoidLine & line)
{
  AvoidLineArray ret{line};
  fillAdditionalInfoFromPoint(data, ret);
  return ret.front();
}

void fillAdditionalInfoFromPoint(const AvoidancePlanningData & data, AvoidLineArray & lines)
{
  if (lines.empty()) {
    return;
  }

  const auto & path = data.reference_path;
  const auto & arc = data.arclength_from_ego;

  // calc longitudinal
  for (auto & sl : lines) {
    sl.start_idx = findNearestIndex(path.points, sl.start.position);
    sl.start_longitudinal = arc.at(sl.start_idx);
    sl.end_idx = findNearestIndex(path.points, sl.end.position);
    sl.end_longitudinal = arc.at(sl.end_idx);
  }
}

void fillAdditionalInfoFromLongitudinal(const AvoidancePlanningData & data, AvoidLine & line)
{
  const auto & path = data.reference_path;
  const auto & arc = data.arclength_from_ego;

  line.start_idx = findPathIndexFromArclength(arc, line.start_longitudinal);
  line.start = path.points.at(line.start_idx).point.pose;
  line.end_idx = findPathIndexFromArclength(arc, line.end_longitudinal);
  line.end = path.points.at(line.end_idx).point.pose;
}

void fillAdditionalInfoFromLongitudinal(
  const AvoidancePlanningData & data, AvoidOutlines & outlines)
{
  for (auto & outline : outlines) {
    fillAdditionalInfoFromLongitudinal(data, outline.avoid_line);
    fillAdditionalInfoFromLongitudinal(data, outline.return_line);

    std::for_each(outline.middle_lines.begin(), outline.middle_lines.end(), [&](auto & line) {
      fillAdditionalInfoFromLongitudinal(data, line);
    });
  }
}

void fillAdditionalInfoFromLongitudinal(const AvoidancePlanningData & data, AvoidLineArray & lines)
{
  const auto & path = data.reference_path;
  const auto & arc = data.arclength_from_ego;

  for (auto & sl : lines) {
    sl.start_idx = findPathIndexFromArclength(arc, sl.start_longitudinal);
    sl.start = path.points.at(sl.start_idx).point.pose;
    sl.end_idx = findPathIndexFromArclength(arc, sl.end_longitudinal);
    sl.end = path.points.at(sl.end_idx).point.pose;
  }
}

AvoidLineArray combineRawShiftLinesWithUniqueCheck(
  const AvoidLineArray & base_lines, const AvoidLineArray & added_lines)
{
  // TODO(Horibe) parametrize
  const auto isSimilar = [](const AvoidLine & a, const AvoidLine & b) {
    using tier4_autoware_utils::calcDistance2d;
    if (calcDistance2d(a.start, b.start) > 1.0) {
      return false;
    }
    if (calcDistance2d(a.end, b.end) > 1.0) {
      return false;
    }
    if (std::abs(a.end_shift_length - b.end_shift_length) > 0.5) {
      return false;
    }
    return true;
  };
  const auto hasSameObjectId = [](const auto & a, const auto & b) {
    return a.object.object.object_id == b.object.object.object_id;
  };

  auto combined = base_lines;  // initialized
  for (const auto & added_line : added_lines) {
    bool skip = false;

    for (const auto & base_line : base_lines) {
      if (hasSameObjectId(added_line, base_line) && isSimilar(added_line, base_line)) {
        skip = true;
        break;
      }
    }
    if (!skip) {
      combined.push_back(added_line);
    }
  }

  return combined;
}

lanelet::ConstLanelets getAdjacentLane(
  const std::shared_ptr<const PlannerData> & planner_data,
  const std::shared_ptr<AvoidanceParameters> & parameters, const bool is_right_shift)
{
  const auto & rh = planner_data->route_handler;
  const auto & forward_distance = parameters->object_check_max_forward_distance;
  const auto & backward_distance = parameters->safety_check_backward_distance;
  const auto & vehicle_pose = planner_data->self_odometry->pose.pose;

  lanelet::ConstLanelet current_lane;
  if (!rh->getClosestLaneletWithinRoute(vehicle_pose, &current_lane)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("behavior_path_planner").get_child("avoidance"),
      "failed to find closest lanelet within route!!!");
    return {};  // TODO(Satoshi Ota)
  }

  const auto ego_succeeding_lanes =
    rh->getLaneletSequence(current_lane, vehicle_pose, backward_distance, forward_distance);

  lanelet::ConstLanelets lanes{};
  for (const auto & lane : ego_succeeding_lanes) {
    const auto opt_left_lane = rh->getLeftLanelet(lane, true, false);
    if (!is_right_shift && opt_left_lane) {
      lanes.push_back(opt_left_lane.value());
    }

    const auto opt_right_lane = rh->getRightLanelet(lane, true, false);
    if (is_right_shift && opt_right_lane) {
      lanes.push_back(opt_right_lane.value());
    }

    const auto right_opposite_lanes = rh->getRightOppositeLanelets(lane);
    if (is_right_shift && !right_opposite_lanes.empty()) {
      lanes.push_back(right_opposite_lanes.front());
    }
  }

  return lanes;
}

std::vector<ExtendedPredictedObject> getSafetyCheckTargetObjects(
  const AvoidancePlanningData & data, const std::shared_ptr<const PlannerData> & planner_data,
  const std::shared_ptr<AvoidanceParameters> & parameters, const bool is_right_shift,
  DebugData & debug)
{
  const auto & p = parameters;
  const auto check_right_lanes =
    (is_right_shift && p->check_shift_side_lane) || (!is_right_shift && p->check_other_side_lane);
  const auto check_left_lanes =
    (!is_right_shift && p->check_shift_side_lane) || (is_right_shift && p->check_other_side_lane);

  std::vector<ExtendedPredictedObject> target_objects;

  const auto time_horizon = std::max(
    parameters->ego_predicted_path_params.time_horizon_for_front_object,
    parameters->ego_predicted_path_params.time_horizon_for_rear_object);

  const auto append = [&](const auto & objects) {
    std::for_each(objects.objects.begin(), objects.objects.end(), [&](const auto & object) {
      target_objects.push_back(utils::path_safety_checker::transform(
        object, time_horizon, parameters->ego_predicted_path_params.time_resolution));
    });
  };

  const auto to_predicted_objects = [&p, &parameters](const auto & objects) {
    PredictedObjects ret{};
    std::for_each(objects.begin(), objects.end(), [&p, &ret, &parameters](const auto & object) {
      if (filtering_utils::isSafetyCheckTargetObjectType(object.object, parameters)) {
        ret.objects.push_back(object.object);
      }
    });
    return ret;
  };

  const auto unavoidable_objects = [&data]() {
    ObjectDataArray ret;
    std::for_each(data.target_objects.begin(), data.target_objects.end(), [&](const auto & object) {
      if (!object.is_avoidable) {
        ret.push_back(object);
      }
    });
    return ret;
  }();

  // check right lanes
  if (check_right_lanes) {
    const auto check_lanes = getAdjacentLane(planner_data, p, true);

    if (p->check_other_object) {
      const auto [targets, others] = utils::path_safety_checker::separateObjectsByLanelets(
        to_predicted_objects(data.other_objects), check_lanes,
        utils::path_safety_checker::isCentroidWithinLanelet);
      append(targets);
    }

    if (p->check_unavoidable_object) {
      const auto [targets, others] = utils::path_safety_checker::separateObjectsByLanelets(
        to_predicted_objects(unavoidable_objects), check_lanes,
        utils::path_safety_checker::isCentroidWithinLanelet);
      append(targets);
    }

    debug.safety_check_lanes.insert(
      debug.safety_check_lanes.end(), check_lanes.begin(), check_lanes.end());
  }

  // check left lanes
  if (check_left_lanes) {
    const auto check_lanes = getAdjacentLane(planner_data, p, false);

    if (p->check_other_object) {
      const auto [targets, others] = utils::path_safety_checker::separateObjectsByLanelets(
        to_predicted_objects(data.other_objects), check_lanes,
        utils::path_safety_checker::isCentroidWithinLanelet);
      append(targets);
    }

    if (p->check_unavoidable_object) {
      const auto [targets, others] = utils::path_safety_checker::separateObjectsByLanelets(
        to_predicted_objects(unavoidable_objects), check_lanes,
        utils::path_safety_checker::isCentroidWithinLanelet);
      append(targets);
    }

    debug.safety_check_lanes.insert(
      debug.safety_check_lanes.end(), check_lanes.begin(), check_lanes.end());
  }

  // check current lanes
  if (p->check_current_lane) {
    const auto check_lanes = data.current_lanelets;

    if (p->check_other_object) {
      const auto [targets, others] = utils::path_safety_checker::separateObjectsByLanelets(
        to_predicted_objects(data.other_objects), check_lanes,
        utils::path_safety_checker::isCentroidWithinLanelet);
      append(targets);
    }

    if (p->check_unavoidable_object) {
      const auto [targets, others] = utils::path_safety_checker::separateObjectsByLanelets(
        to_predicted_objects(unavoidable_objects), check_lanes,
        utils::path_safety_checker::isCentroidWithinLanelet);
      append(targets);
    }

    debug.safety_check_lanes.insert(
      debug.safety_check_lanes.end(), check_lanes.begin(), check_lanes.end());
  }

  return target_objects;
}

std::pair<PredictedObjects, PredictedObjects> separateObjectsByPath(
  const PathWithLaneId & path, const std::shared_ptr<const PlannerData> & planner_data,
  const AvoidancePlanningData & data, const std::shared_ptr<AvoidanceParameters> & parameters,
  const double object_check_forward_distance, const bool is_running, DebugData & debug)
{
  PredictedObjects target_objects;
  PredictedObjects other_objects;

  double max_offset = 0.0;
  for (const auto & object_parameter : parameters->object_parameters) {
    const auto p = object_parameter.second;
    const auto offset =
      2.0 * p.envelope_buffer_margin + p.safety_buffer_lateral + p.avoid_margin_lateral;
    max_offset = std::max(max_offset, offset);
  }

  const auto detection_area =
    createVehiclePolygon(planner_data->parameters.vehicle_info, max_offset);
  const auto ego_idx = planner_data->findEgoIndex(path.points);

  Polygon2d attention_area;
  for (size_t i = 0; i < path.points.size() - 1; ++i) {
    const auto & p_ego_front = path.points.at(i).point.pose;
    const auto & p_ego_back = path.points.at(i + 1).point.pose;

    const auto distance_from_ego = calcSignedArcLength(path.points, ego_idx, i);
    if (distance_from_ego > object_check_forward_distance) {
      break;
    }

    const auto ego_one_step_polygon = createOneStepPolygon(p_ego_front, p_ego_back, detection_area);

    std::vector<Polygon2d> unions;
    boost::geometry::union_(attention_area, ego_one_step_polygon, unions);
    if (!unions.empty()) {
      attention_area = unions.front();
      boost::geometry::correct(attention_area);
    }
  }

  // expand detection area width only when the module is running.
  if (is_running) {
    constexpr int PER_CIRCLE = 36;
    constexpr double MARGIN = 1.0;  // [m]
    boost::geometry::strategy::buffer::distance_symmetric<double> distance_strategy(MARGIN);
    boost::geometry::strategy::buffer::join_round join_strategy(PER_CIRCLE);
    boost::geometry::strategy::buffer::end_round end_strategy(PER_CIRCLE);
    boost::geometry::strategy::buffer::point_circle circle_strategy(PER_CIRCLE);
    boost::geometry::strategy::buffer::side_straight side_strategy;
    boost::geometry::model::multi_polygon<Polygon2d> result;
    // Create the buffer of a multi polygon
    boost::geometry::buffer(
      attention_area, result, distance_strategy, side_strategy, join_strategy, end_strategy,
      circle_strategy);
    if (!result.empty()) {
      attention_area = result.front();
    }
  }

  debug.detection_area = toMsg(attention_area, data.reference_pose.position.z);

  const auto objects = planner_data->dynamic_object->objects;
  std::for_each(objects.begin(), objects.end(), [&](const auto & object) {
    const auto obj_polygon = tier4_autoware_utils::toPolygon2d(object);
    if (boost::geometry::disjoint(obj_polygon, attention_area)) {
      other_objects.objects.push_back(object);
    } else {
      target_objects.objects.push_back(object);
    }
  });

  return std::make_pair(target_objects, other_objects);
}

DrivableLanes generateExpandDrivableLanes(
  const lanelet::ConstLanelet & lanelet, const std::shared_ptr<const PlannerData> & planner_data,
  const std::shared_ptr<AvoidanceParameters> & parameters)
{
  const auto & route_handler = planner_data->route_handler;

  DrivableLanes current_drivable_lanes;
  current_drivable_lanes.left_lane = lanelet;
  current_drivable_lanes.right_lane = lanelet;

  if (!parameters->use_adjacent_lane) {
    return current_drivable_lanes;
  }

  // 1. get left/right side lanes
  const auto update_left_lanelets = [&](const lanelet::ConstLanelet & target_lane) {
    const auto all_left_lanelets = route_handler->getAllLeftSharedLinestringLanelets(
      target_lane, parameters->use_opposite_lane, true);
    if (!all_left_lanelets.empty()) {
      current_drivable_lanes.left_lane = all_left_lanelets.back();  // leftmost lanelet
      pushUniqueVector(
        current_drivable_lanes.middle_lanes,
        lanelet::ConstLanelets(all_left_lanelets.begin(), all_left_lanelets.end() - 1));
    }
  };
  const auto update_right_lanelets = [&](const lanelet::ConstLanelet & target_lane) {
    const auto all_right_lanelets = route_handler->getAllRightSharedLinestringLanelets(
      target_lane, parameters->use_opposite_lane, true);
    if (!all_right_lanelets.empty()) {
      current_drivable_lanes.right_lane = all_right_lanelets.back();  // rightmost lanelet
      pushUniqueVector(
        current_drivable_lanes.middle_lanes,
        lanelet::ConstLanelets(all_right_lanelets.begin(), all_right_lanelets.end() - 1));
    }
  };

  update_left_lanelets(lanelet);
  update_right_lanelets(lanelet);

  // 2.1 when there are multiple lanes whose previous lanelet is the same
  const auto get_next_lanes_from_same_previous_lane =
    [&route_handler](const lanelet::ConstLanelet & lane) {
      // get previous lane, and return false if previous lane does not exist
      lanelet::ConstLanelets prev_lanes;
      if (!route_handler->getPreviousLaneletsWithinRoute(lane, &prev_lanes)) {
        return lanelet::ConstLanelets{};
      }

      lanelet::ConstLanelets next_lanes;
      for (const auto & prev_lane : prev_lanes) {
        const auto next_lanes_from_prev = route_handler->getNextLanelets(prev_lane);
        pushUniqueVector(next_lanes, next_lanes_from_prev);
      }
      return next_lanes;
    };

  const auto next_lanes_for_right =
    get_next_lanes_from_same_previous_lane(current_drivable_lanes.right_lane);
  const auto next_lanes_for_left =
    get_next_lanes_from_same_previous_lane(current_drivable_lanes.left_lane);

  // 2.2 look for neighbor lane recursively, where end line of the lane is connected to end line
  // of the original lane
  const auto update_drivable_lanes =
    [&](const lanelet::ConstLanelets & next_lanes, const bool is_left) {
      for (const auto & next_lane : next_lanes) {
        const auto & edge_lane =
          is_left ? current_drivable_lanes.left_lane : current_drivable_lanes.right_lane;
        if (next_lane.id() == edge_lane.id()) {
          continue;
        }

        const auto & left_lane = is_left ? next_lane : edge_lane;
        const auto & right_lane = is_left ? edge_lane : next_lane;
        if (!isEndPointsConnected(left_lane, right_lane)) {
          continue;
        }

        if (is_left) {
          current_drivable_lanes.left_lane = next_lane;
        } else {
          current_drivable_lanes.right_lane = next_lane;
        }

        const auto & middle_lanes = current_drivable_lanes.middle_lanes;
        const auto has_same_lane = std::any_of(
          middle_lanes.begin(), middle_lanes.end(),
          [&edge_lane](const auto & lane) { return lane.id() == edge_lane.id(); });

        if (!has_same_lane) {
          if (is_left) {
            if (current_drivable_lanes.right_lane.id() != edge_lane.id()) {
              current_drivable_lanes.middle_lanes.push_back(edge_lane);
            }
          } else {
            if (current_drivable_lanes.left_lane.id() != edge_lane.id()) {
              current_drivable_lanes.middle_lanes.push_back(edge_lane);
            }
          }
        }

        return true;
      }
      return false;
    };

  const auto expand_drivable_area_recursively =
    [&](const lanelet::ConstLanelets & next_lanes, const bool is_left) {
      // NOTE: set max search num to avoid infinity loop for drivable area expansion
      constexpr size_t max_recursive_search_num = 3;
      for (size_t i = 0; i < max_recursive_search_num; ++i) {
        const bool is_update_kept = update_drivable_lanes(next_lanes, is_left);
        if (!is_update_kept) {
          break;
        }
        if (i == max_recursive_search_num - 1) {
          RCLCPP_ERROR(
            rclcpp::get_logger("behavior_path_planner").get_child("avoidance"),
            "Drivable area expansion reaches max iteration.");
        }
      }
    };
  expand_drivable_area_recursively(next_lanes_for_right, false);
  expand_drivable_area_recursively(next_lanes_for_left, true);

  // 3. update again for new left/right lanes
  update_left_lanelets(current_drivable_lanes.left_lane);
  update_right_lanelets(current_drivable_lanes.right_lane);

  // 4. compensate that current_lane is in either of left_lane, right_lane or middle_lanes.
  if (
    current_drivable_lanes.left_lane.id() != lanelet.id() &&
    current_drivable_lanes.right_lane.id() != lanelet.id()) {
    current_drivable_lanes.middle_lanes.push_back(lanelet);
  }

  return current_drivable_lanes;
}

double calcDistanceToAvoidStartLine(
  const lanelet::ConstLanelets & lanelets, const PathWithLaneId & path,
  const std::shared_ptr<const PlannerData> & planner_data,
  const std::shared_ptr<AvoidanceParameters> & parameters)
{
  if (lanelets.empty()) {
    return std::numeric_limits<double>::lowest();
  }

  double distance_to_return_dead_line = std::numeric_limits<double>::lowest();

  // dead line stop factor(traffic light)
  if (parameters->enable_dead_line_for_traffic_light) {
    const auto to_traffic_light = calcDistanceToRedTrafficLight(lanelets, path, planner_data);
    if (to_traffic_light.has_value()) {
      distance_to_return_dead_line = std::max(
        distance_to_return_dead_line,
        to_traffic_light.value() + parameters->dead_line_buffer_for_traffic_light);
    }
  }

  return distance_to_return_dead_line;
}

double calcDistanceToReturnDeadLine(
  const lanelet::ConstLanelets & lanelets, const PathWithLaneId & path,
  const std::shared_ptr<const PlannerData> & planner_data,
  const std::shared_ptr<AvoidanceParameters> & parameters)
{
  if (lanelets.empty()) {
    return std::numeric_limits<double>::max();
  }

  double distance_to_return_dead_line = std::numeric_limits<double>::max();

  // dead line stop factor(traffic light)
  if (parameters->enable_dead_line_for_traffic_light) {
    const auto to_traffic_light = calcDistanceToRedTrafficLight(lanelets, path, planner_data);
    if (to_traffic_light.has_value()) {
      distance_to_return_dead_line = std::min(
        distance_to_return_dead_line,
        to_traffic_light.value() - parameters->dead_line_buffer_for_traffic_light);
    }
  }

  // dead line for goal
  if (parameters->enable_dead_line_for_goal) {
    if (planner_data->route_handler->isInGoalRouteSection(lanelets.back())) {
      const auto & ego_pos = planner_data->self_odometry->pose.pose.position;
      const auto to_goal_distance =
        calcSignedArcLength(path.points, ego_pos, path.points.size() - 1);
      distance_to_return_dead_line = std::min(
        distance_to_return_dead_line, to_goal_distance - parameters->dead_line_buffer_for_goal);
    }
  }

  return distance_to_return_dead_line;
}

TurnSignalInfo calcTurnSignalInfo(
  const ShiftedPath & path, const ShiftLine & shift_line, const double current_shift_length,
  const AvoidancePlanningData & data, const std::shared_ptr<const PlannerData> & planner_data)
{
  constexpr double THRESHOLD = 0.1;
  const auto & p = planner_data->parameters;
  const auto & rh = planner_data->route_handler;
  const auto & ego_pose = planner_data->self_odometry->pose.pose;
  const auto & ego_speed = planner_data->self_odometry->twist.twist.linear.x;

  if (shift_line.start_idx + 1 > path.shift_length.size()) {
    RCLCPP_WARN(rclcpp::get_logger(__func__), "index inconsistency.");
    return {};
  }

  if (shift_line.start_idx + 1 > path.path.points.size()) {
    RCLCPP_WARN(rclcpp::get_logger(__func__), "index inconsistency.");
    return {};
  }

  if (shift_line.end_idx + 1 > path.shift_length.size()) {
    RCLCPP_WARN(rclcpp::get_logger(__func__), "index inconsistency.");
    return {};
  }

  if (shift_line.end_idx + 1 > path.path.points.size()) {
    RCLCPP_WARN(rclcpp::get_logger(__func__), "index inconsistency.");
    return {};
  }

  const auto start_shift_length = path.shift_length.at(shift_line.start_idx);
  const auto end_shift_length = path.shift_length.at(shift_line.end_idx);
  const auto relative_shift_length = end_shift_length - start_shift_length;

  // If shift length is shorter than the threshold, it does not need to turn on blinkers
  if (std::fabs(relative_shift_length) < p.turn_signal_shift_length_threshold) {
    return {};
  }

  // If the vehicle does not shift anymore, we turn off the blinker
  if (std::fabs(path.shift_length.at(shift_line.end_idx) - current_shift_length) < THRESHOLD) {
    return {};
  }

  const auto get_command = [](const auto & shift_length) {
    return shift_length > 0.0 ? TurnIndicatorsCommand::ENABLE_LEFT
                              : TurnIndicatorsCommand::ENABLE_RIGHT;
  };

  const auto signal_prepare_distance =
    std::max(ego_speed * p.turn_signal_search_time, p.turn_signal_minimum_search_distance);
  const auto ego_front_to_shift_start =
    calcSignedArcLength(path.path.points, ego_pose.position, shift_line.start_idx) -
    p.vehicle_info.max_longitudinal_offset_m;

  if (signal_prepare_distance < ego_front_to_shift_start) {
    return {};
  }

  const auto blinker_start_pose = path.path.points.at(shift_line.start_idx).point.pose;
  const auto blinker_end_pose = path.path.points.at(shift_line.end_idx).point.pose;
  const auto get_start_pose = [&](const auto & ego_to_shift_start) {
    return ego_to_shift_start ? ego_pose : blinker_start_pose;
  };

  TurnSignalInfo turn_signal_info{};
  turn_signal_info.desired_start_point = get_start_pose(ego_front_to_shift_start);
  turn_signal_info.desired_end_point = blinker_end_pose;
  turn_signal_info.required_start_point = blinker_start_pose;
  turn_signal_info.required_end_point = blinker_end_pose;
  turn_signal_info.turn_signal.command = get_command(relative_shift_length);

  if (!p.turn_signal_on_swerving) {
    return turn_signal_info;
  }

  lanelet::ConstLanelet lanelet;
  if (!rh->getClosestLaneletWithinRoute(shift_line.end, &lanelet)) {
    return {};
  }

  const auto left_lanelets = rh->getAllLeftSharedLinestringLanelets(lanelet, true, true);
  const auto right_lanelets = rh->getAllRightSharedLinestringLanelets(lanelet, true, true);

  if (!existShiftSideLane(
        start_shift_length, end_shift_length, left_lanelets.empty(), right_lanelets.empty())) {
    return {};
  }

  if (!straddleRoadBound(path, shift_line, data.current_lanelets, p.vehicle_info)) {
    return {};
  }

  return turn_signal_info;
}
}  // namespace behavior_path_planner::utils::avoidance
