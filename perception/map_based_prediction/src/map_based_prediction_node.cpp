// Copyright 2021 Tier IV, Inc.
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

#include "map_based_prediction/map_based_prediction_node.hpp"

#include <interpolation/linear_interpolation.hpp>
#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <motion_utils/resample/resample.hpp>
#include <motion_utils/trajectory/trajectory.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/math/constants.hpp>
#include <tier4_autoware_utils/math/normalization.hpp>
#include <tier4_autoware_utils/math/unit_conversion.hpp>
#include <tier4_autoware_utils/ros/uuid_helper.hpp>

#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <tf2/utils.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <glog/logging.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>

namespace map_based_prediction
{

namespace
{
/**
 * @brief First order Low pass filtering
 *
 * @param prev_y previous filtered value
 * @param prev_x previous input value
 * @param x current input value
 * @param cutoff_freq  cutoff frequency in Hz not rad/s (1/s)
 * @param sampling_time  sampling time of discrete system (s)
 *
 * @return double current filtered value
 */
double FirstOrderLowpassFilter(
  const double prev_y, const double prev_x, const double x, const double sampling_time = 0.1,
  const double cutoff_freq = 0.1)
{
  // Eq:  yn = a yn-1 + b (xn-1 + xn)
  const double wt = 2.0 * M_PI * cutoff_freq * sampling_time;
  const double a = (2.0 - wt) / (2.0 + wt);
  const double b = wt / (2.0 + wt);

  return a * prev_y + b * (prev_x + x);
}

/**
 * @brief calc lateral offset from pose to linestring
 *
 * @param boundary_line 2d line strings
 * @param search_pose search point
 * @return double
 */
double calcAbsLateralOffset(
  const lanelet::ConstLineString2d & boundary_line, const geometry_msgs::msg::Pose & search_pose)
{
  std::vector<geometry_msgs::msg::Point> boundary_path(boundary_line.size());
  for (size_t i = 0; i < boundary_path.size(); ++i) {
    const double x = boundary_line[i].x();
    const double y = boundary_line[i].y();
    boundary_path[i] = tier4_autoware_utils::createPoint(x, y, 0.0);
  }

  return std::fabs(motion_utils::calcLateralOffset(boundary_path, search_pose.position));
}

/**
 * @brief init lateral kinematics struct
 *
 * @param lanelet closest lanelet
 * @param pose search pose
 * @return lateral kinematics data struct
 */
LateralKinematicsToLanelet initLateralKinematics(
  const lanelet::ConstLanelet & lanelet, geometry_msgs::msg::Pose pose)
{
  LateralKinematicsToLanelet lateral_kinematics;

  const lanelet::ConstLineString2d left_bound = lanelet.leftBound2d();
  const lanelet::ConstLineString2d right_bound = lanelet.rightBound2d();
  const double left_dist = calcAbsLateralOffset(left_bound, pose);
  const double right_dist = calcAbsLateralOffset(right_bound, pose);

  // calc boundary distance
  lateral_kinematics.dist_from_left_boundary = left_dist;
  lateral_kinematics.dist_from_right_boundary = right_dist;
  // velocities are not init in the first step
  lateral_kinematics.left_lateral_velocity = 0;
  lateral_kinematics.right_lateral_velocity = 0;
  lateral_kinematics.filtered_left_lateral_velocity = 0;
  lateral_kinematics.filtered_right_lateral_velocity = 0;
  return lateral_kinematics;
}

/**
 * @brief calc lateral velocity and filtered velocity of object in a lanelet
 *
 * @param prev_lateral_kinematics previous lateral lanelet kinematics
 * @param current_lateral_kinematics current lateral lanelet kinematics
 * @param dt sampling time [s]
 */
void calcLateralKinematics(
  const LateralKinematicsToLanelet & prev_lateral_kinematics,
  LateralKinematicsToLanelet & current_lateral_kinematics, const double dt, const double cutoff)
{
  // calc velocity via backward difference
  current_lateral_kinematics.left_lateral_velocity =
    (current_lateral_kinematics.dist_from_left_boundary -
     prev_lateral_kinematics.dist_from_left_boundary) /
    dt;
  current_lateral_kinematics.right_lateral_velocity =
    (current_lateral_kinematics.dist_from_right_boundary -
     prev_lateral_kinematics.dist_from_right_boundary) /
    dt;

  // low pass filtering left velocity: default cut_off is 0.6 Hz
  current_lateral_kinematics.filtered_left_lateral_velocity = FirstOrderLowpassFilter(
    prev_lateral_kinematics.filtered_left_lateral_velocity,
    prev_lateral_kinematics.left_lateral_velocity, current_lateral_kinematics.left_lateral_velocity,
    dt, cutoff);
  current_lateral_kinematics.filtered_right_lateral_velocity = FirstOrderLowpassFilter(
    prev_lateral_kinematics.filtered_right_lateral_velocity,
    prev_lateral_kinematics.right_lateral_velocity,
    current_lateral_kinematics.right_lateral_velocity, dt, cutoff);
}

/**
 * @brief look for matching lanelet between current/previous object state and calculate velocity
 *
 * @param prev_obj previous ObjectData
 * @param current_obj current ObjectData to be updated
 * @param routing_graph_ptr_ routing graph pointer
 */
void updateLateralKinematicsVector(
  const ObjectData & prev_obj, ObjectData & current_obj,
  const lanelet::routing::RoutingGraphPtr routing_graph_ptr_, const double lowpass_cutoff)
{
  const double dt = (current_obj.header.stamp.sec - prev_obj.header.stamp.sec) +
                    (current_obj.header.stamp.nanosec - prev_obj.header.stamp.nanosec) * 1e-9;
  if (dt < 1e-6) {
    return;  // do not update
  }

  // look for matching lanelet between current and previous kinematics
  for (auto & current_set : current_obj.lateral_kinematics_set) {
    const auto & current_lane = current_set.first;
    auto & current_lateral_kinematics = current_set.second;

    // 1. has same lanelet
    if (prev_obj.lateral_kinematics_set.count(current_lane) != 0) {
      const auto & prev_lateral_kinematics = prev_obj.lateral_kinematics_set.at(current_lane);
      calcLateralKinematics(
        prev_lateral_kinematics, current_lateral_kinematics, dt, lowpass_cutoff);
      break;
    }
    // 2. successive lanelet
    for (auto & prev_set : prev_obj.lateral_kinematics_set) {
      const auto & prev_lane = prev_set.first;
      const auto & prev_lateral_kinematics = prev_set.second;
      const bool successive_lanelet =
        routing_graph_ptr_->routingRelation(prev_lane, current_lane) ==
        lanelet::routing::RelationType::Successor;
      if (successive_lanelet) {  // lanelet can be connected
        calcLateralKinematics(
          prev_lateral_kinematics, current_lateral_kinematics, dt,
          lowpass_cutoff);  // calc velocity
        break;
      }
    }
  }
}

/**
 * @brief calc absolute normalized yaw difference between lanelet and object
 *
 * @param object
 * @param lanelet
 * @return double
 */
double calcAbsYawDiffBetweenLaneletAndObject(
  const TrackedObject & object, const lanelet::ConstLanelet & lanelet)
{
  const double object_yaw = tf2::getYaw(object.kinematics.pose_with_covariance.pose.orientation);
  const double lane_yaw =
    lanelet::utils::getLaneletAngle(lanelet, object.kinematics.pose_with_covariance.pose.position);
  const double delta_yaw = object_yaw - lane_yaw;
  const double normalized_delta_yaw = tier4_autoware_utils::normalizeRadian(delta_yaw);
  const double abs_norm_delta = std::fabs(normalized_delta_yaw);
  return abs_norm_delta;
}

/**
 * @brief Get the Right LineSharing Lanelets object
 *
 * @param current_lanelet
 * @param lanelet_map_ptr
 * @return lanelet::ConstLanelets
 */
lanelet::ConstLanelets getRightLineSharingLanelets(
  const lanelet::ConstLanelet & current_lanelet, const lanelet::LaneletMapPtr & lanelet_map_ptr)
{
  lanelet::ConstLanelets
    output_lanelets;  // create an empty container of type lanelet::ConstLanelets

  // step1: look for lane sharing current right bound
  lanelet::Lanelets right_lane_candidates =
    lanelet_map_ptr->laneletLayer.findUsages(current_lanelet.rightBound());
  for (auto & candidate : right_lane_candidates) {
    // exclude self lanelet
    if (candidate == current_lanelet) continue;
    // if candidate has linestring as left bound, assign it to output
    if (candidate.leftBound() == current_lanelet.rightBound()) {
      output_lanelets.push_back(candidate);
    }
  }
  return output_lanelets;  // return empty
}

/**
 * @brief Get the Left LineSharing Lanelets object
 *
 * @param current_lanelet
 * @param lanelet_map_ptr
 * @return lanelet::ConstLanelets
 */
lanelet::ConstLanelets getLeftLineSharingLanelets(
  const lanelet::ConstLanelet & current_lanelet, const lanelet::LaneletMapPtr & lanelet_map_ptr)
{
  lanelet::ConstLanelets
    output_lanelets;  // create an empty container of type lanelet::ConstLanelets

  // step1: look for lane sharing current left bound
  lanelet::Lanelets left_lane_candidates =
    lanelet_map_ptr->laneletLayer.findUsages(current_lanelet.leftBound());
  for (auto & candidate : left_lane_candidates) {
    // exclude self lanelet
    if (candidate == current_lanelet) continue;
    // if candidate has linestring as right bound, assign it to output
    if (candidate.rightBound() == current_lanelet.leftBound()) {
      output_lanelets.push_back(candidate);
    }
  }
  return output_lanelets;  // return empty
}

/**
 * @brief Check if the lanelet is isolated in routing graph
 * @param current_lanelet
 * @param lanelet_map_ptr
 */
bool isIsolatedLanelet(
  const lanelet::ConstLanelet & lanelet, lanelet::routing::RoutingGraphPtr & graph)
{
  const auto & following_lanelets = graph->following(lanelet);
  const auto & left_lanelets = graph->lefts(lanelet);
  const auto & right_lanelets = graph->rights(lanelet);
  return left_lanelets.empty() && right_lanelets.empty() && following_lanelets.empty();
}

/**
 * @brief Get the Possible Paths For Isolated Lanelet object
 * @param lanelet
 * @return lanelet::routing::LaneletPaths
 */
lanelet::routing::LaneletPaths getPossiblePathsForIsolatedLanelet(
  const lanelet::ConstLanelet & lanelet)
{
  lanelet::ConstLanelets possible_lanelets;
  possible_lanelets.push_back(lanelet);
  lanelet::routing::LaneletPaths possible_paths;
  // need to initialize path with constant lanelets
  lanelet::routing::LaneletPath possible_path(possible_lanelets);
  possible_paths.push_back(possible_path);
  return possible_paths;
}

/**
 * @brief validate isolated lanelet length has enough length for prediction
 * @param lanelet
 * @param object: object information for calc length threshold
 * @param prediction_time: time horizon[s] for calc length threshold
 * @return bool
 */
bool validateIsolatedLaneletLength(
  const lanelet::ConstLanelet & lanelet, const TrackedObject & object, const double prediction_time)
{
  // get closest center line point to object
  const auto & center_line = lanelet.centerline2d();
  const auto & obj_pos = object.kinematics.pose_with_covariance.pose.position;
  const lanelet::BasicPoint2d obj_point(obj_pos.x, obj_pos.y);
  // get end point of the center line
  const auto & end_point = center_line.back();
  // calc approx distance between closest point and end point
  const double approx_distance = lanelet::geometry::distance2d(obj_point, end_point);
  // calc min length for prediction
  const double abs_speed = std::hypot(
    object.kinematics.twist_with_covariance.twist.linear.x,
    object.kinematics.twist_with_covariance.twist.linear.y);
  const double min_length = abs_speed * prediction_time;
  return approx_distance > min_length;
}

lanelet::ConstLanelets getLanelets(const map_based_prediction::LaneletsData & data)
{
  lanelet::ConstLanelets lanelets;
  for (const auto & lanelet_data : data) {
    lanelets.push_back(lanelet_data.lanelet);
  }

  return lanelets;
}

CrosswalkEdgePoints getCrosswalkEdgePoints(const lanelet::ConstLanelet & crosswalk)
{
  const Eigen::Vector2d r_p_front = crosswalk.rightBound().front().basicPoint2d();
  const Eigen::Vector2d l_p_front = crosswalk.leftBound().front().basicPoint2d();
  const Eigen::Vector2d front_center_point = (r_p_front + l_p_front) / 2.0;

  const Eigen::Vector2d r_p_back = crosswalk.rightBound().back().basicPoint2d();
  const Eigen::Vector2d l_p_back = crosswalk.leftBound().back().basicPoint2d();
  const Eigen::Vector2d back_center_point = (r_p_back + l_p_back) / 2.0;

  return CrosswalkEdgePoints{front_center_point, r_p_front, l_p_front,
                             back_center_point,  r_p_back,  l_p_back};
}

bool withinLanelet(
  const TrackedObject & object, const lanelet::ConstLanelet & lanelet,
  const bool use_yaw_information = false, const float yaw_threshold = 0.6)
{
  using Point = boost::geometry::model::d2::point_xy<double>;

  const auto & obj_pos = object.kinematics.pose_with_covariance.pose.position;
  const Point p_object{obj_pos.x, obj_pos.y};

  auto polygon = lanelet.polygon2d().basicPolygon();
  boost::geometry::correct(polygon);
  bool with_in_polygon = boost::geometry::within(p_object, polygon);

  if (!use_yaw_information) return with_in_polygon;

  // use yaw angle to compare
  const double abs_yaw_diff = calcAbsYawDiffBetweenLaneletAndObject(object, lanelet);
  if (abs_yaw_diff < yaw_threshold) return with_in_polygon;

  return false;
}

bool withinRoadLanelet(
  const TrackedObject & object, const lanelet::LaneletMapPtr & lanelet_map_ptr,
  const bool use_yaw_information = false)
{
  using Point = boost::geometry::model::d2::point_xy<double>;

  const auto & obj_pos = object.kinematics.pose_with_covariance.pose.position;
  const Point p_object{obj_pos.x, obj_pos.y};

  lanelet::BasicPoint2d search_point(obj_pos.x, obj_pos.y);
  // nearest lanelet
  constexpr double search_radius = 10.0;  // [m]
  const auto surrounding_lanelets =
    lanelet::geometry::findNearest(lanelet_map_ptr->laneletLayer, search_point, search_radius);

  for (const auto & lanelet : surrounding_lanelets) {
    if (lanelet.second.hasAttribute(lanelet::AttributeName::Subtype)) {
      lanelet::Attribute attr = lanelet.second.attribute(lanelet::AttributeName::Subtype);
      if (
        attr.value() == lanelet::AttributeValueString::Crosswalk ||
        attr.value() == lanelet::AttributeValueString::Walkway) {
        continue;
      }
    }

    if (withinLanelet(object, lanelet.second, use_yaw_information)) {
      return true;
    }
  }

  return false;
}

boost::optional<CrosswalkEdgePoints> isReachableCrosswalkEdgePoints(
  const TrackedObject & object, const CrosswalkEdgePoints & edge_points,
  const lanelet::LaneletMapPtr & lanelet_map_ptr, const double time_horizon,
  const double min_object_vel)
{
  using Point = boost::geometry::model::d2::point_xy<double>;
  using Line = boost::geometry::model::linestring<Point>;

  const auto & obj_pos = object.kinematics.pose_with_covariance.pose.position;
  const auto & obj_vel = object.kinematics.twist_with_covariance.twist.linear;
  const auto yaw = tier4_autoware_utils::getRPY(object.kinematics.pose_with_covariance.pose).z;

  const auto & p1 = edge_points.front_center_point;
  const auto & p2 = edge_points.back_center_point;

  CrosswalkEdgePoints ret{p1, {}, {}, p2, {}, {}};
  auto distance_pedestrian_to_p1 = std::hypot(p1.x() - obj_pos.x, p1.y() - obj_pos.y);
  auto distance_pedestrian_to_p2 = std::hypot(p2.x() - obj_pos.x, p2.y() - obj_pos.y);

  if (distance_pedestrian_to_p2 < distance_pedestrian_to_p1) {
    ret.swap();
    std::swap(distance_pedestrian_to_p1, distance_pedestrian_to_p2);
  }

  constexpr double stop_velocity_th = 0.14;  // [m/s]
  const auto estimated_velocity = std::hypot(obj_vel.x, obj_vel.y);
  const auto is_stop_object = estimated_velocity < stop_velocity_th;
  const auto velocity = std::max(min_object_vel, estimated_velocity);

  lanelet::BasicPoint2d search_point(obj_pos.x, obj_pos.y);
  // nearest lanelet
  const auto surrounding_lanelets = lanelet::geometry::findNearest(
    lanelet_map_ptr->laneletLayer, search_point, time_horizon * velocity);

  bool first_intersect_load = false;
  bool second_intersect_load = false;
  std::vector<Point> intersects_first;
  std::vector<Point> intersects_second;
  for (const auto & lanelet : surrounding_lanelets) {
    if (withinLanelet(object, lanelet.second)) {
      return {};
    }

    lanelet::Attribute attr = lanelet.second.attribute(lanelet::AttributeName::Subtype);
    if (attr.value() != "road") {
      continue;
    }

    {
      const Line object_to_entry_point{
        {obj_pos.x, obj_pos.y}, {ret.front_center_point.x(), ret.front_center_point.y()}};
      std::vector<Point> tmp_intersects;
      boost::geometry::intersection(
        object_to_entry_point, lanelet.second.polygon2d().basicPolygon(), tmp_intersects);
      for (const auto & p : tmp_intersects) {
        intersects_first.push_back(p);
      }
    }

    {
      const Line object_to_entry_point{
        {obj_pos.x, obj_pos.y}, {ret.back_center_point.x(), ret.back_center_point.y()}};
      std::vector<Point> tmp_intersects;
      boost::geometry::intersection(
        object_to_entry_point, lanelet.second.polygon2d().basicPolygon(), tmp_intersects);
      for (const auto & p : tmp_intersects) {
        intersects_second.push_back(p);
      }
    }
  }

  if (1 < intersects_first.size()) {
    first_intersect_load = true;
  }

  if (1 < intersects_second.size()) {
    second_intersect_load = true;
  }

  if (first_intersect_load && second_intersect_load) {
    return {};
  }

  if (first_intersect_load && !second_intersect_load) {
    ret.swap();
  }

  const Eigen::Vector2d pedestrian_to_crosswalk(
    (ret.front_center_point.x() + ret.back_center_point.x()) / 2.0 - obj_pos.x,
    (ret.front_center_point.y() + ret.back_center_point.y()) / 2.0 - obj_pos.y);
  const Eigen::Vector2d pedestrian_heading_direction(
    obj_vel.x * std::cos(yaw), obj_vel.x * std::sin(yaw));
  const auto reachable =
    std::min(distance_pedestrian_to_p1, distance_pedestrian_to_p2) < velocity * time_horizon;
  const auto heading_for_crosswalk =
    pedestrian_to_crosswalk.dot(pedestrian_heading_direction) > 0.0;

  if ((reachable && heading_for_crosswalk) || (reachable && is_stop_object)) {
    return ret;
  }

  return {};
}

bool hasPotentialToReach(
  const TrackedObject & object, const Eigen::Vector2d & center_point,
  const Eigen::Vector2d & right_point, const Eigen::Vector2d & left_point,
  const double time_horizon, const double min_object_vel,
  const double max_crosswalk_user_delta_yaw_threshold_for_lanelet)
{
  const auto & obj_pos = object.kinematics.pose_with_covariance.pose.position;
  const auto & obj_vel = object.kinematics.twist_with_covariance.twist.linear;
  const auto yaw = tier4_autoware_utils::getRPY(object.kinematics.pose_with_covariance.pose).z;

  constexpr double stop_velocity_th = 0.14;  // [m/s]
  const auto estimated_velocity = std::hypot(obj_vel.x, obj_vel.y);
  const auto is_stop_object = estimated_velocity < stop_velocity_th;
  const auto velocity = std::max(min_object_vel, estimated_velocity);

  const double pedestrian_to_crosswalk_center_direction =
    std::atan2(center_point.y() - obj_pos.y, center_point.x() - obj_pos.x);

  const auto
    [pedestrian_to_crosswalk_right_rel_direction, pedestrian_to_crosswalk_left_rel_direction] =
      [&]() {
        const double pedestrian_to_crosswalk_right_direction =
          std::atan2(right_point.y() - obj_pos.y, right_point.x() - obj_pos.x);
        const double pedestrian_to_crosswalk_left_direction =
          std::atan2(left_point.y() - obj_pos.y, left_point.x() - obj_pos.x);
        return std::make_pair(
          tier4_autoware_utils::normalizeRadian(
            pedestrian_to_crosswalk_right_direction - pedestrian_to_crosswalk_center_direction),
          tier4_autoware_utils::normalizeRadian(
            pedestrian_to_crosswalk_left_direction - pedestrian_to_crosswalk_center_direction));
      }();

  const double pedestrian_heading_rel_direction = [&]() {
    const double pedestrian_heading_direction =
      std::atan2(obj_vel.x * std::sin(yaw), obj_vel.x * std::cos(yaw));
    return tier4_autoware_utils::normalizeRadian(
      pedestrian_heading_direction - pedestrian_to_crosswalk_center_direction);
  }();

  const double pedestrian_to_crosswalk_min_rel_direction = std::min(
    pedestrian_to_crosswalk_right_rel_direction, pedestrian_to_crosswalk_left_rel_direction);
  const double pedestrian_to_crosswalk_max_rel_direction = std::max(
    pedestrian_to_crosswalk_right_rel_direction, pedestrian_to_crosswalk_left_rel_direction);
  const double pedestrian_vel_angle_against_crosswalk = [&]() {
    if (pedestrian_heading_rel_direction < pedestrian_to_crosswalk_min_rel_direction) {
      return pedestrian_to_crosswalk_min_rel_direction - pedestrian_heading_rel_direction;
    }
    if (pedestrian_to_crosswalk_max_rel_direction < pedestrian_heading_rel_direction) {
      return pedestrian_to_crosswalk_max_rel_direction - pedestrian_heading_rel_direction;
    }
    return 0.0;
  }();
  const auto heading_for_crosswalk = std::abs(pedestrian_vel_angle_against_crosswalk) <
                                     max_crosswalk_user_delta_yaw_threshold_for_lanelet;
  const auto reachable = std::hypot(center_point.x() - obj_pos.x, center_point.y() - obj_pos.y) <
                         velocity * time_horizon;

  if (reachable && (heading_for_crosswalk || is_stop_object)) {
    return true;
  }

  return false;
}

/**
 * @brief change label for prediction
 *
 * @param label
 * @return ObjectClassification::_label_type
 */
ObjectClassification::_label_type changeLabelForPrediction(
  const ObjectClassification::_label_type & label, const TrackedObject & object,
  const lanelet::LaneletMapPtr & lanelet_map_ptr_)
{
  // for car like vehicle do not change labels
  switch (label) {
    case ObjectClassification::CAR:
    case ObjectClassification::BUS:
    case ObjectClassification::TRUCK:
    case ObjectClassification::TRAILER:
    case ObjectClassification::UNKNOWN:
      return label;

    case ObjectClassification::MOTORCYCLE:
    case ObjectClassification::BICYCLE: {  // if object is within road lanelet and satisfies yaw
                                           // constraints
      const bool within_road_lanelet = withinRoadLanelet(object, lanelet_map_ptr_, true);
      const float high_speed_threshold =
        tier4_autoware_utils::kmph2mps(25.0);  // High speed bicycle 25 km/h
      // calc abs speed from x and y velocity
      const double abs_speed = std::hypot(
        object.kinematics.twist_with_covariance.twist.linear.x,
        object.kinematics.twist_with_covariance.twist.linear.y);
      const bool high_speed_object = abs_speed > high_speed_threshold;

      // if the object is within lanelet, do the same estimation with vehicle
      if (within_road_lanelet) return ObjectClassification::MOTORCYCLE;
      // high speed object outside road lanelet will move like unknown object
      // return ObjectClassification::UNKNOWN; // temporary disabled
      if (high_speed_object) return label;  // Do nothing for now
      return ObjectClassification::BICYCLE;
    }

    case ObjectClassification::PEDESTRIAN: {
      const bool within_road_lanelet = withinRoadLanelet(object, lanelet_map_ptr_, true);
      const float max_velocity_for_human_mps =
        tier4_autoware_utils::kmph2mps(25.0);  // Max human being motion speed is 25km/h
      const double abs_speed = std::hypot(
        object.kinematics.twist_with_covariance.twist.linear.x,
        object.kinematics.twist_with_covariance.twist.linear.y);
      const bool high_speed_object = abs_speed > max_velocity_for_human_mps;
      // fast, human-like object: like segway
      if (within_road_lanelet && high_speed_object) return label;  // currently do nothing
      // return ObjectClassification::MOTORCYCLE;
      if (high_speed_object) return label;  // currently do nothing
      // fast human outside road lanelet will move like unknown object
      // return ObjectClassification::UNKNOWN;
      return label;
    }

    default:
      return label;
  }
}

StringStamped createStringStamped(const rclcpp::Time & now, const double data)
{
  StringStamped msg;
  msg.stamp = now;
  msg.data = std::to_string(data);
  return msg;
}

// NOTE: These two functions are copied from the route_handler package.
lanelet::Lanelets getRightOppositeLanelets(
  const std::shared_ptr<lanelet::LaneletMap> & lanelet_map_ptr,
  const lanelet::ConstLanelet & lanelet)
{
  const auto opposite_candidate_lanelets =
    lanelet_map_ptr->laneletLayer.findUsages(lanelet.rightBound().invert());

  lanelet::Lanelets opposite_lanelets;
  for (const auto & candidate_lanelet : opposite_candidate_lanelets) {
    if (candidate_lanelet.leftBound().id() == lanelet.rightBound().id()) {
      continue;
    }

    opposite_lanelets.push_back(candidate_lanelet);
  }

  return opposite_lanelets;
}

lanelet::Lanelets getLeftOppositeLanelets(
  const std::shared_ptr<lanelet::LaneletMap> & lanelet_map_ptr,
  const lanelet::ConstLanelet & lanelet)
{
  const auto opposite_candidate_lanelets =
    lanelet_map_ptr->laneletLayer.findUsages(lanelet.leftBound().invert());

  lanelet::Lanelets opposite_lanelets;
  for (const auto & candidate_lanelet : opposite_candidate_lanelets) {
    if (candidate_lanelet.rightBound().id() == lanelet.leftBound().id()) {
      continue;
    }

    opposite_lanelets.push_back(candidate_lanelet);
  }

  return opposite_lanelets;
}

void replaceObjectYawWithLaneletsYaw(
  const LaneletsData & current_lanelets, TrackedObject & transformed_object)
{
  // return if no lanelet is found
  if (current_lanelets.empty()) return;
  auto & pose_with_cov = transformed_object.kinematics.pose_with_covariance;
  // for each lanelet, calc lanelet angle and calculate mean angle
  double sum_x = 0.0;
  double sum_y = 0.0;
  for (const auto & current_lanelet : current_lanelets) {
    const auto lanelet_angle =
      lanelet::utils::getLaneletAngle(current_lanelet.lanelet, pose_with_cov.pose.position);
    sum_x += std::cos(lanelet_angle);
    sum_y += std::sin(lanelet_angle);
  }
  const double mean_yaw_angle = std::atan2(sum_y, sum_x);
  double roll, pitch, yaw;
  tf2::Quaternion original_quaternion;
  tf2::fromMsg(pose_with_cov.pose.orientation, original_quaternion);
  tf2::Matrix3x3(original_quaternion).getRPY(roll, pitch, yaw);
  tf2::Quaternion filtered_quaternion;
  filtered_quaternion.setRPY(roll, pitch, mean_yaw_angle);
  pose_with_cov.pose.orientation = tf2::toMsg(filtered_quaternion);
}

}  // namespace

MapBasedPredictionNode::MapBasedPredictionNode(const rclcpp::NodeOptions & node_options)
: Node("map_based_prediction", node_options), debug_accumulated_time_(0.0)
{
  google::InitGoogleLogging("map_based_prediction_node");
  google::InstallFailureSignalHandler();
  enable_delay_compensation_ = declare_parameter<bool>("enable_delay_compensation");
  prediction_time_horizon_ = declare_parameter<double>("prediction_time_horizon");
  lateral_control_time_horizon_ =
    declare_parameter<double>("lateral_control_time_horizon");  // [s] for lateral control point
  prediction_sampling_time_interval_ = declare_parameter<double>("prediction_sampling_delta_time");
  min_velocity_for_map_based_prediction_ =
    declare_parameter<double>("min_velocity_for_map_based_prediction");
  min_crosswalk_user_velocity_ = declare_parameter<double>("min_crosswalk_user_velocity");
  max_crosswalk_user_delta_yaw_threshold_for_lanelet_ =
    declare_parameter<double>("max_crosswalk_user_delta_yaw_threshold_for_lanelet");
  dist_threshold_for_searching_lanelet_ =
    declare_parameter<double>("dist_threshold_for_searching_lanelet");
  delta_yaw_threshold_for_searching_lanelet_ =
    declare_parameter<double>("delta_yaw_threshold_for_searching_lanelet");
  sigma_lateral_offset_ = declare_parameter<double>("sigma_lateral_offset");
  sigma_yaw_angle_deg_ = declare_parameter<double>("sigma_yaw_angle_deg");
  object_buffer_time_length_ = declare_parameter<double>("object_buffer_time_length");
  history_time_length_ = declare_parameter<double>("history_time_length");

  check_lateral_acceleration_constraints_ =
    declare_parameter<bool>("check_lateral_acceleration_constraints");
  max_lateral_accel_ = declare_parameter<double>("max_lateral_accel");
  min_acceleration_before_curve_ = declare_parameter<double>("min_acceleration_before_curve");

  {  // lane change detection
    lane_change_detection_method_ = declare_parameter<std::string>("lane_change_detection.method");

    // lane change detection by time_to_change_lane
    dist_threshold_to_bound_ = declare_parameter<double>(
      "lane_change_detection.time_to_change_lane.dist_threshold_for_lane_change_detection");  // 1m
    time_threshold_to_bound_ = declare_parameter<double>(
      "lane_change_detection.time_to_change_lane.time_threshold_for_lane_change_detection");
    cutoff_freq_of_velocity_lpf_ = declare_parameter<double>(
      "lane_change_detection.time_to_change_lane.cutoff_freq_of_velocity_for_lane_change_"
      "detection");

    // lane change detection by lat_diff_distance
    dist_ratio_threshold_to_left_bound_ = declare_parameter<double>(
      "lane_change_detection.lat_diff_distance.dist_ratio_threshold_to_left_bound");
    dist_ratio_threshold_to_right_bound_ = declare_parameter<double>(
      "lane_change_detection.lat_diff_distance.dist_ratio_threshold_to_right_bound");
    diff_dist_threshold_to_left_bound_ = declare_parameter<double>(
      "lane_change_detection.lat_diff_distance.diff_dist_threshold_to_left_bound");
    diff_dist_threshold_to_right_bound_ = declare_parameter<double>(
      "lane_change_detection.lat_diff_distance.diff_dist_threshold_to_right_bound");

    num_continuous_state_transition_ =
      declare_parameter<int>("lane_change_detection.num_continuous_state_transition");

    consider_only_routable_neighbours_ =
      declare_parameter<bool>("lane_change_detection.consider_only_routable_neighbours");
  }
  reference_path_resolution_ = declare_parameter<double>("reference_path_resolution");
  /* prediction path will disabled when the estimated path length exceeds lanelet length. This
   * parameter control the estimated path length = vx * th * (rate)  */
  prediction_time_horizon_rate_for_validate_lane_length_ =
    declare_parameter<double>("prediction_time_horizon_rate_for_validate_shoulder_lane_length");

  path_generator_ = std::make_shared<PathGenerator>(
    prediction_time_horizon_, lateral_control_time_horizon_, prediction_sampling_time_interval_,
    min_crosswalk_user_velocity_);

  sub_objects_ = this->create_subscription<TrackedObjects>(
    "~/input/objects", 1,
    std::bind(&MapBasedPredictionNode::objectsCallback, this, std::placeholders::_1));
  sub_map_ = this->create_subscription<HADMapBin>(
    "/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&MapBasedPredictionNode::mapCallback, this, std::placeholders::_1));

  pub_objects_ = this->create_publisher<PredictedObjects>("~/output/objects", rclcpp::QoS{1});
  pub_debug_markers_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("maneuver", rclcpp::QoS{1});
  pub_calculation_time_ = create_publisher<StringStamped>("~/debug/calculation_time", 1);

  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&MapBasedPredictionNode::onParam, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult MapBasedPredictionNode::onParam(
  const std::vector<rclcpp::Parameter> & parameters)
{
  using tier4_autoware_utils::updateParam;

  updateParam(parameters, "max_lateral_accel", max_lateral_accel_);
  updateParam(parameters, "min_acceleration_before_curve", min_acceleration_before_curve_);
  updateParam(
    parameters, "check_lateral_acceleration_constraints", check_lateral_acceleration_constraints_);

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

PredictedObjectKinematics MapBasedPredictionNode::convertToPredictedKinematics(
  const TrackedObjectKinematics & tracked_object)
{
  PredictedObjectKinematics output;
  output.initial_pose_with_covariance = tracked_object.pose_with_covariance;
  output.initial_twist_with_covariance = tracked_object.twist_with_covariance;
  output.initial_acceleration_with_covariance = tracked_object.acceleration_with_covariance;
  return output;
}

PredictedObject MapBasedPredictionNode::convertToPredictedObject(
  const TrackedObject & tracked_object)
{
  PredictedObject predicted_object;
  predicted_object.kinematics = convertToPredictedKinematics(tracked_object.kinematics);
  predicted_object.classification = tracked_object.classification;
  predicted_object.object_id = tracked_object.object_id;
  predicted_object.shape = tracked_object.shape;
  predicted_object.existence_probability = tracked_object.existence_probability;

  return predicted_object;
}

void MapBasedPredictionNode::mapCallback(const HADMapBin::ConstSharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "[Map Based Prediction]: Start loading lanelet");
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(
    *msg, lanelet_map_ptr_, &traffic_rules_ptr_, &routing_graph_ptr_);
  RCLCPP_INFO(get_logger(), "[Map Based Prediction]: Map is loaded");

  const auto all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_ptr_);
  const auto crosswalks = lanelet::utils::query::crosswalkLanelets(all_lanelets);
  const auto walkways = lanelet::utils::query::walkwayLanelets(all_lanelets);
  crosswalks_.insert(crosswalks_.end(), crosswalks.begin(), crosswalks.end());
  crosswalks_.insert(crosswalks_.end(), walkways.begin(), walkways.end());
}

void MapBasedPredictionNode::objectsCallback(const TrackedObjects::ConstSharedPtr in_objects)
{
  stop_watch_.tic();
  // Guard for map pointer and frame transformation
  if (!lanelet_map_ptr_) {
    return;
  }

  auto world2map_transform = transform_listener_.getTransform(
    "map",                        // target
    in_objects->header.frame_id,  // src
    in_objects->header.stamp, rclcpp::Duration::from_seconds(0.1));
  auto map2world_transform = transform_listener_.getTransform(
    in_objects->header.frame_id,  // target
    "map",                        // src
    in_objects->header.stamp, rclcpp::Duration::from_seconds(0.1));
  auto debug_map2lidar_transform = transform_listener_.getTransform(
    "base_link",  // target
    "map",        // src
    rclcpp::Time(), rclcpp::Duration::from_seconds(0.1));

  if (!world2map_transform || !map2world_transform || !debug_map2lidar_transform) {
    return;
  }

  // Remove old objects information in object history
  const double objects_detected_time = rclcpp::Time(in_objects->header.stamp).seconds();
  removeOldObjectsHistory(objects_detected_time);

  // result output
  PredictedObjects output;
  output.header = in_objects->header;
  output.header.frame_id = "map";

  // result debug
  visualization_msgs::msg::MarkerArray debug_markers;

  for (const auto & object : in_objects->objects) {
    std::string object_id = tier4_autoware_utils::toHexString(object.object_id);
    TrackedObject transformed_object = object;

    // transform object frame if it's based on map frame
    if (in_objects->header.frame_id != "map") {
      geometry_msgs::msg::PoseStamped pose_in_map;
      geometry_msgs::msg::PoseStamped pose_orig;
      pose_orig.pose = object.kinematics.pose_with_covariance.pose;
      tf2::doTransform(pose_orig, pose_in_map, *world2map_transform);
      transformed_object.kinematics.pose_with_covariance.pose = pose_in_map.pose;
    }

    // get tracking label and update it for the prediction
    const auto & label_ = transformed_object.classification.front().label;
    const auto label = changeLabelForPrediction(label_, object, lanelet_map_ptr_);

    switch (label) {
      case ObjectClassification::PEDESTRIAN:
      case ObjectClassification::BICYCLE: {
        const auto predicted_object_crosswalk =
          getPredictedObjectAsCrosswalkUser(transformed_object);
        output.objects.push_back(predicted_object_crosswalk);
        break;
      }
      case ObjectClassification::CAR:
      case ObjectClassification::BUS:
      case ObjectClassification::TRAILER:
      case ObjectClassification::MOTORCYCLE:
      case ObjectClassification::TRUCK: {
        // Update object yaw and velocity
        updateObjectData(transformed_object);

        // Get Closest Lanelet
        const auto current_lanelets = getCurrentLanelets(transformed_object);

        // Update Objects History
        updateObjectsHistory(output.header, transformed_object, current_lanelets);

        // For off lane obstacles
        if (current_lanelets.empty()) {
          PredictedPath predicted_path =
            path_generator_->generatePathForOffLaneVehicle(transformed_object);
          predicted_path.confidence = 1.0;
          if (predicted_path.path.empty()) break;

          auto predicted_object_vehicle = convertToPredictedObject(transformed_object);
          predicted_object_vehicle.kinematics.predicted_paths.push_back(predicted_path);
          output.objects.push_back(predicted_object_vehicle);
          break;
        }

        // For too-slow vehicle
        const double abs_obj_speed = std::hypot(
          transformed_object.kinematics.twist_with_covariance.twist.linear.x,
          transformed_object.kinematics.twist_with_covariance.twist.linear.y);
        if (std::fabs(abs_obj_speed) < min_velocity_for_map_based_prediction_) {
          PredictedPath predicted_path =
            path_generator_->generatePathForLowSpeedVehicle(transformed_object);
          predicted_path.confidence = 1.0;
          if (predicted_path.path.empty()) break;

          auto predicted_slow_object = convertToPredictedObject(transformed_object);
          predicted_slow_object.kinematics.predicted_paths.push_back(predicted_path);
          output.objects.push_back(predicted_slow_object);
          break;
        }

        // Get Predicted Reference Path for Each Maneuver and current lanelets
        // return: <probability, paths>
        const auto ref_paths =
          getPredictedReferencePath(transformed_object, current_lanelets, objects_detected_time);

        // If predicted reference path is empty, assume this object is out of the lane
        if (ref_paths.empty()) {
          PredictedPath predicted_path =
            path_generator_->generatePathForLowSpeedVehicle(transformed_object);
          predicted_path.confidence = 1.0;
          if (predicted_path.path.empty()) break;

          auto predicted_object_out_of_lane = convertToPredictedObject(transformed_object);
          predicted_object_out_of_lane.kinematics.predicted_paths.push_back(predicted_path);
          output.objects.push_back(predicted_object_out_of_lane);
          break;
        }

        // Get Debug Marker for On Lane Vehicles
        const auto max_prob_path = std::max_element(
          ref_paths.begin(), ref_paths.end(),
          [](const PredictedRefPath & a, const PredictedRefPath & b) {
            return a.probability < b.probability;
          });
        const auto debug_marker =
          getDebugMarker(object, max_prob_path->maneuver, debug_markers.markers.size());
        debug_markers.markers.push_back(debug_marker);

        // Fix object angle if its orientation unreliable (e.g. far object by radar sensor)
        // This prevent bending predicted path
        TrackedObject yaw_fixed_transformed_object = transformed_object;
        if (
          transformed_object.kinematics.orientation_availability ==
          autoware_auto_perception_msgs::msg::TrackedObjectKinematics::UNAVAILABLE) {
          replaceObjectYawWithLaneletsYaw(current_lanelets, yaw_fixed_transformed_object);
        }
        // Generate Predicted Path
        std::vector<PredictedPath> predicted_paths;
        double min_avg_curvature = std::numeric_limits<double>::max();
        PredictedPath path_with_smallest_avg_curvature;

        for (const auto & ref_path : ref_paths) {
          PredictedPath predicted_path = path_generator_->generatePathForOnLaneVehicle(
            yaw_fixed_transformed_object, ref_path.path);
          if (predicted_path.path.empty()) continue;

          if (!check_lateral_acceleration_constraints_) {
            predicted_path.confidence = ref_path.probability;
            predicted_paths.push_back(predicted_path);
            continue;
          }

          // Check lat. acceleration constraints
          const auto trajectory_with_const_velocity =
            toTrajectoryPoints(predicted_path, abs_obj_speed);

          if (isLateralAccelerationConstraintSatisfied(
                trajectory_with_const_velocity, prediction_sampling_time_interval_)) {
            predicted_path.confidence = ref_path.probability;
            predicted_paths.push_back(predicted_path);
            continue;
          }

          // Calculate curvature assuming the trajectory points interval is constant
          // In case all paths are deleted, a copy of the straightest path is kept

          constexpr double curvature_calculation_distance = 2.0;
          constexpr double points_interval = 1.0;
          const size_t idx_dist = static_cast<size_t>(
            std::max(static_cast<int>((curvature_calculation_distance) / points_interval), 1));
          const auto curvature_v =
            calcTrajectoryCurvatureFrom3Points(trajectory_with_const_velocity, idx_dist);
          if (curvature_v.empty()) {
            continue;
          }
          const auto curvature_avg =
            std::accumulate(curvature_v.begin(), curvature_v.end(), 0.0) / curvature_v.size();
          if (curvature_avg < min_avg_curvature) {
            min_avg_curvature = curvature_avg;
            path_with_smallest_avg_curvature = predicted_path;
            path_with_smallest_avg_curvature.confidence = ref_path.probability;
          }
        }

        if (predicted_paths.empty()) predicted_paths.push_back(path_with_smallest_avg_curvature);
        // Normalize Path Confidence and output the predicted object

        float sum_confidence = 0.0;
        for (const auto & predicted_path : predicted_paths) {
          sum_confidence += predicted_path.confidence;
        }
        const float min_sum_confidence_value = 1e-3;
        sum_confidence = std::max(sum_confidence, min_sum_confidence_value);

        auto predicted_object = convertToPredictedObject(transformed_object);

        for (auto & predicted_path : predicted_paths) {
          predicted_path.confidence = predicted_path.confidence / sum_confidence;
          if (predicted_object.kinematics.predicted_paths.size() >= 100) break;
          predicted_object.kinematics.predicted_paths.push_back(predicted_path);
        }
        output.objects.push_back(predicted_object);
        break;
      }
      default: {
        auto predicted_unknown_object = convertToPredictedObject(transformed_object);
        PredictedPath predicted_path =
          path_generator_->generatePathForNonVehicleObject(transformed_object);
        predicted_path.confidence = 1.0;

        predicted_unknown_object.kinematics.predicted_paths.push_back(predicted_path);
        output.objects.push_back(predicted_unknown_object);
        break;
      }
    }
  }

  // Publish Results
  pub_objects_->publish(output);
  pub_debug_markers_->publish(debug_markers);
  const auto calculation_time_msg = createStringStamped(now(), stop_watch_.toc());
  pub_calculation_time_->publish(calculation_time_msg);
}

bool MapBasedPredictionNode::doesPathCrossAnyFence(const PredictedPath & predicted_path)
{
  const lanelet::ConstLineStrings3d & all_fences =
    lanelet::utils::query::getAllFences(lanelet_map_ptr_);
  for (const auto & fence_line : all_fences) {
    if (doesPathCrossFence(predicted_path, fence_line)) {
      return true;
    }
  }
  return false;
}

bool MapBasedPredictionNode::doesPathCrossFence(
  const PredictedPath & predicted_path, const lanelet::ConstLineString3d & fence_line)
{
  // check whether the predicted path cross with fence
  for (size_t i = 0; i < predicted_path.path.size(); ++i) {
    for (size_t j = 0; j < fence_line.size() - 1; ++j) {
      if (isIntersecting(
            predicted_path.path[i].position, predicted_path.path[i + 1].position, fence_line[j],
            fence_line[j + 1])) {
        return true;
      }
    }
  }
  return false;
}

bool MapBasedPredictionNode::isIntersecting(
  const geometry_msgs::msg::Point & point1, const geometry_msgs::msg::Point & point2,
  const lanelet::ConstPoint3d & point3, const lanelet::ConstPoint3d & point4)
{
  const auto p1 = tier4_autoware_utils::createPoint(point1.x, point1.y, 0.0);
  const auto p2 = tier4_autoware_utils::createPoint(point2.x, point2.y, 0.0);
  const auto p3 = tier4_autoware_utils::createPoint(point3.x(), point3.y(), 0.0);
  const auto p4 = tier4_autoware_utils::createPoint(point4.x(), point4.y(), 0.0);
  const auto intersection = tier4_autoware_utils::intersect(p1, p2, p3, p4);
  return intersection.has_value();
}

PredictedObject MapBasedPredictionNode::getPredictedObjectAsCrosswalkUser(
  const TrackedObject & object)
{
  auto predicted_object = convertToPredictedObject(object);
  {
    PredictedPath predicted_path = path_generator_->generatePathForNonVehicleObject(object);
    predicted_path.confidence = 1.0;

    predicted_object.kinematics.predicted_paths.push_back(predicted_path);
  }

  boost::optional<lanelet::ConstLanelet> crossing_crosswalk{boost::none};
  for (const auto & crosswalk : crosswalks_) {
    if (withinLanelet(object, crosswalk)) {
      crossing_crosswalk = crosswalk;
      break;
    }
  }

  // If the object is in the crosswalk, generate path to the crosswalk edge
  if (crossing_crosswalk) {
    const auto edge_points = getCrosswalkEdgePoints(crossing_crosswalk.get());

    if (hasPotentialToReach(
          object, edge_points.front_center_point, edge_points.front_right_point,
          edge_points.front_left_point, std::numeric_limits<double>::max(),
          min_crosswalk_user_velocity_, max_crosswalk_user_delta_yaw_threshold_for_lanelet_)) {
      PredictedPath predicted_path =
        path_generator_->generatePathToTargetPoint(object, edge_points.front_center_point);
      predicted_path.confidence = 1.0;
      predicted_object.kinematics.predicted_paths.push_back(predicted_path);
    }

    if (hasPotentialToReach(
          object, edge_points.back_center_point, edge_points.back_right_point,
          edge_points.back_left_point, std::numeric_limits<double>::max(),
          min_crosswalk_user_velocity_, max_crosswalk_user_delta_yaw_threshold_for_lanelet_)) {
      PredictedPath predicted_path =
        path_generator_->generatePathToTargetPoint(object, edge_points.back_center_point);
      predicted_path.confidence = 1.0;
      predicted_object.kinematics.predicted_paths.push_back(predicted_path);
    }

    // If the object is not crossing the crosswalk, in the road lanelets, try to find the closest
    // crosswalk and generate path to the crosswalk edge
  } else if (withinRoadLanelet(object, lanelet_map_ptr_)) {
    lanelet::ConstLanelet closest_crosswalk{};
    const auto & obj_pose = object.kinematics.pose_with_covariance.pose;
    const auto found_closest_crosswalk =
      lanelet::utils::query::getClosestLanelet(crosswalks_, obj_pose, &closest_crosswalk);

    if (found_closest_crosswalk) {
      const auto edge_points = getCrosswalkEdgePoints(closest_crosswalk);

      if (hasPotentialToReach(
            object, edge_points.front_center_point, edge_points.front_right_point,
            edge_points.front_left_point, prediction_time_horizon_ * 2.0,
            min_crosswalk_user_velocity_, max_crosswalk_user_delta_yaw_threshold_for_lanelet_)) {
        PredictedPath predicted_path =
          path_generator_->generatePathToTargetPoint(object, edge_points.front_center_point);
        predicted_path.confidence = 1.0;
        predicted_object.kinematics.predicted_paths.push_back(predicted_path);
      }

      if (hasPotentialToReach(
            object, edge_points.back_center_point, edge_points.back_right_point,
            edge_points.back_left_point, prediction_time_horizon_ * 2.0,
            min_crosswalk_user_velocity_, max_crosswalk_user_delta_yaw_threshold_for_lanelet_)) {
        PredictedPath predicted_path =
          path_generator_->generatePathToTargetPoint(object, edge_points.back_center_point);
        predicted_path.confidence = 1.0;
        predicted_object.kinematics.predicted_paths.push_back(predicted_path);
      }
    }

    // If the object is not crossing the crosswalk, not in the road lanelets, try to find the edge
    // points for all crosswalks and generate path to the crosswalk edge
  } else {
    for (const auto & crosswalk : crosswalks_) {
      const auto edge_points = getCrosswalkEdgePoints(crosswalk);

      const auto reachable_first = hasPotentialToReach(
        object, edge_points.front_center_point, edge_points.front_right_point,
        edge_points.front_left_point, prediction_time_horizon_, min_crosswalk_user_velocity_,
        max_crosswalk_user_delta_yaw_threshold_for_lanelet_);
      const auto reachable_second = hasPotentialToReach(
        object, edge_points.back_center_point, edge_points.back_right_point,
        edge_points.back_left_point, prediction_time_horizon_, min_crosswalk_user_velocity_,
        max_crosswalk_user_delta_yaw_threshold_for_lanelet_);

      if (!reachable_first && !reachable_second) {
        continue;
      }

      const auto reachable_crosswalk = isReachableCrosswalkEdgePoints(
        object, edge_points, lanelet_map_ptr_, prediction_time_horizon_,
        min_crosswalk_user_velocity_);

      if (!reachable_crosswalk) {
        continue;
      }

      PredictedPath predicted_path =
        path_generator_->generatePathForCrosswalkUser(object, reachable_crosswalk.get());
      predicted_path.confidence = 1.0;

      if (predicted_path.path.empty()) {
        continue;
      }
      // If the predicted path to the crosswalk is crossing the fence, don't use it
      if (doesPathCrossAnyFence(predicted_path)) {
        continue;
      }

      predicted_object.kinematics.predicted_paths.push_back(predicted_path);
    }
  }

  const auto n_path = predicted_object.kinematics.predicted_paths.size();
  for (auto & predicted_path : predicted_object.kinematics.predicted_paths) {
    predicted_path.confidence = 1.0 / n_path;
  }

  return predicted_object;
}

void MapBasedPredictionNode::updateObjectData(TrackedObject & object)
{
  if (
    object.kinematics.orientation_availability ==
    autoware_auto_perception_msgs::msg::DetectedObjectKinematics::AVAILABLE) {
    return;
  }

  // Compute yaw angle from the velocity and position of the object
  const auto & object_pose = object.kinematics.pose_with_covariance.pose;
  const auto & object_twist = object.kinematics.twist_with_covariance.twist;
  const auto future_object_pose = tier4_autoware_utils::calcOffsetPose(
    object_pose, object_twist.linear.x * 0.1, object_twist.linear.y * 0.1, 0.0);

  // assumption: the object vx is much larger than vy
  if (object.kinematics.twist_with_covariance.twist.linear.x >= 0.0) return;

  switch (object.kinematics.orientation_availability) {
    case autoware_auto_perception_msgs::msg::DetectedObjectKinematics::SIGN_UNKNOWN: {
      const auto original_yaw =
        tf2::getYaw(object.kinematics.pose_with_covariance.pose.orientation);
      // flip the angle
      object.kinematics.pose_with_covariance.pose.orientation =
        tier4_autoware_utils::createQuaternionFromYaw(tier4_autoware_utils::pi + original_yaw);
      break;
    }
    default: {
      const auto updated_object_yaw =
        tier4_autoware_utils::calcAzimuthAngle(object_pose.position, future_object_pose.position);

      object.kinematics.pose_with_covariance.pose.orientation =
        tier4_autoware_utils::createQuaternionFromYaw(updated_object_yaw);
      break;
    }
  }
  object.kinematics.twist_with_covariance.twist.linear.x *= -1.0;
  object.kinematics.twist_with_covariance.twist.linear.y *= -1.0;

  return;
}

void MapBasedPredictionNode::removeOldObjectsHistory(const double current_time)
{
  std::vector<std::string> invalid_object_id;
  for (auto iter = objects_history_.begin(); iter != objects_history_.end(); ++iter) {
    const std::string object_id = iter->first;
    std::deque<ObjectData> & object_data = iter->second;

    // If object data is empty, we are going to delete the buffer for the obstacle
    if (object_data.empty()) {
      invalid_object_id.push_back(object_id);
      continue;
    }

    const double latest_object_time = rclcpp::Time(object_data.back().header.stamp).seconds();

    // Delete Old Objects
    if (current_time - latest_object_time > 2.0) {
      invalid_object_id.push_back(object_id);
      continue;
    }

    // Delete old information
    while (!object_data.empty()) {
      const double post_object_time = rclcpp::Time(object_data.front().header.stamp).seconds();
      if (current_time - post_object_time > 2.0) {
        // Delete Old Position
        object_data.pop_front();
      } else {
        break;
      }
    }

    if (object_data.empty()) {
      invalid_object_id.push_back(object_id);
      continue;
    }
  }

  for (const auto & key : invalid_object_id) {
    objects_history_.erase(key);
  }
}

LaneletsData MapBasedPredictionNode::getCurrentLanelets(const TrackedObject & object)
{
  // obstacle point
  lanelet::BasicPoint2d search_point(
    object.kinematics.pose_with_covariance.pose.position.x,
    object.kinematics.pose_with_covariance.pose.position.y);

  // nearest lanelet
  std::vector<std::pair<double, lanelet::Lanelet>> surrounding_lanelets =
    lanelet::geometry::findNearest(lanelet_map_ptr_->laneletLayer, search_point, 10);

  {  // Step 1. Search same directional lanelets
    // No Closest Lanelets
    if (surrounding_lanelets.empty()) {
      return {};
    }

    LaneletsData object_lanelets;
    std::optional<std::pair<double, lanelet::Lanelet>> closest_lanelet{std::nullopt};
    for (const auto & lanelet : surrounding_lanelets) {
      // Check if the close lanelets meet the necessary condition for start lanelets and
      // Check if similar lanelet is inside the object lanelet
      if (!checkCloseLaneletCondition(lanelet, object) || isDuplicated(lanelet, object_lanelets)) {
        continue;
      }

      // Memorize closest lanelet
      // NOTE: The object may be outside the lanelet.
      if (!closest_lanelet || lanelet.first < closest_lanelet->first) {
        closest_lanelet = lanelet;
      }

      // Check if the obstacle is inside of this lanelet
      constexpr double epsilon = 1e-3;
      if (lanelet.first < epsilon) {
        const auto object_lanelet =
          LaneletData{lanelet.second, calculateLocalLikelihood(lanelet.second, object)};
        object_lanelets.push_back(object_lanelet);
      }
    }

    if (!object_lanelets.empty()) {
      return object_lanelets;
    }
    if (closest_lanelet) {
      return LaneletsData{LaneletData{
        closest_lanelet->second, calculateLocalLikelihood(closest_lanelet->second, object)}};
    }
  }

  {  // Step 2. Search opposite directional lanelets
    // Get opposite lanelets and calculate distance to search point.
    std::vector<std::pair<double, lanelet::Lanelet>> surrounding_opposite_lanelets;
    for (const auto & surrounding_lanelet : surrounding_lanelets) {
      for (const auto & left_opposite_lanelet :
           getLeftOppositeLanelets(lanelet_map_ptr_, surrounding_lanelet.second)) {
        const double distance = lanelet::geometry::distance2d(left_opposite_lanelet, search_point);
        surrounding_opposite_lanelets.push_back(std::make_pair(distance, left_opposite_lanelet));
      }
      for (const auto & right_opposite_lanelet :
           getRightOppositeLanelets(lanelet_map_ptr_, surrounding_lanelet.second)) {
        const double distance = lanelet::geometry::distance2d(right_opposite_lanelet, search_point);
        surrounding_opposite_lanelets.push_back(std::make_pair(distance, right_opposite_lanelet));
      }
    }

    std::optional<std::pair<double, lanelet::Lanelet>> opposite_closest_lanelet{std::nullopt};
    for (const auto & lanelet : surrounding_opposite_lanelets) {
      // Check if the close lanelets meet the necessary condition for start lanelets
      // except for distance checking
      if (!checkCloseLaneletCondition(lanelet, object)) {
        continue;
      }

      // Memorize closest lanelet
      if (!opposite_closest_lanelet || lanelet.first < opposite_closest_lanelet->first) {
        opposite_closest_lanelet = lanelet;
      }
    }
    if (opposite_closest_lanelet) {
      return LaneletsData{LaneletData{
        opposite_closest_lanelet->second,
        calculateLocalLikelihood(opposite_closest_lanelet->second, object)}};
    }
  }

  return LaneletsData{};
}

bool MapBasedPredictionNode::checkCloseLaneletCondition(
  const std::pair<double, lanelet::Lanelet> & lanelet, const TrackedObject & object)
{
  // Step1. If we only have one point in the centerline, we will ignore the lanelet
  if (lanelet.second.centerline().size() <= 1) {
    return false;
  }

  // If the object is in the objects history, we check if the target lanelet is
  // inside the current lanelets id or following lanelets
  const std::string object_id = tier4_autoware_utils::toHexString(object.object_id);
  if (objects_history_.count(object_id) != 0) {
    const std::vector<lanelet::ConstLanelet> & possible_lanelet =
      objects_history_.at(object_id).back().future_possible_lanelets;

    bool not_in_possible_lanelet =
      std::find(possible_lanelet.begin(), possible_lanelet.end(), lanelet.second) ==
      possible_lanelet.end();
    if (!possible_lanelet.empty() && not_in_possible_lanelet) {
      return false;
    }
  }

  // Step2. Calculate the angle difference between the lane angle and obstacle angle
  const double object_yaw = tf2::getYaw(object.kinematics.pose_with_covariance.pose.orientation);
  const double lane_yaw = lanelet::utils::getLaneletAngle(
    lanelet.second, object.kinematics.pose_with_covariance.pose.position);
  const double delta_yaw = object_yaw - lane_yaw;
  const double normalized_delta_yaw = tier4_autoware_utils::normalizeRadian(delta_yaw);
  const double abs_norm_delta = std::fabs(normalized_delta_yaw);

  // Step3. Check if the closest lanelet is valid, and add all
  // of the lanelets that are below max_dist and max_delta_yaw
  const double object_vel = object.kinematics.twist_with_covariance.twist.linear.x;
  const bool is_yaw_reversed =
    M_PI - delta_yaw_threshold_for_searching_lanelet_ < abs_norm_delta && object_vel < 0.0;
  if (dist_threshold_for_searching_lanelet_ < lanelet.first) {
    return false;
  }
  if (!is_yaw_reversed && delta_yaw_threshold_for_searching_lanelet_ < abs_norm_delta) {
    return false;
  }

  return true;
}

float MapBasedPredictionNode::calculateLocalLikelihood(
  const lanelet::Lanelet & current_lanelet, const TrackedObject & object) const
{
  const auto & obj_point = object.kinematics.pose_with_covariance.pose.position;

  // compute yaw difference between the object and lane
  const double obj_yaw = tf2::getYaw(object.kinematics.pose_with_covariance.pose.orientation);
  const double lane_yaw = lanelet::utils::getLaneletAngle(current_lanelet, obj_point);
  const double delta_yaw = obj_yaw - lane_yaw;
  const double abs_norm_delta_yaw = std::fabs(tier4_autoware_utils::normalizeRadian(delta_yaw));

  // compute lateral distance
  const auto centerline = current_lanelet.centerline();
  std::vector<geometry_msgs::msg::Point> converted_centerline;
  for (const auto & p : centerline) {
    const auto converted_p = lanelet::utils::conversion::toGeomMsgPt(p);
    converted_centerline.push_back(converted_p);
  }
  const double lat_dist =
    std::fabs(motion_utils::calcLateralOffset(converted_centerline, obj_point));

  // Compute Chi-squared distributed (Equation (8) in the paper)
  const double sigma_d = sigma_lateral_offset_;  // Standard Deviation for lateral position
  const double sigma_yaw = M_PI * sigma_yaw_angle_deg_ / 180.0;  // Standard Deviation for yaw angle
  const Eigen::Vector2d delta(lat_dist, abs_norm_delta_yaw);
  const Eigen::Matrix2d P_inv =
    (Eigen::Matrix2d() << 1.0 / (sigma_d * sigma_d), 0.0, 0.0, 1.0 / (sigma_yaw * sigma_yaw))
      .finished();
  const double MINIMUM_DISTANCE = 1e-6;
  const double dist = std::max(delta.dot(P_inv * delta), MINIMUM_DISTANCE);

  return static_cast<float>(1.0 / dist);
}

void MapBasedPredictionNode::updateObjectsHistory(
  const std_msgs::msg::Header & header, const TrackedObject & object,
  const LaneletsData & current_lanelets_data)
{
  std::string object_id = tier4_autoware_utils::toHexString(object.object_id);
  const auto current_lanelets = getLanelets(current_lanelets_data);

  ObjectData single_object_data;
  single_object_data.header = header;
  single_object_data.current_lanelets = current_lanelets;
  single_object_data.future_possible_lanelets = current_lanelets;
  single_object_data.pose = object.kinematics.pose_with_covariance.pose;
  const double object_yaw = tf2::getYaw(object.kinematics.pose_with_covariance.pose.orientation);
  single_object_data.pose.orientation = tier4_autoware_utils::createQuaternionFromYaw(object_yaw);
  single_object_data.time_delay = std::fabs((this->get_clock()->now() - header.stamp).seconds());
  single_object_data.twist = object.kinematics.twist_with_covariance.twist;

  // Init lateral kinematics
  for (const auto & current_lane : current_lanelets) {
    const LateralKinematicsToLanelet lateral_kinematics =
      initLateralKinematics(current_lane, single_object_data.pose);
    single_object_data.lateral_kinematics_set[current_lane] = lateral_kinematics;
  }

  if (objects_history_.count(object_id) == 0) {
    // New Object(Create a new object in object histories)
    std::deque<ObjectData> object_data = {single_object_data};
    objects_history_.emplace(object_id, object_data);
  } else {
    // Object that is already in the object buffer
    std::deque<ObjectData> & object_data = objects_history_.at(object_id);
    // get previous object data and update
    const auto prev_object_data = object_data.back();
    updateLateralKinematicsVector(
      prev_object_data, single_object_data, routing_graph_ptr_, cutoff_freq_of_velocity_lpf_);

    object_data.push_back(single_object_data);
  }
}

std::vector<PredictedRefPath> MapBasedPredictionNode::getPredictedReferencePath(
  const TrackedObject & object, const LaneletsData & current_lanelets_data,
  const double object_detected_time)
{
  const double obj_vel = std::hypot(
    object.kinematics.twist_with_covariance.twist.linear.x,
    object.kinematics.twist_with_covariance.twist.linear.y);

  std::vector<PredictedRefPath> all_ref_paths;
  for (const auto & current_lanelet_data : current_lanelets_data) {
    // parameter for lanelet::routing::PossiblePathsParams
    const double search_dist = prediction_time_horizon_ * obj_vel +
                               lanelet::utils::getLaneletLength3d(current_lanelet_data.lanelet);
    lanelet::routing::PossiblePathsParams possible_params{search_dist, {}, 0, false, true};
    const double validate_time_horizon =
      prediction_time_horizon_ * prediction_time_horizon_rate_for_validate_lane_length_;

    // lambda function to get possible paths for isolated lanelet
    // isolated is often caused by lanelet with no connection e.g. shoulder-lane
    auto getPathsForNormalOrIsolatedLanelet = [&](const lanelet::ConstLanelet & lanelet) {
      // if lanelet is not isolated, return normal possible paths
      if (!isIsolatedLanelet(lanelet, routing_graph_ptr_)) {
        return routing_graph_ptr_->possiblePaths(lanelet, possible_params);
      }
      // if lanelet is isolated, check if it has enough length
      if (!validateIsolatedLaneletLength(lanelet, object, validate_time_horizon)) {
        return lanelet::routing::LaneletPaths{};
      } else {
        // if lanelet has enough length, return possible paths
        return getPossiblePathsForIsolatedLanelet(lanelet);
      }
    };

    // lambda function to extract left/right lanelets
    auto getLeftOrRightLanelets = [&](
                                    const lanelet::ConstLanelet & lanelet,
                                    const bool get_left) -> std::optional<lanelet::ConstLanelet> {
      const auto opt =
        get_left ? routing_graph_ptr_->left(lanelet) : routing_graph_ptr_->right(lanelet);
      if (!!opt) {
        return *opt;
      }
      if (!consider_only_routable_neighbours_) {
        const auto adjacent = get_left ? routing_graph_ptr_->adjacentLeft(lanelet)
                                       : routing_graph_ptr_->adjacentRight(lanelet);
        if (!!adjacent) {
          return *adjacent;
        }
        // search for unconnected lanelet
        const auto unconnected_lanelets =
          get_left ? getLeftLineSharingLanelets(lanelet, lanelet_map_ptr_)
                   : getRightLineSharingLanelets(lanelet, lanelet_map_ptr_);
        // just return first candidate of unconnected lanelet for now
        if (!unconnected_lanelets.empty()) {
          return unconnected_lanelets.front();
        }
      }

      // if no candidate lanelet found, return empty
      return std::nullopt;
    };

    // Step1. Get the path
    // Step1.1 Get the left lanelet
    lanelet::routing::LaneletPaths left_paths;
    const auto left_lanelet = getLeftOrRightLanelets(current_lanelet_data.lanelet, true);
    if (!!left_lanelet) {
      left_paths = getPathsForNormalOrIsolatedLanelet(left_lanelet.value());
    }

    // Step1.2 Get the right lanelet
    lanelet::routing::LaneletPaths right_paths;
    const auto right_lanelet = getLeftOrRightLanelets(current_lanelet_data.lanelet, false);
    if (!!right_lanelet) {
      right_paths = getPathsForNormalOrIsolatedLanelet(right_lanelet.value());
    }

    // Step1.3 Get the centerline
    lanelet::routing::LaneletPaths center_paths =
      getPathsForNormalOrIsolatedLanelet(current_lanelet_data.lanelet);

    // Skip calculations if all paths are empty
    if (left_paths.empty() && right_paths.empty() && center_paths.empty()) {
      continue;
    }

    // Step2. Predict Object Maneuver
    const Maneuver predicted_maneuver =
      predictObjectManeuver(object, current_lanelet_data, object_detected_time);

    // Step3. Allocate probability for each predicted maneuver
    const auto maneuver_prob =
      calculateManeuverProbability(predicted_maneuver, left_paths, right_paths, center_paths);

    // Step4. add candidate reference paths to the all_ref_paths
    const float path_prob = current_lanelet_data.probability;
    const auto addReferencePathsLocal = [&](const auto & paths, const auto & maneuver) {
      addReferencePaths(object, paths, path_prob, maneuver_prob, maneuver, all_ref_paths);
    };
    addReferencePathsLocal(left_paths, Maneuver::LEFT_LANE_CHANGE);
    addReferencePathsLocal(right_paths, Maneuver::RIGHT_LANE_CHANGE);
    addReferencePathsLocal(center_paths, Maneuver::LANE_FOLLOW);
  }

  return all_ref_paths;
}

/**
 * @brief Do lane change prediction
 * @return predicted manuever (lane follow, left/right lane change)
 */
Maneuver MapBasedPredictionNode::predictObjectManeuver(
  const TrackedObject & object, const LaneletData & current_lanelet_data,
  const double object_detected_time)
{
  // calculate maneuver
  const auto current_maneuver = [&]() {
    if (lane_change_detection_method_ == "time_to_change_lane") {
      return predictObjectManeuverByTimeToLaneChange(
        object, current_lanelet_data, object_detected_time);
    } else if (lane_change_detection_method_ == "lat_diff_distance") {
      return predictObjectManeuverByLatDiffDistance(
        object, current_lanelet_data, object_detected_time);
    }
    throw std::logic_error("Lane change detection method is invalid.");
  }();

  const std::string object_id = tier4_autoware_utils::toHexString(object.object_id);
  if (objects_history_.count(object_id) == 0) {
    return current_maneuver;
  }
  auto & object_info = objects_history_.at(object_id);

  // update maneuver in object history
  if (!object_info.empty()) {
    object_info.back().one_shot_maneuver = current_maneuver;
  }

  // decide maneuver considering previous results
  if (object_info.size() < 2) {
    object_info.back().output_maneuver = current_maneuver;
    return current_maneuver;
  }
  // NOTE: The index of previous maneuver is not object_info.size() - 1
  const auto prev_output_maneuver =
    object_info.at(static_cast<int>(object_info.size()) - 2).output_maneuver;

  for (int i = 0;
       i < std::min(num_continuous_state_transition_, static_cast<int>(object_info.size())); ++i) {
    const auto & tmp_maneuver =
      object_info.at(static_cast<int>(object_info.size()) - 1 - i).one_shot_maneuver;
    if (tmp_maneuver != current_maneuver) {
      object_info.back().output_maneuver = prev_output_maneuver;
      return prev_output_maneuver;
    }
  }

  object_info.back().output_maneuver = current_maneuver;
  return current_maneuver;
}

Maneuver MapBasedPredictionNode::predictObjectManeuverByTimeToLaneChange(
  const TrackedObject & object, const LaneletData & current_lanelet_data,
  const double /*object_detected_time*/)
{
  // Step1. Check if we have the object in the buffer
  const std::string object_id = tier4_autoware_utils::toHexString(object.object_id);
  if (objects_history_.count(object_id) == 0) {
    return Maneuver::LANE_FOLLOW;
  }

  const std::deque<ObjectData> & object_info = objects_history_.at(object_id);

  // Step2. Check if object history length longer than history_time_length
  const int latest_id = static_cast<int>(object_info.size()) - 1;
  // object history is not long enough
  if (latest_id < 1) {
    return Maneuver::LANE_FOLLOW;
  }

  // Step3. get object lateral kinematics
  const auto & latest_info = object_info.at(static_cast<size_t>(latest_id));

  bool not_found_corresponding_lanelet = true;
  double left_dist, right_dist;
  double v_left_filtered, v_right_filtered;
  if (latest_info.lateral_kinematics_set.count(current_lanelet_data.lanelet) != 0) {
    const auto & lateral_kinematics =
      latest_info.lateral_kinematics_set.at(current_lanelet_data.lanelet);
    left_dist = lateral_kinematics.dist_from_left_boundary;
    right_dist = lateral_kinematics.dist_from_right_boundary;
    v_left_filtered = lateral_kinematics.filtered_left_lateral_velocity;
    v_right_filtered = lateral_kinematics.filtered_right_lateral_velocity;
    not_found_corresponding_lanelet = false;
  }

  // return lane follow when catch exception
  if (not_found_corresponding_lanelet) {
    return Maneuver::LANE_FOLLOW;
  }

  const double latest_lane_width = left_dist + right_dist;
  if (latest_lane_width < 1e-3) {
    RCLCPP_ERROR(get_logger(), "[Map Based Prediction]: Lane Width is too small");
    return Maneuver::LANE_FOLLOW;
  }

  // Step 4. check time to reach left/right bound
  const double epsilon = 1e-9;
  const double margin_to_reach_left_bound = left_dist / (std::fabs(v_left_filtered) + epsilon);
  const double margin_to_reach_right_bound = right_dist / (std::fabs(v_right_filtered) + epsilon);

  // Step 5. detect lane change
  if (
    left_dist < right_dist &&                              // in left side,
    left_dist < dist_threshold_to_bound_ &&                // close to boundary,
    v_left_filtered < 0 &&                                 // approaching,
    margin_to_reach_left_bound < time_threshold_to_bound_  // will soon arrive to left bound
  ) {
    return Maneuver::LEFT_LANE_CHANGE;
  } else if (
    right_dist < left_dist &&                               // in right side,
    right_dist < dist_threshold_to_bound_ &&                // close to boundary,
    v_right_filtered < 0 &&                                 // approaching,
    margin_to_reach_right_bound < time_threshold_to_bound_  // will soon arrive to right bound
  ) {
    return Maneuver::RIGHT_LANE_CHANGE;
  }

  return Maneuver::LANE_FOLLOW;
}

Maneuver MapBasedPredictionNode::predictObjectManeuverByLatDiffDistance(
  const TrackedObject & object, const LaneletData & current_lanelet_data,
  const double /*object_detected_time*/)
{
  // Step1. Check if we have the object in the buffer
  const std::string object_id = tier4_autoware_utils::toHexString(object.object_id);
  if (objects_history_.count(object_id) == 0) {
    return Maneuver::LANE_FOLLOW;
  }

  const std::deque<ObjectData> & object_info = objects_history_.at(object_id);
  const double current_time = (this->get_clock()->now()).seconds();

  // Step2. Get the previous id
  int prev_id = static_cast<int>(object_info.size()) - 1;
  while (prev_id >= 0) {
    const double prev_time_delay = object_info.at(prev_id).time_delay;
    const double prev_time =
      rclcpp::Time(object_info.at(prev_id).header.stamp).seconds() + prev_time_delay;
    // if (object_detected_time - prev_time > history_time_length_) {
    if (current_time - prev_time > history_time_length_) {
      break;
    }
    --prev_id;
  }

  if (prev_id < 0) {
    return Maneuver::LANE_FOLLOW;
  }

  // Step3. Get closest previous lanelet ID
  const auto & prev_info = object_info.at(static_cast<size_t>(prev_id));
  const auto prev_pose = prev_info.pose;
  const lanelet::ConstLanelets prev_lanelets =
    object_info.at(static_cast<size_t>(prev_id)).current_lanelets;
  if (prev_lanelets.empty()) {
    return Maneuver::LANE_FOLLOW;
  }
  lanelet::ConstLanelet prev_lanelet = prev_lanelets.front();
  double closest_prev_yaw = std::numeric_limits<double>::max();
  for (const auto & lanelet : prev_lanelets) {
    const double lane_yaw = lanelet::utils::getLaneletAngle(lanelet, prev_pose.position);
    const double delta_yaw = tf2::getYaw(prev_pose.orientation) - lane_yaw;
    const double normalized_delta_yaw = tier4_autoware_utils::normalizeRadian(delta_yaw);
    if (normalized_delta_yaw < closest_prev_yaw) {
      closest_prev_yaw = normalized_delta_yaw;
      prev_lanelet = lanelet;
    }
  }

  // Step4. Check if the vehicle has changed lane
  const auto current_lanelet = current_lanelet_data.lanelet;
  const auto current_pose = object.kinematics.pose_with_covariance.pose;
  const double dist = tier4_autoware_utils::calcDistance2d(prev_pose, current_pose);
  lanelet::routing::LaneletPaths possible_paths =
    routing_graph_ptr_->possiblePaths(prev_lanelet, dist + 2.0, 0, false);
  bool has_lane_changed = true;
  if (prev_lanelet == current_lanelet) {
    has_lane_changed = false;
  } else {
    for (const auto & path : possible_paths) {
      for (const auto & lanelet : path) {
        if (lanelet == current_lanelet) {
          has_lane_changed = false;
          break;
        }
      }
    }
  }

  if (has_lane_changed) {
    return Maneuver::LANE_FOLLOW;
  }

  // Step5. Lane Change Detection
  const lanelet::ConstLineString2d prev_left_bound = prev_lanelet.leftBound2d();
  const lanelet::ConstLineString2d prev_right_bound = prev_lanelet.rightBound2d();
  const lanelet::ConstLineString2d current_left_bound = current_lanelet.leftBound2d();
  const lanelet::ConstLineString2d current_right_bound = current_lanelet.rightBound2d();
  const double prev_left_dist = calcLeftLateralOffset(prev_left_bound, prev_pose);
  const double prev_right_dist = calcRightLateralOffset(prev_right_bound, prev_pose);
  const double current_left_dist = calcLeftLateralOffset(current_left_bound, current_pose);
  const double current_right_dist = calcRightLateralOffset(current_right_bound, current_pose);
  const double prev_lane_width = std::fabs(prev_left_dist) + std::fabs(prev_right_dist);
  const double current_lane_width = std::fabs(current_left_dist) + std::fabs(current_right_dist);
  if (prev_lane_width < 1e-3 || current_lane_width < 1e-3) {
    RCLCPP_ERROR(get_logger(), "[Map Based Prediction]: Lane Width is too small");
    return Maneuver::LANE_FOLLOW;
  }

  const double current_left_dist_ratio = current_left_dist / current_lane_width;
  const double current_right_dist_ratio = current_right_dist / current_lane_width;
  const double diff_left_current_prev = current_left_dist - prev_left_dist;
  const double diff_right_current_prev = current_right_dist - prev_right_dist;

  if (
    current_left_dist_ratio > dist_ratio_threshold_to_left_bound_ &&
    diff_left_current_prev > diff_dist_threshold_to_left_bound_) {
    return Maneuver::LEFT_LANE_CHANGE;
  } else if (
    current_right_dist_ratio < dist_ratio_threshold_to_right_bound_ &&
    diff_right_current_prev < diff_dist_threshold_to_right_bound_) {
    return Maneuver::RIGHT_LANE_CHANGE;
  }

  return Maneuver::LANE_FOLLOW;
}

geometry_msgs::msg::Pose MapBasedPredictionNode::compensateTimeDelay(
  const geometry_msgs::msg::Pose & delayed_pose, const geometry_msgs::msg::Twist & twist,
  const double dt) const
{
  if (!enable_delay_compensation_) {
    return delayed_pose;
  }

  /*  == Nonlinear model ==
   *
   * x_{k+1}   = x_k + vx_k * cos(yaw_k) * dt - vy_k * sin(yaw_k) * dt
   * y_{k+1}   = y_k + vx_k * sin(yaw_k) * dt + vy_k * cos(yaw_k) * dt
   * yaw_{k+1} = yaw_k + (wz_k) * dt
   *
   */

  const double vx = twist.linear.x;
  const double vy = twist.linear.y;
  const double wz = twist.angular.z;
  const double prev_yaw = tf2::getYaw(delayed_pose.orientation);
  const double prev_x = delayed_pose.position.x;
  const double prev_y = delayed_pose.position.y;
  const double prev_z = delayed_pose.position.z;

  const double curr_x = prev_x + vx * std::cos(prev_yaw) * dt - vy * std::sin(prev_yaw) * dt;
  const double curr_y = prev_y + vx * std::sin(prev_yaw) * dt + vy * std::cos(prev_yaw) * dt;
  const double curr_z = prev_z;
  const double curr_yaw = prev_yaw + wz * dt;

  geometry_msgs::msg::Pose current_pose;
  current_pose.position = tier4_autoware_utils::createPoint(curr_x, curr_y, curr_z);
  current_pose.orientation = tier4_autoware_utils::createQuaternionFromYaw(curr_yaw);

  return current_pose;
}

double MapBasedPredictionNode::calcRightLateralOffset(
  const lanelet::ConstLineString2d & boundary_line, const geometry_msgs::msg::Pose & search_pose)
{
  std::vector<geometry_msgs::msg::Point> boundary_path(boundary_line.size());
  for (size_t i = 0; i < boundary_path.size(); ++i) {
    const double x = boundary_line[i].x();
    const double y = boundary_line[i].y();
    boundary_path[i] = tier4_autoware_utils::createPoint(x, y, 0.0);
  }

  return std::fabs(motion_utils::calcLateralOffset(boundary_path, search_pose.position));
}

double MapBasedPredictionNode::calcLeftLateralOffset(
  const lanelet::ConstLineString2d & boundary_line, const geometry_msgs::msg::Pose & search_pose)
{
  return -calcRightLateralOffset(boundary_line, search_pose);
}

void MapBasedPredictionNode::updateFuturePossibleLanelets(
  const TrackedObject & object, const lanelet::routing::LaneletPaths & paths)
{
  std::string object_id = tier4_autoware_utils::toHexString(object.object_id);
  if (objects_history_.count(object_id) == 0) {
    return;
  }

  std::vector<lanelet::ConstLanelet> & possible_lanelets =
    objects_history_.at(object_id).back().future_possible_lanelets;
  for (const auto & path : paths) {
    for (const auto & lanelet : path) {
      bool not_in_buffer = std::find(possible_lanelets.begin(), possible_lanelets.end(), lanelet) ==
                           possible_lanelets.end();
      if (not_in_buffer) {
        possible_lanelets.push_back(lanelet);
      }
    }
  }
}

void MapBasedPredictionNode::addReferencePaths(
  const TrackedObject & object, const lanelet::routing::LaneletPaths & candidate_paths,
  const float path_probability, const ManeuverProbability & maneuver_probability,
  const Maneuver & maneuver, std::vector<PredictedRefPath> & reference_paths)
{
  if (!candidate_paths.empty()) {
    updateFuturePossibleLanelets(object, candidate_paths);
    const auto converted_paths = convertPathType(candidate_paths);
    for (const auto & converted_path : converted_paths) {
      PredictedRefPath predicted_path;
      predicted_path.probability = maneuver_probability.at(maneuver) * path_probability;
      predicted_path.path = converted_path;
      predicted_path.maneuver = maneuver;
      reference_paths.push_back(predicted_path);
    }
  }
}

ManeuverProbability MapBasedPredictionNode::calculateManeuverProbability(
  const Maneuver & predicted_maneuver, const lanelet::routing::LaneletPaths & left_paths,
  const lanelet::routing::LaneletPaths & right_paths,
  const lanelet::routing::LaneletPaths & center_paths)
{
  float left_lane_change_probability = 0.0;
  float right_lane_change_probability = 0.0;
  float lane_follow_probability = 0.0;
  if (!left_paths.empty() && predicted_maneuver == Maneuver::LEFT_LANE_CHANGE) {
    constexpr float LF_PROB_WHEN_LC = 0.9;  // probability for lane follow during lane change
    constexpr float LC_PROB_WHEN_LC = 1.0;  // probability for left lane change
    left_lane_change_probability = LC_PROB_WHEN_LC;
    right_lane_change_probability = 0.0;
    lane_follow_probability = LF_PROB_WHEN_LC;
  } else if (!right_paths.empty() && predicted_maneuver == Maneuver::RIGHT_LANE_CHANGE) {
    constexpr float LF_PROB_WHEN_LC = 0.9;  // probability for lane follow during lane change
    constexpr float RC_PROB_WHEN_LC = 1.0;  // probability for right lane change
    left_lane_change_probability = 0.0;
    right_lane_change_probability = RC_PROB_WHEN_LC;
    lane_follow_probability = LF_PROB_WHEN_LC;
  } else if (!center_paths.empty()) {
    constexpr float LF_PROB = 1.0;  // probability for lane follow
    constexpr float LC_PROB = 0.3;  // probability for left lane change
    constexpr float RC_PROB = 0.3;  // probability for right lane change
    if (predicted_maneuver == Maneuver::LEFT_LANE_CHANGE) {
      // If prediction says left change, but left lane is empty, assume lane follow
      left_lane_change_probability = 0.0;
      right_lane_change_probability = (!right_paths.empty()) ? RC_PROB : 0.0;
    } else if (predicted_maneuver == Maneuver::RIGHT_LANE_CHANGE) {
      // If prediction says right change, but right lane is empty, assume lane follow
      left_lane_change_probability = (!left_paths.empty()) ? LC_PROB : 0.0;
      right_lane_change_probability = 0.0;
    } else {
      // Predicted Maneuver is Lane Follow
      left_lane_change_probability = LC_PROB;
      right_lane_change_probability = RC_PROB;
    }
    lane_follow_probability = LF_PROB;
  } else {
    // Center path is empty
    constexpr float LC_PROB = 1.0;  // probability for left lane change
    constexpr float RC_PROB = 1.0;  // probability for right lane change
    lane_follow_probability = 0.0;

    // If the given lane is empty, the probability goes to 0
    left_lane_change_probability = left_paths.empty() ? 0.0 : LC_PROB;
    right_lane_change_probability = right_paths.empty() ? 0.0 : RC_PROB;
  }

  const float MIN_PROBABILITY = 1e-3;
  const float max_prob = std::max(
    MIN_PROBABILITY, std::max(
                       lane_follow_probability,
                       std::max(left_lane_change_probability, right_lane_change_probability)));

  // Insert Normalized Probability
  ManeuverProbability maneuver_prob;
  maneuver_prob[Maneuver::LEFT_LANE_CHANGE] = left_lane_change_probability / max_prob;
  maneuver_prob[Maneuver::RIGHT_LANE_CHANGE] = right_lane_change_probability / max_prob;
  maneuver_prob[Maneuver::LANE_FOLLOW] = lane_follow_probability / max_prob;

  return maneuver_prob;
}

std::vector<PosePath> MapBasedPredictionNode::convertPathType(
  const lanelet::routing::LaneletPaths & paths)
{
  std::vector<PosePath> converted_paths;
  for (const auto & path : paths) {
    PosePath converted_path;

    // Insert Positions. Note that we start inserting points from previous lanelet
    if (!path.empty()) {
      lanelet::ConstLanelets prev_lanelets = routing_graph_ptr_->previous(path.front());
      if (!prev_lanelets.empty()) {
        lanelet::ConstLanelet prev_lanelet = prev_lanelets.front();
        bool init_flag = true;
        geometry_msgs::msg::Pose prev_p;
        for (const auto & lanelet_p : prev_lanelet.centerline()) {
          geometry_msgs::msg::Pose current_p;
          current_p.position = lanelet::utils::conversion::toGeomMsgPt(lanelet_p);
          if (init_flag) {
            init_flag = false;
            prev_p = current_p;
            continue;
          }

          const double lane_yaw = std::atan2(
            current_p.position.y - prev_p.position.y, current_p.position.x - prev_p.position.x);
          current_p.orientation = tier4_autoware_utils::createQuaternionFromYaw(lane_yaw);
          converted_path.push_back(current_p);
          prev_p = current_p;
        }
      }
    }

    for (const auto & lanelet : path) {
      bool init_flag = true;
      geometry_msgs::msg::Pose prev_p;
      for (const auto & lanelet_p : lanelet.centerline()) {
        geometry_msgs::msg::Pose current_p;
        current_p.position = lanelet::utils::conversion::toGeomMsgPt(lanelet_p);
        if (init_flag) {
          init_flag = false;
          prev_p = current_p;
          continue;
        }

        // Prevent from inserting same points
        if (!converted_path.empty()) {
          const auto last_p = converted_path.back();
          const double tmp_dist = tier4_autoware_utils::calcDistance2d(last_p, current_p);
          if (tmp_dist < 1e-6) {
            prev_p = current_p;
            continue;
          }
        }

        const double lane_yaw = std::atan2(
          current_p.position.y - prev_p.position.y, current_p.position.x - prev_p.position.x);
        current_p.orientation = tier4_autoware_utils::createQuaternionFromYaw(lane_yaw);
        converted_path.push_back(current_p);
        prev_p = current_p;
      }
    }

    // Resample Path
    const auto resampled_converted_path =
      motion_utils::resamplePoseVector(converted_path, reference_path_resolution_);
    converted_paths.push_back(resampled_converted_path);
  }

  return converted_paths;
}

bool MapBasedPredictionNode::isDuplicated(
  const std::pair<double, lanelet::ConstLanelet> & target_lanelet,
  const LaneletsData & lanelets_data)
{
  const double CLOSE_LANELET_THRESHOLD = 0.1;
  for (const auto & lanelet_data : lanelets_data) {
    const auto target_lanelet_end_p = target_lanelet.second.centerline2d().back();
    const auto lanelet_end_p = lanelet_data.lanelet.centerline2d().back();
    const double dist = std::hypot(
      target_lanelet_end_p.x() - lanelet_end_p.x(), target_lanelet_end_p.y() - lanelet_end_p.y());
    if (dist < CLOSE_LANELET_THRESHOLD) {
      return true;
    }
  }

  return false;
}

bool MapBasedPredictionNode::isDuplicated(
  const PredictedPath & predicted_path, const std::vector<PredictedPath> & predicted_paths)
{
  const double CLOSE_PATH_THRESHOLD = 0.1;
  for (const auto & prev_predicted_path : predicted_paths) {
    const auto prev_path_end = prev_predicted_path.path.back().position;
    const auto current_path_end = predicted_path.path.back().position;
    const double dist = tier4_autoware_utils::calcDistance2d(prev_path_end, current_path_end);
    if (dist < CLOSE_PATH_THRESHOLD) {
      return true;
    }
  }

  return false;
}
}  // namespace map_based_prediction

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(map_based_prediction::MapBasedPredictionNode)
