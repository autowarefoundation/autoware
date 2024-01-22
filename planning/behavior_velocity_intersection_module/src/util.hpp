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

#ifndef UTIL_HPP_
#define UTIL_HPP_

#include "interpolated_path_info.hpp"

#include <rclcpp/logger.hpp>
#include <tier4_autoware_utils/geometry/boost_geometry.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_object_kinematics.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_routing/Forward.h>

#include <optional>
#include <set>
#include <utility>
#include <vector>

namespace behavior_velocity_planner::util
{

/**
 * @fn
 * @brief insert a new pose to the path and return its index
 * @return if insertion was successful return the inserted point index
 */
std::optional<size_t> insertPointIndex(
  const geometry_msgs::msg::Pose & in_pose,
  autoware_auto_planning_msgs::msg::PathWithLaneId * inout_path,
  const double ego_nearest_dist_threshold, const double ego_nearest_yaw_threshold);

/**
 * @fn
 * @brief check if a PathPointWithLaneId contains any of the given lane ids
 */
bool hasLaneIds(
  const autoware_auto_planning_msgs::msg::PathPointWithLaneId & p,
  const std::set<lanelet::Id> & ids);

/**
 * @fn
 * @brief find the first contiguous interval of the path points that contains the specified lane ids
 * @return if no interval is found, return null. if the interval [start, end] (inclusive range) is
 * found, returns the pair (start-1, end)
 */
std::optional<std::pair<size_t, size_t>> findLaneIdsInterval(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & p, const std::set<lanelet::Id> & ids);

/**
 * @fn
 * @brief return the index of the first point which is inside the given polygon
 * @param[in] lane_interval the interval of the path points on the intersection
 * @param[in] search_forward flag for search direction
 */
std::optional<size_t> getFirstPointInsidePolygon(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
  const std::pair<size_t, size_t> lane_interval, const lanelet::CompoundPolygon3d & polygon,
  const bool search_forward = true);

/**
 * @fn
 * @brief check if ego is over the target_idx. If the index is same, compare the exact pose
 * @param[in] path path
 * @param[in] closest_idx ego's closest index on the path
 * @param[in] current_pose ego's exact pose
 * @return true if ego is over the target_idx
 */
bool isOverTargetIndex(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const size_t closest_idx,
  const geometry_msgs::msg::Pose & current_pose, const size_t target_idx);

/**
 * @fn
 * @brief check if ego is before the target_idx. If the index is same, compare the exact pose
 * @param[in] path path
 * @param[in] closest_idx ego's closest index on the path
 * @param[in] current_pose ego's exact pose
 * @return true if ego is over the target_idx
 */
bool isBeforeTargetIndex(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const size_t closest_idx,
  const geometry_msgs::msg::Pose & current_pose, const size_t target_idx);

std::optional<tier4_autoware_utils::Polygon2d> getIntersectionArea(
  lanelet::ConstLanelet assigned_lane, lanelet::LaneletMapConstPtr lanelet_map_ptr);

/**
 * @fn
 * @brief check if the given lane has related traffic light
 */
bool hasAssociatedTrafficLight(lanelet::ConstLanelet lane);

/**
 * @fn
 * @brief interpolate PathWithLaneId
 */
std::optional<intersection::InterpolatedPathInfo> generateInterpolatedPath(
  const lanelet::Id lane_id, const std::set<lanelet::Id> & associative_lane_ids,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & input_path, const double ds,
  const rclcpp::Logger logger);

geometry_msgs::msg::Pose getObjectPoseWithVelocityDirection(
  const autoware_auto_perception_msgs::msg::PredictedObjectKinematics & obj_state);

/**
 * @fn
 * @brief this function sorts the set of lanelets topologically using topological sort and merges
 * the lanelets from each root to each end. each branch is merged and returned with the original
 * lanelets
 * @param[in] lanelets the set of lanelets
 * @param[in] routing_graph_ptr the routing graph
 * @return the pair of merged lanelets and their corresponding original lanelets
 */
std::pair<lanelet::ConstLanelets, std::vector<lanelet::ConstLanelets>>
mergeLaneletsByTopologicalSort(
  const lanelet::ConstLanelets & lanelets,
  const lanelet::routing::RoutingGraphPtr routing_graph_ptr);

/**
 * @fn
 * @brief find the index of the first point where vehicle footprint intersects with the given
 * polygon
 */
std::optional<size_t> getFirstPointInsidePolygonByFootprint(
  const lanelet::CompoundPolygon3d & polygon,
  const intersection::InterpolatedPathInfo & interpolated_path_info,
  const tier4_autoware_utils::LinearRing2d & footprint, const double vehicle_length);

std::optional<std::pair<
  size_t /* the index of interpolated PathPoint*/, size_t /* the index of corresponding Polygon */>>
getFirstPointInsidePolygonsByFootprint(
  const std::vector<lanelet::CompoundPolygon3d> & polygons,
  const intersection::InterpolatedPathInfo & interpolated_path_info,
  const tier4_autoware_utils::LinearRing2d & footprint, const double vehicle_length);

std::vector<lanelet::CompoundPolygon3d> getPolygon3dFromLanelets(
  const lanelet::ConstLanelets & ll_vec);

}  // namespace behavior_velocity_planner::util

#endif  // UTIL_HPP_
