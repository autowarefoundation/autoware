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

#ifndef SCENE_MODULE__INTERSECTION__UTIL_HPP_
#define SCENE_MODULE__INTERSECTION__UTIL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <scene_module/intersection/scene_intersection.hpp>

#include <geometry_msgs/msg/point.hpp>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace behavior_velocity_planner
{
namespace util
{
std::optional<size_t> insertPoint(
  const geometry_msgs::msg::Pose & in_pose,
  autoware_auto_planning_msgs::msg::PathWithLaneId * inout_path);

bool hasLaneId(const autoware_auto_planning_msgs::msg::PathPointWithLaneId & p, const int id);
std::optional<std::pair<size_t, size_t>> findLaneIdInterval(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & p, const int lane_id);
std::optional<size_t> getDuplicatedPointIdx(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
  const geometry_msgs::msg::Point & point);

/**
 * @brief get objective polygons for detection area
 */
std::tuple<lanelet::ConstLanelets, lanelet::ConstLanelets> getObjectiveLanelets(
  lanelet::LaneletMapConstPtr lanelet_map_ptr, lanelet::routing::RoutingGraphPtr routing_graph_ptr,
  const int lane_id, const double detection_area_length, const bool tl_arrow_solid_on = false);

struct StopLineIdx
{
  size_t first_inside_lane = 0;
  size_t pass_judge_line = 0;
  size_t stop_line = 0;
  size_t keep_detection_line = 0;
};

/**
 * @brief Generate a stop line and insert it into the path. If the stop line is defined in the map,
 * read it from the map; otherwise, generate a stop line at a position where it will not collide.
 * @param detection_areas used to generate stop line
 * @param original_path   ego-car lane
 * @param target_path     target lane to insert stop point (part of ego-car lane or same to ego-car
 * lane)
 " @param use_stuck_stopline if true, a stop line is generated at the beginning of intersection lane
 * @return nullopt if path is not intersecting with detection areas
 */
std::pair<std::optional<size_t>, std::optional<StopLineIdx>> generateStopLine(
  const int lane_id, const std::vector<lanelet::CompoundPolygon3d> & detection_areas,
  const std::vector<lanelet::CompoundPolygon3d> & conflicting_areas,
  const std::shared_ptr<const PlannerData> & planner_data, const double stop_line_margin,
  const double keep_detection_line_margin, const bool use_stuck_stopline,
  autoware_auto_planning_msgs::msg::PathWithLaneId * original_path,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & target_path, const rclcpp::Logger logger,
  const rclcpp::Clock::SharedPtr clock);

/**
 * @brief Calculate first path index that is in the polygon.
 * @param path     target path
 * @param lane_interval_start the start index of point on the lane
 * @param lane_interval_end the last index of point on the lane
 * @param polygons target polygon
 * @return path point index
 */
std::optional<size_t> getFirstPointInsidePolygons(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const size_t lane_interval_start,
  const size_t lane_interval_end, const int lane_id,
  const std::vector<lanelet::CompoundPolygon3d> & polygons);

/**
 * @brief Get stop point from map if exists
 * @param stop_pose stop point defined on map
 * @return true when the stop point is defined on map.
 */
bool getStopLineIndexFromMap(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const size_t lane_interval_start,
  const size_t lane_interval_end, const int lane_id,
  const std::shared_ptr<const PlannerData> & planner_data, size_t * stop_idx_ip,
  const double dist_thr, const rclcpp::Logger logger);

std::vector<lanelet::CompoundPolygon3d> getPolygon3dFromLaneletsVec(
  const std::vector<lanelet::ConstLanelets> & ll_vec, double clip_length);

std::vector<lanelet::CompoundPolygon3d> getPolygon3dFromLanelets(
  const lanelet::ConstLanelets & ll_vec, double clip_length);

std::vector<lanelet::CompoundPolygon3d> getPolygon3dFromLanelets(
  const lanelet::ConstLanelets & ll_vec);

std::vector<int> getLaneletIdsFromLaneletsVec(const std::vector<lanelet::ConstLanelets> & ll_vec);

lanelet::ConstLanelet generateOffsetLanelet(
  const lanelet::ConstLanelet lanelet, double right_margin, double left_margin);
geometry_msgs::msg::Pose toPose(const geometry_msgs::msg::Point & p);

/**
 * @brief check if ego is over the target_idx. If the index is same, compare the exact pose
 * @param path path
 * @param closest_idx ego's closest index on the path
 * @param current_pose ego's exact pose
 * @return true if ego is over the target_idx
 */
bool isOverTargetIndex(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const int closest_idx,
  const geometry_msgs::msg::Pose & current_pose, const int target_idx);

bool isBeforeTargetIndex(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const int closest_idx,
  const geometry_msgs::msg::Pose & current_pose, const int target_idx);

lanelet::ConstLanelets extendedAdjacentDirectionLanes(
  const lanelet::LaneletMapPtr map, const lanelet::routing::RoutingGraphPtr routing_graph,
  lanelet::ConstLanelet lane);

std::optional<Polygon2d> getIntersectionArea(
  lanelet::ConstLanelet assigned_lane, lanelet::LaneletMapConstPtr lanelet_map_ptr);

bool hasAssociatedTrafficLight(lanelet::ConstLanelet lane);
bool isTrafficLightArrowActivated(
  lanelet::ConstLanelet lane,
  const std::map<int, autoware_auto_perception_msgs::msg::TrafficSignalStamped> & tl_infos);

}  // namespace util
}  // namespace behavior_velocity_planner

#endif  // SCENE_MODULE__INTERSECTION__UTIL_HPP_
