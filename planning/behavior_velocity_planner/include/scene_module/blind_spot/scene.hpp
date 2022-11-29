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

#ifndef SCENE_MODULE__BLIND_SPOT__SCENE_HPP_
#define SCENE_MODULE__BLIND_SPOT__SCENE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <scene_module/scene_module_interface.hpp>
#include <utilization/boost_geometry_helper.hpp>
#include <utilization/state_machine.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_object.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <boost/optional.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <memory>
#include <string>

namespace behavior_velocity_planner
{
struct BlindSpotPolygons
{
  lanelet::CompoundPolygon3d conflict_area;
  lanelet::CompoundPolygon3d detection_area;
};

class BlindSpotModule : public SceneModuleInterface
{
public:
  enum class TurnDirection { LEFT = 0, RIGHT, INVALID };

  struct DebugData
  {
    autoware_auto_planning_msgs::msg::PathWithLaneId path_raw;
    geometry_msgs::msg::Pose virtual_wall_pose;
    geometry_msgs::msg::Pose stop_point_pose;
    geometry_msgs::msg::Pose judge_point_pose;
    geometry_msgs::msg::Polygon conflict_area_for_blind_spot;
    geometry_msgs::msg::Polygon detection_area_for_blind_spot;
    autoware_auto_planning_msgs::msg::PathWithLaneId spline_path;
    autoware_auto_perception_msgs::msg::PredictedObjects conflicting_targets;
  };

public:
  struct PlannerParam
  {
    bool use_pass_judge_line;  //! distance which ego can stop with max brake
    double stop_line_margin;   //! distance from auto-generated stopline to detection_area boundary
    double backward_length;  //! distance[m] from closest path point to the edge of beginning point
    double ignore_width_from_center_line;  //! ignore width from center line from detection_area
    double
      max_future_movement_time;  //! maximum time[second] for considering future movement of object
  };

  BlindSpotModule(
    const int64_t module_id, const int64_t lane_id, std::shared_ptr<const PlannerData> planner_data,
    const PlannerParam & planner_param, const rclcpp::Logger logger,
    const rclcpp::Clock::SharedPtr clock);

  /**
   * @brief plan go-stop velocity at traffic crossing with collision check between reference path
   * and object predicted path
   */
  bool modifyPathVelocity(PathWithLaneId * path, StopReason * stop_reason) override;

  visualization_msgs::msg::MarkerArray createDebugMarkerArray() override;
  visualization_msgs::msg::MarkerArray createVirtualWallMarkerArray() override;

private:
  const int64_t lane_id_;
  TurnDirection turn_direction_;
  bool has_traffic_light_;
  bool is_over_pass_judge_line_;

  // Parameter
  PlannerParam planner_param_;

  /**
   * @brief Check obstacle is in blind spot areas.
   * Condition1: Object's position is in broad blind spot area.
   * Condition2: Object's predicted position is in narrow blind spot area.
   * If both conditions are met, return true
   * @param path path information associated with lane id
   * @param objects_ptr dynamic objects
   * @param closest_idx closest path point index from ego car in path points
   * @return true when an object is detected in blind spot
   */
  bool checkObstacleInBlindSpot(
    lanelet::LaneletMapConstPtr lanelet_map_ptr,
    lanelet::routing::RoutingGraphPtr routing_graph_ptr,
    const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
    const autoware_auto_perception_msgs::msg::PredictedObjects::ConstSharedPtr objects_ptr,
    const int closest_idx, const geometry_msgs::msg::Pose & stop_line_pose) const;

  /**
   * @brief Create half lanelet
   * @param lanelet input lanelet
   * @return Half lanelet
   */
  lanelet::ConstLanelet generateHalfLanelet(const lanelet::ConstLanelet lanelet) const;

  /**
   * @brief Make blind spot areas. Narrow area is made from closest path point to stop line index.
   * Broad area is made from backward expanded point to stop line point
   * @param path path information associated with lane id
   * @param closest_idx closest path point index from ego car in path points
   * @return Blind spot polygons
   */
  boost::optional<BlindSpotPolygons> generateBlindSpotPolygons(
    lanelet::LaneletMapConstPtr lanelet_map_ptr,
    lanelet::routing::RoutingGraphPtr routing_graph_ptr,
    const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const int closest_idx,
    const geometry_msgs::msg::Pose & pose) const;

  /**
   * @brief Get vehicle edge
   * @param vehicle_pose pose of ego vehicle
   * @param vehicle_width width of ego vehicle
   * @param base_link2front length between base link and front of ego vehicle
   * @return edge of ego vehicle
   */
  lanelet::LineString2d getVehicleEdge(
    const geometry_msgs::msg::Pose & vehicle_pose, const double vehicle_width,
    const double base_link2front) const;

  /**
   * @brief Check if object is belong to targeted classes
   * @param object Dynamic object
   * @return True when object belong to targeted classes
   */
  bool isTargetObjectType(const autoware_auto_perception_msgs::msg::PredictedObject & object) const;

  /**
   * @brief Check if at least one of object's predicted position is in area
   * @param object Dynamic object
   * @param area Area defined by polygon
   * @return True when at least one of object's predicted position is in area
   */
  bool isPredictedPathInArea(
    const autoware_auto_perception_msgs::msg::PredictedObject & object,
    const lanelet::CompoundPolygon3d & area) const;

  /**
   * @brief Generate a stop line and insert it into the path.
   * A stop line is at an intersection point of straight path with vehicle path
   * @param detection_areas used to generate stop line
   * @param path            ego-car lane
   * @param stop_line_idx   generated stop line index
   * @param pass_judge_line_idx  generated pass judge line index
   * @return false when generation failed
   */
  bool generateStopLine(
    const lanelet::ConstLanelets straight_lanelets,
    autoware_auto_planning_msgs::msg::PathWithLaneId * path, int * stop_line_idx) const;

  /**
   * @brief Insert a point to target path
   * @param insert_idx_ip insert point index in path_ip
   * @param path_ip interpolated path
   * @param path target path for inserting a point
   * @return inserted point idx in target path, return -1 when could not find valid index
   */
  int insertPoint(
    const int insert_idx_ip, const autoware_auto_planning_msgs::msg::PathWithLaneId path_ip,
    autoware_auto_planning_msgs::msg::PathWithLaneId * path) const;

  /**
   * @brief Calculate first path index that is conflicting lanelets.
   * @param path     target path
   * @param lanelets target lanelets
   * @return path point index
   */
  boost::optional<int> getFirstPointConflictingLanelets(
    const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
    const lanelet::ConstLanelets & lanelets) const;

  /**
   * @brief Get start point from lanelet
   * @param lane_id lane id of objective lanelet
   * @return end point of lanelet
   */
  boost::optional<geometry_msgs::msg::Pose> getStartPointFromLaneLet(const int lane_id) const;

  /**
   * @brief get straight lanelets in intersection
   */
  lanelet::ConstLanelets getStraightLanelets(
    lanelet::LaneletMapConstPtr lanelet_map_ptr,
    lanelet::routing::RoutingGraphPtr routing_graph_ptr, const int lane_id);

  /**
   * @brief Modify objects predicted path. remove path point if the time exceeds timer_thr.
   * @param objects_ptr target objects
   * @param time_thr    time threshold to cut path
   */
  void cutPredictPathWithDuration(
    autoware_auto_perception_msgs::msg::PredictedObjects * objects_ptr,
    const double time_thr) const;

  StateMachine state_machine_;  //! for state

  // Debug
  mutable DebugData debug_data_;
};
}  // namespace behavior_velocity_planner

#endif  // SCENE_MODULE__BLIND_SPOT__SCENE_HPP_
