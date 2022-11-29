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

#ifndef SCENE_MODULE__INTERSECTION__SCENE_INTERSECTION_HPP_
#define SCENE_MODULE__INTERSECTION__SCENE_INTERSECTION_HPP_

#include <rclcpp/rclcpp.hpp>
#include <scene_module/scene_module_interface.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>
#include <utilization/boost_geometry_helper.hpp>
#include <utilization/state_machine.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_object.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace behavior_velocity_planner
{
// first: time, second: distance
using TimeDistanceArray = std::vector<std::pair<double, double>>;

class IntersectionModule : public SceneModuleInterface
{
public:
  struct DebugData
  {
    bool stop_required;
    autoware_auto_planning_msgs::msg::PathWithLaneId path_raw;

    geometry_msgs::msg::Pose slow_wall_pose;
    geometry_msgs::msg::Pose stop_wall_pose;
    geometry_msgs::msg::Pose stop_point_pose;
    geometry_msgs::msg::Pose judge_point_pose;
    geometry_msgs::msg::Polygon ego_lane_polygon;
    geometry_msgs::msg::Polygon stuck_vehicle_detect_area;
    geometry_msgs::msg::Polygon candidate_collision_ego_lane_polygon;
    std::vector<geometry_msgs::msg::Polygon> candidate_collision_object_polygons;
    std::vector<lanelet::ConstLanelet> intersection_detection_lanelets;
    std::vector<lanelet::CompoundPolygon3d> detection_area;
    geometry_msgs::msg::Polygon intersection_area;
    std::vector<lanelet::CompoundPolygon3d> adjacent_area;
    autoware_auto_perception_msgs::msg::PredictedObjects conflicting_targets;
    autoware_auto_perception_msgs::msg::PredictedObjects stuck_targets;
    geometry_msgs::msg::Pose predicted_obj_pose;
  };

public:
  struct PlannerParam
  {
    double state_transit_margin_time;
    double stop_line_margin;  //! distance from auto-generated stopline to detection_area boundary
    double keep_detection_line_margin;  //! distance (toward path end) from generated stop line.
                                        //! keep detection if ego is before this line and ego.vel <
                                        //! keep_detection_vel_thr
    double keep_detection_vel_thr;
    double stuck_vehicle_detect_dist;  //! distance from end point to finish stuck vehicle check
    double
      stuck_vehicle_ignore_dist;   //! distance from intersection start to start stuck vehicle check
    double stuck_vehicle_vel_thr;  //! Threshold of the speed to be recognized as stopped
    double intersection_velocity;  //! used for intersection passing time
    double intersection_max_acc;   //! used for calculating intersection velocity
    double detection_area_margin;  //! used for detecting objects in detection area
    double detection_area_right_margin;  //! used for detecting objects in detection area only right
                                         //! direction
    double detection_area_left_margin;   //! used for detecting objects in detection area only left
                                         //! direction
    double detection_area_length;        //! used to create detection area polygon
    double detection_area_angle_thr;     //! threshold in checking the angle of detecting objects
    double min_predicted_path_confidence;
    //! minimum confidence value of predicted path to use for collision detection
    double external_input_timeout;       //! used to disable external input
    double collision_start_margin_time;  //! start margin time to check collision
    double collision_end_margin_time;    //! end margin time to check collision
    bool use_stuck_stopline;  //! stopline generate before the intersection lanelet when is_stuck.
    double
      assumed_front_car_decel;  //! the expected deceleration of front car when front car as well
    bool enable_front_car_decel_prediction;  //! flag for using above feature
  };

  IntersectionModule(
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
  int64_t lane_id_;
  std::string turn_direction_;
  bool has_traffic_light_;
  bool is_go_out_;

  // Parameter
  PlannerParam planner_param_;

  /**
   * @brief check collision for all lanelet area & predicted objects (call checkPathCollision() as
   * actual collision check algorithm inside this function)
   * @param lanelet_map_ptr  lanelet map
   * @param path             ego-car lane
   * @param detection_area_lanelet_ids  angle check is performed for obstacles using this lanelet
   * ids
   * @param intersection_area associated intersection_area if exists
   * @param objects_ptr      target objects
   * @param closest_idx      ego-car position index on the lane
   * @return true if collision is detected
   */
  bool checkCollision(
    lanelet::LaneletMapConstPtr lanelet_map_ptr,
    const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
    const lanelet::ConstLanelets & detection_area_lanelets,
    const lanelet::ConstLanelets & adjacent_lanelets,
    const std::optional<Polygon2d> & intersection_area,
    const autoware_auto_perception_msgs::msg::PredictedObjects::ConstSharedPtr objects_ptr,
    const int closest_idx, const Polygon2d & stuck_vehicle_detect_area);

  /**
   * @brief Check if there is a stopped vehicle on the ego-lane.
   * @param lanelet_map_ptr lanelet map
   * @param path            ego-car lane
   * @param closest_idx     ego-car position on the lane
   * @param objects_ptr     target objects
   * @return true if exists
   */
  bool checkStuckVehicleInIntersection(
    const autoware_auto_perception_msgs::msg::PredictedObjects::ConstSharedPtr objects_ptr,
    const Polygon2d & stuck_vehicle_detect_area) const;

  /**
   * @brief Calculate the polygon of the path from the ego-car position to the end of the
   * intersection lanelet (+ extra distance).
   * @param lanelet_map_ptr lanelet map
   * @param path            ego-car lane
   * @param closest_idx     ego-car position index on the lane
   * @param extra_dist      extra distance from the end point of the intersection lanelet
   * @param ignore_dist     ignore distance from the start point of the ego-intersection lane
   * @return generated polygon
   */
  Polygon2d generateEgoIntersectionLanePolygon(
    lanelet::LaneletMapConstPtr lanelet_map_ptr,
    const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const int closest_idx,
    const int start_idx, const double extra_dist, const double ignore_dist) const;

  /**
   * @brief Modify objects predicted path. remove path point if the time exceeds timer_thr.
   * @param objects_ptr target objects
   * @param time_thr    time threshold to cut path
   */
  void cutPredictPathWithDuration(
    autoware_auto_perception_msgs::msg::PredictedObjects * objects_ptr,
    const double time_thr) const;

  /**
   * @brief Calculate time that is needed for ego-vehicle to cross the intersection. (to be updated)
   * @param path              ego-car lane
   * @param closest_idx       ego-car position index on the lane
   * @param objective_lane_id lanelet id on ego-car
   * @return calculated time [s]
   */
  TimeDistanceArray calcIntersectionPassingTime(
    const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const int closest_idx,
    const int objective_lane_id) const;

  /**
   * @brief check if the object has a target type for collision check
   * @param object target object
   * @return true if the object has a target type
   */
  bool isTargetCollisionVehicleType(
    const autoware_auto_perception_msgs::msg::PredictedObject & object) const;

  /**
   * @brief check if the object has a target type for stuck check
   * @param object target object
   * @return true if the object has a target type
   */
  bool isTargetStuckVehicleType(
    const autoware_auto_perception_msgs::msg::PredictedObject & object) const;

  /**
   * @brief Whether target tier4_api_msgs::Intersection::status is valid or not
   * @param target_status target tier4_api_msgs::Intersection::status
   * @return rue if the object has a target type
   */
  bool isTargetExternalInputStatus(const int target_status);

  /**
   * @brief Whether the given pose belongs to any target lanelet or not
   * @param pose pose to be checked
   * @param target_lanelet_ids id list of target lanelets
   * @param thresh_angle angle threshold considered to belong to a lanelet
   * @return true if the given pose belongs to any target lanelet
   */
  bool checkAngleForTargetLanelets(
    const geometry_msgs::msg::Pose & pose, const lanelet::ConstLanelets & target_lanelet_ids,
    const double margin = 0);

  /**
   * @brief Get lanes including ego lanelet and next lanelet
   * @param lanelet_map_ptr lanelet map
   * @param path            ego-car lane
   * @return ego lanelet and next lanelet
   */
  lanelet::ConstLanelets getEgoLaneWithNextLane(
    lanelet::LaneletMapConstPtr lanelet_map_ptr,
    const autoware_auto_planning_msgs::msg::PathWithLaneId & path) const;

  /**
   * @brief Calculate distance between closest path point and intersection lanelet along path
   * @param lanelet_map_ptr lanelet map
   * @param path            ego-car lane
   * @param closest_idx     closest path index
   * @return ego lanelet and next lanelet
   */
  double calcDistanceUntilIntersectionLanelet(
    lanelet::LaneletMapConstPtr lanelet_map_ptr,
    const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const size_t closest_idx) const;

  /**
   * @brief Check if the ego is expected to stop in the opposite lane if the front vehicle starts
   * deceleration from current velocity and stop befor the crosswalk. If the stop position of front
   * vehicle is in stuck area, and the position `ego_length` meter behind is in detection area,
   * return true
   * @param ego_poly Polygon of ego_with_next_lane
   * @param closest_lanelet
   * @return true if the ego is expected to stop in the opposite lane
   */
  bool checkFrontVehicleDeceleration(
    lanelet::ConstLanelets & ego_lane_with_next_lane, lanelet::ConstLanelet & closest_lanelet,
    const Polygon2d & stuck_vehicle_detect_area,
    const autoware_auto_perception_msgs::msg::PredictedObject & object) const;
  StateMachine state_machine_;  //! for state

  // Debug
  mutable DebugData debug_data_;
};
}  // namespace behavior_velocity_planner

#endif  // SCENE_MODULE__INTERSECTION__SCENE_INTERSECTION_HPP_
