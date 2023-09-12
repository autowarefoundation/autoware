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

#ifndef SCENE_INTERSECTION_HPP_
#define SCENE_INTERSECTION_HPP_

#include "util_type.hpp"

#include <behavior_velocity_planner_common/scene_module_interface.hpp>
#include <behavior_velocity_planner_common/utilization/boost_geometry_helper.hpp>
#include <behavior_velocity_planner_common/utilization/state_machine.hpp>
#include <grid_map_core/grid_map_core.hpp>
#include <motion_utils/marker/virtual_wall_marker_creator.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <std_msgs/msg/string.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <algorithm>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace behavior_velocity_planner
{

using TimeDistanceArray = std::vector<std::pair<double /* time*/, double /* distance*/>>;

class IntersectionModule : public SceneModuleInterface
{
public:
  struct PlannerParam
  {
    struct Common
    {
      double attention_area_margin;     //! used for detecting objects in attention area
      double attention_area_length;     //! used to create attention area polygon
      double attention_area_angle_thr;  //! threshold in checking the angle of detecting objects
      double stop_line_margin;  //! distance from auto-generated stopline to detection_area boundary
      double intersection_velocity;  //! used for intersection passing time
      double intersection_max_acc;   //! used for calculating intersection velocity
      double stop_overshoot_margin;  //! overshoot margin for stuck, collision detection
      bool use_intersection_area;
      bool consider_wrong_direction_vehicle;
      double path_interpolation_ds;
    } common;
    struct StuckVehicle
    {
      bool use_stuck_stopline;  //! stopline generate before the intersection lanelet when is_stuck.
      double stuck_vehicle_detect_dist;  //! distance from end point to finish stuck vehicle check
      double stuck_vehicle_vel_thr;      //! Threshold of the speed to be recognized as stopped
      /*
      double
        assumed_front_car_decel;  //! the expected deceleration of front car when front car as well
      bool enable_front_car_decel_prediction;  //! flag for using above feature
      */
      double timeout_private_area;
    } stuck_vehicle;
    struct CollisionDetection
    {
      double min_predicted_path_confidence;
      //! minimum confidence value of predicted path to use for collision detection
      double minimum_ego_predicted_velocity;  //! used to calculate ego's future velocity profile
      double state_transit_margin_time;
      struct Normal
      {
        double collision_start_margin_time;  //! start margin time to check collision
        double collision_end_margin_time;    //! end margin time to check collision
      } normal;
      struct Relaxed
      {
        double collision_start_margin_time;
        double collision_end_margin_time;
      } relaxed;
      double keep_detection_vel_thr;  //! keep detection if ego is ego.vel < keep_detection_vel_thr
    } collision_detection;
    struct Occlusion
    {
      bool enable;
      double occlusion_attention_area_length;  //! used for occlusion detection
      bool enable_creeping;
      double occlusion_creep_velocity;  //! the creep velocity to occlusion limit stop line
      double peeking_offset;
      int free_space_max;
      int occupied_min;
      bool do_dp;
      double before_creep_stop_time;
      double min_vehicle_brake_for_rss;
      double max_vehicle_velocity_for_rss;
      double denoise_kernel;
      std::vector<double> possible_object_bbox;
      double ignore_parked_vehicle_speed_threshold;
      double stop_release_margin_time;
    } occlusion;
  };

  /*
  enum OcclusionState {
    NONE,
    BEFORE_FIRST_STOP_LINE,
    WAIT_FIRST_STOP_LINE,
    CREEP_SECOND_STOP_LINE,
    COLLISION_DETECTED,
  };
  */

  using Indecisive = std::monostate;
  struct StuckStop
  {
    size_t stop_line_idx;
    // NOTE: this is optional because stuck vehicle detection is possible
    // even if the detection area is empty.
    // Still this may be required for RTC's default stop line
    bool is_detection_area_empty;
    util::IntersectionStopLines stop_lines;
  };
  struct NonOccludedCollisionStop
  {
    size_t stop_line_idx;
    util::IntersectionStopLines stop_lines;
  };
  struct FirstWaitBeforeOcclusion
  {
    size_t first_stop_line_idx;
    size_t occlusion_stop_line_idx;
    bool is_actually_occlusion_cleared;
    util::IntersectionStopLines stop_lines;
  };
  struct PeekingTowardOcclusion
  {
    size_t stop_line_idx;
    // NOTE: if intersection_occlusion is disapproved externally through RTC,
    // it indicates "is_forcefully_occluded"
    bool is_actually_occlusion_cleared;
    util::IntersectionStopLines stop_lines;
  };
  struct OccludedCollisionStop
  {
    size_t stop_line_idx;
    size_t occlusion_stop_line_idx;
    bool is_actually_occlusion_cleared;
    util::IntersectionStopLines stop_lines;
  };
  struct Safe
  {
    // NOTE: if RTC is disapproved status, default stop lines are still needed.
    util::IntersectionStopLines stop_lines;
  };
  struct TrafficLightArrowSolidOn
  {
    bool collision_detected;
    util::IntersectionStopLines stop_lines;
  };
  using DecisionResult = std::variant<
    Indecisive,                // internal process error, or over the pass judge line
    StuckStop,                 // detected stuck vehicle
    NonOccludedCollisionStop,  // detected collision while FOV is clear
    FirstWaitBeforeOcclusion,  // stop for a while before peeking to occlusion
    PeekingTowardOcclusion,    // peeking into occlusion while collision is not detected
    OccludedCollisionStop,     // occlusion and collision are both detected
    Safe,                      // judge as safe
    TrafficLightArrowSolidOn   // only detect vehicles violating traffic rules
    >;

  IntersectionModule(
    const int64_t module_id, const int64_t lane_id, std::shared_ptr<const PlannerData> planner_data,
    const PlannerParam & planner_param, const std::set<int> & associative_ids,
    const bool is_private_area, const bool enable_occlusion_detection, rclcpp::Node & node,
    const rclcpp::Logger logger, const rclcpp::Clock::SharedPtr clock);

  /**
   * @brief plan go-stop velocity at traffic crossing with collision check between reference path
   * and object predicted path
   */
  bool modifyPathVelocity(PathWithLaneId * path, StopReason * stop_reason) override;

  visualization_msgs::msg::MarkerArray createDebugMarkerArray() override;
  motion_utils::VirtualWalls createVirtualWalls() override;

  const std::set<int> & getAssociativeIds() const { return associative_ids_; }

  UUID getOcclusionUUID() const { return occlusion_uuid_; }
  bool getOcclusionSafety() const { return occlusion_safety_; }
  double getOcclusionDistance() const { return occlusion_stop_distance_; }
  void setOcclusionActivation(const bool activation) { occlusion_activated_ = activation; }
  bool isOcclusionFirstStopRequired() { return occlusion_first_stop_required_; }

private:
  rclcpp::Node & node_;
  const int64_t lane_id_;
  const std::set<int> associative_ids_;
  std::string turn_direction_;
  bool is_go_out_ = false;
  bool is_permanent_go_ = false;
  bool is_peeking_ = false;
  // Parameter
  PlannerParam planner_param_;
  std::optional<util::IntersectionLanelets> intersection_lanelets_;
  // for an intersection lane, its associative lanes are those that share same parent lanelet and
  // have same turn_direction

  // for occlusion detection
  const bool enable_occlusion_detection_;
  std::optional<std::vector<util::DiscretizedLane>> occlusion_attention_divisions_;
  // OcclusionState prev_occlusion_state_ = OcclusionState::NONE;
  StateMachine collision_state_machine_;     //! for stable collision checking
  StateMachine before_creep_state_machine_;  //! for two phase stop
  StateMachine occlusion_stop_state_machine_;
  // NOTE: uuid_ is base member

  // for stuck vehicle detection
  const bool is_private_area_;
  StateMachine stuck_private_area_timeout_;

  // for RTC
  const UUID occlusion_uuid_;
  bool occlusion_safety_ = true;
  double occlusion_stop_distance_;
  bool occlusion_activated_ = true;
  // for first stop in two-phase stop
  const UUID occlusion_first_stop_uuid_;
  bool occlusion_first_stop_required_ = false;

  void initializeRTCStatus();
  void prepareRTCStatus(
    const DecisionResult &, const autoware_auto_planning_msgs::msg::PathWithLaneId & path);

  DecisionResult modifyPathVelocityDetail(PathWithLaneId * path, StopReason * stop_reason);

  bool checkStuckVehicle(
    const std::shared_ptr<const PlannerData> & planner_data,
    const util::PathLanelets & path_lanelets,
    const util::InterpolatedPathInfo & interpolated_path_info,
    const util::IntersectionStopLines & intersection_stop_lines);

  autoware_auto_perception_msgs::msg::PredictedObjects filterTargetObjects(
    const lanelet::ConstLanelets & attention_area_lanelets,
    const lanelet::ConstLanelets & adjacent_lanelets,
    const std::optional<Polygon2d> & intersection_area) const;

  bool checkCollision(
    const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
    const autoware_auto_perception_msgs::msg::PredictedObjects & target_objects,
    const util::PathLanelets & path_lanelets, const int closest_idx, const double time_delay,
    const bool tl_arrow_solid_on);

  bool isOcclusionCleared(
    const nav_msgs::msg::OccupancyGrid & occ_grid,
    const std::vector<lanelet::CompoundPolygon3d> & attention_areas,
    const lanelet::ConstLanelets & adjacent_lanelets,
    const lanelet::CompoundPolygon3d & first_attention_area,
    const util::InterpolatedPathInfo & interpolated_path_info,
    const std::vector<util::DiscretizedLane> & lane_divisions,
    const std::vector<autoware_auto_perception_msgs::msg::PredictedObject> &
      parked_attention_objects,
    const double occlusion_dist_thr);

  /*
  bool IntersectionModule::checkFrontVehicleDeceleration(
    lanelet::ConstLanelets & ego_lane_with_next_lane, lanelet::ConstLanelet & closest_lanelet,
    const Polygon2d & stuck_vehicle_detect_area,
    const autoware_auto_perception_msgs::msg::PredictedObject & object,
    const double assumed_front_car_decel);
  */

  util::DebugData debug_data_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr decision_state_pub_;
};

}  // namespace behavior_velocity_planner

#endif  // SCENE_INTERSECTION_HPP_
