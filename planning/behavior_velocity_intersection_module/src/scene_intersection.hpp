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
#include <behavior_velocity_planner_common/utilization/state_machine.hpp>
#include <motion_utils/marker/virtual_wall_marker_creator.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <std_msgs/msg/string.hpp>
#include <tier4_debug_msgs/msg/float64_multi_array_stamped.hpp>

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
      double attention_area_length;
      double attention_area_margin;
      double attention_area_angle_threshold;
      bool use_intersection_area;
      double default_stopline_margin;
      double stopline_overshoot_margin;
      double path_interpolation_ds;
      double max_accel;
      double max_jerk;
      double delay_response_time;
    } common;

    struct TurnDirection
    {
      bool left;
      bool right;
      bool straight;
    };

    struct StuckVehicle
    {
      TurnDirection turn_direction;
      bool use_stuck_stopline;
      double stuck_vehicle_detect_dist;
      double stuck_vehicle_velocity_threshold;
      bool disable_against_private_lane;
    } stuck_vehicle;

    struct YieldStuck
    {
      TurnDirection turn_direction;
      double distance_threshold;
    } yield_stuck;

    struct CollisionDetection
    {
      bool consider_wrong_direction_vehicle;
      double collision_detection_hold_time;
      double min_predicted_path_confidence;
      double keep_detection_velocity_threshold;
      struct VelocityProfile
      {
        bool use_upstream;
        double minimum_upstream_velocity;
        double default_velocity;
        double minimum_default_velocity;
      } velocity_profile;
      struct FullyPrioritized
      {
        double collision_start_margin_time;
        double collision_end_margin_time;
      } fully_prioritized;
      struct PartiallyPrioritized
      {
        double collision_start_margin_time;
        double collision_end_margin_time;
      } partially_prioritized;
      struct NotPrioritized
      {
        double collision_start_margin_time;
        double collision_end_margin_time;
      } not_prioritized;
      struct YieldOnGreeTrafficLight
      {
        double distance_to_assigned_lanelet_start;
        double duration;
        double object_dist_to_stopline;
      } yield_on_green_traffic_light;
      struct IgnoreOnAmberTrafficLight
      {
        double object_expected_deceleration;
      } ignore_on_amber_traffic_light;
      struct IgnoreOnRedTrafficLight
      {
        double object_margin_to_path;
      } ignore_on_red_traffic_light;
    } collision_detection;

    struct Occlusion
    {
      bool enable;
      double occlusion_attention_area_length;
      int free_space_max;
      int occupied_min;
      double denoise_kernel;
      double attention_lane_crop_curvature_threshold;
      double attention_lane_curvature_calculation_ds;
      struct CreepDuringPeeking
      {
        bool enable;
        double creep_velocity;
      } creep_during_peeking;
      double peeking_offset;
      double occlusion_required_clearance_distance;
      std::vector<double> possible_object_bbox;
      double ignore_parked_vehicle_speed_threshold;
      double occlusion_detection_hold_time;
      double temporal_stop_time_before_peeking;
      bool temporal_stop_before_attention_area;
      double creep_velocity_without_traffic_light;
      double static_occlusion_with_traffic_light_timeout;
    } occlusion;

    struct Debug
    {
      std::vector<int64_t> ttc;
    } debug;
  };

  enum OcclusionType {
    NOT_OCCLUDED,
    STATICALLY_OCCLUDED,
    DYNAMICALLY_OCCLUDED,
    RTC_OCCLUDED,  // actual occlusion does not exist, only disapproved by RTC
  };

  struct Indecisive
  {
    std::string error;
  };
  struct StuckStop
  {
    size_t closest_idx{0};
    size_t stuck_stopline_idx{0};
    std::optional<size_t> occlusion_stopline_idx{std::nullopt};
  };
  struct YieldStuckStop
  {
    size_t closest_idx{0};
    size_t stuck_stopline_idx{0};
  };
  struct NonOccludedCollisionStop
  {
    size_t closest_idx{0};
    size_t collision_stopline_idx{0};
    size_t occlusion_stopline_idx{0};
  };
  struct FirstWaitBeforeOcclusion
  {
    bool is_actually_occlusion_cleared{false};
    size_t closest_idx{0};
    size_t first_stopline_idx{0};
    size_t occlusion_stopline_idx{0};
  };
  // A state peeking to occlusion limit line in the presence of traffic light
  struct PeekingTowardOcclusion
  {
    // NOTE: if intersection_occlusion is disapproved externally through RTC,
    // it indicates "is_forcefully_occluded"
    bool is_actually_occlusion_cleared{false};
    bool temporal_stop_before_attention_required{false};
    size_t closest_idx{0};
    size_t collision_stopline_idx{0};
    size_t first_attention_stopline_idx{0};
    size_t occlusion_stopline_idx{0};
    // if null, it is dynamic occlusion and shows up intersection_occlusion(dyn)
    // if valid, it contains the remaining time to release the static occlusion stuck and shows up
    // intersection_occlusion(x.y)
    std::optional<double> static_occlusion_timeout{std::nullopt};
  };
  // A state detecting both collision and occlusion in the presence of traffic light
  struct OccludedCollisionStop
  {
    bool is_actually_occlusion_cleared{false};
    bool temporal_stop_before_attention_required{false};
    size_t closest_idx{0};
    size_t collision_stopline_idx{0};
    size_t first_attention_stopline_idx{0};
    size_t occlusion_stopline_idx{0};
    // if null, it is dynamic occlusion and shows up intersection_occlusion(dyn)
    // if valid, it contains the remaining time to release the static occlusion stuck
    std::optional<double> static_occlusion_timeout{std::nullopt};
  };
  struct OccludedAbsenceTrafficLight
  {
    bool is_actually_occlusion_cleared{false};
    bool collision_detected{false};
    bool temporal_stop_before_attention_required{false};
    size_t closest_idx{0};
    size_t first_attention_area_stopline_idx{0};
    size_t peeking_limit_line_idx{0};
  };
  struct Safe
  {
    // NOTE: if RTC is disapproved status, default stop lines are still needed.
    size_t closest_idx{0};
    size_t collision_stopline_idx{0};
    size_t occlusion_stopline_idx{0};
  };
  struct FullyPrioritized
  {
    bool collision_detected{false};
    size_t closest_idx{0};
    size_t collision_stopline_idx{0};
    size_t occlusion_stopline_idx{0};
  };
  using DecisionResult = std::variant<
    Indecisive,                   // internal process error, or over the pass judge line
    StuckStop,                    // detected stuck vehicle
    YieldStuckStop,               // detected yield stuck vehicle
    NonOccludedCollisionStop,     // detected collision while FOV is clear
    FirstWaitBeforeOcclusion,     // stop for a while before peeking to occlusion
    PeekingTowardOcclusion,       // peeking into occlusion while collision is not detected
    OccludedCollisionStop,        // occlusion and collision are both detected
    OccludedAbsenceTrafficLight,  // occlusion is detected in the absence of traffic light
    Safe,                         // judge as safe
    FullyPrioritized              // only detect vehicles violating traffic rules
    >;

  IntersectionModule(
    const int64_t module_id, const int64_t lane_id, std::shared_ptr<const PlannerData> planner_data,
    const PlannerParam & planner_param, const std::set<lanelet::Id> & associative_ids,
    const std::string & turn_direction, const bool has_traffic_light,
    const bool enable_occlusion_detection, rclcpp::Node & node, const rclcpp::Logger logger,
    const rclcpp::Clock::SharedPtr clock);

  /**
   * @brief plan go-stop velocity at traffic crossing with collision check between reference path
   * and object predicted path
   */
  bool modifyPathVelocity(PathWithLaneId * path, StopReason * stop_reason) override;

  visualization_msgs::msg::MarkerArray createDebugMarkerArray() override;
  motion_utils::VirtualWalls createVirtualWalls() override;

  const std::set<lanelet::Id> & getAssociativeIds() const { return associative_ids_; }

  UUID getOcclusionUUID() const { return occlusion_uuid_; }
  bool getOcclusionSafety() const { return occlusion_safety_; }
  double getOcclusionDistance() const { return occlusion_stop_distance_; }
  void setOcclusionActivation(const bool activation) { occlusion_activated_ = activation; }
  bool isOcclusionFirstStopRequired() { return occlusion_first_stop_required_; }

private:
  rclcpp::Node & node_;
  const int64_t lane_id_;
  const std::set<lanelet::Id> associative_ids_;
  const std::string turn_direction_;
  const bool has_traffic_light_;

  bool is_go_out_{false};
  bool is_permanent_go_{false};
  DecisionResult prev_decision_result_{Indecisive{""}};
  OcclusionType prev_occlusion_status_;

  // Parameter
  PlannerParam planner_param_;

  std::optional<util::IntersectionLanelets> intersection_lanelets_{std::nullopt};

  // for occlusion detection
  const bool enable_occlusion_detection_;
  std::optional<std::vector<lanelet::ConstLineString3d>> occlusion_attention_divisions_{
    std::nullopt};
  StateMachine collision_state_machine_;     //! for stable collision checking
  StateMachine before_creep_state_machine_;  //! for two phase stop
  StateMachine occlusion_stop_state_machine_;
  StateMachine temporal_stop_before_attention_state_machine_;
  StateMachine static_occlusion_timeout_state_machine_;

  // for pseudo-collision detection when ego just entered intersection on green light and upcoming
  // vehicles are very slow
  std::optional<rclcpp::Time> initial_green_light_observed_time_{std::nullopt};

  // for RTC
  const UUID occlusion_uuid_;
  bool occlusion_safety_{true};
  double occlusion_stop_distance_{0.0};
  bool occlusion_activated_{true};
  // for first stop in two-phase stop
  bool occlusion_first_stop_required_{false};

  void initializeRTCStatus();
  void prepareRTCStatus(
    const DecisionResult &, const autoware_auto_planning_msgs::msg::PathWithLaneId & path);

  DecisionResult modifyPathVelocityDetail(PathWithLaneId * path, StopReason * stop_reason);

  bool checkStuckVehicle(
    const std::shared_ptr<const PlannerData> & planner_data,
    const util::PathLanelets & path_lanelets);

  bool checkYieldStuckVehicle(
    const util::TargetObjects & target_objects,
    const util::InterpolatedPathInfo & interpolated_path_info,
    const lanelet::ConstLanelets & attention_lanelets);

  util::TargetObjects generateTargetObjects(
    const util::IntersectionLanelets & intersection_lanelets,
    const std::optional<Polygon2d> & intersection_area) const;

  bool checkCollision(
    const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
    util::TargetObjects * target_objects, const util::PathLanelets & path_lanelets,
    const size_t closest_idx, const size_t last_intersection_stopline_candidate_idx,
    const double time_delay, const util::TrafficPrioritizedLevel & traffic_prioritized_level);

  OcclusionType getOcclusionStatus(
    const nav_msgs::msg::OccupancyGrid & occ_grid,
    const std::vector<lanelet::CompoundPolygon3d> & attention_areas,
    const lanelet::ConstLanelets & adjacent_lanelets,
    const lanelet::CompoundPolygon3d & first_attention_area,
    const util::InterpolatedPathInfo & interpolated_path_info,
    const std::vector<lanelet::ConstLineString3d> & lane_divisions,
    const util::TargetObjects & target_objects, const geometry_msgs::msg::Pose & current_pose,
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
  rclcpp::Publisher<tier4_debug_msgs::msg::Float64MultiArrayStamped>::SharedPtr ego_ttc_pub_;
  rclcpp::Publisher<tier4_debug_msgs::msg::Float64MultiArrayStamped>::SharedPtr object_ttc_pub_;
};

}  // namespace behavior_velocity_planner

#endif  // SCENE_INTERSECTION_HPP_
