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

#include "decision_result.hpp"
#include "interpolated_path_info.hpp"
#include "intersection_lanelets.hpp"
#include "intersection_stoplines.hpp"
#include "result.hpp"

#include <behavior_velocity_planner_common/scene_module_interface.hpp>
#include <behavior_velocity_planner_common/utilization/state_machine.hpp>
#include <motion_utils/marker/virtual_wall_marker_creator.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <std_msgs/msg/string.hpp>
#include <tier4_debug_msgs/msg/float64_multi_array_stamped.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_routing/Forward.h>

#include <memory>
#include <optional>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace behavior_velocity_planner
{

/**
 * @struct
 * @brief see the document for more details of IntersectionStopLines
 */
struct TargetObject
{
  autoware_auto_perception_msgs::msg::PredictedObject object;
  std::optional<lanelet::ConstLanelet> attention_lanelet{std::nullopt};
  std::optional<lanelet::ConstLineString3d> stopline{std::nullopt};
  std::optional<double> dist_to_stopline{std::nullopt};
  void calc_dist_to_stopline();
};

/**
 * @struct
 * @brief categorize TargetObjects
 */
struct TargetObjects
{
  std_msgs::msg::Header header;
  std::vector<TargetObject> attention_objects;
  std::vector<TargetObject> parked_attention_objects;
  std::vector<TargetObject> intersection_area_objects;
  std::vector<TargetObject> all_attention_objects;  // TODO(Mamoru Sobue): avoid copy
};

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
      bool enable_pass_judge_before_default_stopline;
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
    //! occlusion is not detected
    NOT_OCCLUDED,
    //! occlusion is not caused by dynamic objects
    STATICALLY_OCCLUDED,
    //! occlusion is caused by dynamic objects
    DYNAMICALLY_OCCLUDED,
    //! actual occlusion does not exist, only disapproved by RTC
    RTC_OCCLUDED,
  };

  struct DebugData
  {
    std::optional<geometry_msgs::msg::Pose> collision_stop_wall_pose{std::nullopt};
    std::optional<geometry_msgs::msg::Pose> occlusion_stop_wall_pose{std::nullopt};
    std::optional<geometry_msgs::msg::Pose> occlusion_first_stop_wall_pose{std::nullopt};
    std::optional<geometry_msgs::msg::Pose> first_pass_judge_wall_pose{std::nullopt};
    bool passed_first_pass_judge{false};
    bool passed_second_pass_judge{false};
    std::optional<geometry_msgs::msg::Pose> second_pass_judge_wall_pose{std::nullopt};
    std::optional<std::vector<lanelet::CompoundPolygon3d>> attention_area{std::nullopt};
    std::optional<std::vector<lanelet::CompoundPolygon3d>> occlusion_attention_area{std::nullopt};
    std::optional<lanelet::CompoundPolygon3d> ego_lane{std::nullopt};
    std::optional<std::vector<lanelet::CompoundPolygon3d>> adjacent_area{std::nullopt};
    std::optional<lanelet::CompoundPolygon3d> first_attention_area{std::nullopt};
    std::optional<lanelet::CompoundPolygon3d> second_attention_area{std::nullopt};
    std::optional<geometry_msgs::msg::Polygon> stuck_vehicle_detect_area{std::nullopt};
    std::optional<std::vector<lanelet::CompoundPolygon3d>> yield_stuck_detect_area{std::nullopt};
    std::optional<geometry_msgs::msg::Polygon> candidate_collision_ego_lane_polygon{std::nullopt};
    std::vector<geometry_msgs::msg::Polygon> candidate_collision_object_polygons;
    autoware_auto_perception_msgs::msg::PredictedObjects conflicting_targets;
    autoware_auto_perception_msgs::msg::PredictedObjects amber_ignore_targets;
    autoware_auto_perception_msgs::msg::PredictedObjects red_overshoot_ignore_targets;
    autoware_auto_perception_msgs::msg::PredictedObjects stuck_targets;
    autoware_auto_perception_msgs::msg::PredictedObjects yield_stuck_targets;
    std::vector<geometry_msgs::msg::Polygon> occlusion_polygons;
    std::optional<std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point>>
      nearest_occlusion_projection{std::nullopt};
    autoware_auto_perception_msgs::msg::PredictedObjects blocking_attention_objects;
    std::optional<geometry_msgs::msg::Pose> absence_traffic_light_creep_wall{std::nullopt};
    std::optional<double> static_occlusion_with_traffic_light_timeout{std::nullopt};
  };

  using TimeDistanceArray = std::vector<std::pair<double /* time*/, double /* distance*/>>;

  /**
   * @struct
   * @brief categorize traffic light priority
   */
  enum class TrafficPrioritizedLevel {
    //! The target lane's traffic signal is red or the ego's traffic signal has an arrow.
    FULLY_PRIORITIZED = 0,
    //! The target lane's traffic signal is amber
    PARTIALLY_PRIORITIZED,
    //! The target lane's traffic signal is green
    NOT_PRIORITIZED
  };
  /** @} */

  IntersectionModule(
    const int64_t module_id, const int64_t lane_id, std::shared_ptr<const PlannerData> planner_data,
    const PlannerParam & planner_param, const std::set<lanelet::Id> & associative_ids,
    const std::string & turn_direction, const bool has_traffic_light, rclcpp::Node & node,
    const rclcpp::Logger logger, const rclcpp::Clock::SharedPtr clock);

  /**
   ***********************************************************
   ***********************************************************
   ***********************************************************
   * @defgroup primary-function [fn] primary functions
   * the entrypoint of this module is modifyPathVelocity() function that calculates safety decision
   * of latest context and send it to RTC and then react to RTC approval. The reaction to RTC
   * approval may not be based on the latest decision of this module depending on the auto-mode
   * configuration. For module side it is not visible if the module is operating in auto-mode or
   * manual-module. At first, initializeRTCStatus() is called to reset the safety value of
   * INTERSECTION and INTERSECTION_OCCLUSION. Then modifyPathVelocityDetail() is called to analyze
   * the context. Then prepareRTCStatus() is called to set the safety value of INTERSECTION and
   * INTERSECTION_OCCLUSION.
   * @{
   */
  bool modifyPathVelocity(PathWithLaneId * path, StopReason * stop_reason) override;
  /** @}*/

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

  /**
   ***********************************************************
   ***********************************************************
   ***********************************************************
   * @defgroup const-variables [var] const variables
   * following variables are unique to this intersection lanelet or to this module
   * @{
   */
  //! lanelet of this intersection
  const lanelet::Id lane_id_;

  //! associative(sibling) lanelets ids
  const std::set<lanelet::Id> associative_ids_;

  //! turn_direction of this lane
  const std::string turn_direction_;

  //! flag if this intersection is traffic controlled
  const bool has_traffic_light_;

  //! RTC uuid for INTERSECTION_OCCLUSION
  const UUID occlusion_uuid_;
  /** @}*/

private:
  /**
   ***********************************************************
   ***********************************************************
   ***********************************************************
   * @defgroup semi-const-variables [var] semi-const variables
   * following variables are immutable once initialized
   * @{
   */
  PlannerParam planner_param_;

  //! cache IntersectionLanelets struct
  std::optional<intersection::IntersectionLanelets> intersection_lanelets_{std::nullopt};

  //! cache discretized occlusion detection lanelets
  std::optional<std::vector<lanelet::ConstLineString3d>> occlusion_attention_divisions_{
    std::nullopt};
  /** @}*/

private:
  /**
   ***********************************************************
   ***********************************************************
   ***********************************************************
   * @defgroup pass-judge-variable [var] pass judge variables
   * following variables are state variables that depends on how the vehicle passed the intersection
   * @{
   */
  //! if true, this module never commands to STOP anymore
  bool is_permanent_go_{false};

  //! for checking if ego is over the pass judge lines because previously the situation was SAFE
  intersection::DecisionResult prev_decision_result_{intersection::Indecisive{""}};

  //! flag if ego passed the 1st_pass_judge_line while peeking. If this is true, 1st_pass_judge_line
  //! is treated as the same position as occlusion_peeking_stopline
  bool passed_1st_judge_line_while_peeking_{false};

  //! save the time when ego passed the 1st/2nd_pass_judge_line with safe decision. If collision is
  //! expected after these variables are non-null, then it is the fault of past perception failure
  //! at these time.
  std::optional<rclcpp::Time> safely_passed_1st_judge_line_time_{std::nullopt};
  std::optional<rclcpp::Time> safely_passed_2nd_judge_line_time_{std::nullopt};
  /** @}*/

private:
  /**
   ***********************************************************
   ***********************************************************
   ***********************************************************
   * @defgroup collision-variables [var] collision detection
   * @{
   */
  //! debouncing for stable SAFE decision
  StateMachine collision_state_machine_;
  /** @} */

private:
  /**
   ***********************************************************
   ***********************************************************
   ***********************************************************
   * @defgroup occlusion-variables [var] occlusion detection variables
   * @{
   */
  OcclusionType prev_occlusion_status_;

  //! debouncing for the first brief stop at the default stopline
  StateMachine before_creep_state_machine_;

  //! debouncing for stable CLEARED decision
  StateMachine occlusion_stop_state_machine_;

  //! debouncing for the brief stop at the boundary of attention area(if required by the flag)
  StateMachine temporal_stop_before_attention_state_machine_;

  //! time counter for the stuck detection due to occlusion caused static objects
  StateMachine static_occlusion_timeout_state_machine_;
  /** @} */

  std::optional<rclcpp::Time> initial_green_light_observed_time_{std::nullopt};

private:
  /**
   ***********************************************************
   ***********************************************************
   ***********************************************************
   * @defgroup RTC-variables [var] RTC variables
   *
   * intersection module has additional rtc_interface_ for INTERSECTION_OCCLUSION in addition to the
   * default rtc_interface of SceneModuleManagerInterfaceWithRTC. activated_ is the derived member
   * of this module which is updated by the RTC config/service, so it should be read-only in this
   * module. occlusion_safety_ and occlusion_stop_distance_ are the corresponding RTC value for
   * INTERSECTION_OCCLUSION.
   * @{
   */
  bool occlusion_safety_{true};
  double occlusion_stop_distance_{0.0};
  bool occlusion_activated_{true};
  bool occlusion_first_stop_required_{false};
  /** @}*/

private:
  /**
   ***********************************************************
   ***********************************************************
   ***********************************************************
   * @ingroup primary-functions
   * @{
   */
  /**
   * @brief set all RTC variable to true(safe) and -INF
   */
  void initializeRTCStatus();

  /**
   * @brief analyze traffic_light/occupancy/objects context and return DecisionResult
   */
  intersection::DecisionResult modifyPathVelocityDetail(
    PathWithLaneId * path, StopReason * stop_reason);

  /**
   * @brief set RTC value according to calculated DecisionResult
   */
  void prepareRTCStatus(
    const intersection::DecisionResult &,
    const autoware_auto_planning_msgs::msg::PathWithLaneId & path);

  /**
   * @brief act based on current RTC approval
   */
  void reactRTCApproval(
    const intersection::DecisionResult & decision_result,
    autoware_auto_planning_msgs::msg::PathWithLaneId * path, StopReason * stop_reason);
  /** @}*/

private:
  /**
   ***********************************************************
   ***********************************************************
   ***********************************************************
   * @defgroup prepare-data [fn] basic data construction
   * @{
   */
  /**
   * @struct
   */
  struct BasicData
  {
    intersection::InterpolatedPathInfo interpolated_path_info;
    intersection::IntersectionStopLines intersection_stoplines;
    intersection::PathLanelets path_lanelets;
  };

  /**
   * @brief prepare basic data structure
   * @return return IntersectionStopLines if all data is valid, otherwise Indecisive
   * @note if successful, it is ensure that intersection_lanelets_,
   * intersection_lanelets.first_conflicting_lane are not null
   *
   * To simplify modifyPathVelocityDetail(), this function is used at first
   */
  intersection::Result<BasicData, intersection::Indecisive> prepareIntersectionData(
    const bool is_prioritized, PathWithLaneId * path);

  /**
   * @brief find the associated stopline road marking of assigned lanelet
   */
  std::optional<size_t> getStopLineIndexFromMap(
    const intersection::InterpolatedPathInfo & interpolated_path_info,
    lanelet::ConstLanelet assigned_lanelet);

  /**
   * @brief generate IntersectionStopLines
   */
  std::optional<intersection::IntersectionStopLines> generateIntersectionStopLines(
    lanelet::ConstLanelet assigned_lanelet,
    const lanelet::CompoundPolygon3d & first_conflicting_area,
    const lanelet::ConstLanelet & first_attention_lane,
    const std::optional<lanelet::CompoundPolygon3d> & second_attention_area_opt,
    const intersection::InterpolatedPathInfo & interpolated_path_info,
    autoware_auto_planning_msgs::msg::PathWithLaneId * original_path);

  /**
   * @brief generate IntersectionLanelets
   */
  intersection::IntersectionLanelets generateObjectiveLanelets(
    lanelet::LaneletMapConstPtr lanelet_map_ptr,
    lanelet::routing::RoutingGraphPtr routing_graph_ptr,
    const lanelet::ConstLanelet assigned_lanelet);

  /**
   * @brief generate PathLanelets
   */
  std::optional<intersection::PathLanelets> generatePathLanelets(
    const lanelet::ConstLanelets & lanelets_on_path,
    const intersection::InterpolatedPathInfo & interpolated_path_info,
    const lanelet::CompoundPolygon3d & first_conflicting_area,
    const std::vector<lanelet::CompoundPolygon3d> & conflicting_areas,
    const std::optional<lanelet::CompoundPolygon3d> & first_attention_area,
    const std::vector<lanelet::CompoundPolygon3d> & attention_areas, const size_t closest_idx);

  /**
   * @brief generate discretized detection lane linestring.
   */
  std::vector<lanelet::ConstLineString3d> generateDetectionLaneDivisions(
    lanelet::ConstLanelets detection_lanelets,
    const lanelet::routing::RoutingGraphPtr routing_graph_ptr, const double resolution);
  /** @} */

private:
  /**
   * @defgroup utility [fn] utility member function
   * @{
   */
  void stoppedAtPositionForDuration(
    const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const size_t position,
    const double duration, StateMachine * state_machine);
  /** @} */

private:
  /**
   ***********************************************************
   ***********************************************************
   ***********************************************************
   * @defgroup get-traffic-light [fn] traffic light
   * @{
   */
  /**
   * @brief check if associated traffic light is green
   */
  bool isGreenSolidOn() const;

  /**
   * @brief find TrafficPrioritizedLevel
   */
  TrafficPrioritizedLevel getTrafficPrioritizedLevel() const;
  /** @} */

private:
  /**
   * @brief categorize target objects
   */
  TargetObjects generateTargetObjects(
    const intersection::IntersectionLanelets & intersection_lanelets,
    const std::optional<Polygon2d> & intersection_area) const;

private:
  /**
   ***********************************************************
   ***********************************************************
   ***********************************************************
   * @defgroup yield [fn] check stuck
   * @{
   */
  /**
   * @brief check stuck status
   * @attention this function has access to value() of intersection_lanelets_,
   * intersection_lanelets.first_conflicting_lane(). They are ensured in prepareIntersectionData()
   */
  std::optional<intersection::StuckStop> isStuckStatus(
    const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
    const intersection::IntersectionStopLines & intersection_stoplines,
    const intersection::PathLanelets & path_lanelets) const;

  bool isTargetStuckVehicleType(
    const autoware_auto_perception_msgs::msg::PredictedObject & object) const;

  /**
   * @brief check stuck
   */
  bool checkStuckVehicleInIntersection(const intersection::PathLanelets & path_lanelets) const;
  /** @} */

private:
  /**
   ***********************************************************
   ***********************************************************
   ***********************************************************
   * @defgroup yield [fn] check yield stuck
   * @{
   */
  /**
   * @brief check yield stuck status
   * @attention this function has access to value() of intersection_lanelets_,
   * intersection_stoplines.default_stopline, intersection_stoplines.first_attention_stopline
   */
  std::optional<intersection::YieldStuckStop> isYieldStuckStatus(
    const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
    const intersection::InterpolatedPathInfo & interpolated_path_info,
    const intersection::IntersectionStopLines & intersection_stoplines,
    const TargetObjects & target_objects) const;

  /**
   * @brief check yield stuck
   */
  bool checkYieldStuckVehicleInIntersection(
    const TargetObjects & target_objects,
    const intersection::InterpolatedPathInfo & interpolated_path_info,
    const lanelet::ConstLanelets & attention_lanelets) const;
  /** @} */

private:
  /**
   ***********************************************************
   ***********************************************************
   ***********************************************************
   * @defgroup occlusion [fn] check occlusion
   * @{
   */
  /**
   * @brief check occlusion status
   * @attention this function has access to value() of occlusion_attention_divisions_,
   * intersection_lanelets.first_attention_area()
   */
  std::tuple<
    OcclusionType, bool /* module detection with margin */,
    bool /* reconciled occlusion disapproval */>
  getOcclusionStatus(
    const TrafficPrioritizedLevel & traffic_prioritized_level,
    const intersection::InterpolatedPathInfo & interpolated_path_info,
    const intersection::IntersectionLanelets & intersection_lanelets,
    const TargetObjects & target_objects);

  /**
   * @brief calculate detected occlusion status(NOT | STATICALLY | DYNAMICALLY)
   */
  OcclusionType detectOcclusion(
    const std::vector<lanelet::CompoundPolygon3d> & attention_areas,
    const lanelet::ConstLanelets & adjacent_lanelets,
    const lanelet::CompoundPolygon3d & first_attention_area,
    const intersection::InterpolatedPathInfo & interpolated_path_info,
    const std::vector<lanelet::ConstLineString3d> & lane_divisions,
    const TargetObjects & target_objects);
  /** @} */

private:
  /**
   ***********************************************************
   ***********************************************************
   ***********************************************************
   * @defgroup pass-judge-decision [fn] pass judge decision
   * @{
   */
  /**
   * @brief check if ego is already over the pass judge line
   * @return if ego is over both 1st/2nd pass judge lines, return Indecisive, else return
   * (is_over_1st_pass_judge, is_over_2nd_pass_judge)
   * @attention this function has access to value() of intersection_stoplines.default_stopline,
   * intersection_stoplines.occlusion_stopline
   */
  intersection::Result<
    intersection::Indecisive,
    std::pair<bool /* is_over_1st_pass_judge */, bool /* is_over_2nd_pass_judge */>>
  isOverPassJudgeLinesStatus(
    const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const bool is_occlusion_state,
    const intersection::IntersectionStopLines & intersection_stoplines);
  /** @} */

private:
  /**
   ***********************************************************
   ***********************************************************
   ***********************************************************
   * @defgroup collision-detection [fn] check collision
   * @{
   */
  bool isTargetCollisionVehicleType(
    const autoware_auto_perception_msgs::msg::PredictedObject & object) const;

  /**
   * @brief check if there are any objects around the stoplines on the attention areas when ego
   * entered the intersection on green light
   * @return return NonOccludedCollisionStop if there are vehicle within the margin for some
   * duration from ego's entry to yield
   * @attention this function has access to value() of
   * intersection_stoplines.occlusion_peeking_stopline
   */
  std::optional<intersection::NonOccludedCollisionStop> isGreenPseudoCollisionStatus(
    const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
    const size_t collision_stopline_idx,
    const intersection::IntersectionStopLines & intersection_stoplines,
    const TargetObjects & target_objects);

  /**
   * @brief check collision
   */
  bool checkCollision(
    const autoware_auto_planning_msgs::msg::PathWithLaneId & path, TargetObjects * target_objects,
    const intersection::PathLanelets & path_lanelets, const size_t closest_idx,
    const size_t last_intersection_stopline_candidate_idx, const double time_delay,
    const TrafficPrioritizedLevel & traffic_prioritized_level);

  std::optional<size_t> checkAngleForTargetLanelets(
    const geometry_msgs::msg::Pose & pose, const lanelet::ConstLanelets & target_lanelets,
    const bool is_parked_vehicle) const;

  void cutPredictPathWithDuration(TargetObjects * target_objects, const double time_thr);

  /**
   * @brief calculate ego vehicle profile along the path inside the intersection as the sequence of
   * (time of arrival, traveled distance) from current ego position
   */
  TimeDistanceArray calcIntersectionPassingTime(
    const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const size_t closest_idx,
    const size_t last_intersection_stopline_candidate_idx, const double time_delay,
    tier4_debug_msgs::msg::Float64MultiArrayStamped * debug_ttc_array);
  /** @} */

  /*
  bool IntersectionModule::checkFrontVehicleDeceleration(
    lanelet::ConstLanelets & ego_lane_with_next_lane, lanelet::ConstLanelet & closest_lanelet,
    const Polygon2d & stuck_vehicle_detect_area,
    const autoware_auto_perception_msgs::msg::PredictedObject & object,
    const double assumed_front_car_decel);
  */

  mutable DebugData debug_data_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr decision_state_pub_;
  rclcpp::Publisher<tier4_debug_msgs::msg::Float64MultiArrayStamped>::SharedPtr ego_ttc_pub_;
  rclcpp::Publisher<tier4_debug_msgs::msg::Float64MultiArrayStamped>::SharedPtr object_ttc_pub_;
};

}  // namespace behavior_velocity_planner

#endif  // SCENE_INTERSECTION_HPP_
