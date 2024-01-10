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
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace behavior_velocity_planner
{

using TimeDistanceArray = std::vector<std::pair<double /* time*/, double /* distance*/>>;

struct DebugData
{
  std::optional<geometry_msgs::msg::Pose> collision_stop_wall_pose{std::nullopt};
  std::optional<geometry_msgs::msg::Pose> occlusion_stop_wall_pose{std::nullopt};
  std::optional<geometry_msgs::msg::Pose> occlusion_first_stop_wall_pose{std::nullopt};
  std::optional<geometry_msgs::msg::Pose> pass_judge_wall_pose{std::nullopt};
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

/**
 * @struct
 * @brief see the document for more details of IntersectionLanelets
 */
struct IntersectionLanelets
{
public:
  /**
   * update conflicting lanelets and traffic priority information
   */
  void update(
    const bool is_prioritized, const util::InterpolatedPathInfo & interpolated_path_info,
    const tier4_autoware_utils::LinearRing2d & footprint, const double vehicle_length,
    lanelet::routing::RoutingGraphPtr routing_graph_ptr);

  const lanelet::ConstLanelets & attention() const
  {
    return is_prioritized_ ? attention_non_preceding_ : attention_;
  }
  const std::vector<std::optional<lanelet::ConstLineString3d>> & attention_stoplines() const
  {
    return is_prioritized_ ? attention_non_preceding_stoplines_ : attention_stoplines_;
  }
  const lanelet::ConstLanelets & conflicting() const { return conflicting_; }
  const lanelet::ConstLanelets & adjacent() const { return adjacent_; }
  const lanelet::ConstLanelets & occlusion_attention() const
  {
    return is_prioritized_ ? attention_non_preceding_ : occlusion_attention_;
  }
  const lanelet::ConstLanelets & attention_non_preceding() const
  {
    return attention_non_preceding_;
  }
  const std::vector<lanelet::CompoundPolygon3d> & attention_area() const
  {
    return is_prioritized_ ? attention_non_preceding_area_ : attention_area_;
  }
  const std::vector<lanelet::CompoundPolygon3d> & conflicting_area() const
  {
    return conflicting_area_;
  }
  const std::vector<lanelet::CompoundPolygon3d> & adjacent_area() const { return adjacent_area_; }
  const std::vector<lanelet::CompoundPolygon3d> & occlusion_attention_area() const
  {
    return occlusion_attention_area_;
  }
  const std::optional<lanelet::ConstLanelet> & first_conflicting_lane() const
  {
    return first_conflicting_lane_;
  }
  const std::optional<lanelet::CompoundPolygon3d> & first_conflicting_area() const
  {
    return first_conflicting_area_;
  }
  const std::optional<lanelet::ConstLanelet> & first_attention_lane() const
  {
    return first_attention_lane_;
  }
  const std::optional<lanelet::CompoundPolygon3d> & first_attention_area() const
  {
    return first_attention_area_;
  }
  const std::optional<lanelet::ConstLanelet> & second_attention_lane() const
  {
    return second_attention_lane_;
  }
  const std::optional<lanelet::CompoundPolygon3d> & second_attention_area() const
  {
    return second_attention_area_;
  }

  /**
   * the set of attention lanelets which is topologically merged
   */
  lanelet::ConstLanelets attention_;
  std::vector<lanelet::CompoundPolygon3d> attention_area_;

  /**
   * the stop lines for each attention_ lanelets
   */
  std::vector<std::optional<lanelet::ConstLineString3d>> attention_stoplines_;

  /**
   * the conflicting part of attention lanelets
   */
  lanelet::ConstLanelets attention_non_preceding_;
  std::vector<lanelet::CompoundPolygon3d> attention_non_preceding_area_;

  /**
   * the stop lines for each attention_non_preceding_
   */
  std::vector<std::optional<lanelet::ConstLineString3d>> attention_non_preceding_stoplines_;

  /**
   * the conflicting lanelets of the objective intersection lanelet
   */
  lanelet::ConstLanelets conflicting_;
  std::vector<lanelet::CompoundPolygon3d> conflicting_area_;

  /**
   *
   */
  lanelet::ConstLanelets adjacent_;
  std::vector<lanelet::CompoundPolygon3d> adjacent_area_;

  /**
   * the set of attention lanelets for occlusion detection which is topologically merged
   */
  lanelet::ConstLanelets occlusion_attention_;
  std::vector<lanelet::CompoundPolygon3d> occlusion_attention_area_;

  /**
   * the vector of sum of each occlusion_attention lanelet
   */
  std::vector<double> occlusion_attention_size_;

  /**
   * the first conflicting lanelet which ego path points intersect for the first time
   */
  std::optional<lanelet::ConstLanelet> first_conflicting_lane_{std::nullopt};
  std::optional<lanelet::CompoundPolygon3d> first_conflicting_area_{std::nullopt};

  /**
   * the first attention lanelet which ego path points intersect for the first time
   */
  std::optional<lanelet::ConstLanelet> first_attention_lane_{std::nullopt};
  std::optional<lanelet::CompoundPolygon3d> first_attention_area_{std::nullopt};

  /**
   * the second attention lanelet which ego path points intersect next to the
   * first_attention_lanelet
   */
  bool second_attention_lane_empty_{false};
  std::optional<lanelet::ConstLanelet> second_attention_lane_{std::nullopt};
  std::optional<lanelet::CompoundPolygon3d> second_attention_area_{std::nullopt};

  /**
   * flag if the intersection is prioritized by the traffic light
   */
  bool is_prioritized_{false};
};

/**
 * @struct
 * @brief see the document for more details of IntersectionStopLines
 */
struct IntersectionStopLines
{
  size_t closest_idx{0};

  /**
   * stuck_stopline is null if ego path does not intersect with first_conflicting_area
   */
  std::optional<size_t> stuck_stopline{std::nullopt};

  /**
   * default_stopline is null if it is calculated negative from first_attention_stopline
   */
  std::optional<size_t> default_stopline{std::nullopt};

  /**
   * first_attention_stopline is null if ego footprint along the path does not intersect with
   * attention area. if path[0] satisfies the condition, it is 0
   */
  std::optional<size_t> first_attention_stopline{std::nullopt};

  /**
   * second_attention_stopline is null if ego footprint along the path does not intersect with
   * second_attention_lane. if path[0] satisfies the condition, it is 0
   */
  std::optional<size_t> second_attention_stopline{std::nullopt};

  /**
   * occlusion_peeking_stopline is null if path[0] is already inside the attention area
   */
  std::optional<size_t> occlusion_peeking_stopline{std::nullopt};

  /**
   * first_pass_judge_line is before first_attention_stopline by the braking distance. if its value
   * is calculated negative, it is 0
   */
  size_t first_pass_judge_line{0};

  /**
   * second_pass_judge_line is before second_attention_stopline by the braking distance. if its
   * value is calculated negative, it is 0
   */
  size_t second_pass_judge_line{0};

  /**
   * occlusion_wo_tl_pass_judge_line is null if ego footprint along the path does not intersect with
   * the centerline of the first_attention_lane
   */
  size_t occlusion_wo_tl_pass_judge_line{0};
};

/**
 * @struct
 * @brief see the document for more details of PathLanelets
 */
struct PathLanelets
{
  lanelet::ConstLanelets prev;
  // lanelet::ConstLanelet entry2ego; this is included in `all` if exists
  lanelet::ConstLanelet
    ego_or_entry2exit;  // this is `assigned lane` part of the path(not from
                        // ego) if ego is before the intersection, otherwise from ego to exit
  std::optional<lanelet::ConstLanelet> next =
    std::nullopt;  // this is nullopt is the goal is inside intersection
  lanelet::ConstLanelets all;
  lanelet::ConstLanelets
    conflicting_interval_and_remaining;  // the left/right-most interval of path conflicting with
                                         // conflicting lanelets plus the next lane part of the
                                         // path
};

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
    //! occlusion is not detected
    NOT_OCCLUDED,
    //! occlusion is not caused by dynamic objects
    STATICALLY_OCCLUDED,
    //! occlusion is caused by dynamic objects
    DYNAMICALLY_OCCLUDED,
    //! actual occlusion does not exist, only disapproved by RTC
    RTC_OCCLUDED,
  };

  /**
   * @struct
   * @brief Internal error or ego already passed pass_judge_line
   */
  struct Indecisive
  {
    std::string error;
  };
  /**
   * @struct
   * @brief detected stuck vehicle
   */
  struct StuckStop
  {
    size_t closest_idx{0};
    size_t stuck_stopline_idx{0};
    std::optional<size_t> occlusion_stopline_idx{std::nullopt};
  };
  /**
   * @struct
   * @brief yielded by vehicle on the attention area
   */
  struct YieldStuckStop
  {
    size_t closest_idx{0};
    size_t stuck_stopline_idx{0};
  };
  /**
   * @struct
   * @brief only collision is detected
   */
  struct NonOccludedCollisionStop
  {
    size_t closest_idx{0};
    size_t collision_stopline_idx{0};
    size_t occlusion_stopline_idx{0};
  };
  /**
   * @struct
   * @brief occlusion is detected so ego needs to stop at the default stop line position
   */
  struct FirstWaitBeforeOcclusion
  {
    bool is_actually_occlusion_cleared{false};
    size_t closest_idx{0};
    size_t first_stopline_idx{0};
    size_t occlusion_stopline_idx{0};
  };
  /**
   * @struct
   * @brief ego is approaching the boundary of attention area in the presence of traffic light
   */
  struct PeekingTowardOcclusion
  {
    //! if intersection_occlusion is disapproved externally through RTC, it indicates
    //! "is_forcefully_occluded"
    bool is_actually_occlusion_cleared{false};
    bool temporal_stop_before_attention_required{false};
    size_t closest_idx{0};
    size_t collision_stopline_idx{0};
    size_t first_attention_stopline_idx{0};
    size_t occlusion_stopline_idx{0};
    //! if null, it is dynamic occlusion and shows up intersection_occlusion(dyn). if valid, it
    //! contains the remaining time to release the static occlusion stuck and shows up
    //! intersection_occlusion(x.y)
    std::optional<double> static_occlusion_timeout{std::nullopt};
  };
  /**
   * @struct
   * @brief both collision and occlusion are detected in the presence of traffic light
   */
  struct OccludedCollisionStop
  {
    bool is_actually_occlusion_cleared{false};
    bool temporal_stop_before_attention_required{false};
    size_t closest_idx{0};
    size_t collision_stopline_idx{0};
    size_t first_attention_stopline_idx{0};
    size_t occlusion_stopline_idx{0};
    //! if null, it is dynamic occlusion and shows up intersection_occlusion(dyn). if valid, it
    //! contains the remaining time to release the static occlusion stuck
    std::optional<double> static_occlusion_timeout{std::nullopt};
  };
  /**
   * @struct
   * @brief at least occlusion is detected in the absence of traffic light
   */
  struct OccludedAbsenceTrafficLight
  {
    bool is_actually_occlusion_cleared{false};
    bool collision_detected{false};
    bool temporal_stop_before_attention_required{false};
    size_t closest_idx{0};
    size_t first_attention_area_stopline_idx{0};
    size_t peeking_limit_line_idx{0};
  };
  /**
   * @struct
   * @brief both collision and occlusion are not detected
   */
  struct Safe
  {
    size_t closest_idx{0};
    size_t collision_stopline_idx{0};
    size_t occlusion_stopline_idx{0};
  };
  /**
   * @struct
   * @brief traffic light is red or arrow signal
   */
  struct FullyPrioritized
  {
    bool collision_detected{false};
    size_t closest_idx{0};
    size_t collision_stopline_idx{0};
    size_t occlusion_stopline_idx{0};
  };
  using DecisionResult = std::variant<
    Indecisive,                   //! internal process error, or over the pass judge line
    StuckStop,                    //! detected stuck vehicle
    YieldStuckStop,               //! detected yield stuck vehicle
    NonOccludedCollisionStop,     //! detected collision while FOV is clear
    FirstWaitBeforeOcclusion,     //! stop for a while before peeking to occlusion
    PeekingTowardOcclusion,       //! peeking into occlusion while collision is not detected
    OccludedCollisionStop,        //! occlusion and collision are both detected
    OccludedAbsenceTrafficLight,  //! occlusion is detected in the absence of traffic light
    Safe,                         //! judge as safe
    FullyPrioritized              //! only detect vehicles violating traffic rules
    >;

  IntersectionModule(
    const int64_t module_id, const int64_t lane_id, std::shared_ptr<const PlannerData> planner_data,
    const PlannerParam & planner_param, const std::set<lanelet::Id> & associative_ids,
    const std::string & turn_direction, const bool has_traffic_light,
    const bool enable_occlusion_detection, rclcpp::Node & node, const rclcpp::Logger logger,
    const rclcpp::Clock::SharedPtr clock);

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

  // Parameter
  PlannerParam planner_param_;

  std::optional<IntersectionLanelets> intersection_lanelets_{std::nullopt};

  // for pass judge decision
  bool is_go_out_{false};
  bool is_permanent_go_{false};
  DecisionResult prev_decision_result_{Indecisive{""}};
  OcclusionType prev_occlusion_status_;

  // for occlusion detection
  const bool enable_occlusion_detection_;
  std::optional<std::vector<lanelet::ConstLineString3d>> occlusion_attention_divisions_{
    std::nullopt};                        //! for caching discretized occlusion detection lanelets
  StateMachine collision_state_machine_;  //! for stable collision checking
  StateMachine before_creep_state_machine_;    //! for two phase stop
  StateMachine occlusion_stop_state_machine_;  //! for stable occlusion detection
  StateMachine temporal_stop_before_attention_state_machine_;
  StateMachine static_occlusion_timeout_state_machine_;

  std::optional<rclcpp::Time> initial_green_light_observed_time_{std::nullopt};

  // for RTC
  const UUID occlusion_uuid_;
  bool occlusion_safety_{true};
  double occlusion_stop_distance_{0.0};
  bool occlusion_activated_{true};
  bool occlusion_first_stop_required_{false};

  /**
   * @fn
   * @brief set all RTC variable to safe and -inf
   */
  void initializeRTCStatus();
  /**
   * @fn
   * @brief analyze traffic_light/occupancy/objects context and return DecisionResult
   */
  DecisionResult modifyPathVelocityDetail(PathWithLaneId * path, StopReason * stop_reason);
  /**
   * @fn
   * @brief set RTC value according to calculated DecisionResult
   */
  void prepareRTCStatus(
    const DecisionResult &, const autoware_auto_planning_msgs::msg::PathWithLaneId & path);

  /**
   * @fn
   * @brief find TrafficPrioritizedLevel
   */
  TrafficPrioritizedLevel getTrafficPrioritizedLevel(lanelet::ConstLanelet lane);

  /**
   * @fn
   * @brief generate IntersectionLanelets
   */
  IntersectionLanelets getObjectiveLanelets(
    lanelet::LaneletMapConstPtr lanelet_map_ptr,
    lanelet::routing::RoutingGraphPtr routing_graph_ptr,
    const lanelet::ConstLanelet assigned_lanelet, const lanelet::ConstLanelets & lanelets_on_path);

  /**
   * @fn
   * @brief generate IntersectionStopLines
   */
  std::optional<IntersectionStopLines> generateIntersectionStopLines(
    lanelet::ConstLanelet assigned_lanelet,
    const lanelet::CompoundPolygon3d & first_conflicting_area,
    const lanelet::ConstLanelet & first_attention_lane,
    const std::optional<lanelet::CompoundPolygon3d> & second_attention_area_opt,
    const util::InterpolatedPathInfo & interpolated_path_info,
    autoware_auto_planning_msgs::msg::PathWithLaneId * original_path);

  /**
   * @fn
   * @brief find the associated stopline road marking of assigned lanelet
   */
  std::optional<size_t> getStopLineIndexFromMap(
    const util::InterpolatedPathInfo & interpolated_path_info,
    lanelet::ConstLanelet assigned_lanelet);

  /**
   * @fn
   * @brief generate PathLanelets
   */
  std::optional<PathLanelets> generatePathLanelets(
    const lanelet::ConstLanelets & lanelets_on_path,
    const util::InterpolatedPathInfo & interpolated_path_info,
    const lanelet::CompoundPolygon3d & first_conflicting_area,
    const std::vector<lanelet::CompoundPolygon3d> & conflicting_areas,
    const std::optional<lanelet::CompoundPolygon3d> & first_attention_area,
    const std::vector<lanelet::CompoundPolygon3d> & attention_areas, const size_t closest_idx);

  /**
   * @fn
   * @brief check stuck
   */
  bool checkStuckVehicleInIntersection(const PathLanelets & path_lanelets, DebugData * debug_data);

  /**
   * @fn
   * @brief check yield stuck
   */
  bool checkYieldStuckVehicleInIntersection(
    const TargetObjects & target_objects, const util::InterpolatedPathInfo & interpolated_path_info,
    const lanelet::ConstLanelets & attention_lanelets, DebugData * debug_data);

  /**
   * @fn
   * @brief categorize target objects
   */
  TargetObjects generateTargetObjects(
    const IntersectionLanelets & intersection_lanelets,
    const std::optional<Polygon2d> & intersection_area) const;

  /**
   * @fn
   * @brief check collision
   */
  bool checkCollision(
    const autoware_auto_planning_msgs::msg::PathWithLaneId & path, TargetObjects * target_objects,
    const PathLanelets & path_lanelets, const size_t closest_idx,
    const size_t last_intersection_stopline_candidate_idx, const double time_delay,
    const TrafficPrioritizedLevel & traffic_prioritized_level);

  std::optional<size_t> checkAngleForTargetLanelets(
    const geometry_msgs::msg::Pose & pose, const lanelet::ConstLanelets & target_lanelets,
    const bool is_parked_vehicle) const;

  void cutPredictPathWithDuration(TargetObjects * target_objects, const double time_thr);

  /**
   * @fn
   * @brief calculate ego vehicle profile along the path inside the intersection as the sequence of
   * (time of arrival, traveled distance) from current ego position
   */
  TimeDistanceArray calcIntersectionPassingTime(
    const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const size_t closest_idx,
    const size_t last_intersection_stopline_candidate_idx, const double time_delay,
    tier4_debug_msgs::msg::Float64MultiArrayStamped * debug_ttc_array);

  std::vector<lanelet::ConstLineString3d> generateDetectionLaneDivisions(
    lanelet::ConstLanelets detection_lanelets,
    const lanelet::routing::RoutingGraphPtr routing_graph_ptr, const double resolution);

  /**
   * @fn
   * @brief check occlusion status
   */
  OcclusionType getOcclusionStatus(
    const std::vector<lanelet::CompoundPolygon3d> & attention_areas,
    const lanelet::ConstLanelets & adjacent_lanelets,
    const lanelet::CompoundPolygon3d & first_attention_area,
    const util::InterpolatedPathInfo & interpolated_path_info,
    const std::vector<lanelet::ConstLineString3d> & lane_divisions,
    const TargetObjects & target_objects);

  /*
   * @fn
   * @brief check if associated traffic light is green
   */
  bool isGreenSolidOn(lanelet::ConstLanelet lane);

  /*
  bool IntersectionModule::checkFrontVehicleDeceleration(
    lanelet::ConstLanelets & ego_lane_with_next_lane, lanelet::ConstLanelet & closest_lanelet,
    const Polygon2d & stuck_vehicle_detect_area,
    const autoware_auto_perception_msgs::msg::PredictedObject & object,
    const double assumed_front_car_decel);
  */

  DebugData debug_data_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr decision_state_pub_;
  rclcpp::Publisher<tier4_debug_msgs::msg::Float64MultiArrayStamped>::SharedPtr ego_ttc_pub_;
  rclcpp::Publisher<tier4_debug_msgs::msg::Float64MultiArrayStamped>::SharedPtr object_ttc_pub_;
};

}  // namespace behavior_velocity_planner

#endif  // SCENE_INTERSECTION_HPP_
