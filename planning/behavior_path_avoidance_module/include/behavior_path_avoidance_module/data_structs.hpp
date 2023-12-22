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

#ifndef BEHAVIOR_PATH_AVOIDANCE_MODULE__DATA_STRUCTS_HPP_
#define BEHAVIOR_PATH_AVOIDANCE_MODULE__DATA_STRUCTS_HPP_

#include "behavior_path_planner_common/data_manager.hpp"
#include "behavior_path_planner_common/utils/path_safety_checker/path_safety_checker_parameters.hpp"
#include "behavior_path_planner_common/utils/path_shifter/path_shifter.hpp"

#include <rclcpp/time.hpp>
#include <tier4_autoware_utils/geometry/boost_geometry.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <tier4_planning_msgs/msg/avoidance_debug_msg_array.hpp>

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>

#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace behavior_path_planner
{
using autoware_auto_perception_msgs::msg::PredictedObject;
using autoware_auto_perception_msgs::msg::PredictedPath;
using autoware_auto_planning_msgs::msg::PathWithLaneId;

using tier4_autoware_utils::Point2d;
using tier4_autoware_utils::Polygon2d;
using tier4_planning_msgs::msg::AvoidanceDebugMsgArray;

using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;

using behavior_path_planner::utils::path_safety_checker::CollisionCheckDebug;

using route_handler::Direction;

struct ObjectParameter
{
  bool is_avoidance_target{false};

  bool is_safety_check_target{false};

  size_t execute_num{1};

  double moving_speed_threshold{0.0};

  double moving_time_threshold{1.0};

  double max_expand_ratio{0.0};

  double envelope_buffer_margin{0.0};

  double avoid_margin_lateral{1.0};

  double safety_buffer_lateral{1.0};

  double safety_buffer_longitudinal{0.0};

  bool use_conservative_buffer_longitudinal{true};
};

struct AvoidanceParameters
{
  // path resample interval for avoidance planning path.
  double resample_interval_for_planning = 0.3;

  // path resample interval for output path. Too short interval increases
  // computational cost for latter modules.
  double resample_interval_for_output = 3.0;

  // enable avoidance to be perform only in lane with same direction
  bool use_adjacent_lane{true};

  // enable avoidance to be perform in opposite lane direction
  // to use this, enable_avoidance_over_same_direction need to be set to true.
  bool use_opposite_lane{true};

  // if this param is true, it reverts avoidance path when the path is no longer needed.
  bool enable_cancel_maneuver{false};

  // enable avoidance for all parking vehicle
  bool enable_force_avoidance_for_stopped_vehicle{false};

  // enable yield maneuver.
  bool enable_yield_maneuver{false};

  // enable yield maneuver.
  bool enable_yield_maneuver_during_shifting{false};

  // disable path update
  bool disable_path_update{false};

  // use hatched road markings for avoidance
  bool use_hatched_road_markings{false};

  // use intersection area for avoidance
  bool use_intersection_areas{false};

  // consider avoidance return dead line
  bool enable_dead_line_for_goal{false};
  bool enable_dead_line_for_traffic_light{false};

  // module try to return original path to keep this distance from edge point of the path.
  double dead_line_buffer_for_goal{0.0};
  double dead_line_buffer_for_traffic_light{0.0};

  // max deceleration for
  double max_deceleration{0.0};

  // max jerk
  double max_jerk{0.0};

  // comfortable deceleration
  double nominal_deceleration{0.0};

  // comfortable jerk
  double nominal_jerk{0.0};

  // To prevent large acceleration while avoidance.
  double max_acceleration{0.0};

  // upper distance for envelope polygon expansion.
  double upper_distance_for_polygon_expansion{0.0};

  // lower distance for envelope polygon expansion.
  double lower_distance_for_polygon_expansion{0.0};

  // Vehicles whose distance to the center of the path is
  // less than this will not be considered for avoidance.
  double threshold_distance_object_is_on_center{0.0};

  // execute only when there is no intersection behind of the stopped vehicle.
  double object_ignore_section_traffic_light_in_front_distance{0.0};

  // execute only when there is no crosswalk near the stopped vehicle.
  double object_ignore_section_crosswalk_in_front_distance{0.0};

  // execute only when there is no crosswalk near the stopped vehicle.
  double object_ignore_section_crosswalk_behind_distance{0.0};

  // distance to avoid object detection
  bool use_static_detection_area{true};
  double object_check_min_forward_distance{0.0};
  double object_check_max_forward_distance{0.0};
  double object_check_backward_distance{0.0};
  double object_check_yaw_deviation{0.0};

  // if the distance between object and goal position is less than this parameter, the module ignore
  // the object.
  double object_check_goal_distance{0.0};

  // use in judge whether the vehicle is parking object on road shoulder
  double object_check_shiftable_ratio{0.0};

  // minimum road shoulder width. maybe 0.5 [m]
  double object_check_min_road_shoulder_width{0.0};

  // force avoidance
  double threshold_time_force_avoidance_for_stopped_vehicle{0.0};
  double force_avoidance_distance_threshold{0.0};

  // when complete avoidance motion, there is a distance margin with the object
  // for longitudinal direction
  double longitudinal_collision_margin_min_distance{0.0};

  // when complete avoidance motion, there is a time margin with the object
  // for longitudinal direction
  double longitudinal_collision_margin_time{0.0};

  // parameters for safety check area
  bool enable_safety_check{false};
  bool check_current_lane{false};
  bool check_shift_side_lane{false};
  bool check_other_side_lane{false};

  // parameters for safety check target.
  bool check_unavoidable_object{false};
  bool check_other_object{false};

  // parameters for collision check.
  bool check_all_predicted_path{false};

  // find adjacent lane vehicles
  double safety_check_backward_distance{0.0};

  // transit hysteresis (unsafe to safe)
  size_t hysteresis_factor_safe_count;
  double hysteresis_factor_expand_rate{0.0};

  // keep target velocity in yield maneuver
  double yield_velocity{0.0};

  // maximum stop distance
  double stop_max_distance{0.0};

  // stop buffer
  double stop_buffer{0.0};

  // start avoidance after this time to avoid sudden path change
  double min_prepare_time{0.0};
  double max_prepare_time{0.0};

  // Even if the vehicle speed is zero, avoidance will start after a distance of this much.
  double min_prepare_distance{0.0};

  // minimum slow down speed
  double min_slow_down_speed{0.0};

  // slow down speed buffer
  double buf_slow_down_speed{0.0};

  // nominal avoidance sped
  double nominal_avoidance_speed{0.0};

  // The margin is configured so that the generated avoidance trajectory does not come near to the
  // road shoulder.
  double soft_road_shoulder_margin{1.0};

  // The margin is configured so that the generated avoidance trajectory does not come near to the
  // road shoulder.
  double hard_road_shoulder_margin{1.0};

  // Even if the obstacle is very large, it will not avoid more than this length for right direction
  double max_right_shift_length{0.0};

  // Even if the obstacle is very large, it will not avoid more than this length for left direction
  double max_left_shift_length{0.0};

  // Validate vehicle departure from driving lane.
  double max_deviation_from_lane{0.0};

  // To prevent large acceleration while avoidance.
  double max_lateral_acceleration{0.0};

  // For the compensation of the detection lost. Once an object is observed, it is registered and
  // will be used for planning from the next time. If the object is not observed, it counts up the
  // lost_count and the registered object will be removed when the count exceeds this max count.
  double object_last_seen_threshold{0.0};

  // The avoidance path generation is performed when the shift distance of the
  // avoidance points is greater than this threshold.
  // In multiple targets case: if there are multiple vehicles in a row to be avoided, no new
  // avoidance path will be generated unless their lateral margin difference exceeds this value.
  double lateral_execution_threshold{0.0};

  // shift lines whose shift length is less than threshold is added a request with other large shift
  // line.
  double lateral_small_shift_threshold{0.0};

  // use for judge if the ego is shifting or not.
  double lateral_avoid_check_threshold{0.0};

  // For shift line generation process. The continuous shift length is quantized by this value.
  double quantize_filter_threshold{0.0};

  // For shift line generation process. Merge small shift lines. (First step)
  double same_grad_filter_1_threshold{0.0};

  // For shift line generation process. Merge small shift lines. (Second step)
  double same_grad_filter_2_threshold{0.0};

  // For shift line generation process. Merge small shift lines. (Third step)
  double same_grad_filter_3_threshold{0.0};

  // For shift line generation process. Remove sharp(=jerky) shift line.
  double sharp_shift_filter_threshold{0.0};

  // policy
  bool use_shorten_margin_immediately{false};

  // policy
  std::string policy_approval{"per_shift_line"};

  // policy
  std::string policy_deceleration{"best_effort"};

  // policy
  std::string policy_lateral_margin{"best_effort"};

  // target velocity matrix
  std::vector<double> velocity_map;

  // Minimum lateral jerk limitation map.
  std::vector<double> lateral_min_jerk_map;

  // Maximum lateral jerk limitation map.
  std::vector<double> lateral_max_jerk_map;

  // Maximum lateral acceleration limitation map.
  std::vector<double> lateral_max_accel_map;

  // parameters depend on object class
  std::unordered_map<uint8_t, ObjectParameter> object_parameters;

  // ego predicted path params.
  utils::path_safety_checker::EgoPredictedPathParams ego_predicted_path_params{};

  // rss parameters
  utils::path_safety_checker::RSSparams rss_params{};

  // clip left and right bounds for objects
  bool enable_bound_clipping{false};

  // debug
  bool publish_debug_marker = false;
  bool print_debug_info = false;
};

struct ObjectData  // avoidance target
{
  ObjectData() = default;

  ObjectData(PredictedObject obj, double lat, double lon, double len, double overhang)
  : object(std::move(obj)),
    to_centerline(lat),
    longitudinal(lon),
    length(len),
    overhang_dist(overhang)
  {
  }

  PredictedObject object;

  // object behavior.
  enum class Behavior {
    NONE = 0,
    MERGING,
    DEVIATING,
  };
  Behavior behavior{Behavior::NONE};

  // lateral position of the CoM, in Frenet coordinate from ego-pose

  double to_centerline{0.0};

  // longitudinal position of the CoM, in Frenet coordinate from ego-pose
  double longitudinal{0.0};

  // longitudinal length of vehicle, in Frenet coordinate
  double length{0.0};

  // lateral distance to the closest footprint, in Frenet coordinate
  double overhang_dist{0.0};

  // lateral shiftable ratio
  double shiftable_ratio{0.0};

  // distance factor for perception noise (0.0~1.0)
  double distance_factor{0.0};

  // count up when object disappeared. Removed when it exceeds threshold.
  rclcpp::Time last_seen;
  double lost_time{0.0};

  // count up when object moved. Removed when it exceeds threshold.
  rclcpp::Time last_stop;
  double move_time{0.0};

  // object stopping duration
  rclcpp::Time last_move;
  double stop_time{0.0};

  // store the information of the lanelet which the object's overhang is currently occupying
  lanelet::ConstLanelet overhang_lanelet;

  // the position at the detected moment
  Pose init_pose;

  // the position of the overhang
  Pose overhang_pose;

  // envelope polygon
  Polygon2d envelope_poly{};

  // envelope polygon centroid
  Point2d centroid{};

  // lateral distance from overhang to the road shoulder
  double to_road_shoulder_distance{0.0};

  // to intersection
  double to_stop_factor_distance{std::numeric_limits<double>::infinity()};

  // to stop line distance
  double to_stop_line{std::numeric_limits<double>::infinity()};

  // if lateral margin is NOT enough, the ego must avoid the object.
  bool avoid_required{false};

  // is avoidable by behavior module
  bool is_avoidable{false};

  // is stoppable under the constraints
  bool is_stoppable{false};

  // is within intersection area
  bool is_within_intersection{false};

  // object direction.
  Direction direction{Direction::NONE};

  // unavoidable reason
  std::string reason{};

  // lateral avoid margin
  std::optional<double> avoid_margin{std::nullopt};

  // the nearest bound point (use in road shoulder distance calculation)
  std::optional<Point> nearest_bound_point{std::nullopt};
};
using ObjectDataArray = std::vector<ObjectData>;

/*
 * Shift point with additional info for avoidance planning
 */
struct AvoidLine : public ShiftLine
{
  // object side
  bool object_on_right = true;

  // Distance from ego to start point in Frenet
  double start_longitudinal = 0.0;

  // Distance from ego to end point in Frenet
  double end_longitudinal = 0.0;

  // for unique_id
  UUID id{};

  // for the case the point is created by merge other points
  std::vector<UUID> parent_ids{};

  // corresponding object
  ObjectData object{};

  double getRelativeLength() const { return end_shift_length - start_shift_length; }

  double getRelativeLongitudinal() const { return end_longitudinal - start_longitudinal; }

  double getGradient() const { return getRelativeLength() / getRelativeLongitudinal(); }
};
using AvoidLineArray = std::vector<AvoidLine>;

struct AvoidOutline
{
  AvoidOutline(AvoidLine avoid_line, AvoidLine return_line)
  : avoid_line{std::move(avoid_line)}, return_line{std::move(return_line)}
  {
  }

  AvoidLine avoid_line{};

  AvoidLine return_line{};

  AvoidLineArray middle_lines{};
};
using AvoidOutlines = std::vector<AvoidOutline>;

/*
 * avoidance state
 */
enum class AvoidanceState {
  NOT_AVOID = 0,
  AVOID_EXECUTE,
  YIELD,
  AVOID_PATH_READY,
  AVOID_PATH_NOT_READY,
};

/*
 * Common data for avoidance planning
 */
struct AvoidancePlanningData
{
  // ego final state
  AvoidanceState state{AvoidanceState::NOT_AVOID};

  // un-shifted pose (for current lane detection)
  Pose reference_pose;

  // reference path (before shifting)
  PathWithLaneId reference_path;

  // reference path (pre-resampled reference path)
  PathWithLaneId reference_path_rough;

  // closest reference_path index for reference_pose
  size_t ego_closest_path_index;

  // arclength vector of the reference_path from ego.
  // If the point is behind ego_pose, the value is negative.
  std::vector<double> arclength_from_ego;

  // current driving lanelet
  lanelet::ConstLanelets current_lanelets;
  lanelet::ConstLanelets extend_lanelets;

  // output path
  ShiftedPath candidate_path;

  // avoidance target objects
  ObjectDataArray target_objects;

  // the others
  ObjectDataArray other_objects;

  // nearest object that should be avoid
  std::optional<ObjectData> stop_target_object{std::nullopt};

  // new shift point
  AvoidLineArray new_shift_line{};

  // safe shift point
  AvoidLineArray safe_shift_line{};

  std::vector<DrivableLanes> drivable_lanes{};

  lanelet::BasicLineString3d right_bound{};

  lanelet::BasicLineString3d left_bound{};

  bool safe{false};

  bool valid{false};

  bool success{false};

  bool comfortable{false};

  bool avoid_required{false};

  bool yield_required{false};

  bool found_avoidance_path{false};

  double to_stop_line{std::numeric_limits<double>::max()};

  double to_start_point{std::numeric_limits<double>::lowest()};

  double to_return_point{std::numeric_limits<double>::max()};
};

/*
 * Data struct for shift line generation
 */
struct ShiftLineData
{
  std::vector<double> shift_line;

  std::vector<double> pos_shift_line;

  std::vector<double> neg_shift_line;

  std::vector<double> shift_line_grad;

  std::vector<double> pos_shift_line_grad;

  std::vector<double> neg_shift_line_grad;

  std::vector<double> forward_grad;

  std::vector<double> backward_grad;

  std::vector<std::vector<double>> shift_line_history;
};

/*
 * Debug information for marker array
 */
struct DebugData
{
  geometry_msgs::msg::Polygon detection_area;

  lanelet::ConstLineStrings3d bounds;

  // combine process
  AvoidLineArray step1_registered_shift_line;
  AvoidLineArray step1_current_shift_line;
  AvoidLineArray step1_filled_shift_line;
  AvoidLineArray step1_merged_shift_line;
  AvoidLineArray step1_combined_shift_line;
  AvoidLineArray step1_return_shift_line;
  AvoidLineArray step1_front_shift_line;

  // create outline process
  AvoidLineArray step2_merged_shift_line;

  // trimming process
  AvoidLineArray step3_quantize_filtered;
  AvoidLineArray step3_noise_filtered;
  AvoidLineArray step3_grad_filtered_1st;
  AvoidLineArray step3_grad_filtered_2nd;
  AvoidLineArray step3_grad_filtered_3rd;

  // registered process
  AvoidLineArray step4_new_shift_line;

  // shift length
  std::vector<double> pos_shift;
  std::vector<double> neg_shift;
  std::vector<double> total_shift;
  std::vector<double> output_shift;

  // shift grad
  std::vector<double> pos_shift_grad;
  std::vector<double> neg_shift_grad;
  std::vector<double> total_forward_grad;
  std::vector<double> total_backward_grad;

  // shift path
  std::vector<double> proposed_spline_shift;

  // avoidance require objects
  ObjectDataArray unavoidable_objects;

  // avoidance unsafe objects
  ObjectDataArray unsafe_objects;

  // tmp for plot
  PathWithLaneId center_line;

  // safety check area
  lanelet::ConstLanelets safety_check_lanes;

  // collision check debug map
  utils::path_safety_checker::CollisionCheckDebugMap collision_check;

  // debug msg array
  AvoidanceDebugMsgArray avoidance_debug_msg_array;
};

}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_AVOIDANCE_MODULE__DATA_STRUCTS_HPP_
