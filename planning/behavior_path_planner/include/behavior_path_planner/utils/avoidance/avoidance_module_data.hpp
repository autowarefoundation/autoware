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

#ifndef BEHAVIOR_PATH_PLANNER__UTILS__AVOIDANCE__AVOIDANCE_MODULE_DATA_HPP_
#define BEHAVIOR_PATH_PLANNER__UTILS__AVOIDANCE__AVOIDANCE_MODULE_DATA_HPP_

#include "behavior_path_planner/utils/path_shifter/path_shifter.hpp"

#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <tier4_planning_msgs/msg/avoidance_debug_msg_array.hpp>

#include <lanelet2_core/geometry/Lanelet.h>

#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
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
using geometry_msgs::msg::TransformStamped;

struct ObjectParameter
{
  bool is_target{false};

  double moving_speed_threshold{0.0};

  double moving_time_threshold{1.0};

  double max_expand_ratio{0.0};

  double envelope_buffer_margin{0.0};

  double avoid_margin_lateral{1.0};

  double safety_buffer_lateral{1.0};

  double safety_buffer_longitudinal{0.0};
};

struct AvoidanceParameters
{
  // path resample interval for avoidance planning path.
  double resample_interval_for_planning = 0.3;

  // path resample interval for output path. Too short interval increases
  // computational cost for latter modules.
  double resample_interval_for_output = 3.0;

  // lanelet expand length for right side to find avoidance target vehicles
  double detection_area_right_expand_dist = 0.0;

  // lanelet expand length for left side to find avoidance target vehicles
  double detection_area_left_expand_dist = 1.0;

  // enable avoidance to be perform only in lane with same direction
  bool use_adjacent_lane{true};

  // enable avoidance to be perform in opposite lane direction
  // to use this, enable_avoidance_over_same_direction need to be set to true.
  bool use_opposite_lane{true};

  // enable update path when if detected objects on planner data is gone.
  bool enable_update_path_when_object_is_gone{false};

  // enable avoidance for all parking vehicle
  bool enable_force_avoidance_for_stopped_vehicle{false};

  // enable safety check. if avoidance path is NOT safe, the ego will execute yield maneuver
  bool enable_safety_check{false};

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

  // constrains
  bool use_constraints_for_decel{false};

  // max deceleration for
  double max_deceleration;

  // max jerk
  double max_jerk;

  // comfortable deceleration
  double nominal_deceleration;

  // comfortable jerk
  double nominal_jerk;

  // To prevent large acceleration while avoidance.
  double max_acceleration;

  // upper distance for envelope polygon expansion.
  double upper_distance_for_polygon_expansion;

  // lower distance for envelope polygon expansion.
  double lower_distance_for_polygon_expansion;

  // Vehicles whose distance to the center of the path is
  // less than this will not be considered for avoidance.
  double threshold_distance_object_is_on_center;

  // execute only when there is no intersection behind of the stopped vehicle.
  double object_ignore_section_traffic_light_in_front_distance;

  // execute only when there is no crosswalk near the stopped vehicle.
  double object_ignore_section_crosswalk_in_front_distance;

  // execute only when there is no crosswalk near the stopped vehicle.
  double object_ignore_section_crosswalk_behind_distance;

  // distance to avoid object detection
  double object_check_forward_distance;

  // continue to detect backward vehicles as avoidance targets until they are this distance away
  double object_check_backward_distance;

  // if the distance between object and goal position is less than this parameter, the module ignore
  // the object.
  double object_check_goal_distance;

  // use in judge whether the vehicle is parking object on road shoulder
  double object_check_shiftable_ratio;

  // minimum road shoulder width. maybe 0.5 [m]
  double object_check_min_road_shoulder_width;

  // force avoidance
  double threshold_time_force_avoidance_for_stopped_vehicle;

  // when complete avoidance motion, there is a distance margin with the object
  // for longitudinal direction
  double longitudinal_collision_margin_min_distance;

  // when complete avoidance motion, there is a time margin with the object
  // for longitudinal direction
  double longitudinal_collision_margin_time;

  // find adjacent lane vehicles
  double safety_check_backward_distance;

  // minimum longitudinal margin for vehicles in adjacent lane
  double safety_check_min_longitudinal_margin;

  // safety check time horizon
  double safety_check_time_horizon;

  // use in RSS calculation
  double safety_check_idling_time;

  // use in RSS calculation
  double safety_check_accel_for_rss;

  // transit hysteresis (unsafe to safe)
  double safety_check_hysteresis_factor;

  // don't output new candidate path if the offset between ego and path is larger than this.
  double safety_check_ego_offset;

  // keep target velocity in yield maneuver
  double yield_velocity;

  // maximum stop distance
  double stop_max_distance;

  // stop buffer
  double stop_buffer;

  // start avoidance after this time to avoid sudden path change
  double prepare_time;

  // Even if the vehicle speed is zero, avoidance will start after a distance of this much.
  double min_prepare_distance;

  // minimum slow down speed
  double min_slow_down_speed;

  // slow down speed buffer
  double buf_slow_down_speed;

  // nominal avoidance sped
  double nominal_avoidance_speed;

  // The margin is configured so that the generated avoidance trajectory does not come near to the
  // road shoulder.
  double road_shoulder_safety_margin{1.0};

  // Even if the obstacle is very large, it will not avoid more than this length for right direction
  double max_right_shift_length;

  // Even if the obstacle is very large, it will not avoid more than this length for left direction
  double max_left_shift_length;

  // To prevent large acceleration while avoidance.
  double max_lateral_acceleration;

  // For the compensation of the detection lost. Once an object is observed, it is registered and
  // will be used for planning from the next time. If the object is not observed, it counts up the
  // lost_count and the registered object will be removed when the count exceeds this max count.
  double object_last_seen_threshold;

  // The avoidance path generation is performed when the shift distance of the
  // avoidance points is greater than this threshold.
  // In multiple targets case: if there are multiple vehicles in a row to be avoided, no new
  // avoidance path will be generated unless their lateral margin difference exceeds this value.
  double lateral_execution_threshold;

  // shift lines whose shift length is less than threshold is added a request with other large shift
  // line.
  double lateral_small_shift_threshold;

  // For shift line generation process. The continuous shift length is quantized by this value.
  double quantize_filter_threshold;

  // For shift line generation process. Merge small shift lines. (First step)
  double same_grad_filter_1_threshold;

  // For shift line generation process. Merge small shift lines. (Second step)
  double same_grad_filter_2_threshold;

  // For shift line generation process. Merge small shift lines. (Third step)
  double same_grad_filter_3_threshold;

  // For shift line generation process. Remove sharp(=jerky) shift line.
  double sharp_shift_filter_threshold;

  // target velocity matrix
  std::vector<double> velocity_map;

  // Minimum lateral jerk limitation map.
  std::vector<double> lateral_min_jerk_map;

  // Maximum lateral jerk limitation map.
  std::vector<double> lateral_max_jerk_map;

  // Maximum lateral acceleration limitation map.
  std::vector<double> lateral_max_accel_map;

  // target velocity matrix
  std::vector<double> target_velocity_matrix;

  // matrix col size
  size_t col_size;

  // parameters depend on object class
  std::unordered_map<uint8_t, ObjectParameter> object_parameters;

  // clip left and right bounds for objects
  bool enable_bound_clipping{false};

  // debug
  bool publish_debug_marker = false;
  bool print_debug_info = false;
};

struct ObjectData  // avoidance target
{
  ObjectData() = default;
  ObjectData(const PredictedObject & obj, double lat, double lon, double len, double overhang)
  : object(obj), lateral(lat), longitudinal(lon), length(len), overhang_dist(overhang)
  {
  }

  PredictedObject object;

  // lateral position of the CoM, in Frenet coordinate from ego-pose
  double lateral;

  // longitudinal position of the CoM, in Frenet coordinate from ego-pose
  double longitudinal;

  // longitudinal length of vehicle, in Frenet coordinate
  double length;

  // lateral distance to the closest footprint, in Frenet coordinate
  double overhang_dist;

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

  // unavoidable reason
  std::string reason{""};

  // lateral avoid margin
  // NOTE: If margin is less than the minimum margin threshold, boost::none will be set.
  boost::optional<double> avoid_margin{boost::none};
};
using ObjectDataArray = std::vector<ObjectData>;

/*
 * Shift point with additional info for avoidance planning
 */
struct AvoidLine : public ShiftLine
{
  // Distance from ego to start point in Frenet
  double start_longitudinal = 0.0;

  // Distance from ego to end point in Frenet
  double end_longitudinal = 0.0;

  // for unique_id
  uint64_t id = 0;

  // for the case the point is created by merge other points
  std::vector<uint64_t> parent_ids{};

  // corresponding object
  ObjectData object{};

  double getRelativeLength() const { return end_shift_length - start_shift_length; }

  double getRelativeLongitudinal() const { return end_longitudinal - start_longitudinal; }

  double getGradient() const { return getRelativeLength() / getRelativeLongitudinal(); }
};
using AvoidLineArray = std::vector<AvoidLine>;

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

  // output path
  ShiftedPath candidate_path;

  // avoidance target objects
  ObjectDataArray target_objects;

  // the others
  ObjectDataArray other_objects;

  // nearest object that should be avoid
  boost::optional<ObjectData> stop_target_object{boost::none};

  // raw shift point
  AvoidLineArray unapproved_raw_sl{};

  // new shift point
  AvoidLineArray unapproved_new_sl{};

  // safe shift point
  AvoidLineArray safe_new_sl{};

  bool safe{false};

  bool avoiding_now{false};

  bool avoid_required{false};

  bool yield_required{false};

  bool found_avoidance_path{false};

  double to_stop_line{std::numeric_limits<double>::max()};
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
 * Data struct for longitudinal margin
 */
struct MarginData
{
  Pose pose{};

  bool enough_lateral_margin{true};

  double longitudinal_distance{std::numeric_limits<double>::max()};

  double longitudinal_margin{std::numeric_limits<double>::lowest()};

  double vehicle_width;

  double base_link2front;

  double base_link2rear;
};
using MarginDataArray = std::vector<MarginData>;

/*
 * Debug information for marker array
 */
struct DebugData
{
  std::shared_ptr<lanelet::ConstLanelets> expanded_lanelets;
  std::shared_ptr<lanelet::ConstLanelets> current_lanelets;

  lanelet::ConstLineStrings3d bounds;

  AvoidLineArray current_shift_lines;  // in path shifter
  AvoidLineArray new_shift_lines;      // in path shifter

  AvoidLineArray registered_raw_shift;
  AvoidLineArray current_raw_shift;
  AvoidLineArray extra_return_shift;

  AvoidLineArray merged;
  AvoidLineArray gap_filled;
  AvoidLineArray trim_similar_grad_shift;
  AvoidLineArray quantized;
  AvoidLineArray trim_small_shift;
  AvoidLineArray trim_similar_grad_shift_second;
  AvoidLineArray trim_similar_grad_shift_third;
  AvoidLineArray trim_momentary_return;
  AvoidLineArray trim_too_sharp_shift;

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

  bool exist_adjacent_objects{false};

  // future pose
  PathWithLaneId path_with_planned_velocity;

  // margin
  MarginDataArray margin_data_array;

  // avoidance require objects
  ObjectDataArray unavoidable_objects;

  // avoidance unsafe objects
  ObjectDataArray unsafe_objects;

  // tmp for plot
  PathWithLaneId center_line;

  AvoidanceDebugMsgArray avoidance_debug_msg_array;
};

}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__UTILS__AVOIDANCE__AVOIDANCE_MODULE_DATA_HPP_
