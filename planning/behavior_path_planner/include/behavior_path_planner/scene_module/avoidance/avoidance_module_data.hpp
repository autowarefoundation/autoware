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

#ifndef BEHAVIOR_PATH_PLANNER__SCENE_MODULE__AVOIDANCE__AVOIDANCE_MODULE_DATA_HPP_
#define BEHAVIOR_PATH_PLANNER__SCENE_MODULE__AVOIDANCE__AVOIDANCE_MODULE_DATA_HPP_

#include "behavior_path_planner/path_shifter/path_shifter.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/path.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <memory>
#include <string>
#include <vector>

namespace behavior_path_planner
{
using autoware_auto_perception_msgs::msg::PredictedObject;
using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseStamped;

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
  bool enable_avoidance_over_same_direction{true};

  // enable avoidance to be perform in opposite lane direction
  // to use this, enable_avoidance_over_same_direction need to be set to true.
  bool enable_avoidance_over_opposite_direction{true};

  // Vehicles whose distance to the center of the path is
  // less than this will not be considered for avoidance.
  double threshold_distance_object_is_on_center;

  // vehicles with speed greater than this will not be avoided
  double threshold_speed_object_is_stopped;

  // distance to avoid object detection
  double object_check_forward_distance;

  // continue to detect backward vehicles as avoidance targets until they are this distance away
  double object_check_backward_distance;

  // we want to keep this lateral margin when avoiding
  double lateral_collision_margin;
  // a buffer in case lateral_collision_margin is set to 0. Will throw error
  // don't ever set this value to 0
  double lateral_collision_safety_buffer{0.5};

  // when complete avoidance motion, there is a distance margin with the object
  // for longitudinal direction
  double longitudinal_collision_margin_min_distance;

  // when complete avoidance motion, there is a time margin with the object
  // for longitudinal direction
  double longitudinal_collision_margin_time;

  // start avoidance after this time to avoid sudden path change
  double prepare_time;

  // Even if the vehicle speed is zero, avoidance will start after a distance of this much.
  double min_prepare_distance;

  // minimum distance while avoiding TODO(Horibe): will be changed to jerk constraint later
  double min_avoidance_distance;

  // minimum speed for jerk calculation in a nominal situation, i.e. there is an enough
  // distance for avoidance, and the object is very far from ego. In that case, the
  // vehicle speed is unknown passing along the object. Then use this speed as a minimum.
  // Note: This parameter is needed because we have to plan an avoidance path in advance
  //       without knowing the speed of the distant path.
  double min_nominal_avoidance_speed;

  // minimum speed for jerk calculation in a tight situation, i.e. there is NOT an enough
  // distance for avoidance. Need a sharp avoidance path to avoid the object.
  double min_sharp_avoidance_speed;

  // The margin is configured so that the generated avoidance trajectory does not come near to the
  // road shoulder.
  double road_shoulder_safety_margin{1.0};

  // Even if the obstacle is very large, it will not avoid more than this length for right direction
  double max_right_shift_length;

  // Even if the obstacle is very large, it will not avoid more than this length for left direction
  double max_left_shift_length;

  // Avoidance path is generated with this jerk.
  // If there is no margin, the jerk increases up to max lateral jerk.
  double nominal_lateral_jerk;

  // if the avoidance path exceeds this lateral jerk, it will be not used anymore.
  double max_lateral_jerk;

  // For the compensation of the detection lost. Once an object is observed, it is registered and
  // will be used for planning from the next time. If the object is not observed, it counts up the
  // lost_count and the registered object will be removed when the count exceeds this max count.
  int object_hold_max_count;

  // For velocity planning to avoid acceleration during avoidance.
  // Speeds smaller than this are not inserted.
  double min_avoidance_speed_for_acc_prevention;

  // To prevent large acceleration while avoidance. The max velocity is limited with this
  // acceleration.
  double max_avoidance_acceleration;

  // if distance between vehicle front and shift end point is larger than this length,
  // turn signal is not turned on.
  double avoidance_search_distance;

  // debug
  bool publish_debug_marker = false;
  bool print_debug_info = false;
};

struct ObjectData  // avoidance target
{
  ObjectData() = default;
  ObjectData(const PredictedObject & obj, double lat, double lon, double overhang)
  : object(obj), lateral(lat), longitudinal(lon), overhang_dist(overhang)
  {
  }

  PredictedObject object;

  // lateral position of the CoM, in Frenet coordinate from ego-pose
  double lateral;

  // longitudinal position of the CoM, in Frenet coordinate from ego-pose
  double longitudinal;

  // lateral distance to the closest footprint, in Frenet coordinate
  double overhang_dist;

  // count up when object disappeared. Removed when it exceeds threshold.
  int lost_count = 0;

  // store the information of the lanelet which the object's overhang is currently occupying
  lanelet::ConstLanelet overhang_lanelet;

  // the position of the overhang
  Pose overhang_pose;

  // lateral distance from overhang to the road shoulder
  double to_road_shoulder_distance{0.0};
};
using ObjectDataArray = std::vector<ObjectData>;

/*
 * Shift point with additional info for avoidance planning
 */
struct AvoidPoint : public ShiftPoint
{
  // relative shift length from start to end point
  double start_length = 0.0;

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

  double getRelativeLength() const { return length - start_length; }

  double getRelativeLongitudinal() const { return end_longitudinal - start_longitudinal; }

  double getGradient() const { return getRelativeLength() / getRelativeLongitudinal(); }
};
using AvoidPointArray = std::vector<AvoidPoint>;

/*
 * Common data for avoidance planning
 */
struct AvoidancePlanningData
{
  // un-shifted pose (for current lane detection)
  Pose reference_pose;

  // reference path (before shifting)
  PathWithLaneId reference_path;

  // closest reference_path index for reference_pose
  size_t ego_closest_path_index;

  // arclength vector of the reference_path from ego.
  // If the point is behind ego_pose, the value is negative.
  std::vector<double> arclength_from_ego;

  // current driving lanelet
  lanelet::ConstLanelets current_lanelets;

  // avoidance target objects
  ObjectDataArray objects;
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
  std::shared_ptr<lanelet::ConstLanelets> expanded_lanelets;
  std::shared_ptr<lanelet::ConstLanelets> current_lanelets;
  std::shared_ptr<lanelet::ConstLineStrings3d> farthest_linestring_from_overhang;

  AvoidPointArray current_shift_points;  // in path shifter
  AvoidPointArray new_shift_points;      // in path shifter

  AvoidPointArray registered_raw_shift;
  AvoidPointArray current_raw_shift;
  AvoidPointArray extra_return_shift;

  AvoidPointArray merged;
  AvoidPointArray trim_similar_grad_shift;
  AvoidPointArray quantized;
  AvoidPointArray trim_small_shift;
  AvoidPointArray trim_similar_grad_shift_second;
  AvoidPointArray trim_momentary_return;
  AvoidPointArray trim_too_sharp_shift;
  std::vector<double> pos_shift;
  std::vector<double> neg_shift;
  std::vector<double> total_shift;
  std::vector<double> output_shift;

  // tmp for plot
  PathWithLaneId center_line;
};

}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__AVOIDANCE__AVOIDANCE_MODULE_DATA_HPP_
