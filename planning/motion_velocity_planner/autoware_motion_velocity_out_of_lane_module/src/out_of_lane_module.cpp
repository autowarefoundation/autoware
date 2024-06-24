// Copyright 2024 TIER IV, Inc. All rights reserved.
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

#include "out_of_lane_module.hpp"

#include "calculate_slowdown_points.hpp"
#include "debug.hpp"
#include "decisions.hpp"
#include "filter_predicted_objects.hpp"
#include "footprint.hpp"
#include "lanelets_selection.hpp"
#include "overlapping_range.hpp"
#include "types.hpp"

#include <autoware/motion_utils/trajectory/interpolation.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/universe_utils/ros/parameter.hpp>
#include <autoware/universe_utils/ros/update_param.hpp>
#include <autoware/universe_utils/system/stop_watch.hpp>

#include <boost/geometry/algorithms/intersects.hpp>

#include <lanelet2_core/geometry/LaneletMap.h>

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::motion_velocity_planner
{

using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

void OutOfLaneModule::init(rclcpp::Node & node, const std::string & module_name)
{
  module_name_ = module_name;
  logger_ = node.get_logger();
  clock_ = node.get_clock();
  init_parameters(node);
  velocity_factor_interface_.init(autoware::motion_utils::PlanningBehavior::ROUTE_OBSTACLE);

  debug_publisher_ =
    node.create_publisher<visualization_msgs::msg::MarkerArray>("~/" + ns_ + "/debug_markers", 1);
  virtual_wall_publisher_ =
    node.create_publisher<visualization_msgs::msg::MarkerArray>("~/" + ns_ + "/virtual_walls", 1);
  processing_time_publisher_ = std::make_shared<autoware::universe_utils::ProcessingTimePublisher>(
    &node, "~/debug/" + ns_ + "/processing_time_ms");
}
void OutOfLaneModule::init_parameters(rclcpp::Node & node)
{
  using autoware::universe_utils::getOrDeclareParameter;
  auto & pp = params_;

  pp.mode = getOrDeclareParameter<std::string>(node, ns_ + ".mode");
  pp.skip_if_already_overlapping =
    getOrDeclareParameter<bool>(node, ns_ + ".skip_if_already_overlapping");

  pp.time_threshold = getOrDeclareParameter<double>(node, ns_ + ".threshold.time_threshold");
  pp.intervals_ego_buffer = getOrDeclareParameter<double>(node, ns_ + ".intervals.ego_time_buffer");
  pp.intervals_obj_buffer =
    getOrDeclareParameter<double>(node, ns_ + ".intervals.objects_time_buffer");
  pp.ttc_threshold = getOrDeclareParameter<double>(node, ns_ + ".ttc.threshold");

  pp.objects_min_vel = getOrDeclareParameter<double>(node, ns_ + ".objects.minimum_velocity");
  pp.objects_use_predicted_paths =
    getOrDeclareParameter<bool>(node, ns_ + ".objects.use_predicted_paths");
  pp.objects_min_confidence =
    getOrDeclareParameter<double>(node, ns_ + ".objects.predicted_path_min_confidence");
  pp.objects_dist_buffer = getOrDeclareParameter<double>(node, ns_ + ".objects.distance_buffer");
  pp.objects_cut_predicted_paths_beyond_red_lights =
    getOrDeclareParameter<bool>(node, ns_ + ".objects.cut_predicted_paths_beyond_red_lights");

  pp.overlap_min_dist = getOrDeclareParameter<double>(node, ns_ + ".overlap.minimum_distance");
  pp.overlap_extra_length = getOrDeclareParameter<double>(node, ns_ + ".overlap.extra_length");

  pp.skip_if_over_max_decel =
    getOrDeclareParameter<bool>(node, ns_ + ".action.skip_if_over_max_decel");
  pp.precision = getOrDeclareParameter<double>(node, ns_ + ".action.precision");
  pp.min_decision_duration = getOrDeclareParameter<double>(node, ns_ + ".action.min_duration");
  pp.dist_buffer = getOrDeclareParameter<double>(node, ns_ + ".action.distance_buffer");
  pp.slow_velocity = getOrDeclareParameter<double>(node, ns_ + ".action.slowdown.velocity");
  pp.slow_dist_threshold =
    getOrDeclareParameter<double>(node, ns_ + ".action.slowdown.distance_threshold");
  pp.stop_dist_threshold =
    getOrDeclareParameter<double>(node, ns_ + ".action.stop.distance_threshold");

  pp.ego_min_velocity = getOrDeclareParameter<double>(node, ns_ + ".ego.min_assumed_velocity");
  pp.extra_front_offset = getOrDeclareParameter<double>(node, ns_ + ".ego.extra_front_offset");
  pp.extra_rear_offset = getOrDeclareParameter<double>(node, ns_ + ".ego.extra_rear_offset");
  pp.extra_left_offset = getOrDeclareParameter<double>(node, ns_ + ".ego.extra_left_offset");
  pp.extra_right_offset = getOrDeclareParameter<double>(node, ns_ + ".ego.extra_right_offset");
  const auto vehicle_info = autoware::vehicle_info_utils::VehicleInfoUtils(node).getVehicleInfo();
  pp.front_offset = vehicle_info.max_longitudinal_offset_m;
  pp.rear_offset = vehicle_info.min_longitudinal_offset_m;
  pp.left_offset = vehicle_info.max_lateral_offset_m;
  pp.right_offset = vehicle_info.min_lateral_offset_m;
}

void OutOfLaneModule::update_parameters(const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware::universe_utils::updateParam;
  auto & pp = params_;
  updateParam(parameters, ns_ + ".mode", pp.mode);
  updateParam(parameters, ns_ + ".skip_if_already_overlapping", pp.skip_if_already_overlapping);

  updateParam(parameters, ns_ + ".threshold.time_threshold", pp.time_threshold);
  updateParam(parameters, ns_ + ".intervals.ego_time_buffer", pp.intervals_ego_buffer);
  updateParam(parameters, ns_ + ".intervals.objects_time_buffer", pp.intervals_obj_buffer);
  updateParam(parameters, ns_ + ".ttc.threshold", pp.ttc_threshold);

  updateParam(parameters, ns_ + ".objects.minimum_velocity", pp.objects_min_vel);
  updateParam(parameters, ns_ + ".objects.use_predicted_paths", pp.objects_use_predicted_paths);
  updateParam(
    parameters, ns_ + ".objects.predicted_path_min_confidence", pp.objects_min_confidence);
  updateParam(parameters, ns_ + ".objects.distance_buffer", pp.objects_dist_buffer);
  updateParam(
    parameters, ns_ + ".objects.cut_predicted_paths_beyond_red_lights",
    pp.objects_cut_predicted_paths_beyond_red_lights);

  updateParam(parameters, ns_ + ".overlap.minimum_distance", pp.overlap_min_dist);
  updateParam(parameters, ns_ + ".overlap.extra_length", pp.overlap_extra_length);

  updateParam(parameters, ns_ + ".action.skip_if_over_max_decel", pp.skip_if_over_max_decel);
  updateParam(parameters, ns_ + ".action.precision", pp.precision);
  updateParam(parameters, ns_ + ".action.min_duration", pp.min_decision_duration);
  updateParam(parameters, ns_ + ".action.distance_buffer", pp.dist_buffer);
  updateParam(parameters, ns_ + ".action.slowdown.velocity", pp.slow_velocity);
  updateParam(parameters, ns_ + ".action.slowdown.distance_threshold", pp.slow_dist_threshold);
  updateParam(parameters, ns_ + ".action.stop.distance_threshold", pp.stop_dist_threshold);

  updateParam(parameters, ns_ + ".ego.min_assumed_velocity", pp.ego_min_velocity);
  updateParam(parameters, ns_ + ".ego.extra_front_offset", pp.extra_front_offset);
  updateParam(parameters, ns_ + ".ego.extra_rear_offset", pp.extra_rear_offset);
  updateParam(parameters, ns_ + ".ego.extra_left_offset", pp.extra_left_offset);
  updateParam(parameters, ns_ + ".ego.extra_right_offset", pp.extra_right_offset);
}

VelocityPlanningResult OutOfLaneModule::plan(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & ego_trajectory_points,
  const std::shared_ptr<const PlannerData> planner_data)
{
  VelocityPlanningResult result;
  autoware::universe_utils::StopWatch<std::chrono::microseconds> stopwatch;
  stopwatch.tic();
  out_of_lane::EgoData ego_data;
  ego_data.pose = planner_data->current_odometry.pose.pose;
  ego_data.trajectory_points = ego_trajectory_points;
  ego_data.first_trajectory_idx =
    autoware::motion_utils::findNearestSegmentIndex(ego_trajectory_points, ego_data.pose.position);
  ego_data.velocity = planner_data->current_odometry.twist.twist.linear.x;
  ego_data.max_decel = planner_data->velocity_smoother_->getMinDecel();
  stopwatch.tic("calculate_trajectory_footprints");
  const auto current_ego_footprint =
    out_of_lane::calculate_current_ego_footprint(ego_data, params_, true);
  const auto trajectory_footprints =
    out_of_lane::calculate_trajectory_footprints(ego_data, params_);
  const auto calculate_trajectory_footprints_us = stopwatch.toc("calculate_trajectory_footprints");
  // Calculate lanelets to ignore and consider
  stopwatch.tic("calculate_lanelets");
  const auto trajectory_lanelets =
    out_of_lane::calculate_trajectory_lanelets(ego_data, planner_data->route_handler);
  const auto ignored_lanelets = out_of_lane::calculate_ignored_lanelets(
    ego_data, trajectory_lanelets, planner_data->route_handler, params_);
  const auto other_lanelets = out_of_lane::calculate_other_lanelets(
    ego_data, trajectory_lanelets, ignored_lanelets, planner_data->route_handler, params_);
  const auto calculate_lanelets_us = stopwatch.toc("calculate_lanelets");

  debug_data_.reset_data();
  debug_data_.trajectory_points = ego_trajectory_points;
  debug_data_.footprints = trajectory_footprints;
  debug_data_.trajectory_lanelets = trajectory_lanelets;
  debug_data_.ignored_lanelets = ignored_lanelets;
  debug_data_.other_lanelets = other_lanelets;
  debug_data_.first_trajectory_idx = ego_data.first_trajectory_idx;

  if (params_.skip_if_already_overlapping) {
    debug_data_.current_footprint = current_ego_footprint;
    const auto overlapped_lanelet_it =
      std::find_if(other_lanelets.begin(), other_lanelets.end(), [&](const auto & ll) {
        return boost::geometry::intersects(ll.polygon2d().basicPolygon(), current_ego_footprint);
      });
    if (overlapped_lanelet_it != other_lanelets.end()) {
      debug_data_.current_overlapped_lanelets.push_back(*overlapped_lanelet_it);
      RCLCPP_DEBUG(logger_, "Ego is already overlapping a lane, skipping the module\n");
      return result;
    }
  }
  // Calculate overlapping ranges
  stopwatch.tic("calculate_overlapping_ranges");
  const auto ranges = out_of_lane::calculate_overlapping_ranges(
    trajectory_footprints, trajectory_lanelets, other_lanelets, params_);
  const auto calculate_overlapping_ranges_us = stopwatch.toc("calculate_overlapping_ranges");
  // Calculate stop and slowdown points
  out_of_lane::DecisionInputs inputs;
  inputs.ranges = ranges;
  inputs.ego_data = ego_data;
  stopwatch.tic("filter_predicted_objects");
  inputs.objects = out_of_lane::filter_predicted_objects(planner_data, ego_data, params_);
  const auto filter_predicted_objects_us = stopwatch.toc("filter_predicted_objects");
  inputs.route_handler = planner_data->route_handler;
  inputs.lanelets = other_lanelets;
  stopwatch.tic("calculate_decisions");
  const auto decisions = out_of_lane::calculate_decisions(inputs, params_, logger_);
  const auto calculate_decisions_us = stopwatch.toc("calculate_decisions");
  stopwatch.tic("calc_slowdown_points");
  if (  // reset the previous inserted point if the timer expired
    prev_inserted_point_ &&
    (clock_->now() - prev_inserted_point_time_).seconds() > params_.min_decision_duration)
    prev_inserted_point_.reset();
  auto point_to_insert =
    out_of_lane::calculate_slowdown_point(ego_data, decisions, prev_inserted_point_, params_);
  const auto calc_slowdown_points_us = stopwatch.toc("calc_slowdown_points");
  stopwatch.tic("insert_slowdown_points");
  debug_data_.slowdowns.clear();
  if (  // reset the timer if there is no previous inserted point or if we avoid the same lane
    point_to_insert &&
    (!prev_inserted_point_ || prev_inserted_point_->slowdown.lane_to_avoid.id() ==
                                point_to_insert->slowdown.lane_to_avoid.id()))
    prev_inserted_point_time_ = clock_->now();
  // reuse previous stop point if there is no new one or if its velocity is not higher than the new
  // one and its arc length is lower
  const auto should_use_prev_inserted_point = [&]() {
    if (
      point_to_insert && prev_inserted_point_ &&
      prev_inserted_point_->slowdown.velocity <= point_to_insert->slowdown.velocity) {
      const auto arc_length = autoware::motion_utils::calcSignedArcLength(
        ego_trajectory_points, 0LU, point_to_insert->point.pose.position);
      const auto prev_arc_length = autoware::motion_utils::calcSignedArcLength(
        ego_trajectory_points, 0LU, prev_inserted_point_->point.pose.position);
      return prev_arc_length < arc_length;
    }
    return !point_to_insert && prev_inserted_point_;
  }();
  if (should_use_prev_inserted_point) {
    // if the trajectory changed the prev point is no longer on the trajectory so we project it
    const auto insert_arc_length = autoware::motion_utils::calcSignedArcLength(
      ego_trajectory_points, 0LU, prev_inserted_point_->point.pose.position);
    prev_inserted_point_->point.pose =
      autoware::motion_utils::calcInterpolatedPose(ego_trajectory_points, insert_arc_length);
    point_to_insert = prev_inserted_point_;
  }
  if (point_to_insert) {
    prev_inserted_point_ = point_to_insert;
    RCLCPP_DEBUG(logger_, "Avoiding lane %lu", point_to_insert->slowdown.lane_to_avoid.id());
    debug_data_.slowdowns = {*point_to_insert};
    if (point_to_insert->slowdown.velocity == 0.0)
      result.stop_points.push_back(point_to_insert->point.pose.position);
    else
      result.slowdown_intervals.emplace_back(
        point_to_insert->point.pose.position, point_to_insert->point.pose.position,
        point_to_insert->slowdown.velocity);

    const auto is_approaching = autoware::motion_utils::calcSignedArcLength(
                                  ego_trajectory_points, ego_data.pose.position,
                                  point_to_insert->point.pose.position) > 0.1 &&
                                ego_data.velocity > 0.1;
    const auto status = is_approaching ? autoware::motion_utils::VelocityFactor::APPROACHING
                                       : autoware::motion_utils::VelocityFactor::STOPPED;
    velocity_factor_interface_.set(
      ego_trajectory_points, ego_data.pose, point_to_insert->point.pose, status, "out_of_lane");
    result.velocity_factor = velocity_factor_interface_.get();
  } else if (!decisions.empty()) {
    RCLCPP_WARN(logger_, "Could not insert stop point (would violate max deceleration limits)");
  }
  const auto insert_slowdown_points_us = stopwatch.toc("insert_slowdown_points");
  debug_data_.ranges = inputs.ranges;

  const auto total_time_us = stopwatch.toc();
  RCLCPP_DEBUG(
    logger_,
    "Total time = %2.2fus\n"
    "\tcalculate_lanelets = %2.0fus\n"
    "\tcalculate_trajectory_footprints = %2.0fus\n"
    "\tcalculate_overlapping_ranges = %2.0fus\n"
    "\tfilter_pred_objects = %2.0fus\n"
    "\tcalculate_decisions = %2.0fus\n"
    "\tcalc_slowdown_points = %2.0fus\n"
    "\tinsert_slowdown_points = %2.0fus\n",
    total_time_us, calculate_lanelets_us, calculate_trajectory_footprints_us,
    calculate_overlapping_ranges_us, filter_predicted_objects_us, calculate_decisions_us,
    calc_slowdown_points_us, insert_slowdown_points_us);
  debug_publisher_->publish(out_of_lane::debug::create_debug_marker_array(debug_data_));
  virtual_wall_marker_creator.add_virtual_walls(
    out_of_lane::debug::create_virtual_walls(debug_data_, params_));
  virtual_wall_publisher_->publish(virtual_wall_marker_creator.create_markers(clock_->now()));
  std::map<std::string, double> processing_times;
  processing_times["calculate_lanelets"] = calculate_lanelets_us / 1000;
  processing_times["calculate_trajectory_footprints"] = calculate_trajectory_footprints_us / 1000;
  processing_times["calculate_overlapping_ranges"] = calculate_overlapping_ranges_us / 1000;
  processing_times["filter_pred_objects"] = filter_predicted_objects_us / 1000;
  processing_times["calculate_decision"] = calculate_decisions_us / 1000;
  processing_times["calc_slowdown_points"] = calc_slowdown_points_us / 1000;
  processing_times["insert_slowdown_points"] = insert_slowdown_points_us / 1000;
  processing_times["Total"] = total_time_us / 1000;
  processing_time_publisher_->publish(processing_times);
  return result;
}

}  // namespace autoware::motion_velocity_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::motion_velocity_planner::OutOfLaneModule,
  autoware::motion_velocity_planner::PluginModuleInterface)
