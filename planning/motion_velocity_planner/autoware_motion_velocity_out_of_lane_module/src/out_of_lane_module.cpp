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
#include "filter_predicted_objects.hpp"
#include "footprint.hpp"
#include "lanelets_selection.hpp"
#include "out_of_lane_collisions.hpp"
#include "types.hpp"

#include <autoware/motion_utils/trajectory/interpolation.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/motion_velocity_planner_common/planner_data.hpp>
#include <autoware/route_handler/route_handler.hpp>
#include <autoware/universe_utils/geometry/boost_geometry.hpp>
#include <autoware/universe_utils/ros/parameter.hpp>
#include <autoware/universe_utils/ros/update_param.hpp>
#include <autoware/universe_utils/system/stop_watch.hpp>
#include <traffic_light_utils/traffic_light_utils.hpp>

#include <boost/geometry/algorithms/envelope.hpp>
#include <boost/geometry/algorithms/intersects.hpp>
#include <boost/geometry/geometries/multi_polygon.hpp>

#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>

#include <algorithm>
#include <map>
#include <memory>
#include <string>
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
  velocity_factor_interface_.init(motion_utils::PlanningBehavior::ROUTE_OBSTACLE);

  debug_publisher_ =
    node.create_publisher<visualization_msgs::msg::MarkerArray>("~/" + ns_ + "/debug_markers", 1);
  virtual_wall_publisher_ =
    node.create_publisher<visualization_msgs::msg::MarkerArray>("~/" + ns_ + "/virtual_walls", 1);
  processing_diag_publisher_ = std::make_shared<universe_utils::ProcessingTimePublisher>(
    &node, "~/debug/" + ns_ + "/processing_time_ms_diag");
  processing_time_publisher_ = node.create_publisher<tier4_debug_msgs::msg::Float64Stamped>(
    "~/debug/" + ns_ + "/processing_time_ms", 1);
}
void OutOfLaneModule::init_parameters(rclcpp::Node & node)
{
  using universe_utils::getOrDeclareParameter;
  auto & pp = params_;

  pp.mode = getOrDeclareParameter<std::string>(node, ns_ + ".mode");
  pp.skip_if_already_overlapping =
    getOrDeclareParameter<bool>(node, ns_ + ".skip_if_already_overlapping");
  pp.ignore_lane_changeable_lanelets =
    getOrDeclareParameter<bool>(node, ns_ + ".ignore_overlaps_over_lane_changeable_lanelets");
  pp.max_arc_length = getOrDeclareParameter<double>(node, ns_ + ".max_arc_length");

  pp.time_threshold = getOrDeclareParameter<double>(node, ns_ + ".threshold.time_threshold");
  pp.ttc_threshold = getOrDeclareParameter<double>(node, ns_ + ".ttc.threshold");

  pp.objects_min_vel = getOrDeclareParameter<double>(node, ns_ + ".objects.minimum_velocity");
  pp.objects_min_confidence =
    getOrDeclareParameter<double>(node, ns_ + ".objects.predicted_path_min_confidence");
  pp.objects_cut_predicted_paths_beyond_red_lights =
    getOrDeclareParameter<bool>(node, ns_ + ".objects.cut_predicted_paths_beyond_red_lights");
  pp.objects_ignore_behind_ego =
    getOrDeclareParameter<bool>(node, ns_ + ".objects.ignore_behind_ego");

  pp.precision = getOrDeclareParameter<double>(node, ns_ + ".action.precision");
  pp.min_decision_duration = getOrDeclareParameter<double>(node, ns_ + ".action.min_duration");
  pp.lon_dist_buffer =
    getOrDeclareParameter<double>(node, ns_ + ".action.longitudinal_distance_buffer");
  pp.lat_dist_buffer = getOrDeclareParameter<double>(node, ns_ + ".action.lateral_distance_buffer");
  pp.slow_velocity = getOrDeclareParameter<double>(node, ns_ + ".action.slowdown.velocity");
  pp.stop_dist_threshold =
    getOrDeclareParameter<double>(node, ns_ + ".action.stop.distance_threshold");

  pp.extra_front_offset = getOrDeclareParameter<double>(node, ns_ + ".ego.extra_front_offset");
  pp.extra_rear_offset = getOrDeclareParameter<double>(node, ns_ + ".ego.extra_rear_offset");
  pp.extra_left_offset = getOrDeclareParameter<double>(node, ns_ + ".ego.extra_left_offset");
  pp.extra_right_offset = getOrDeclareParameter<double>(node, ns_ + ".ego.extra_right_offset");
  const auto vehicle_info = vehicle_info_utils::VehicleInfoUtils(node).getVehicleInfo();
  pp.front_offset = vehicle_info.max_longitudinal_offset_m;
  pp.rear_offset = vehicle_info.min_longitudinal_offset_m;
  pp.left_offset = vehicle_info.max_lateral_offset_m;
  pp.right_offset = vehicle_info.min_lateral_offset_m;
}

void OutOfLaneModule::update_parameters(const std::vector<rclcpp::Parameter> & parameters)
{
  using universe_utils::updateParam;
  auto & pp = params_;
  updateParam(parameters, ns_ + ".mode", pp.mode);
  updateParam(parameters, ns_ + ".skip_if_already_overlapping", pp.skip_if_already_overlapping);
  updateParam(parameters, ns_ + ".max_arc_length", pp.max_arc_length);
  updateParam(
    parameters, ns_ + ".ignore_overlaps_over_lane_changeable_lanelets",
    pp.ignore_lane_changeable_lanelets);

  updateParam(parameters, ns_ + ".threshold.time_threshold", pp.time_threshold);
  updateParam(parameters, ns_ + ".ttc.threshold", pp.ttc_threshold);

  updateParam(parameters, ns_ + ".objects.minimum_velocity", pp.objects_min_vel);
  updateParam(
    parameters, ns_ + ".objects.predicted_path_min_confidence", pp.objects_min_confidence);
  updateParam(
    parameters, ns_ + ".objects.cut_predicted_paths_beyond_red_lights",
    pp.objects_cut_predicted_paths_beyond_red_lights);
  updateParam(parameters, ns_ + ".objects.ignore_behind_ego", pp.objects_ignore_behind_ego);

  updateParam(parameters, ns_ + ".action.precision", pp.precision);
  updateParam(parameters, ns_ + ".action.min_duration", pp.min_decision_duration);
  updateParam(parameters, ns_ + ".action.longitudinal_distance_buffer", pp.lon_dist_buffer);
  updateParam(parameters, ns_ + ".action.lateral_distance_buffer", pp.lat_dist_buffer);
  updateParam(parameters, ns_ + ".action.slowdown.velocity", pp.slow_velocity);
  updateParam(parameters, ns_ + ".action.stop.distance_threshold", pp.stop_dist_threshold);

  updateParam(parameters, ns_ + ".ego.extra_front_offset", pp.extra_front_offset);
  updateParam(parameters, ns_ + ".ego.extra_rear_offset", pp.extra_rear_offset);
  updateParam(parameters, ns_ + ".ego.extra_left_offset", pp.extra_left_offset);
  updateParam(parameters, ns_ + ".ego.extra_right_offset", pp.extra_right_offset);
}

void OutOfLaneModule::limit_trajectory_size(
  out_of_lane::EgoData & ego_data,
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & ego_trajectory_points,
  const double max_arc_length)
{
  ego_data.first_trajectory_idx =
    motion_utils::findNearestSegmentIndex(ego_trajectory_points, ego_data.pose.position);
  ego_data.longitudinal_offset_to_first_trajectory_index =
    motion_utils::calcLongitudinalOffsetToSegment(
      ego_trajectory_points, ego_data.first_trajectory_idx, ego_data.pose.position);
  auto l = -ego_data.longitudinal_offset_to_first_trajectory_index;
  ego_data.trajectory_points.push_back(ego_trajectory_points[ego_data.first_trajectory_idx]);
  for (auto i = ego_data.first_trajectory_idx + 1; i < ego_trajectory_points.size(); ++i) {
    l += universe_utils::calcDistance2d(ego_trajectory_points[i - 1], ego_trajectory_points[i]);
    if (l >= max_arc_length) {
      break;
    }
    ego_data.trajectory_points.push_back(ego_trajectory_points[i]);
  }
}

void OutOfLaneModule::calculate_min_stop_and_slowdown_distances(
  out_of_lane::EgoData & ego_data, const PlannerData & planner_data,
  std::optional<geometry_msgs::msg::Pose> & previous_slowdown_pose_, const double slow_velocity)
{
  ego_data.min_stop_distance = planner_data.calculate_min_deceleration_distance(0.0).value_or(0.0);
  ego_data.min_slowdown_distance =
    planner_data.calculate_min_deceleration_distance(slow_velocity).value_or(0.0);
  if (previous_slowdown_pose_) {
    // Ensure we do not remove the previous slowdown point due to the min distance limit
    const auto previous_slowdown_pose_arc_length = motion_utils::calcSignedArcLength(
      ego_data.trajectory_points, ego_data.first_trajectory_idx, previous_slowdown_pose_->position);
    ego_data.min_stop_distance =
      std::min(previous_slowdown_pose_arc_length, ego_data.min_stop_distance);
    ego_data.min_slowdown_distance =
      std::min(previous_slowdown_pose_arc_length, ego_data.min_slowdown_distance);
  }
  ego_data.min_stop_arc_length = motion_utils::calcSignedArcLength(
                                   ego_data.trajectory_points, 0UL, ego_data.first_trajectory_idx) +
                                 ego_data.longitudinal_offset_to_first_trajectory_index +
                                 ego_data.min_stop_distance;
}

void prepare_stop_lines_rtree(
  out_of_lane::EgoData & ego_data, const PlannerData & planner_data, const double search_distance)
{
  std::vector<out_of_lane::StopLineNode> rtree_nodes;
  const auto bbox = lanelet::BoundingBox2d(
    lanelet::BasicPoint2d{
      ego_data.pose.position.x - search_distance, ego_data.pose.position.y - search_distance},
    lanelet::BasicPoint2d{
      ego_data.pose.position.x + search_distance, ego_data.pose.position.y + search_distance});
  out_of_lane::StopLineNode stop_line_node;
  for (const auto & ll :
       planner_data.route_handler->getLaneletMapPtr()->laneletLayer.search(bbox)) {
    for (const auto & element : ll.regulatoryElementsAs<lanelet::TrafficLight>()) {
      const auto traffic_signal_stamped = planner_data.get_traffic_signal(element->id());
      if (
        traffic_signal_stamped.has_value() && element->stopLine().has_value() &&
        traffic_light_utils::isTrafficSignalStop(ll, traffic_signal_stamped.value().signal)) {
        stop_line_node.second.stop_line.clear();
        for (const auto & p : element->stopLine()->basicLineString()) {
          stop_line_node.second.stop_line.emplace_back(p.x(), p.y());
        }
        // use a longer stop line to also cut predicted paths that slightly go around the stop line
        const auto diff =
          stop_line_node.second.stop_line.back() - stop_line_node.second.stop_line.front();
        stop_line_node.second.stop_line.front() -= diff * 0.5;
        stop_line_node.second.stop_line.back() += diff * 0.5;
        stop_line_node.second.lanelets = planner_data.route_handler->getPreviousLanelets(ll);
        stop_line_node.first =
          boost::geometry::return_envelope<universe_utils::Box2d>(stop_line_node.second.stop_line);
        rtree_nodes.push_back(stop_line_node);
      }
    }
  }
  ego_data.stop_lines_rtree = {rtree_nodes.begin(), rtree_nodes.end()};
}

out_of_lane::OutOfLaneData prepare_out_of_lane_data(
  const out_of_lane::EgoData & ego_data, const route_handler::RouteHandler & route_handler)
{
  out_of_lane::OutOfLaneData out_of_lane_data;
  out_of_lane_data.outside_points = out_of_lane::calculate_out_of_lane_points(ego_data);
  out_of_lane::calculate_overlapped_lanelets(out_of_lane_data, route_handler);
  out_of_lane::prepare_out_of_lane_areas_rtree(out_of_lane_data);
  return out_of_lane_data;
}

VelocityPlanningResult OutOfLaneModule::plan(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & ego_trajectory_points,
  const std::shared_ptr<const PlannerData> planner_data)
{
  VelocityPlanningResult result;
  universe_utils::StopWatch<std::chrono::microseconds> stopwatch;
  stopwatch.tic();

  stopwatch.tic("preprocessing");
  out_of_lane::EgoData ego_data;
  ego_data.pose = planner_data->current_odometry.pose.pose;
  limit_trajectory_size(ego_data, ego_trajectory_points, params_.max_arc_length);
  calculate_min_stop_and_slowdown_distances(
    ego_data, *planner_data, previous_slowdown_pose_, params_.slow_velocity);
  prepare_stop_lines_rtree(ego_data, *planner_data, params_.max_arc_length);
  const auto preprocessing_us = stopwatch.toc("preprocessing");

  stopwatch.tic("calculate_trajectory_footprints");
  ego_data.current_footprint =
    out_of_lane::calculate_current_ego_footprint(ego_data, params_, true);
  ego_data.trajectory_footprints = out_of_lane::calculate_trajectory_footprints(ego_data, params_);
  const auto calculate_trajectory_footprints_us = stopwatch.toc("calculate_trajectory_footprints");

  stopwatch.tic("calculate_lanelets");
  out_of_lane::calculate_drivable_lane_polygons(ego_data, *planner_data->route_handler);
  const auto calculate_lanelets_us = stopwatch.toc("calculate_lanelets");

  stopwatch.tic("calculate_out_of_lane_areas");
  auto out_of_lane_data = prepare_out_of_lane_data(ego_data, *planner_data->route_handler);
  const auto calculate_out_of_lane_areas_us = stopwatch.toc("calculate_out_of_lane_areas");

  stopwatch.tic("filter_predicted_objects");
  const auto objects = out_of_lane::filter_predicted_objects(*planner_data, ego_data, params_);
  const auto filter_predicted_objects_us = stopwatch.toc("filter_predicted_objects");

  stopwatch.tic("calculate_time_collisions");
  out_of_lane::calculate_objects_time_collisions(out_of_lane_data, objects.objects);
  const auto calculate_time_collisions_us = stopwatch.toc("calculate_time_collisions");

  stopwatch.tic("calculate_times");
  // calculate times
  out_of_lane::calculate_collisions_to_avoid(out_of_lane_data, ego_data.trajectory_points, params_);
  const auto calculate_times_us = stopwatch.toc("calculate_times");

  if (
    params_.skip_if_already_overlapping && !ego_data.drivable_lane_polygons.empty() &&
    !lanelet::geometry::within(ego_data.current_footprint, ego_data.drivable_lane_polygons)) {
    RCLCPP_WARN(logger_, "Ego is already out of lane, skipping the module\n");
    debug_publisher_->publish(out_of_lane::debug::create_debug_marker_array(
      ego_data, out_of_lane_data, objects, debug_data_));
    return result;
  }

  if (  // reset the previous inserted point if the timer expired
    previous_slowdown_pose_ &&
    (clock_->now() - previous_slowdown_time_).seconds() > params_.min_decision_duration) {
    previous_slowdown_pose_.reset();
  }

  stopwatch.tic("calculate_slowdown_point");
  auto slowdown_pose = out_of_lane::calculate_slowdown_point(ego_data, out_of_lane_data, params_);
  const auto calculate_slowdown_point_us = stopwatch.toc("calculate_slowdown_point");

  if (  // reset the timer if there is no previous inserted point
    slowdown_pose && (!previous_slowdown_pose_)) {
    previous_slowdown_time_ = clock_->now();
  }
  // reuse previous stop pose if there is no new one or if its velocity is not higher than the new
  // one and its arc length is lower
  const auto should_use_previous_pose = [&]() {
    if (slowdown_pose && previous_slowdown_pose_) {
      const auto arc_length =
        motion_utils::calcSignedArcLength(ego_trajectory_points, 0LU, slowdown_pose->position);
      const auto prev_arc_length = motion_utils::calcSignedArcLength(
        ego_trajectory_points, 0LU, previous_slowdown_pose_->position);
      return prev_arc_length < arc_length;
    }
    return !slowdown_pose && previous_slowdown_pose_;
  }();
  if (should_use_previous_pose) {
    // if the trajectory changed the prev point is no longer on the trajectory so we project it
    const auto new_arc_length = motion_utils::calcSignedArcLength(
      ego_trajectory_points, ego_data.first_trajectory_idx, previous_slowdown_pose_->position);
    slowdown_pose = motion_utils::calcInterpolatedPose(ego_trajectory_points, new_arc_length);
  }
  if (slowdown_pose) {
    const auto arc_length =
      motion_utils::calcSignedArcLength(
        ego_trajectory_points, ego_data.first_trajectory_idx, slowdown_pose->position) -
      ego_data.longitudinal_offset_to_first_trajectory_index;
    const auto slowdown_velocity =
      arc_length <= params_.stop_dist_threshold ? 0.0 : params_.slow_velocity;
    previous_slowdown_pose_ = slowdown_pose;
    if (slowdown_velocity == 0.0) {
      result.stop_points.push_back(slowdown_pose->position);
    } else {
      result.slowdown_intervals.emplace_back(
        slowdown_pose->position, slowdown_pose->position, slowdown_velocity);
    }

    const auto is_approaching =
      motion_utils::calcSignedArcLength(
        ego_trajectory_points, ego_data.pose.position, slowdown_pose->position) > 0.1 &&
      planner_data->current_odometry.twist.twist.linear.x > 0.1;
    const auto status = is_approaching ? motion_utils::VelocityFactor::APPROACHING
                                       : motion_utils::VelocityFactor::STOPPED;
    velocity_factor_interface_.set(
      ego_trajectory_points, ego_data.pose, *slowdown_pose, status, "out_of_lane");
    result.velocity_factor = velocity_factor_interface_.get();
    virtual_wall_marker_creator.add_virtual_walls(
      out_of_lane::debug::create_virtual_walls(*slowdown_pose, slowdown_velocity == 0.0, params_));
    virtual_wall_publisher_->publish(virtual_wall_marker_creator.create_markers(clock_->now()));
  } else if (std::any_of(
               out_of_lane_data.outside_points.begin(), out_of_lane_data.outside_points.end(),
               [](const auto & p) { return p.to_avoid; })) {
    RCLCPP_WARN(
      logger_, "[out_of_lane] Could not insert slowdown point because of deceleration limits");
  }

  stopwatch.tic("gen_debug");
  const auto markers =
    out_of_lane::debug::create_debug_marker_array(ego_data, out_of_lane_data, objects, debug_data_);
  const auto markers_us = stopwatch.toc("gen_debug");
  stopwatch.tic("pub");
  debug_publisher_->publish(markers);
  const auto pub_markers_us = stopwatch.toc("pub");
  const auto total_time_us = stopwatch.toc();
  std::map<std::string, double> processing_times;
  processing_times["preprocessing"] = preprocessing_us / 1000;
  processing_times["calculate_lanelets"] = calculate_lanelets_us / 1000;
  processing_times["calculate_trajectory_footprints"] = calculate_trajectory_footprints_us / 1000;
  processing_times["calculate_out_of_lane_areas"] = calculate_out_of_lane_areas_us / 1000;
  processing_times["filter_pred_objects"] = filter_predicted_objects_us / 1000;
  processing_times["calculate_time_collisions"] = calculate_time_collisions_us / 1000;
  processing_times["calculate_times"] = calculate_times_us / 1000;
  processing_times["calculate_slowdown_point"] = calculate_slowdown_point_us / 1000;
  processing_times["generate_markers"] = markers_us / 1000;
  processing_times["publish_markers"] = pub_markers_us / 1000;
  processing_times["Total"] = total_time_us / 1000;
  processing_diag_publisher_->publish(processing_times);
  tier4_debug_msgs::msg::Float64Stamped processing_time_msg;
  processing_time_msg.stamp = clock_->now();
  processing_time_msg.data = processing_times["Total"];
  processing_time_publisher_->publish(processing_time_msg);
  return result;
}

}  // namespace autoware::motion_velocity_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::motion_velocity_planner::OutOfLaneModule,
  autoware::motion_velocity_planner::PluginModuleInterface)
