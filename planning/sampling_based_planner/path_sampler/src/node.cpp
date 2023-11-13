// Copyright 2023 TIER IV, Inc.
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

#include "path_sampler/node.hpp"

#include "interpolation/spline_interpolation_points_2d.hpp"
#include "motion_utils/marker/marker_helper.hpp"
#include "path_sampler/path_generation.hpp"
#include "path_sampler/prepare_inputs.hpp"
#include "path_sampler/utils/geometry_utils.hpp"
#include "path_sampler/utils/trajectory_utils.hpp"
#include "rclcpp/time.hpp"
#include "sampler_common/constraints/hard_constraint.hpp"
#include "sampler_common/constraints/soft_constraint.hpp"

#include <boost/geometry/algorithms/distance.hpp>

#include <chrono>
#include <limits>

namespace path_sampler
{
namespace
{
template <class T>
std::vector<T> concatVectors(const std::vector<T> & prev_vector, const std::vector<T> & next_vector)
{
  std::vector<T> concatenated_vector;
  concatenated_vector.insert(concatenated_vector.end(), prev_vector.begin(), prev_vector.end());
  concatenated_vector.insert(concatenated_vector.end(), next_vector.begin(), next_vector.end());
  return concatenated_vector;
}

StringStamped createStringStamped(const rclcpp::Time & now, const std::string & data)
{
  StringStamped msg;
  msg.stamp = now;
  msg.data = data;
  return msg;
}

bool hasZeroVelocity(const TrajectoryPoint & traj_point)
{
  constexpr double zero_vel = 0.0001;
  return std::abs(traj_point.longitudinal_velocity_mps) < zero_vel;
}
}  // namespace

PathSampler::PathSampler(const rclcpp::NodeOptions & node_options)
: Node("path_sampler", node_options),
  vehicle_info_(vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo()),
  time_keeper_ptr_(std::make_shared<TimeKeeper>())
{
  // interface publisher
  traj_pub_ = create_publisher<Trajectory>("~/output/path", 1);
  virtual_wall_pub_ = create_publisher<MarkerArray>("~/virtual_wall", 1);

  // interface subscriber
  path_sub_ = create_subscription<Path>(
    "~/input/path", 1, std::bind(&PathSampler::onPath, this, std::placeholders::_1));
  odom_sub_ = create_subscription<Odometry>(
    "~/input/odometry", 1, [this](const Odometry::SharedPtr msg) { ego_state_ptr_ = msg; });
  objects_sub_ = create_subscription<PredictedObjects>(
    "~/input/objects", 1, std::bind(&PathSampler::objectsCallback, this, std::placeholders::_1));

  // debug publisher
  debug_markers_pub_ = create_publisher<MarkerArray>("~/debug/marker", 1);
  debug_calculation_time_pub_ = create_publisher<StringStamped>("~/debug/calculation_time", 1);

  {  // parameters
    params_.constraints.hard.max_curvature =
      declare_parameter<double>("constraints.hard.max_curvature");
    params_.constraints.hard.min_curvature =
      declare_parameter<double>("constraints.hard.min_curvature");
    params_.constraints.soft.lateral_deviation_weight =
      declare_parameter<double>("constraints.soft.lateral_deviation_weight");
    params_.constraints.soft.length_weight =
      declare_parameter<double>("constraints.soft.length_weight");
    params_.constraints.soft.curvature_weight =
      declare_parameter<double>("constraints.soft.curvature_weight");
    params_.sampling.enable_frenet = declare_parameter<bool>("sampling.enable_frenet");
    params_.sampling.enable_bezier = declare_parameter<bool>("sampling.enable_bezier");
    params_.sampling.resolution = declare_parameter<double>("sampling.resolution");
    params_.sampling.previous_path_reuse_points_nb =
      declare_parameter<int>("sampling.previous_path_reuse_points_nb");
    params_.sampling.target_lengths =
      declare_parameter<std::vector<double>>("sampling.target_lengths");
    params_.sampling.target_lateral_positions =
      declare_parameter<std::vector<double>>("sampling.target_lateral_positions");
    params_.sampling.nb_target_lateral_positions =
      declare_parameter<int>("sampling.nb_target_lateral_positions");
    params_.sampling.frenet.target_lateral_velocities =
      declare_parameter<std::vector<double>>("sampling.frenet.target_lateral_velocities");
    params_.sampling.frenet.target_lateral_accelerations =
      declare_parameter<std::vector<double>>("sampling.frenet.target_lateral_accelerations");
    params_.sampling.bezier.nb_k = declare_parameter<int>("sampling.bezier.nb_k");
    params_.sampling.bezier.mk_min = declare_parameter<double>("sampling.bezier.mk_min");
    params_.sampling.bezier.mk_max = declare_parameter<double>("sampling.bezier.mk_max");
    params_.sampling.bezier.nb_t = declare_parameter<int>("sampling.bezier.nb_t");
    params_.sampling.bezier.mt_min = declare_parameter<double>("sampling.bezier.mt_min");
    params_.sampling.bezier.mt_max = declare_parameter<double>("sampling.bezier.mt_max");
    params_.preprocessing.force_zero_deviation =
      declare_parameter<bool>("preprocessing.force_zero_initial_deviation");
    params_.preprocessing.force_zero_heading =
      declare_parameter<bool>("preprocessing.force_zero_initial_heading");
    params_.preprocessing.smooth_reference =
      declare_parameter<bool>("preprocessing.smooth_reference_trajectory");
    params_.constraints.ego_footprint = vehicle_info_.createFootprint();
    params_.constraints.ego_width = vehicle_info_.vehicle_width_m;
    params_.constraints.ego_length = vehicle_info_.vehicle_length_m;

    // parameter for debug info
    time_keeper_ptr_->enable_calculation_time_info =
      declare_parameter<bool>("debug.enable_calculation_time_info");
    debug_id_ = static_cast<size_t>(declare_parameter<int>("debug.id"));

    // parameters for ego nearest search
    ego_nearest_param_ = EgoNearestParam(this);

    // parameters for trajectory
    traj_param_ = TrajectoryParam(this);
  }

  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&PathSampler::onParam, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult PathSampler::onParam(
  const std::vector<rclcpp::Parameter> & parameters)
{
  using tier4_autoware_utils::updateParam;

  updateParam(parameters, "constraints.hard.max_curvature", params_.constraints.hard.max_curvature);
  updateParam(parameters, "constraints.hard.min_curvature", params_.constraints.hard.min_curvature);
  updateParam(
    parameters, "constraints.soft.lateral_deviation_weight",
    params_.constraints.soft.lateral_deviation_weight);
  updateParam(parameters, "constraints.soft.length_weight", params_.constraints.soft.length_weight);
  updateParam(
    parameters, "constraints.soft.curvature_weight", params_.constraints.soft.curvature_weight);
  updateParam(parameters, "sampling.enable_frenet", params_.sampling.enable_frenet);
  updateParam(parameters, "sampling.enable_bezier", params_.sampling.enable_bezier);
  updateParam(parameters, "sampling.resolution", params_.sampling.resolution);
  updateParam(
    parameters, "sampling.previous_path_reuse_points_nb",
    params_.sampling.previous_path_reuse_points_nb);
  updateParam(parameters, "sampling.target_lengths", params_.sampling.target_lengths);
  updateParam(
    parameters, "sampling.target_lateral_positions", params_.sampling.target_lateral_positions);
  updateParam(
    parameters, "sampling.nb_target_lateral_positions",
    params_.sampling.nb_target_lateral_positions);
  updateParam(
    parameters, "sampling.frenet.target_lateral_velocities",
    params_.sampling.frenet.target_lateral_velocities);
  updateParam(
    parameters, "sampling.frenet.target_lateral_accelerations",
    params_.sampling.frenet.target_lateral_accelerations);
  updateParam(parameters, "sampling.bezier.nb_k", params_.sampling.bezier.nb_k);
  updateParam(parameters, "sampling.bezier.mk_min", params_.sampling.bezier.mk_min);
  updateParam(parameters, "sampling.bezier.mk_max", params_.sampling.bezier.mk_max);
  updateParam(parameters, "sampling.bezier.nb_t", params_.sampling.bezier.nb_t);
  updateParam(parameters, "sampling.bezier.mt_min", params_.sampling.bezier.mt_min);
  updateParam(parameters, "sampling.bezier.mt_max", params_.sampling.bezier.mt_max);
  updateParam(
    parameters, "preprocessing.force_zero_initial_deviation",
    params_.preprocessing.force_zero_deviation);
  updateParam(
    parameters, "preprocessing.force_zero_initial_heading",
    params_.preprocessing.force_zero_heading);
  updateParam(
    parameters, "preprocessing.smooth_reference_trajectory",
    params_.preprocessing.smooth_reference);
  updateParam(
    parameters, "debug.enable_calculation_time_info",
    time_keeper_ptr_->enable_calculation_time_info);
  updateParam(parameters, "debug.id", debug_id_);
  // parameters for ego nearest search
  ego_nearest_param_.onParam(parameters);

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

void PathSampler::resetPreviousData()
{
  prev_path_.reset();
}

void PathSampler::objectsCallback(const PredictedObjects::SharedPtr msg)
{
  in_objects_ptr_ = msg;
}

sampler_common::State PathSampler::getPlanningState(
  sampler_common::State & state, const sampler_common::transform::Spline2D & path_spline) const
{
  state.frenet = path_spline.frenet(state.pose);
  if (params_.preprocessing.force_zero_deviation) {
    state.pose = path_spline.cartesian(state.frenet.s);
  }
  if (params_.preprocessing.force_zero_heading) {
    state.heading = path_spline.yaw(state.frenet.s);
  }
  state.curvature = path_spline.curvature(state.frenet.s);
  return state;
}

void PathSampler::onPath(const Path::SharedPtr path_ptr)
{
  time_keeper_ptr_->init();
  time_keeper_ptr_->tic(__func__);

  // check if data is ready and valid
  if (!isDataReady(*path_ptr, *get_clock())) {
    return;
  }

  // 0. return if path is backward
  // TODO(Maxime): support backward path
  if (!motion_utils::isDrivingForward(path_ptr->points)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Backward path is NOT supported. Just converting path to trajectory");
  }
  // 1. create planner data
  const auto planner_data = createPlannerData(*path_ptr);
  // 2. generate trajectory
  const auto generated_traj_points = generateTrajectory(planner_data);
  // 3. extend trajectory to connect the optimized trajectory and the following path smoothly
  if (!generated_traj_points.empty()) {
    auto full_traj_points = extendTrajectory(planner_data.traj_points, generated_traj_points);
    const auto output_traj_msg =
      trajectory_utils::createTrajectory(path_ptr->header, full_traj_points);
    traj_pub_->publish(output_traj_msg);
  } else {
    auto stopping_traj = trajectory_utils::convertToTrajectoryPoints(planner_data.traj_points);
    for (auto & p : stopping_traj) p.longitudinal_velocity_mps = 0.0;
    const auto output_traj_msg =
      trajectory_utils::createTrajectory(path_ptr->header, stopping_traj);
    traj_pub_->publish(output_traj_msg);
  }

  time_keeper_ptr_->toc(__func__, "");
  *time_keeper_ptr_ << "========================================";
  time_keeper_ptr_->endLine();

  // publish calculation_time
  // NOTE: This function must be called after measuring onPath calculation time
  const auto calculation_time_msg = createStringStamped(now(), time_keeper_ptr_->getLog());
  debug_calculation_time_pub_->publish(calculation_time_msg);
}

bool PathSampler::isDataReady(const Path & path, rclcpp::Clock clock) const
{
  if (!ego_state_ptr_) {
    RCLCPP_INFO_SKIPFIRST_THROTTLE(get_logger(), clock, 5000, "Waiting for ego pose and twist.");
    return false;
  }

  if (path.points.size() < 2) {
    RCLCPP_INFO_SKIPFIRST_THROTTLE(get_logger(), clock, 5000, "Path points size is less than 1.");
    return false;
  }

  if (path.left_bound.empty() || path.right_bound.empty()) {
    RCLCPP_INFO_SKIPFIRST_THROTTLE(
      get_logger(), clock, 5000, "Left or right bound in path is empty.");
    return false;
  }

  return true;
}

PlannerData PathSampler::createPlannerData(const Path & path) const
{
  // create planner data
  PlannerData planner_data;
  planner_data.header = path.header;
  planner_data.traj_points = trajectory_utils::convertToTrajectoryPoints(path.points);
  planner_data.left_bound = path.left_bound;
  planner_data.right_bound = path.right_bound;
  planner_data.ego_pose = ego_state_ptr_->pose.pose;
  planner_data.ego_vel = ego_state_ptr_->twist.twist.linear.x;
  return planner_data;
}

void PathSampler::copyZ(
  const std::vector<TrajectoryPoint> & from_traj, std::vector<TrajectoryPoint> & to_traj)
{
  if (from_traj.empty() || to_traj.empty()) return;
  to_traj.front().pose.position.z = from_traj.front().pose.position.z;
  if (from_traj.size() < 2 || to_traj.size() < 2) return;
  auto from = from_traj.begin() + 1;
  auto s_from = tier4_autoware_utils::calcDistance2d(from->pose, std::next(from)->pose);
  auto s_to = 0.0;
  auto s_from_prev = 0.0;
  for (auto to = to_traj.begin() + 1; to + 1 != to_traj.end(); ++to) {
    s_to += tier4_autoware_utils::calcDistance2d(std::prev(to)->pose, to->pose);
    for (; s_from < s_to && from + 1 != from_traj.end(); ++from) {
      s_from_prev = s_from;
      s_from += tier4_autoware_utils::calcDistance2d(from->pose, std::next(from)->pose);
    }
    const auto ratio = (s_to - s_from_prev) / (s_from - s_from_prev);
    to->pose.position.z = std::prev(from)->pose.position.z +
                          ratio * (from->pose.position.z - std::prev(from)->pose.position.z);
  }
  to_traj.back().pose.position.z = from->pose.position.z;
}

void PathSampler::copyVelocity(
  const std::vector<TrajectoryPoint> & from_traj, std::vector<TrajectoryPoint> & to_traj,
  const geometry_msgs::msg::Pose & ego_pose)
{
  if (to_traj.empty() || from_traj.empty()) return;

  const auto closest_fn = [&](const auto & p1, const auto & p2) {
    return tier4_autoware_utils::calcDistance2d(p1.pose, ego_pose) <=
           tier4_autoware_utils::calcDistance2d(p2.pose, ego_pose);
  };
  const auto first_from = std::min_element(from_traj.begin(), from_traj.end() - 1, closest_fn);
  const auto first_to = std::min_element(to_traj.begin(), to_traj.end() - 1, closest_fn);

  auto to = to_traj.begin();
  for (; to != first_to; ++to)
    to->longitudinal_velocity_mps = first_from->longitudinal_velocity_mps;

  auto from = first_from;
  auto s_from = tier4_autoware_utils::calcDistance2d(from->pose, std::next(from)->pose);
  auto s_to = 0.0;
  auto s_from_prev = 0.0;
  for (; to + 1 != to_traj.end(); ++to) {
    s_to += tier4_autoware_utils::calcDistance2d(to->pose, std::next(to)->pose);
    for (; s_from < s_to && from + 1 != from_traj.end(); ++from) {
      s_from_prev = s_from;
      s_from += tier4_autoware_utils::calcDistance2d(from->pose, std::next(from)->pose);
    }
    if (
      from->longitudinal_velocity_mps == 0.0 || std::prev(from)->longitudinal_velocity_mps == 0.0) {
      to->longitudinal_velocity_mps = 0.0;
    } else {
      const auto ratio = (s_to - s_from_prev) / (s_from - s_from_prev);
      to->longitudinal_velocity_mps =
        std::prev(from)->longitudinal_velocity_mps +
        ratio * (from->longitudinal_velocity_mps - std::prev(from)->longitudinal_velocity_mps);
    }
  }
  to_traj.back().longitudinal_velocity_mps = from->longitudinal_velocity_mps;
}

std::vector<TrajectoryPoint> PathSampler::generateTrajectory(const PlannerData & planner_data)
{
  time_keeper_ptr_->tic(__func__);

  const auto & input_traj_points = planner_data.traj_points;

  auto generated_traj_points = generatePath(planner_data);

  copyVelocity(input_traj_points, generated_traj_points, planner_data.ego_pose);
  copyZ(input_traj_points, generated_traj_points);
  publishDebugMarker(generated_traj_points);

  time_keeper_ptr_->toc(__func__, " ");
  return generated_traj_points;
}

std::vector<TrajectoryPoint> PathSampler::generatePath(const PlannerData & planner_data)
{
  std::vector<TrajectoryPoint> trajectory;
  time_keeper_ptr_->tic(__func__);
  const auto & p = planner_data;

  const auto path_spline = preparePathSpline(p.traj_points, params_.preprocessing.smooth_reference);
  sampler_common::State current_state;
  current_state.pose = {planner_data.ego_pose.position.x, planner_data.ego_pose.position.y};
  current_state.heading = tf2::getYaw(planner_data.ego_pose.orientation);

  const auto planning_state = getPlanningState(current_state, path_spline);
  prepareConstraints(params_.constraints, *in_objects_ptr_, p.left_bound, p.right_bound);

  auto candidate_paths = generateCandidatePaths(planning_state, path_spline, 0.0, params_);
  if (prev_path_ && prev_path_->lengths.size() > 1) {
    // Update previous path
    constexpr auto max_deviation = 2.0;  // [m] TODO(Maxime): param
    const auto closest_iter = std::min_element(
      prev_path_->points.begin(), prev_path_->points.end() - 1,
      [&](const auto & p1, const auto & p2) {
        return boost::geometry::distance(p1, current_state.pose) <=
               boost::geometry::distance(p2, current_state.pose);
      });
    if (
      closest_iter != prev_path_->points.end() &&
      boost::geometry::distance(*closest_iter, current_state.pose) <= max_deviation) {
      const auto current_idx = std::distance(prev_path_->points.begin(), closest_iter);
      const auto length_offset = prev_path_->lengths[current_idx];
      for (auto & l : prev_path_->lengths) l -= length_offset;
      constexpr auto behind_dist = -5.0;  // [m] TODO(Maxime): param
      auto behind_idx = current_idx;
      while (behind_idx > 0 && prev_path_->lengths[behind_idx] > behind_dist) --behind_idx;
      *prev_path_ = *prev_path_->subset(behind_idx, prev_path_->points.size());

      // Use previous path for replanning
      const auto prev_path_length = prev_path_->lengths.back();
      const auto reuse_step = prev_path_length / params_.sampling.previous_path_reuse_points_nb;
      for (double reuse_length = reuse_step; reuse_length <= prev_path_length;
           reuse_length += reuse_step) {
        sampler_common::State reuse_state;
        size_t reuse_idx = 0;
        for (reuse_idx = 0; reuse_idx + 1 < prev_path_->lengths.size() &&
                            prev_path_->lengths[reuse_idx] < reuse_length;
             ++reuse_idx)
          ;
        if (reuse_idx == 0UL) continue;
        const auto reused_path = *prev_path_->subset(0UL, reuse_idx);
        reuse_state.curvature = reused_path.curvatures.back();
        reuse_state.pose = reused_path.points.back();
        reuse_state.heading = reused_path.yaws.back();
        reuse_state.frenet = path_spline.frenet(reuse_state.pose);
        auto paths = generateCandidatePaths(reuse_state, path_spline, reuse_length, params_);
        for (auto & p : paths) candidate_paths.push_back(reused_path.extend(p));
      }
    } else {
      resetPreviousData();
    }
  }
  debug_data_.footprints.clear();
  for (auto & path : candidate_paths) {
    const auto footprint =
      sampler_common::constraints::checkHardConstraints(path, params_.constraints);
    debug_data_.footprints.push_back(footprint);
    sampler_common::constraints::calculateCost(path, params_.constraints, path_spline);
  }
  const auto best_path_idx = [](const auto & paths) {
    auto min_cost = std::numeric_limits<double>::max();
    size_t best_path_idx = 0;
    for (auto i = 0LU; i < paths.size(); ++i) {
      if (paths[i].constraint_results.isValid() && paths[i].cost < min_cost) {
        best_path_idx = i;
        min_cost = paths[i].cost;
      }
    }
    return min_cost < std::numeric_limits<double>::max() ? std::optional<size_t>(best_path_idx)
                                                         : std::nullopt;
  };
  const auto selected_path_idx = best_path_idx(candidate_paths);
  if (selected_path_idx) {
    const auto & selected_path = candidate_paths[*selected_path_idx];
    trajectory = trajectory_utils::convertToTrajectoryPoints(selected_path);
    prev_path_ = selected_path;
  } else {
    RCLCPP_WARN(
      get_logger(), "No valid path found (out of %lu) outputting %s\n", candidate_paths.size(),
      prev_path_ ? "previous path" : "stopping path");
    int coll = 0;
    int da = 0;
    int k = 0;
    for (const auto & p : candidate_paths) {
      coll += static_cast<int>(!p.constraint_results.collision);
      da += static_cast<int>(!p.constraint_results.drivable_area);
      k += static_cast<int>(!p.constraint_results.curvature);
    }
    RCLCPP_WARN(get_logger(), "\tInvalid coll/da/k = %d/%d/%d\n", coll, da, k);
    if (prev_path_) trajectory = trajectory_utils::convertToTrajectoryPoints(*prev_path_);
  }
  time_keeper_ptr_->toc(__func__, "    ");
  debug_data_.previous_sampled_candidates_nb = debug_data_.sampled_candidates.size();
  debug_data_.sampled_candidates = candidate_paths;
  debug_data_.obstacles = params_.constraints.obstacle_polygons;
  return trajectory;
}

void PathSampler::publishVirtualWall(const geometry_msgs::msg::Pose & stop_pose) const
{
  time_keeper_ptr_->tic(__func__);

  const auto virtual_wall_marker = motion_utils::createStopVirtualWallMarker(
    stop_pose, "outside drivable area", now(), 0, vehicle_info_.max_longitudinal_offset_m);

  virtual_wall_pub_->publish(virtual_wall_marker);
  time_keeper_ptr_->toc(__func__, "      ");
}

void PathSampler::publishDebugMarker(const std::vector<TrajectoryPoint> & traj_points) const
{
  (void)traj_points;

  time_keeper_ptr_->tic(__func__);

  // debug marker
  time_keeper_ptr_->tic("getDebugMarker");
  visualization_msgs::msg::MarkerArray markers;
  if (debug_markers_pub_->get_subscription_count() > 0LU) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.header.stamp = this->now();
    m.action = m.ADD;
    m.id = 0UL;
    m.type = m.LINE_STRIP;
    m.color.a = 1.0;
    m.scale.x = 0.02;
    m.ns = "candidates";
    for (const auto & c : debug_data_.sampled_candidates) {
      m.points.clear();
      for (const auto & p : c.points)
        m.points.push_back(geometry_msgs::msg::Point().set__x(p.x()).set__y(p.y()));
      if (c.constraint_results.isValid()) {
        m.color.g = 1.0;
        m.color.r = 0.0;
      } else {
        m.color.r = 1.0;
        m.color.g = 0.0;
      }
      markers.markers.push_back(m);
      ++m.id;
    }
    if (prev_path_) {
      m.ns = "previous_path";
      m.id = 0UL;
      m.points.clear();
      for (const auto & p : prev_path_->points)
        m.points.push_back(geometry_msgs::msg::Point().set__x(p.x()).set__y(p.y()));
      m.color.r = 0.0;
      m.color.g = 0.0;
      m.color.b = 1.0;
      markers.markers.push_back(m);
    }
    m.ns = "footprint";
    m.id = 0UL;
    m.type = m.POINTS;
    m.points.clear();
    m.color.r = 1.0;
    m.color.g = 0.0;
    m.color.b = 1.0;
    m.scale.y = 0.02;
    if (!debug_data_.footprints.empty()) {
      m.action = m.ADD;
      for (const auto & p :
           debug_data_.footprints[std::min(debug_id_, debug_data_.footprints.size())])
        m.points.push_back(geometry_msgs::msg::Point().set__x(p.x()).set__y(p.y()));
    } else {
      m.action = m.DELETE;
    }
    m.ns = "debug_path";
    m.id = 0UL;
    m.type = m.POINTS;
    m.points.clear();
    m.color.g = 1.0;
    m.color.b = 0.0;
    m.scale.y = 0.04;
    if (!debug_data_.sampled_candidates.empty()) {
      m.action = m.ADD;
      for (const auto & p :
           debug_data_
             .sampled_candidates[std::min(debug_id_, debug_data_.sampled_candidates.size())]
             .points)
        m.points.push_back(geometry_msgs::msg::Point().set__x(p.x()).set__y(p.y()));
    } else {
      m.action = m.DELETE;
    }
    markers.markers.push_back(m);
    m.type = m.LINE_STRIP;
    m.ns = "obstacles";
    m.id = 0UL;
    m.color.r = 1.0;
    m.color.g = 0.0;
    m.color.b = 0.0;
    for (const auto & obs : debug_data_.obstacles) {
      m.points.clear();
      for (const auto & p : obs.outer())
        m.points.push_back(geometry_msgs::msg::Point().set__x(p.x()).set__y(p.y()));
      markers.markers.push_back(m);
      ++m.id;
    }
    m.action = m.DELETE;
    m.ns = "candidates";
    for (m.id = debug_data_.sampled_candidates.size();
         static_cast<size_t>(m.id) < debug_data_.previous_sampled_candidates_nb; ++m.id)
      markers.markers.push_back(m);
  }
  time_keeper_ptr_->toc("getDebugMarker", "      ");

  time_keeper_ptr_->tic("publishDebugMarker");
  debug_markers_pub_->publish(markers);
  time_keeper_ptr_->toc("publishDebugMarker", "      ");

  time_keeper_ptr_->toc(__func__, "    ");
}

std::vector<TrajectoryPoint> PathSampler::extendTrajectory(
  const std::vector<TrajectoryPoint> & traj_points,
  const std::vector<TrajectoryPoint> & optimized_traj_points) const
{
  time_keeper_ptr_->tic(__func__);

  const auto & start_pose = optimized_traj_points.front().pose;

  // prepend the generated trajectory: join it at the start with the reference path
  const size_t start_traj_seg_idx =
    trajectory_utils::findEgoSegmentIndex(traj_points, start_pose, ego_nearest_param_);
  const auto prepended_traj_points = [&]() {
    if (start_traj_seg_idx == 0UL) return traj_points;
    const auto pre_traj =
      std::vector<TrajectoryPoint>(traj_points.begin(), traj_points.begin() + start_traj_seg_idx);
    return concatVectors(pre_traj, optimized_traj_points);
  }();

  // expand the generated trajectory: join it at the end with the reference path
  constexpr double joint_traj_length_for_smoothing = 5.0;
  const auto & end_pose = prepended_traj_points.back().pose;
  const size_t end_traj_seg_idx =
    trajectory_utils::findEgoSegmentIndex(traj_points, end_pose, ego_nearest_param_);
  const auto end_upto_traj_point_idx = trajectory_utils::getPointIndexAfter(
    traj_points, end_pose.position, end_traj_seg_idx, joint_traj_length_for_smoothing);

  // calculate full trajectory points
  const auto full_traj_points = [&]() {
    if (!end_upto_traj_point_idx) {
      return prepended_traj_points;
    }
    const auto extended_traj_points = std::vector<TrajectoryPoint>{
      traj_points.begin() + *end_upto_traj_point_idx, traj_points.end()};
    return concatVectors(prepended_traj_points, extended_traj_points);
  }();

  // resample trajectory points
  auto resampled_traj_points = trajectory_utils::resampleTrajectoryPoints(
    full_traj_points, traj_param_.output_delta_arc_length);

  // update velocity on joint
  for (size_t i = end_traj_seg_idx + 1; i <= end_upto_traj_point_idx; ++i) {
    if (hasZeroVelocity(traj_points.at(i))) {
      if (i != 0 && !hasZeroVelocity(traj_points.at(i - 1))) {
        // Here is when current point is 0 velocity, but previous point is not 0 velocity.
        const auto & input_stop_pose = traj_points.at(i).pose;
        const size_t stop_seg_idx = trajectory_utils::findEgoSegmentIndex(
          resampled_traj_points, input_stop_pose, ego_nearest_param_);

        // calculate and insert stop pose on output trajectory
        trajectory_utils::insertStopPoint(resampled_traj_points, input_stop_pose, stop_seg_idx);
      }
    }
  }
  time_keeper_ptr_->toc(__func__, "  ");
  return resampled_traj_points;
}
}  // namespace path_sampler

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(path_sampler::PathSampler)
