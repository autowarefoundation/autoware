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

#include "obstacle_avoidance_planner/eb_path_optimizer.hpp"

#include "obstacle_avoidance_planner/mpt_optimizer.hpp"
#include "obstacle_avoidance_planner/process_cv.hpp"
#include "obstacle_avoidance_planner/util.hpp"

#include <opencv2/opencv.hpp>
#include <osqp_interface/osqp_interface.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_object.hpp>
#include <autoware_auto_planning_msgs/msg/path.hpp>
#include <autoware_auto_planning_msgs/msg/path_point.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>

#include <boost/optional.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <chrono>
#include <limits>
#include <memory>
#include <vector>

EBPathOptimizer::EBPathOptimizer(
  const bool is_showing_debug_info, const QPParam qp_param, const TrajectoryParam traj_pram,
  const ConstrainParam constrain_param, const VehicleParam & vehicle_param,
  const MPTParam & mpt_param)
: logger_ros_clock_(RCL_ROS_TIME),
  is_showing_debug_info_(is_showing_debug_info),
  epsilon_(1e-8),
  qp_param_(qp_param),
  traj_param_(traj_pram),
  constrain_param_(constrain_param),
  vehicle_param_(vehicle_param)
{
  geometry_msgs::msg::Vector3 keep_space_shape;
  keep_space_shape.x = constrain_param_.keep_space_shape_x;
  keep_space_shape.y = constrain_param_.keep_space_shape_y;
  keep_space_shape_ptr_ = std::make_unique<geometry_msgs::msg::Vector3>(keep_space_shape);
  mpt_optimizer_ptr_ = std::make_unique<MPTOptimizer>(
    is_showing_debug_info, qp_param, traj_pram, constrain_param, vehicle_param, mpt_param);
  initializeSolver();
}

EBPathOptimizer::~EBPathOptimizer() {}

void EBPathOptimizer::initializeSolver()
{
  Eigen::MatrixXd p = makePMatrix();
  default_a_matrix_ = makeAMatrix();

  std::vector<double> q(traj_param_.num_sampling_points * 2, 0.0);
  std::vector<double> lower_bound(traj_param_.num_sampling_points * 2, 0.0);
  std::vector<double> upper_bound(traj_param_.num_sampling_points * 2, 0.0);
  osqp_solver_ptr_ = std::make_unique<autoware::common::osqp::OSQPInterface>(
    p, default_a_matrix_, q, lower_bound, upper_bound, qp_param_.eps_abs);
  osqp_solver_ptr_->updateEpsRel(qp_param_.eps_rel);
  osqp_solver_ptr_->updateMaxIter(qp_param_.max_iteration);
  ex_osqp_solver_ptr_ = std::make_unique<autoware::common::osqp::OSQPInterface>(
    p, default_a_matrix_, q, lower_bound, upper_bound, qp_param_.eps_abs);
  ex_osqp_solver_ptr_->updateEpsRel(qp_param_.eps_rel);
  ex_osqp_solver_ptr_->updateMaxIter(qp_param_.max_iteration);
  vis_osqp_solver_ptr_ = std::make_unique<autoware::common::osqp::OSQPInterface>(
    p, default_a_matrix_, q, lower_bound, upper_bound, qp_param_.eps_abs);
  vis_osqp_solver_ptr_->updateEpsRel(qp_param_.eps_rel);
  vis_osqp_solver_ptr_->updateMaxIter(qp_param_.max_iteration);
}

/* make positive semidefinite matrix for objective function
   reference: https://ieeexplore.ieee.org/document/7402333 */
Eigen::MatrixXd EBPathOptimizer::makePMatrix()
{
  Eigen::MatrixXd P =
    Eigen::MatrixXd::Zero(traj_param_.num_sampling_points * 2, traj_param_.num_sampling_points * 2);
  for (int r = 0; r < traj_param_.num_sampling_points * 2; r++) {
    for (int c = 0; c < traj_param_.num_sampling_points * 2; c++) {
      if (r == c) {
        P(r, c) = 6;
      } else if (std::abs(c - r) == 1) {
        P(r, c) = -4;
      } else if (std::abs(c - r) == 2) {
        P(r, c) = 1;
      } else {
        P(r, c) = 0;
      }
    }
  }
  return P;
}

// make default linear constrain matrix
Eigen::MatrixXd EBPathOptimizer::makeAMatrix()
{
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(
    traj_param_.num_sampling_points * 2, traj_param_.num_sampling_points * 2);
  for (int i = 0; i < traj_param_.num_sampling_points * 2; i++) {
    if (i < traj_param_.num_sampling_points) {
      A(i, i + traj_param_.num_sampling_points) = 1;
    } else {
      A(i, i - traj_param_.num_sampling_points) = 1;
    }
  }
  return A;
}

boost::optional<Trajectories> EBPathOptimizer::generateOptimizedTrajectory(
  const bool enable_avoidance, const geometry_msgs::msg::Pose & ego_pose,
  const autoware_auto_planning_msgs::msg::Path & path,
  const std::unique_ptr<Trajectories> & prev_trajs,
  const std::vector<autoware_auto_perception_msgs::msg::PredictedObject> & objects,
  DebugData * debug_data)
{
  // processing drivable area
  auto t_start1 = std::chrono::high_resolution_clock::now();
  CVMaps cv_maps = process_cv::getMaps(enable_avoidance, path, objects, traj_param_, debug_data);
  auto t_end1 = std::chrono::high_resolution_clock::now();
  float elapsed_ms1 = std::chrono::duration<float, std::milli>(t_end1 - t_start1).count();
  RCLCPP_INFO_EXPRESSION(
    rclcpp::get_logger("EBPathOptimizer"), is_showing_debug_info_,
    "Processing driveable area time: = %f [ms]", elapsed_ms1);

  // get candidate points for optimization
  CandidatePoints candidate_points = getCandidatePoints(
    ego_pose, path.points, prev_trajs, cv_maps.drivable_area, path.drivable_area.info, debug_data);
  if (candidate_points.fixed_points.empty() && candidate_points.non_fixed_points.empty()) {
    RCLCPP_INFO_EXPRESSION(
      rclcpp::get_logger("EBPathOptimizer"), is_showing_debug_info_,
      "return boost::none since empty candidate points");
    return boost::none;
  }

  // get optimized smooth points
  const auto opt_traj_points = getOptimizedTrajectory(
    enable_avoidance, path, candidate_points, cv_maps.clearance_map,
    cv_maps.only_objects_clearance_map, debug_data);
  if (!opt_traj_points) {
    RCLCPP_INFO_EXPRESSION(
      rclcpp::get_logger("EBPathOptimizer"), is_showing_debug_info_,
      "return boost::none since smoothing failed");
    return boost::none;
  }

  const auto mpt_trajs = mpt_optimizer_ptr_->getModelPredictiveTrajectory(
    enable_avoidance, opt_traj_points.get(), path.points, prev_trajs, cv_maps, ego_pose,
    debug_data);
  if (!mpt_trajs) {
    RCLCPP_INFO_EXPRESSION(
      rclcpp::get_logger("EBPathOptimizer"), is_showing_debug_info_,
      "return boost::none since mpt failed");
    return boost::none;
  }

  auto t_start2 = std::chrono::high_resolution_clock::now();
  std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> extended_traj_points =
    getExtendedOptimizedTrajectory(path.points, mpt_trajs.get().mpt, debug_data);
  auto t_end2 = std::chrono::high_resolution_clock::now();
  float elapsed_ms2 = std::chrono::duration<float, std::milli>(t_end2 - t_start2).count();
  RCLCPP_INFO_EXPRESSION(
    rclcpp::get_logger("EBPathOptimizer"), is_showing_debug_info_,
    "Extending trajectory time: = %f [ms]", elapsed_ms2);

  Trajectories traj;
  traj.smoothed_trajectory = opt_traj_points.get();
  traj.mpt_ref_points = mpt_trajs.get().ref_points;
  traj.model_predictive_trajectory = mpt_trajs.get().mpt;
  traj.extended_trajectory = extended_traj_points;
  debug_data->smoothed_points = opt_traj_points.get();
  return traj;
}

boost::optional<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>
EBPathOptimizer::getOptimizedTrajectory(
  [[maybe_unused]] const bool enable_avoidance, const autoware_auto_planning_msgs::msg::Path & path,
  const CandidatePoints & candidate_points, const cv::Mat & clearance_map,
  const cv::Mat & only_objects_clearance_map, DebugData * debug_data)
{
  // get constrain rectangles around each point
  std::vector<geometry_msgs::msg::Point> interpolated_points = util::getInterpolatedPoints(
    candidate_points.fixed_points, candidate_points.non_fixed_points,
    traj_param_.delta_arc_length_for_optimization);
  if (interpolated_points.empty()) {
    return boost::none;
  }

  debug_data->interpolated_points = interpolated_points;
  const int farthest_idx = std::min(
    (traj_param_.num_sampling_points - 1), static_cast<int>(interpolated_points.size() - 1));
  const int num_fixed_points =
    getNumFixedPoints(candidate_points.fixed_points, interpolated_points, farthest_idx);
  const int straight_line_idx = getStraightLineIdx(
    interpolated_points, farthest_idx, only_objects_clearance_map, path.drivable_area.info,
    debug_data->straight_points);
  std::vector<geometry_msgs::msg::Point> padded_interpolated_points =
    getPaddedInterpolatedPoints(interpolated_points, farthest_idx);
  auto t_start1 = std::chrono::high_resolution_clock::now();
  const auto rectangles = getConstrainRectangleVec(
    path, padded_interpolated_points, num_fixed_points, farthest_idx, straight_line_idx,
    clearance_map, only_objects_clearance_map);
  auto t_end1 = std::chrono::high_resolution_clock::now();
  float elapsed_ms1 = std::chrono::duration<float, std::milli>(t_end1 - t_start1).count();
  RCLCPP_INFO_EXPRESSION(
    rclcpp::get_logger("EBPathOptimizer"), is_showing_debug_info_,
    "Make constrain rectangle time: = %f [ms]", elapsed_ms1);
  if (!rectangles) {
    return boost::none;
  }

  const auto traj_points = calculateTrajectory(
    padded_interpolated_points, rectangles.get(), farthest_idx, OptMode::Normal);
  debug_data->smoothed_points = traj_points;
  return traj_points;
}

std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>
EBPathOptimizer::getExtendedOptimizedTrajectory(
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & optimized_points,
  DebugData * debug_data)
{
  if (static_cast<int>(optimized_points.size()) <= traj_param_.num_fix_points_for_extending) {
    RCLCPP_INFO_THROTTLE(
      rclcpp::get_logger("EBPathOptimizer"), logger_ros_clock_,
      std::chrono::milliseconds(10000).count(), "[Avoidance] Not extend trajectory");
    return std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>{};
  }
  const double accum_arc_length = util::getArcLength(optimized_points);
  if (
    accum_arc_length > traj_param_.trajectory_length ||
    util::getLastExtendedPoint(
      path_points.back(), optimized_points.back().pose,
      traj_param_.delta_yaw_threshold_for_closest_point,
      traj_param_.max_dist_for_extending_end_point)) {
    RCLCPP_INFO_THROTTLE(
      rclcpp::get_logger("EBPathOptimizer"), logger_ros_clock_,
      std::chrono::milliseconds(10000).count(), "[Avoidance] Not extend trajectory");
    return std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>{};
  }
  const int default_idx = -1;
  int begin_idx = util::getNearestIdx(
    path_points, optimized_points.back().pose, default_idx,
    traj_param_.delta_yaw_threshold_for_closest_point);
  if (begin_idx == default_idx) {
    RCLCPP_INFO_THROTTLE(
      rclcpp::get_logger("EBPathOptimizer"), logger_ros_clock_,
      std::chrono::milliseconds(10000).count(),
      "[Avoidance] Not extend traj since could not find nearest idx from last opt point");
    return std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>{};
  }
  begin_idx = std::min(
    begin_idx + traj_param_.num_offset_for_begin_idx, static_cast<int>(path_points.size()) - 1);
  const double extending_trajectory_length = traj_param_.trajectory_length - accum_arc_length;
  const int end_extended_path_idx =
    getEndPathIdx(path_points, begin_idx, extending_trajectory_length);
  std::vector<geometry_msgs::msg::Pose> non_fixed_points;
  for (int i = begin_idx; i <= end_extended_path_idx; i++) {
    non_fixed_points.push_back(path_points[i].pose);
  }

  std::vector<geometry_msgs::msg::Pose> fixed_points;
  for (int i = traj_param_.num_fix_points_for_extending; i > 0; i--) {
    fixed_points.push_back(optimized_points[optimized_points.size() - i].pose);
  }

  const double delta_arc_length_for_extending_trajectory = std::fmax(
    traj_param_.delta_arc_length_for_optimization,
    static_cast<float>(extending_trajectory_length / traj_param_.num_sampling_points));
  std::vector<geometry_msgs::msg::Point> interpolated_points = util::getInterpolatedPoints(
    fixed_points, non_fixed_points, delta_arc_length_for_extending_trajectory);
  if (interpolated_points.empty()) {
    RCLCPP_INFO_THROTTLE(
      rclcpp::get_logger("EBPathOptimizer"), logger_ros_clock_,
      std::chrono::milliseconds(1000).count(),
      "[Avoidance] Not extend traj since empty interpolated points");
    return std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>{};
  }
  const int farthest_idx = std::min(
    (traj_param_.num_sampling_points - 1), static_cast<int>(interpolated_points.size() - 1));
  std::vector<geometry_msgs::msg::Point> padded_interpolated_points =
    getPaddedInterpolatedPoints(interpolated_points, farthest_idx);
  const double arc_length = util::getArcLength(fixed_points);
  const int num_fix_points =
    static_cast<int>(arc_length) / delta_arc_length_for_extending_trajectory;
  std::vector<ConstrainRectangle> constrain_rectangles =
    getConstrainRectangleVec(path_points, padded_interpolated_points, num_fix_points, farthest_idx);

  const auto extended_traj_points = calculateTrajectory(
    padded_interpolated_points, constrain_rectangles, farthest_idx, OptMode::Extending);

  const int default_extend_begin_idx = 0;
  const int extend_begin_idx = util::getNearestIdxOverPoint(
    extended_traj_points, optimized_points.back().pose, default_extend_begin_idx,
    traj_param_.delta_yaw_threshold_for_closest_point);

  std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> extended_traj;
  for (std::size_t i = extend_begin_idx; i < extended_traj_points.size(); i++) {
    extended_traj.push_back(extended_traj_points[i]);
  }

  debug_data->fixed_points_for_extending = fixed_points;
  debug_data->non_fixed_points_for_extending = non_fixed_points;
  debug_data->constrain_rectangles_for_extending = constrain_rectangles;
  debug_data->interpolated_points_for_extending = interpolated_points;
  return extended_traj;
}

std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> EBPathOptimizer::calculateTrajectory(
  const std::vector<geometry_msgs::msg::Point> & padded_interpolated_points,
  const std::vector<ConstrainRectangle> & constrain_rectangles, const int farthest_idx,
  const OptMode & opt_mode)
{
  // update constrain for QP based on constrain rectangles
  updateConstrain(padded_interpolated_points, constrain_rectangles, opt_mode);

  // solve QP and get optimized trajectory
  auto t_start2 = std::chrono::high_resolution_clock::now();
  std::vector<double> optimized_points = solveQP(opt_mode);
  auto t_end2 = std::chrono::high_resolution_clock::now();
  float elapsed_ms2 = std::chrono::duration<float, std::milli>(t_end2 - t_start2).count();
  RCLCPP_INFO_EXPRESSION(
    rclcpp::get_logger("EBPathOptimizer"), is_showing_debug_info_, "Optimization time: = %f [ms]",
    elapsed_ms2);
  auto traj_points =
    convertOptimizedPointsToTrajectory(optimized_points, constrain_rectangles, farthest_idx);
  return traj_points;
}

std::vector<double> EBPathOptimizer::solveQP(const OptMode & opt_mode)
{
  std::vector<double> optimized_points;
  if (opt_mode == OptMode::Normal) {
    osqp_solver_ptr_->updateEpsRel(qp_param_.eps_rel);
    osqp_solver_ptr_->updateEpsAbs(qp_param_.eps_abs);
    auto result = osqp_solver_ptr_->optimize();
    optimized_points = std::get<0>(result);
    util::logOSQPSolutionStatus(std::get<3>(result));
  } else if (opt_mode == OptMode::Extending) {
    ex_osqp_solver_ptr_->updateEpsAbs(qp_param_.eps_abs_for_extending);
    ex_osqp_solver_ptr_->updateEpsRel(qp_param_.eps_rel_for_extending);
    auto result = ex_osqp_solver_ptr_->optimize();
    optimized_points = std::get<0>(result);
    util::logOSQPSolutionStatus(std::get<3>(result));
  } else if (opt_mode == OptMode::Visualizing) {
    vis_osqp_solver_ptr_->updateEpsAbs(qp_param_.eps_abs_for_visualizing);
    vis_osqp_solver_ptr_->updateEpsRel(qp_param_.eps_rel_for_visualizing);
    auto result = vis_osqp_solver_ptr_->optimize();
    optimized_points = std::get<0>(result);
    util::logOSQPSolutionStatus(std::get<3>(result));
  }
  return optimized_points;
}

std::vector<geometry_msgs::msg::Pose> EBPathOptimizer::getFixedPoints(
  const geometry_msgs::msg::Pose & ego_pose,
  [[maybe_unused]] const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
  const std::unique_ptr<Trajectories> & prev_trajs, [[maybe_unused]] const cv::Mat & drivable_area,
  [[maybe_unused]] const nav_msgs::msg::MapMetaData & map_info)
{
  /* use of prev_traj_points(fine resolution) instead of prev_opt_traj(coarse resolution)
     stabilize trajectory's yaw*/
  if (prev_trajs) {
    if (prev_trajs->smoothed_trajectory.empty()) {
      std::vector<geometry_msgs::msg::Pose> empty_points;
      return empty_points;
    }
    const int default_idx = 0;
    int begin_idx = util::getNearestIdx(
      prev_trajs->smoothed_trajectory, ego_pose, default_idx,
      traj_param_.delta_yaw_threshold_for_closest_point);
    const int backward_fixing_idx = std::max(
      static_cast<int>(
        begin_idx -
        traj_param_.backward_fixing_distance / traj_param_.delta_arc_length_for_trajectory),
      0);
    const int forward_fixing_idx = std::min(
      static_cast<int>(
        begin_idx +
        traj_param_.forward_fixing_distance / traj_param_.delta_arc_length_for_trajectory),
      static_cast<int>(prev_trajs->smoothed_trajectory.size() - 1));
    std::vector<geometry_msgs::msg::Pose> fixed_points;
    for (int i = backward_fixing_idx; i <= forward_fixing_idx; i++) {
      fixed_points.push_back(prev_trajs->smoothed_trajectory.at(i).pose);
    }
    return fixed_points;
  } else {
    std::vector<geometry_msgs::msg::Pose> empty_points;
    return empty_points;
  }
}

CandidatePoints EBPathOptimizer::getCandidatePoints(
  const geometry_msgs::msg::Pose & ego_pose,
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
  const std::unique_ptr<Trajectories> & prev_trajs, const cv::Mat & drivable_area,
  const nav_msgs::msg::MapMetaData & map_info, DebugData * debug_data)
{
  const std::vector<geometry_msgs::msg::Pose> fixed_points =
    getFixedPoints(ego_pose, path_points, prev_trajs, drivable_area, map_info);
  if (fixed_points.empty()) {
    CandidatePoints candidate_points = getDefaultCandidatePoints(path_points);
    return candidate_points;
  }
  const int default_idx = -1;
  int begin_idx = util::getNearestIdx(
    path_points, fixed_points.back(), default_idx,
    traj_param_.delta_yaw_threshold_for_closest_point);
  if (begin_idx == default_idx) {
    CandidatePoints candidate_points;
    candidate_points.fixed_points = fixed_points;
    candidate_points.begin_path_idx = path_points.size();
    candidate_points.end_path_idx = path_points.size();
    return candidate_points;
  }
  begin_idx = std::min(
    begin_idx + traj_param_.num_offset_for_begin_idx, static_cast<int>(path_points.size()) - 1);

  std::vector<geometry_msgs::msg::Pose> non_fixed_points;
  for (size_t i = begin_idx; i < path_points.size(); i++) {
    non_fixed_points.push_back(path_points[i].pose);
  }
  CandidatePoints candidate_points;
  candidate_points.fixed_points = fixed_points;
  candidate_points.non_fixed_points = non_fixed_points;
  candidate_points.begin_path_idx = begin_idx;
  candidate_points.end_path_idx = path_points.size() - 1;

  debug_data->fixed_points = candidate_points.fixed_points;
  debug_data->non_fixed_points = candidate_points.non_fixed_points;
  return candidate_points;
}

std::vector<geometry_msgs::msg::Point> EBPathOptimizer::getPaddedInterpolatedPoints(
  const std::vector<geometry_msgs::msg::Point> & interpolated_points, const int farthest_point_idx)
{
  std::vector<geometry_msgs::msg::Point> padded_interpolated_points;
  for (int i = 0; i < traj_param_.num_sampling_points; i++) {
    if (i > farthest_point_idx) {
      padded_interpolated_points.push_back(interpolated_points[farthest_point_idx]);
    } else {
      padded_interpolated_points.push_back(interpolated_points[i]);
    }
  }
  return padded_interpolated_points;
}

CandidatePoints EBPathOptimizer::getDefaultCandidatePoints(
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points)
{
  double accum_arc_length = 0;
  int end_path_idx = 0;
  std::vector<geometry_msgs::msg::Pose> fixed_points;
  for (size_t i = 0; i < path_points.size(); i++) {
    if (i > 0) {
      accum_arc_length +=
        util::calculate2DDistance(path_points[i].pose.position, path_points[i - 1].pose.position);
    }
    if (
      accum_arc_length >
      traj_param_.num_sampling_points * traj_param_.delta_arc_length_for_optimization) {
      break;
    }
    end_path_idx = i;
    fixed_points.push_back(path_points[i].pose);
  }
  CandidatePoints candidate_points;
  candidate_points.fixed_points = fixed_points;
  candidate_points.begin_path_idx = 0;
  candidate_points.end_path_idx = end_path_idx;
  return candidate_points;
}

bool EBPathOptimizer::isPointInsideDrivableArea(
  const geometry_msgs::msg::Point & point, const cv::Mat & drivable_area,
  const nav_msgs::msg::MapMetaData & map_info)
{
  bool is_inside = true;
  unsigned char occupancy_value = std::numeric_limits<unsigned char>::max();
  const auto image_point = util::transformMapToOptionalImage(point, map_info);
  if (image_point) {
    occupancy_value = drivable_area.ptr<unsigned char>(
      static_cast<int>(image_point.get().y))[static_cast<int>(image_point.get().x)];
  }
  if (!image_point || occupancy_value < epsilon_) {
    is_inside = false;
  }
  return is_inside;
}

int EBPathOptimizer::getEndPathIdx(
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
  const int begin_path_idx, const double required_trajectory_length)
{
  double accum_dist = 0;
  int end_path_idx = begin_path_idx;
  for (std::size_t i = begin_path_idx; i < path_points.size(); i++) {
    if (static_cast<int>(i) > begin_path_idx) {
      const double dist =
        util::calculate2DDistance(path_points[i].pose.position, path_points[i - 1].pose.position);
      accum_dist += dist;
    }
    end_path_idx = i;
    if (accum_dist > required_trajectory_length) {
      break;
    }
  }
  return end_path_idx;
}

int EBPathOptimizer::getEndPathIdxInsideArea(
  const geometry_msgs::msg::Pose & ego_pose,
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
  const int begin_path_idx, const cv::Mat & drivable_area,
  const nav_msgs::msg::MapMetaData & map_info)
{
  int end_path_idx = path_points.size() - 1;
  const int default_idx = 0;
  const int initial_idx = util::getNearestIdx(
    path_points, ego_pose, default_idx, traj_param_.delta_yaw_threshold_for_closest_point);
  for (std::size_t i = initial_idx; i < path_points.size(); i++) {
    geometry_msgs::msg::Point p = path_points[i].pose.position;
    geometry_msgs::msg::Point top_image_point;
    end_path_idx = i;
    if (util::transformMapToImage(p, map_info, top_image_point)) {
      const unsigned char value = drivable_area.ptr<unsigned char>(
        static_cast<int>(top_image_point.y))[static_cast<int>(top_image_point.x)];
      if (value < epsilon_) {
        break;
      }
    } else {
      break;
    }
  }

  int end_path_idx_inside_drivable_area = begin_path_idx;
  for (int i = end_path_idx; i >= begin_path_idx; i--) {
    geometry_msgs::msg::Point rel_top_left_point;
    rel_top_left_point.x = vehicle_param_.length;
    rel_top_left_point.y = vehicle_param_.width * 0.5;
    geometry_msgs::msg::Point abs_top_left_point =
      util::transformToAbsoluteCoordinate2D(rel_top_left_point, path_points[i].pose);
    geometry_msgs::msg::Point top_left_image_point;

    geometry_msgs::msg::Point rel_top_right_point;
    rel_top_right_point.x = vehicle_param_.length;
    rel_top_right_point.y = -vehicle_param_.width * 0.5;
    geometry_msgs::msg::Point abs_top_right_point =
      util::transformToAbsoluteCoordinate2D(rel_top_right_point, path_points[i].pose);
    geometry_msgs::msg::Point top_right_image_point;
    if (
      util::transformMapToImage(abs_top_left_point, map_info, top_left_image_point) &&
      util::transformMapToImage(abs_top_right_point, map_info, top_right_image_point)) {
      const unsigned char top_left_occupancy_value = drivable_area.ptr<unsigned char>(
        static_cast<int>(top_left_image_point.y))[static_cast<int>(top_left_image_point.x)];
      const unsigned char top_right_occupancy_value = drivable_area.ptr<unsigned char>(
        static_cast<int>(top_right_image_point.y))[static_cast<int>(top_right_image_point.x)];
      if (top_left_occupancy_value > epsilon_ && top_right_occupancy_value > epsilon_) {
        end_path_idx_inside_drivable_area = i;
        break;
      }
    }
  }
  return end_path_idx_inside_drivable_area;
}

int EBPathOptimizer::getNumFixedPoints(
  const std::vector<geometry_msgs::msg::Pose> & fixed_points,
  const std::vector<geometry_msgs::msg::Point> & interpolated_points, const int farthest_idx)
{
  int num_fixed_points = 0;
  if (!fixed_points.empty() && !interpolated_points.empty()) {
    std::vector<geometry_msgs::msg::Point> interpolated_points =
      util::getInterpolatedPoints(fixed_points, traj_param_.delta_arc_length_for_optimization);
    num_fixed_points = interpolated_points.size();
  }
  num_fixed_points = std::min(num_fixed_points, farthest_idx);
  return num_fixed_points;
}

std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>
EBPathOptimizer::convertOptimizedPointsToTrajectory(
  const std::vector<double> optimized_points, const std::vector<ConstrainRectangle> & constraints,
  const int farthest_idx)
{
  std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> traj_points;
  for (int i = 0; i <= farthest_idx; i++) {
    autoware_auto_planning_msgs::msg::TrajectoryPoint tmp_point;
    tmp_point.pose.position.x = optimized_points[i];
    tmp_point.pose.position.y = optimized_points[i + traj_param_.num_sampling_points];
    tmp_point.longitudinal_velocity_mps = constraints[i].velocity;
    traj_points.push_back(tmp_point);
  }
  for (size_t i = 0; i < traj_points.size(); i++) {
    if (i > 0) {
      traj_points[i].pose.orientation = util::getQuaternionFromPoints(
        traj_points[i].pose.position, traj_points[i - 1].pose.position);
    } else if (i == 0 && traj_points.size() > 1) {
      traj_points[i].pose.orientation = util::getQuaternionFromPoints(
        traj_points[i + 1].pose.position, traj_points[i].pose.position);
    }
  }
  return traj_points;
}

boost::optional<std::vector<ConstrainRectangle>> EBPathOptimizer::getConstrainRectangleVec(
  const autoware_auto_planning_msgs::msg::Path & path,
  const std::vector<geometry_msgs::msg::Point> & interpolated_points, const int num_fixed_points,
  const int farthest_point_idx, const int straight_idx,
  [[maybe_unused]] const cv::Mat & clearance_map, const cv::Mat & only_objects_clearance_map)
{
  const nav_msgs::msg::MapMetaData map_info = path.drivable_area.info;
  std::vector<ConstrainRectangle> smooth_constrain_rects(traj_param_.num_sampling_points);
  for (int i = 0; i < traj_param_.num_sampling_points; i++) {
    const Anchor anchor = getAnchor(interpolated_points, i, path.points);
    if (i == 0 || i == 1 || i >= farthest_point_idx - 1 || i < num_fixed_points - 1) {
      const auto rect = getConstrainRectangle(anchor, constrain_param_.clearance_for_fixing);
      const auto updated_rect = getUpdatedConstrainRectangle(
        rect, anchor.pose.position, map_info, only_objects_clearance_map);
      smooth_constrain_rects[i] = updated_rect;
    } else if (  // NOLINT
      i >= num_fixed_points - traj_param_.num_joint_buffer_points &&
      i <= num_fixed_points + traj_param_.num_joint_buffer_points) {
      const auto rect = getConstrainRectangle(anchor, constrain_param_.clearance_for_joint);
      const auto updated_rect = getUpdatedConstrainRectangle(
        rect, anchor.pose.position, map_info, only_objects_clearance_map);
      smooth_constrain_rects[i] = updated_rect;
    } else if (i >= straight_idx) {
      const auto rect = getConstrainRectangle(anchor, constrain_param_.clearance_for_straight_line);
      const auto updated_rect = getUpdatedConstrainRectangle(
        rect, anchor.pose.position, map_info, only_objects_clearance_map);
      smooth_constrain_rects[i] = updated_rect;
    } else {
      const auto rect =
        getConstrainRectangle(anchor, constrain_param_.clearance_for_only_smoothing);
      const auto updated_rect = getUpdatedConstrainRectangle(
        rect, anchor.pose.position, map_info, only_objects_clearance_map);
      smooth_constrain_rects[i] = updated_rect;
    }
  }
  return smooth_constrain_rects;
}

boost::optional<std::vector<ConstrainRectangle>> EBPathOptimizer::getConstrainRectangleVec(
  const bool enable_avoidance, const autoware_auto_planning_msgs::msg::Path & path,
  const std::vector<geometry_msgs::msg::Point> & interpolated_points, const int num_fixed_points,
  const int farthest_point_idx, const int straight_idx, const cv::Mat & clearance_map,
  const cv::Mat & only_objects_clearance_map, DebugData * debug_data)
{
  const nav_msgs::msg::MapMetaData map_info = path.drivable_area.info;
  std::vector<ConstrainRectangle> object_road_constrain_ranges(traj_param_.num_sampling_points);
  std::vector<ConstrainRectangle> road_constrain_ranges(traj_param_.num_sampling_points);
  std::vector<ConstrainRectangle> only_smooth_constrain_ranges(traj_param_.num_sampling_points);
  for (int i = 0; i < traj_param_.num_sampling_points; i++) {
    const Anchor anchor = getAnchor(interpolated_points, i, path.points);
    if (i == 0 || i == 1 || i >= farthest_point_idx - 1 || i < num_fixed_points - 1) {
      const ConstrainRectangle rectangle =
        getConstrainRectangle(anchor, constrain_param_.clearance_for_fixing);
      object_road_constrain_ranges[i] = getUpdatedConstrainRectangle(
        rectangle, anchor.pose.position, map_info, only_objects_clearance_map);
      road_constrain_ranges[i] = rectangle;
      only_smooth_constrain_ranges[i] = rectangle;
    } else {
      if (
        i >= num_fixed_points - traj_param_.num_joint_buffer_points &&
        i <= num_fixed_points + traj_param_.num_joint_buffer_points) {
        const ConstrainRectangle rectangle =
          getConstrainRectangle(path.points, anchor, clearance_map, map_info);
        object_road_constrain_ranges[i] = getUpdatedConstrainRectangle(
          rectangle, anchor.pose.position, map_info, only_objects_clearance_map);
        road_constrain_ranges[i] = rectangle;
        only_smooth_constrain_ranges[i] = rectangle;
      } else {
        if (i >= straight_idx) {
          const ConstrainRectangle rectangle =
            getConstrainRectangle(anchor, constrain_param_.clearance_for_straight_line);
          object_road_constrain_ranges[i] = getUpdatedConstrainRectangle(
            rectangle, anchor.pose.position, map_info, only_objects_clearance_map);
          road_constrain_ranges[i] = rectangle;
          only_smooth_constrain_ranges[i] = rectangle;
        } else {
          const ConstrainRectangles constrain_rectangles =
            getConstrainRectangles(anchor, clearance_map, only_objects_clearance_map, map_info);
          object_road_constrain_ranges[i] = constrain_rectangles.object_constrain_rectangle;
          road_constrain_ranges[i] = constrain_rectangles.road_constrain_rectangle;
          only_smooth_constrain_ranges[i] =
            getConstrainRectangle(anchor, constrain_param_.clearance_for_only_smoothing);
        }
      }
    }
  }
  debug_data->foa_data =
    getFOAData(object_road_constrain_ranges, interpolated_points, farthest_point_idx);
  boost::optional<std::vector<ConstrainRectangle>> constrain_ranges =
    getPostProcessedConstrainRectangles(
      enable_avoidance, object_road_constrain_ranges, road_constrain_ranges,
      only_smooth_constrain_ranges, interpolated_points, path.points, farthest_point_idx,
      num_fixed_points, straight_idx, debug_data);
  return constrain_ranges;
}

std::vector<ConstrainRectangle> EBPathOptimizer::getConstrainRectangleVec(
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
  const std::vector<geometry_msgs::msg::Point> & interpolated_points, const int num_fixed_points,
  const int farthest_point_idx)
{
  std::vector<ConstrainRectangle> only_smooth_constrain_ranges(traj_param_.num_sampling_points);
  for (int i = 0; i < traj_param_.num_sampling_points; i++) {
    const Anchor anchor = getAnchor(interpolated_points, i, path_points);
    if (i < num_fixed_points || i >= farthest_point_idx - 1) {
      ConstrainRectangle rectangle = getConstrainRectangle(anchor, 0);
      only_smooth_constrain_ranges[i] = rectangle;
    } else {
      if (
        i >= num_fixed_points &&
        i <= num_fixed_points + traj_param_.num_joint_buffer_points_for_extending) {
        ConstrainRectangle rectangle =
          getConstrainRectangle(anchor, constrain_param_.range_for_extend_joint);
        only_smooth_constrain_ranges[i] = rectangle;
      } else {
        ConstrainRectangle rectangle =
          getConstrainRectangle(anchor, constrain_param_.clearance_for_only_smoothing);
        only_smooth_constrain_ranges[i] = rectangle;
      }
    }
  }
  return only_smooth_constrain_ranges;
}

boost::optional<std::vector<ConstrainRectangle>>
EBPathOptimizer::getPostProcessedConstrainRectangles(
  const bool enable_avoidance, const std::vector<ConstrainRectangle> & object_constrains,
  const std::vector<ConstrainRectangle> & road_constrains,
  const std::vector<ConstrainRectangle> & only_smooth_constrains,
  const std::vector<geometry_msgs::msg::Point> & interpolated_points,
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
  const int farthest_point_idx, const int num_fixed_points, const int straight_idx,
  DebugData * debug_data) const
{
  bool is_using_road_constrain = false;
  if (!enable_avoidance) {
    is_using_road_constrain = true;
  }
  bool is_using_only_smooth_constrain = false;
  if (isFixingPathPoint(path_points)) {
    is_using_only_smooth_constrain = true;
  }
  if (constrain_param_.is_getting_constraints_close2path_points) {
    return getConstrainRectanglesClose2PathPoints(
      is_using_only_smooth_constrain, is_using_road_constrain, object_constrains, road_constrains,
      only_smooth_constrains, debug_data);

  } else {
    return getConstrainRectanglesWithinArea(
      is_using_only_smooth_constrain, is_using_road_constrain, farthest_point_idx, num_fixed_points,
      straight_idx, object_constrains, road_constrains, only_smooth_constrains, interpolated_points,
      path_points, debug_data);
  }
}

boost::optional<std::vector<ConstrainRectangle>>
EBPathOptimizer::getConstrainRectanglesClose2PathPoints(
  const bool is_using_only_smooth_constrain, const bool is_using_road_constrain,
  const std::vector<ConstrainRectangle> & object_constrains,
  const std::vector<ConstrainRectangle> & road_constrains,
  const std::vector<ConstrainRectangle> & only_smooth_constrains, DebugData * debug_data) const
{
  if (is_using_only_smooth_constrain) {
    return only_smooth_constrains;
  }
  if (is_using_road_constrain) {
    return getValidConstrainRectangles(road_constrains, only_smooth_constrains, debug_data);
  } else {
    return getValidConstrainRectangles(object_constrains, only_smooth_constrains, debug_data);
  }
}

boost::optional<std::vector<ConstrainRectangle>> EBPathOptimizer::getValidConstrainRectangles(
  const std::vector<ConstrainRectangle> & constrains,
  const std::vector<ConstrainRectangle> & only_smooth_constrains, DebugData * debug_data) const
{
  bool only_smooth = true;
  int debug_cnt = 0;
  for (const auto & rect : constrains) {
    if (rect.is_empty_driveable_area) {
      debug_data->constrain_rectangles = constrains;
      RCLCPP_INFO_EXPRESSION(
        rclcpp::get_logger("EBPathOptimizer"), is_showing_debug_info_, "Constraint failed at %d",
        debug_cnt);
      return boost::none;
    }
    if (!rect.is_including_only_smooth_range) {
      RCLCPP_INFO_EXPRESSION(
        rclcpp::get_logger("EBPathOptimizer"), is_showing_debug_info_,
        "Constraint does not include only smooth range at %d", debug_cnt);
      only_smooth = false;
    }
    debug_cnt++;
  }
  if (only_smooth) {
    debug_data->constrain_rectangles = only_smooth_constrains;
    return only_smooth_constrains;
  } else {
    debug_data->constrain_rectangles = constrains;
    return constrains;
  }
}

boost::optional<std::vector<ConstrainRectangle>> EBPathOptimizer::getConstrainRectanglesWithinArea(
  bool is_using_only_smooth_constrain, bool is_using_road_constrain, const int farthest_point_idx,
  const int num_fixed_points, const int straight_idx,
  const std::vector<ConstrainRectangle> & object_constrains,
  const std::vector<ConstrainRectangle> & road_constrains,
  const std::vector<ConstrainRectangle> & only_smooth_constrains,
  const std::vector<geometry_msgs::msg::Point> & interpolated_points,
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
  DebugData * debug_data) const
{
  if (is_using_road_constrain) {
    debug_data->constrain_rectangles = road_constrains;
  } else if (is_using_only_smooth_constrain) {
    debug_data->constrain_rectangles = only_smooth_constrains;
  } else {
    debug_data->constrain_rectangles = object_constrains;
  }

  std::vector<ConstrainRectangle> constrain_ranges(traj_param_.num_sampling_points);
  int origin_dynamic_joint_idx = traj_param_.num_sampling_points;
  for (int i = 0; i < traj_param_.num_sampling_points; i++) {
    if (isPreFixIdx(i, farthest_point_idx, num_fixed_points, straight_idx)) {
      constrain_ranges[i] = only_smooth_constrains[i];
      if (object_constrains[i].is_empty_driveable_area) {
        is_using_road_constrain = true;
        return boost::none;
      }
    } else {
      if (
        i > origin_dynamic_joint_idx &&
        i <= origin_dynamic_joint_idx + traj_param_.num_joint_buffer_points) {
        const Anchor anchor = getAnchor(interpolated_points, i, path_points);
        ConstrainRectangle rectangle =
          getConstrainRectangle(anchor, constrain_param_.clearance_for_joint);
        constrain_ranges[i] = rectangle;
      } else if (is_using_only_smooth_constrain) {
        constrain_ranges[i] = only_smooth_constrains[i];
      } else if (is_using_road_constrain) {
        constrain_ranges[i] = road_constrains[i];
        if (constrain_ranges[i].is_empty_driveable_area) {
          RCLCPP_INFO_EXPRESSION(
            rclcpp::get_logger("EBPathOptimizer"), is_showing_debug_info_,
            "Only road clearance optimization failed at %d", i);
          is_using_only_smooth_constrain = true;
          origin_dynamic_joint_idx = i;
          return boost::none;
        }
      } else {
        constrain_ranges[i] = object_constrains[i];
        if (constrain_ranges[i].is_empty_driveable_area) {
          RCLCPP_INFO_EXPRESSION(
            rclcpp::get_logger("EBPathOptimizer"), is_showing_debug_info_,
            "Object clearance optimization failed at %d", i);
          return boost::none;
        }
      }
    }
  }
  return constrain_ranges;
}

bool EBPathOptimizer::isPreFixIdx(
  const int target_idx, const int farthest_point_idx, const int num_fixed_points,
  const int straight_idx) const
{
  if (
    target_idx == 0 || target_idx == 1 || target_idx >= farthest_point_idx - 1 ||
    target_idx < num_fixed_points - 1 ||
    (target_idx >= num_fixed_points - traj_param_.num_joint_buffer_points &&
     target_idx <= num_fixed_points + traj_param_.num_joint_buffer_points) ||
    target_idx >= straight_idx) {
    return true;
  } else {
    return false;
  }
}

bool EBPathOptimizer::isClose2Object(
  const geometry_msgs::msg::Point & point, const nav_msgs::msg::MapMetaData & map_info,
  const cv::Mat & only_objects_clearance_map, const double distance_threshold) const
{
  const auto image_point = util::transformMapToOptionalImage(point, map_info);
  if (!image_point) {
    return false;
  }
  const float object_clearance = only_objects_clearance_map.ptr<float>(static_cast<int>(
                                   image_point.get().y))[static_cast<int>(image_point.get().x)] *
                                 map_info.resolution;
  if (object_clearance < distance_threshold) {
    return true;
  }
  return false;
}

void EBPathOptimizer::updateConstrain(
  const std::vector<geometry_msgs::msg::Point> & interpolated_points,
  const std::vector<ConstrainRectangle> & rectangle_points, const OptMode & opt_mode)
{
  Eigen::MatrixXd A = default_a_matrix_;
  std::vector<double> lower_bound(traj_param_.num_sampling_points * 2, 0.0);
  std::vector<double> upper_bound(traj_param_.num_sampling_points * 2, 0.0);
  for (int i = 0; i < traj_param_.num_sampling_points; ++i) {
    Constrain constrain =
      getConstrainFromConstrainRectangle(interpolated_points[i], rectangle_points[i]);
    A(i, i) = constrain.top_and_bottom.x_coef;
    A(i, i + traj_param_.num_sampling_points) = constrain.top_and_bottom.y_coef;
    A(i + traj_param_.num_sampling_points, i) = constrain.left_and_right.x_coef;
    A(i + traj_param_.num_sampling_points, i + traj_param_.num_sampling_points) =
      constrain.left_and_right.y_coef;
    lower_bound[i] = constrain.top_and_bottom.lower_bound;
    upper_bound[i] = constrain.top_and_bottom.upper_bound;
    lower_bound[i + traj_param_.num_sampling_points] = constrain.left_and_right.lower_bound;
    upper_bound[i + traj_param_.num_sampling_points] = constrain.left_and_right.upper_bound;
  }
  if (opt_mode == OptMode::Normal) {
    osqp_solver_ptr_->updateBounds(lower_bound, upper_bound);
    osqp_solver_ptr_->updateA(A);
  } else if (opt_mode == OptMode::Extending) {
    ex_osqp_solver_ptr_->updateBounds(lower_bound, upper_bound);
    ex_osqp_solver_ptr_->updateA(A);
  } else if (opt_mode == OptMode::Visualizing) {
    vis_osqp_solver_ptr_->updateBounds(lower_bound, upper_bound);
    vis_osqp_solver_ptr_->updateA(A);
  }
}

Rectangle EBPathOptimizer::getAbsShapeRectangle(
  const Rectangle & rel_shape_rectangle_points, const geometry_msgs::msg::Point & offset_point,
  const geometry_msgs::msg::Pose & origin) const
{
  geometry_msgs::msg::Point abs_target_point =
    util::transformToAbsoluteCoordinate2D(offset_point, origin);

  geometry_msgs::msg::Point abs_top_left;
  abs_top_left.x = (rel_shape_rectangle_points.top_left.x + abs_target_point.x);
  abs_top_left.y = (rel_shape_rectangle_points.top_left.y + abs_target_point.y);

  geometry_msgs::msg::Point abs_top_right;
  abs_top_right.x = (rel_shape_rectangle_points.top_right.x + abs_target_point.x);
  abs_top_right.y = (rel_shape_rectangle_points.top_right.y + abs_target_point.y);

  geometry_msgs::msg::Point abs_bottom_left;
  abs_bottom_left.x = (rel_shape_rectangle_points.bottom_left.x + abs_target_point.x);
  abs_bottom_left.y = (rel_shape_rectangle_points.bottom_left.y + abs_target_point.y);

  geometry_msgs::msg::Point abs_bottom_right;
  abs_bottom_right.x = (rel_shape_rectangle_points.bottom_right.x + abs_target_point.x);
  abs_bottom_right.y = (rel_shape_rectangle_points.bottom_right.y + abs_target_point.y);

  Rectangle abs_shape_rectangle_points;
  abs_shape_rectangle_points.top_left = abs_top_left;
  abs_shape_rectangle_points.top_right = abs_top_right;
  abs_shape_rectangle_points.bottom_left = abs_bottom_left;
  abs_shape_rectangle_points.bottom_right = abs_bottom_right;
  return abs_shape_rectangle_points;
}

geometry_msgs::msg::Pose EBPathOptimizer::getOriginPose(
  const std::vector<geometry_msgs::msg::Point> & interpolated_points, const int interpolated_idx,
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points)
{
  geometry_msgs::msg::Pose pose;
  pose.position = interpolated_points[interpolated_idx];
  if (interpolated_idx > 0) {
    pose.orientation = util::getQuaternionFromPoints(
      interpolated_points[interpolated_idx], interpolated_points[interpolated_idx - 1]);
  } else if (interpolated_idx == 0 && interpolated_points.size() > 1) {
    pose.orientation = util::getQuaternionFromPoints(
      interpolated_points[interpolated_idx + 1], interpolated_points[interpolated_idx]);
  }
  const int default_idx = 0;
  const int nearest_id = util::getNearestIdx(
    path_points, pose, default_idx, traj_param_.delta_yaw_threshold_for_closest_point);
  const geometry_msgs::msg::Quaternion nearest_q = path_points[nearest_id].pose.orientation;
  geometry_msgs::msg::Pose origin;
  origin.position = interpolated_points[interpolated_idx];
  origin.orientation = nearest_q;
  return origin;
}

Anchor EBPathOptimizer::getAnchor(
  const std::vector<geometry_msgs::msg::Point> & interpolated_points, const int interpolated_idx,
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points) const
{
  geometry_msgs::msg::Pose pose;
  pose.position = interpolated_points[interpolated_idx];
  if (interpolated_idx > 0) {
    pose.orientation = util::getQuaternionFromPoints(
      interpolated_points[interpolated_idx], interpolated_points[interpolated_idx - 1]);
  } else if (interpolated_idx == 0 && interpolated_points.size() > 1) {
    pose.orientation = util::getQuaternionFromPoints(
      interpolated_points[interpolated_idx + 1], interpolated_points[interpolated_idx]);
  }
  const int default_idx = 0;
  const int nearest_idx = util::getNearestIdx(
    path_points, pose, default_idx, traj_param_.delta_yaw_threshold_for_closest_point);
  const geometry_msgs::msg::Quaternion nearest_q = path_points[nearest_idx].pose.orientation;
  Anchor anchor;
  anchor.pose.position = interpolated_points[interpolated_idx];
  anchor.pose.orientation = nearest_q;
  anchor.velocity = path_points[nearest_idx].longitudinal_velocity_mps;
  return anchor;
}

boost::optional<std::vector<std::vector<geometry_msgs::msg::Point>>>
EBPathOptimizer::getOccupancyPoints(
  const geometry_msgs::msg::Pose & origin, const cv::Mat & clearance_map,
  const nav_msgs::msg::MapMetaData & map_info) const
{
  geometry_msgs::msg::Point image_point;
  if (!util::transformMapToImage(origin.position, map_info, image_point)) {
    return boost::none;
  }
  const float clearance = std::max(
    clearance_map.ptr<float>(static_cast<int>(image_point.y))[static_cast<int>(image_point.x)] *
      map_info.resolution,
    static_cast<float>(keep_space_shape_ptr_->y));
  const float y_constrain_search_range = clearance - 0.5 * keep_space_shape_ptr_->y;
  int y_side_length = 0;
  for (float y = -y_constrain_search_range; y <= y_constrain_search_range + epsilon_;
       y += map_info.resolution * constrain_param_.coef_y_constrain_search_resolution) {
    y_side_length++;
  }
  const float x_constrain_search_range =
    std::fmin(constrain_param_.max_x_constrain_search_range, y_constrain_search_range);
  int x_side_length = 0;
  for (float x = -1 * x_constrain_search_range; x <= x_constrain_search_range + epsilon_;
       x += map_info.resolution * constrain_param_.coef_x_constrain_search_resolution) {
    x_side_length++;
  }
  if (x_side_length == 0 || y_side_length == 0) {
    return boost::none;
  }
  std::vector<std::vector<geometry_msgs::msg::Point>> occupancy_points(
    x_side_length, std::vector<geometry_msgs::msg::Point>(y_side_length));
  int x_idx_in_occupancy_map = 0;
  int y_idx_in_occupancy_map = 0;
  for (float x = -1 * x_constrain_search_range; x <= x_constrain_search_range + epsilon_;
       x += map_info.resolution * constrain_param_.coef_x_constrain_search_resolution) {
    for (float y = -1 * y_constrain_search_range; y <= y_constrain_search_range + epsilon_;
         y += map_info.resolution * constrain_param_.coef_y_constrain_search_resolution) {
      geometry_msgs::msg::Point relative_point;
      relative_point.x = x;
      relative_point.y = y;
      geometry_msgs::msg::Point abs_point =
        util::transformToAbsoluteCoordinate2D(relative_point, origin);
      const int x_idx = x_side_length - x_idx_in_occupancy_map - 1;
      const int y_idx = y_side_length - y_idx_in_occupancy_map - 1;
      if (x_idx < 0 || x_idx >= x_side_length || y_idx < 0 || y_idx >= y_side_length) {
        continue;
      }
      occupancy_points[x_idx][y_idx] = abs_point;
      y_idx_in_occupancy_map++;
    }
    x_idx_in_occupancy_map++;
    y_idx_in_occupancy_map = 0;
  }
  return occupancy_points;
}

Rectangle EBPathOptimizer::getRelShapeRectangle(
  const geometry_msgs::msg::Vector3 & vehicle_shape, const geometry_msgs::msg::Pose & origin) const
{
  geometry_msgs::msg::Point top_left;
  top_left.x = vehicle_shape.x;
  top_left.y = 0.5 * vehicle_shape.y;
  geometry_msgs::msg::Point top_right;
  top_right.x = vehicle_shape.x;
  top_right.y = -0.5 * vehicle_shape.y;
  geometry_msgs::msg::Point bottom_left;
  bottom_left.x = 0.0;
  bottom_left.y = 0.5 * vehicle_shape.y;
  geometry_msgs::msg::Point bottom_right;
  bottom_right.x = 0.0;
  bottom_right.y = -0.5 * vehicle_shape.y;

  geometry_msgs::msg::Pose tmp_origin;
  tmp_origin.orientation = origin.orientation;
  top_left = util::transformToAbsoluteCoordinate2D(top_left, tmp_origin);
  top_right = util::transformToAbsoluteCoordinate2D(top_right, tmp_origin);
  bottom_left = util::transformToAbsoluteCoordinate2D(bottom_left, tmp_origin);
  bottom_right = util::transformToAbsoluteCoordinate2D(bottom_right, tmp_origin);
  Rectangle rectangle;
  rectangle.top_left = top_left;
  rectangle.top_right = top_right;
  rectangle.bottom_left = bottom_left;
  rectangle.bottom_right = bottom_right;
  return rectangle;
}

ConstrainRectangles EBPathOptimizer::getConstrainRectangles(
  const Anchor & anchor, const cv::Mat & clearance_map, const cv::Mat & only_objects_clearance_map,
  const nav_msgs::msg::MapMetaData & map_info) const
{
  const auto occupancy_points_opt = getOccupancyPoints(anchor.pose, clearance_map, map_info);
  const auto image_point = util::transformMapToOptionalImage(anchor.pose.position, map_info);
  if (!image_point || !occupancy_points_opt) {
    ConstrainRectangle rectangle =
      getConstrainRectangle(anchor, constrain_param_.clearance_for_joint);
    rectangle.is_empty_driveable_area = true;
    ConstrainRectangles constrain_rectangles;
    constrain_rectangles.object_constrain_rectangle = rectangle;
    constrain_rectangles.road_constrain_rectangle = rectangle;
    return constrain_rectangles;
  }
  OccupancyMaps occupancy_maps = getOccupancyMaps(
    occupancy_points_opt.get(), anchor.pose, image_point.get(), clearance_map,
    only_objects_clearance_map, map_info);

  ConstrainRectangle object_constrain = getConstrainRectangle(
    occupancy_maps.object_occupancy_map, occupancy_points_opt.get(), anchor, map_info,
    only_objects_clearance_map);

  ConstrainRectangles constrain_rectangles;
  constrain_rectangles.object_constrain_rectangle = getUpdatedConstrainRectangle(
    object_constrain, anchor.pose.position, map_info, only_objects_clearance_map);
  constrain_rectangles.road_constrain_rectangle = getConstrainRectangle(
    occupancy_maps.road_occupancy_map, occupancy_points_opt.get(), anchor, map_info,
    only_objects_clearance_map);
  return constrain_rectangles;
}

OccupancyMaps EBPathOptimizer::getOccupancyMaps(
  const std::vector<std::vector<geometry_msgs::msg::Point>> & occupancy_points,
  const geometry_msgs::msg::Pose & origin_pose,
  const geometry_msgs::msg::Point & origin_point_in_image, const cv::Mat & clearance_map,
  const cv::Mat & only_objects_clearance_map, const nav_msgs::msg::MapMetaData & map_info) const
{
  Rectangle rel_shape_rectangles = getRelShapeRectangle(*keep_space_shape_ptr_, origin_pose);
  const float clearance = std::max(
    clearance_map.ptr<float>(
      static_cast<int>(origin_point_in_image.y))[static_cast<int>(origin_point_in_image.x)] *
      map_info.resolution,
    static_cast<float>(keep_space_shape_ptr_->y));
  const float y_constrain_search_range = clearance - 0.5 * keep_space_shape_ptr_->y;
  const float x_constrain_search_range =
    std::fmin(constrain_param_.max_x_constrain_search_range, y_constrain_search_range);
  std::vector<std::vector<int>> object_occupancy_map(
    occupancy_points.size(), std::vector<int>(occupancy_points.front().size(), 0));
  std::vector<std::vector<int>> road_occupancy_map(
    occupancy_points.size(), std::vector<int>(occupancy_points.front().size(), 0));
  int x_idx_in_occupancy_map = 0;
  int y_idx_in_occupancy_map = 0;
  for (float x = -1 * x_constrain_search_range; x <= x_constrain_search_range + epsilon_;
       x += map_info.resolution * constrain_param_.coef_x_constrain_search_resolution) {
    for (float y = -1 * y_constrain_search_range; y <= y_constrain_search_range + epsilon_;
         y += map_info.resolution * constrain_param_.coef_y_constrain_search_resolution) {
      geometry_msgs::msg::Point rel_target_point;
      rel_target_point.x = x;
      rel_target_point.y = y;
      Rectangle abs_shape_rectangles =
        getAbsShapeRectangle(rel_shape_rectangles, rel_target_point, origin_pose);
      float top_left_clearance = std::numeric_limits<float>::lowest();
      float top_left_objects_clearance = std::numeric_limits<float>::lowest();
      geometry_msgs::msg::Point top_left_image;
      if (util::transformMapToImage(abs_shape_rectangles.top_left, map_info, top_left_image)) {
        top_left_clearance = clearance_map.ptr<float>(static_cast<int>(
                               top_left_image.y))[static_cast<int>(top_left_image.x)] *
                             map_info.resolution;
        top_left_objects_clearance = only_objects_clearance_map.ptr<float>(static_cast<int>(
                                       top_left_image.y))[static_cast<int>(top_left_image.x)] *
                                     map_info.resolution;
      }

      float top_right_clearance = std::numeric_limits<float>::lowest();
      float top_right_objects_clearance = std::numeric_limits<float>::lowest();
      geometry_msgs::msg::Point top_right_image;
      if (util::transformMapToImage(abs_shape_rectangles.top_right, map_info, top_right_image)) {
        top_right_clearance = clearance_map.ptr<float>(static_cast<int>(
                                top_right_image.y))[static_cast<int>(top_right_image.x)] *
                              map_info.resolution;
        top_right_objects_clearance = only_objects_clearance_map.ptr<float>(static_cast<int>(
                                        top_right_image.y))[static_cast<int>(top_right_image.x)] *
                                      map_info.resolution;
      }
      float bottom_left_clearance = std::numeric_limits<float>::lowest();
      float bottom_left_objects_clearance = std::numeric_limits<float>::lowest();
      geometry_msgs::msg::Point bottom_left_image;
      if (util::transformMapToImage(
            abs_shape_rectangles.bottom_left, map_info, bottom_left_image)) {
        bottom_left_clearance = clearance_map.ptr<float>(static_cast<int>(
                                  bottom_left_image.y))[static_cast<int>(bottom_left_image.x)] *
                                map_info.resolution;
        bottom_left_objects_clearance =
          only_objects_clearance_map.ptr<float>(
            static_cast<int>(bottom_left_image.y))[static_cast<int>(bottom_left_image.x)] *
          map_info.resolution;
      }
      float bottom_right_clearance = std::numeric_limits<float>::lowest();
      float bottom_right_objects_clearance = std::numeric_limits<float>::lowest();
      geometry_msgs::msg::Point bottom_right_image;
      if (util::transformMapToImage(
            abs_shape_rectangles.bottom_right, map_info, bottom_right_image)) {
        bottom_right_clearance = clearance_map.ptr<float>(static_cast<int>(
                                   bottom_right_image.y))[static_cast<int>(bottom_right_image.x)] *
                                 map_info.resolution;
        bottom_right_objects_clearance =
          only_objects_clearance_map.ptr<float>(
            static_cast<int>(bottom_right_image.y))[static_cast<int>(bottom_right_image.x)] *
          map_info.resolution;
      }

      const int x_idx = occupancy_points.size() - x_idx_in_occupancy_map - 1;
      const int y_idx = occupancy_points.front().size() - y_idx_in_occupancy_map - 1;
      if (
        x_idx < 0 || x_idx >= static_cast<int>(occupancy_points.size()) || y_idx < 0 ||
        y_idx >= static_cast<int>(occupancy_points.front().size())) {
        continue;
      }
      if (
        top_left_clearance < constrain_param_.clearance_from_road ||
        top_right_clearance < constrain_param_.clearance_from_road ||
        bottom_right_clearance < constrain_param_.clearance_from_road ||
        bottom_left_clearance < constrain_param_.clearance_from_road ||
        top_left_objects_clearance < constrain_param_.clearance_from_object ||
        top_right_objects_clearance < constrain_param_.clearance_from_object ||
        bottom_right_objects_clearance < constrain_param_.clearance_from_object ||
        bottom_left_objects_clearance < constrain_param_.clearance_from_object) {
        object_occupancy_map[x_idx][y_idx] = 1;
      }
      if (
        top_left_clearance < constrain_param_.clearance_from_road ||
        top_right_clearance < constrain_param_.clearance_from_road ||
        bottom_right_clearance < constrain_param_.clearance_from_road ||
        bottom_left_clearance < constrain_param_.clearance_from_road) {
        road_occupancy_map[x_idx][y_idx] = 1;
      }
      y_idx_in_occupancy_map++;
    }
    x_idx_in_occupancy_map++;
    y_idx_in_occupancy_map = 0;
  }
  OccupancyMaps occupancy_maps;
  occupancy_maps.object_occupancy_map = object_occupancy_map;
  occupancy_maps.road_occupancy_map = road_occupancy_map;
  return occupancy_maps;
}

int EBPathOptimizer::getStraightLineIdx(
  const std::vector<geometry_msgs::msg::Point> & interpolated_points, const int farthest_point_idx,
  const cv::Mat & only_objects_clearance_map, const nav_msgs::msg::MapMetaData & map_info,
  std::vector<geometry_msgs::msg::Point> & debug_detected_straight_points)
{
  double prev_yaw = 0;
  int straight_line_idx = farthest_point_idx;
  for (int i = farthest_point_idx; i >= 0; i--) {
    if (i < farthest_point_idx) {
      const double yaw = util::getYawFromPoints(interpolated_points[i + 1], interpolated_points[i]);
      const double delta_yaw = yaw - prev_yaw;
      const double norm_delta_yaw = util::normalizeRadian(delta_yaw);
      float clearance_from_object = std::numeric_limits<float>::max();
      const auto image_point = util::transformMapToOptionalImage(interpolated_points[i], map_info);
      if (image_point) {
        clearance_from_object = only_objects_clearance_map.ptr<float>(static_cast<int>(
                                  image_point.get().y))[static_cast<int>(image_point.get().x)] *
                                map_info.resolution;
      }
      if (
        std::fabs(norm_delta_yaw) > traj_param_.delta_yaw_threshold_for_straight ||
        clearance_from_object < constrain_param_.clearance_from_object_for_straight) {
        break;
      }
      straight_line_idx = i;
      prev_yaw = yaw;
    } else if (i == farthest_point_idx && farthest_point_idx >= 1) {
      const double yaw = util::getYawFromPoints(interpolated_points[i], interpolated_points[i - 1]);
      prev_yaw = yaw;
    }
  }
  for (int i = straight_line_idx; i <= farthest_point_idx; i++) {
    debug_detected_straight_points.push_back(interpolated_points[i]);
  }
  return straight_line_idx;
}

Constrain EBPathOptimizer::getConstrainFromConstrainRectangle(
  const geometry_msgs::msg::Point & interpolated_point, const ConstrainRectangle & constrain_range)
{
  Constrain constrain;
  const double top_dx = constrain_range.top_left.x - constrain_range.top_right.x;
  const double top_dy = constrain_range.top_left.y - constrain_range.top_right.y;
  const double left_dx = constrain_range.top_left.x - constrain_range.bottom_left.x;
  const double left_dy = constrain_range.top_left.y - constrain_range.bottom_left.y;
  if (
    std::fabs(top_dx) < epsilon_ && std::fabs(top_dy) < epsilon_ && std::fabs(left_dx) < epsilon_ &&
    std::fabs(left_dy) < epsilon_) {
    constrain.top_and_bottom.x_coef = 1;
    constrain.top_and_bottom.y_coef = 1;
    constrain.top_and_bottom.lower_bound = interpolated_point.x + interpolated_point.y;
    constrain.top_and_bottom.upper_bound = interpolated_point.x + interpolated_point.y;
    constrain.left_and_right.x_coef = -1;
    constrain.left_and_right.y_coef = 1;
    constrain.left_and_right.lower_bound = interpolated_point.y - interpolated_point.x;
    constrain.left_and_right.upper_bound = interpolated_point.y - interpolated_point.x;
  } else if (std::fabs(top_dx) < epsilon_) {
    constrain.top_and_bottom.x_coef = 1;
    constrain.top_and_bottom.y_coef = epsilon_;
    constrain.top_and_bottom.lower_bound = interpolated_point.x;
    constrain.top_and_bottom.upper_bound = interpolated_point.x;
    constrain.left_and_right =
      getConstrainLines(left_dx, left_dy, constrain_range.top_left, constrain_range.top_right);
  } else if (std::fabs(top_dy) < epsilon_) {
    constrain.top_and_bottom.x_coef = epsilon_;
    constrain.top_and_bottom.y_coef = 1;
    constrain.top_and_bottom.lower_bound = interpolated_point.y;
    constrain.top_and_bottom.upper_bound = interpolated_point.y;
    constrain.left_and_right =
      getConstrainLines(left_dx, left_dy, constrain_range.top_left, constrain_range.top_right);
  } else if (std::fabs(left_dx) < epsilon_) {
    constrain.left_and_right.x_coef = 1;
    constrain.left_and_right.y_coef = epsilon_;
    constrain.left_and_right.lower_bound = interpolated_point.x;
    constrain.left_and_right.upper_bound = interpolated_point.x;
    constrain.top_and_bottom =
      getConstrainLines(top_dx, top_dy, constrain_range.top_left, constrain_range.bottom_left);
  } else if (std::fabs(left_dy) < epsilon_) {
    constrain.left_and_right.x_coef = epsilon_;
    constrain.left_and_right.y_coef = 1;
    constrain.left_and_right.lower_bound = interpolated_point.y;
    constrain.left_and_right.upper_bound = interpolated_point.y;
    constrain.top_and_bottom =
      getConstrainLines(top_dx, top_dy, constrain_range.top_left, constrain_range.bottom_left);
  } else {
    constrain.top_and_bottom =
      getConstrainLines(top_dx, top_dy, constrain_range.top_left, constrain_range.bottom_left);
    constrain.left_and_right =
      getConstrainLines(left_dx, left_dy, constrain_range.top_left, constrain_range.top_right);
  }
  return constrain;
}

ConstrainLines EBPathOptimizer::getConstrainLines(
  const double dx, const double dy, const geometry_msgs::msg::Point & point,
  const geometry_msgs::msg::Point & opposite_point)
{
  ConstrainLines constrain_point;

  const double slope = dy / dx;
  const double intercept = point.y - slope * point.x;
  const double intercept2 = opposite_point.y - slope * opposite_point.x;
  constrain_point.x_coef = -1 * slope;
  constrain_point.y_coef = 1;
  if (intercept > intercept2) {
    constrain_point.lower_bound = intercept2;
    constrain_point.upper_bound = intercept;
  } else {
    constrain_point.lower_bound = intercept;
    constrain_point.upper_bound = intercept2;
  }
  return constrain_point;
}

ConstrainRectangle EBPathOptimizer::getConstrainRectangle(
  const Anchor & anchor, const double clearance) const
{
  ConstrainRectangle constrain_range;
  geometry_msgs::msg::Point top_left;
  top_left.x = clearance;
  top_left.y = clearance;
  constrain_range.top_left = util::transformToAbsoluteCoordinate2D(top_left, anchor.pose);
  geometry_msgs::msg::Point top_right;
  top_right.x = clearance;
  top_right.y = -1 * clearance;
  constrain_range.top_right = util::transformToAbsoluteCoordinate2D(top_right, anchor.pose);
  geometry_msgs::msg::Point bottom_left;
  bottom_left.x = -1 * clearance;
  bottom_left.y = clearance;
  constrain_range.bottom_left = util::transformToAbsoluteCoordinate2D(bottom_left, anchor.pose);
  geometry_msgs::msg::Point bottom_right;
  bottom_right.x = -1 * clearance;
  bottom_right.y = -1 * clearance;
  constrain_range.bottom_right = util::transformToAbsoluteCoordinate2D(bottom_right, anchor.pose);
  constrain_range.velocity = anchor.velocity;
  return constrain_range;
}

ConstrainRectangle EBPathOptimizer::getUpdatedConstrainRectangle(
  const ConstrainRectangle & rectangle, const geometry_msgs::msg::Point & candidate_point,
  const nav_msgs::msg::MapMetaData & map_info, const cv::Mat & only_objects_clearance_map) const
{
  auto rect = rectangle;
  if (isClose2Object(
        candidate_point, map_info, only_objects_clearance_map,
        constrain_param_.min_object_clearance_for_deceleration)) {
    rect.velocity = std::fmin(rect.velocity, traj_param_.max_avoiding_ego_velocity_ms);
  }
  if (isClose2Object(
        candidate_point, map_info, only_objects_clearance_map,
        constrain_param_.min_object_clearance_for_joint)) {
    rect.is_including_only_smooth_range = false;
  }
  return rect;
}

ConstrainRectangle EBPathOptimizer::getConstrainRectangle(
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
  const Anchor & anchor, const cv::Mat & clearance_map,
  const nav_msgs::msg::MapMetaData & map_info) const
{
  const int default_idx = -1;
  const auto interpolated_path_points =
    util::getInterpolatedPoints(path_points, traj_param_.delta_arc_length_for_trajectory);
  const int nearest_idx = util::getNearestIdx(
    interpolated_path_points, anchor.pose, default_idx,
    traj_param_.delta_yaw_threshold_for_closest_point);

  float clearance = std::numeric_limits<float>::lowest();
  geometry_msgs::msg::Point image_point;
  if (util::transformMapToImage(interpolated_path_points[nearest_idx], map_info, image_point)) {
    clearance =
      clearance_map.ptr<float>(static_cast<int>(image_point.y))[static_cast<int>(image_point.x)] *
      map_info.resolution;
  }
  const double dist =
    util::calculate2DDistance(anchor.pose.position, interpolated_path_points[nearest_idx]);
  ConstrainRectangle constrain_rectangle;
  // idx is valid && anchor is close to nearest path point
  if (nearest_idx > default_idx && dist < clearance) {
    constrain_rectangle =
      getConstrainRectangle(anchor, nearest_idx, interpolated_path_points, clearance_map, map_info);
  } else {
    constrain_rectangle = getConstrainRectangle(anchor, constrain_param_.clearance_for_joint);
  }
  return constrain_rectangle;
}

ConstrainRectangle EBPathOptimizer::getConstrainRectangle(
  const Anchor & anchor, const int & nearest_idx,
  const std::vector<geometry_msgs::msg::Point> & interpolated_points, const cv::Mat & clearance_map,
  const nav_msgs::msg::MapMetaData & map_info) const
{
  Anchor replaced_anchor = anchor;
  replaced_anchor.pose.position = interpolated_points[nearest_idx];
  if (nearest_idx > 0) {
    replaced_anchor.pose.orientation = util::getQuaternionFromPoints(
      interpolated_points[nearest_idx], interpolated_points[nearest_idx - 1]);
  } else if (nearest_idx == 0 && interpolated_points.size() > 1) {
    replaced_anchor.pose.orientation = util::getQuaternionFromPoints(
      interpolated_points[nearest_idx + 1], interpolated_points[nearest_idx]);
  }
  ConstrainRectangles rectangles =
    getConstrainRectangles(replaced_anchor, clearance_map, clearance_map, map_info);
  const double rel_plus_y = util::calculate2DDistance(
    rectangles.road_constrain_rectangle.top_left, replaced_anchor.pose.position);
  const double rel_minus_y = util::calculate2DDistance(
    rectangles.road_constrain_rectangle.top_right, replaced_anchor.pose.position);

  ConstrainRectangle constrain_rectangle;
  geometry_msgs::msg::Point top_left;
  top_left.x = constrain_param_.clearance_for_joint;
  top_left.y = rel_plus_y;
  constrain_rectangle.top_left =
    util::transformToAbsoluteCoordinate2D(top_left, replaced_anchor.pose);
  geometry_msgs::msg::Point top_right;
  top_right.x = constrain_param_.clearance_for_joint;
  top_right.y = -1 * rel_minus_y;
  constrain_rectangle.top_right =
    util::transformToAbsoluteCoordinate2D(top_right, replaced_anchor.pose);
  geometry_msgs::msg::Point bottom_left;
  bottom_left.x = -1 * constrain_param_.clearance_for_joint;
  bottom_left.y = rel_plus_y;
  constrain_rectangle.bottom_left =
    util::transformToAbsoluteCoordinate2D(bottom_left, replaced_anchor.pose);
  geometry_msgs::msg::Point bottom_right;
  bottom_right.x = -1 * constrain_param_.clearance_for_joint;
  bottom_right.y = -1 * rel_minus_y;
  constrain_rectangle.bottom_right =
    util::transformToAbsoluteCoordinate2D(bottom_right, replaced_anchor.pose);
  constrain_rectangle.velocity = anchor.velocity;
  return constrain_rectangle;
}

ConstrainRectangle EBPathOptimizer::getConstrainRectangle(
  const std::vector<std::vector<geometry_msgs::msg::Point>> & occupancy_points,
  const util::Rectangle & util_rect, const Anchor & anchor) const
{
  ConstrainRectangle constrain_rectangle;
  constrain_rectangle.bottom_left = occupancy_points[util_rect.min_x_idx][util_rect.max_y_idx];
  constrain_rectangle.bottom_right = occupancy_points[util_rect.min_x_idx][util_rect.min_y_idx];
  constrain_rectangle.top_left = occupancy_points[util_rect.max_x_idx][util_rect.max_y_idx];
  constrain_rectangle.top_right = occupancy_points[util_rect.max_x_idx][util_rect.min_y_idx];

  geometry_msgs::msg::Pose left_pose = anchor.pose;
  left_pose.position = constrain_rectangle.top_left;
  geometry_msgs::msg::Point top_left;
  top_left.x =
    std::fmin(keep_space_shape_ptr_->x, constrain_param_.max_lon_space_for_driveable_constraint);
  top_left.y = 0;
  constrain_rectangle.top_left = util::transformToAbsoluteCoordinate2D(top_left, left_pose);
  geometry_msgs::msg::Point bottom_left;
  bottom_left.x = 0;
  bottom_left.y = 0;
  constrain_rectangle.bottom_left = util::transformToAbsoluteCoordinate2D(bottom_left, left_pose);
  geometry_msgs::msg::Pose right_pose = anchor.pose;
  right_pose.position = constrain_rectangle.top_right;
  geometry_msgs::msg::Point top_right;
  top_right.x =
    std::fmin(keep_space_shape_ptr_->x, constrain_param_.max_lon_space_for_driveable_constraint);
  top_right.y = 0;
  constrain_rectangle.top_right = util::transformToAbsoluteCoordinate2D(top_right, right_pose);
  geometry_msgs::msg::Point bottom_right;
  bottom_right.x = 0;
  bottom_right.y = 0;
  constrain_rectangle.bottom_right =
    util::transformToAbsoluteCoordinate2D(bottom_right, right_pose);
  constrain_rectangle.velocity = anchor.velocity;
  return constrain_rectangle;
}

ConstrainRectangle EBPathOptimizer::getConstrainRectangle(
  const std::vector<std::vector<int>> & occupancy_map,
  const std::vector<std::vector<geometry_msgs::msg::Point>> & occupancy_points,
  const Anchor & anchor, const nav_msgs::msg::MapMetaData & map_info,
  const cv::Mat & only_objects_clearance_map) const
{
  util::Rectangle util_rect = util::getLargestRectangle(occupancy_map);

  ConstrainRectangle constrain_rectangle;
  if (util_rect.area < epsilon_) {
    constrain_rectangle = getConstrainRectangle(anchor, constrain_param_.clearance_for_joint);
    constrain_rectangle.is_empty_driveable_area = true;
  } else {
    constrain_rectangle = getConstrainRectangle(occupancy_points, util_rect, anchor);
  }
  geometry_msgs::msg::Point max_abs_y = occupancy_points[util_rect.max_x_idx][util_rect.max_y_idx];
  geometry_msgs::msg::Point min_abs_y = occupancy_points[util_rect.max_x_idx][util_rect.min_y_idx];
  geometry_msgs::msg::Point max_rel_y =
    util::transformToRelativeCoordinate2D(max_abs_y, anchor.pose);
  geometry_msgs::msg::Point min_rel_y =
    util::transformToRelativeCoordinate2D(min_abs_y, anchor.pose);
  if (
    (max_rel_y.y < -1 * constrain_param_.clearance_for_only_smoothing ||
     min_rel_y.y > constrain_param_.clearance_for_only_smoothing) &&
    isClose2Object(
      anchor.pose.position, map_info, only_objects_clearance_map,
      constrain_param_.clearance_from_object)) {
    constrain_rectangle.is_including_only_smooth_range = false;
  }
  return constrain_rectangle;
}

bool EBPathOptimizer::isFixingPathPoint(
  [[maybe_unused]] const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points)
  const
{
  return false;
}

FOAData EBPathOptimizer::getFOAData(
  const std::vector<ConstrainRectangle> & rectangles,
  const std::vector<geometry_msgs::msg::Point> & interpolated_points, const int farthest_idx)
{
  FOAData foa_data;
  for (const auto & rect : rectangles) {
    if (rect.is_empty_driveable_area) {
      foa_data.is_avoidance_possible = false;
    }
  }
  if (!foa_data.is_avoidance_possible) {
    RCLCPP_WARN(
      rclcpp::get_logger("EBPathOptimizer"),
      "[ObstacleAvoidancePlanner] Fail to make new trajectory from empty drivable area");
  }

  foa_data.constrain_rectangles = rectangles;
  foa_data.avoiding_traj_points = calculateTrajectory(
    interpolated_points, foa_data.constrain_rectangles, farthest_idx, OptMode::Visualizing);
  return foa_data;
}
