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

#include "obstacle_avoidance_planner/utils/utils.hpp"

#include "geometry_msgs/msg/vector3.hpp"

#include <algorithm>
#include <chrono>
#include <limits>
#include <memory>
#include <vector>

EBPathOptimizer::EBPathOptimizer(
  const bool is_showing_debug_info, const TrajectoryParam & traj_param, const EBParam & eb_param,
  const VehicleParam & vehicle_param)
: is_showing_debug_info_(is_showing_debug_info),
  epsilon_(1e-8),
  qp_param_(eb_param.qp_param),
  traj_param_(traj_param),
  eb_param_(eb_param),
  vehicle_param_(vehicle_param)
{
  const Eigen::MatrixXd p = makePMatrix();
  default_a_matrix_ = makeAMatrix();

  const int num_points = eb_param_.num_sampling_points_for_eb;
  const std::vector<double> q(num_points * 2, 0.0);
  const std::vector<double> lower_bound(num_points * 2, 0.0);
  const std::vector<double> upper_bound(num_points * 2, 0.0);

  osqp_solver_ptr_ = std::make_unique<autoware::common::osqp::OSQPInterface>(
    p, default_a_matrix_, q, lower_bound, upper_bound, qp_param_.eps_abs);
  osqp_solver_ptr_->updateEpsRel(qp_param_.eps_rel);
  osqp_solver_ptr_->updateMaxIter(qp_param_.max_iteration);
}

// make positive semidefinite matrix for objective function
// reference: https://ieeexplore.ieee.org/document/7402333
Eigen::MatrixXd EBPathOptimizer::makePMatrix()
{
  const int num_points = eb_param_.num_sampling_points_for_eb;

  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(num_points * 2, num_points * 2);
  for (int r = 0; r < num_points * 2; ++r) {
    for (int c = 0; c < num_points * 2; ++c) {
      if (r == c) {
        P(r, c) = 6.0;
      } else if (std::abs(c - r) == 1) {
        P(r, c) = -4.0;
      } else if (std::abs(c - r) == 2) {
        P(r, c) = 1.0;
      } else {
        P(r, c) = 0.0;
      }
    }
  }
  return P;
}

// make default linear constrain matrix
Eigen::MatrixXd EBPathOptimizer::makeAMatrix()
{
  const int num_points = eb_param_.num_sampling_points_for_eb;

  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(num_points * 2, num_points * 2);
  for (int i = 0; i < num_points * 2; ++i) {
    if (i < num_points) {
      A(i, i + num_points) = 1.0;
    } else {
      A(i, i - num_points) = 1.0;
    }
  }
  return A;
}

boost::optional<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>
EBPathOptimizer::getEBTrajectory(
  const geometry_msgs::msg::Pose & ego_pose, const autoware_auto_planning_msgs::msg::Path & path,
  const std::unique_ptr<Trajectories> & prev_trajs, const double current_ego_vel,
  DebugData & debug_data)
{
  stop_watch_.tic(__func__);

  current_ego_vel_ = current_ego_vel;

  // get candidate points for optimization
  // decide fix or non fix, might not required only for smoothing purpose
  const CandidatePoints candidate_points =
    getCandidatePoints(ego_pose, path.points, prev_trajs, debug_data);
  if (candidate_points.fixed_points.empty() && candidate_points.non_fixed_points.empty()) {
    RCLCPP_INFO_EXPRESSION(
      rclcpp::get_logger("EBPathOptimizer"), is_showing_debug_info_,
      "return boost::none since empty candidate points");
    return boost::none;
  }

  // get optimized smooth points with elastic band
  const auto eb_traj_points = getOptimizedTrajectory(path, candidate_points, debug_data);
  if (!eb_traj_points) {
    RCLCPP_INFO_EXPRESSION(
      rclcpp::get_logger("EBPathOptimizer"), is_showing_debug_info_,
      "return boost::none since smoothing failed");
    return boost::none;
  }

  debug_data.msg_stream << "      " << __func__ << ":= " << stop_watch_.toc(__func__) << " [ms]\n";
  return eb_traj_points;
}

boost::optional<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>
EBPathOptimizer::getOptimizedTrajectory(
  const autoware_auto_planning_msgs::msg::Path & path, const CandidatePoints & candidate_points,
  DebugData & debug_data)
{
  stop_watch_.tic(__func__);

  // get constrain rectangles around each point
  auto full_points = candidate_points.fixed_points;
  full_points.insert(
    full_points.end(), candidate_points.non_fixed_points.begin(),
    candidate_points.non_fixed_points.end());

  // interpolate points for logic purpose
  std::vector<geometry_msgs::msg::Point> interpolated_points =
    interpolation_utils::getInterpolatedPoints(full_points, eb_param_.delta_arc_length_for_eb);
  if (interpolated_points.empty()) {
    return boost::none;
  }

  // clip interpolated points with the length of path
  if (traj_param_.enable_clipping_fixed_traj) {
    if (path.points.size() < 2) {
      return boost::none;
    }
    const auto interpolated_poses =
      points_utils::convertToPosesWithYawEstimation(interpolated_points);
    const auto interpolated_points_end_seg_idx = motion_utils::findNearestSegmentIndex(
      interpolated_poses, path.points.back().pose, 3.0, 0.785);
    if (interpolated_points_end_seg_idx) {
      interpolated_points = std::vector<geometry_msgs::msg::Point>(
        interpolated_points.begin(),
        interpolated_points.begin() + interpolated_points_end_seg_idx.get());
    }
  }

  debug_data.interpolated_points = interpolated_points;
  // number of optimizing points
  const int farthest_idx = std::min(
    (eb_param_.num_sampling_points_for_eb - 1), static_cast<int>(interpolated_points.size() - 1));
  // number of fixed points in interpolated_points
  const int num_fixed_points =
    getNumFixedPoints(candidate_points.fixed_points, interpolated_points, farthest_idx);
  // TODO(murooka) try this printing. something may be wrong
  // std::cerr << num_fixed_points << std::endl;

  // consider straight after `straight_line_idx` and then tighten space for optimization after
  // `straight_line_idx`
  const int straight_line_idx =
    getStraightLineIdx(interpolated_points, farthest_idx, debug_data.straight_points);

  // if `farthest_idx` is lower than `number_of_sampling_points`, duplicate the point at the end of
  // `interpolated_points`
  // This aims for using hotstart by not changing the size of matrix
  std::vector<geometry_msgs::msg::Point> padded_interpolated_points =
    getPaddedInterpolatedPoints(interpolated_points, farthest_idx);

  const auto rectangles = getConstrainRectangleVec(
    path, padded_interpolated_points, num_fixed_points, farthest_idx, straight_line_idx);
  if (!rectangles) {
    return boost::none;
  }

  const auto traj_points =
    calculateTrajectory(padded_interpolated_points, rectangles.get(), farthest_idx, debug_data);
  if (!traj_points) {
    return boost::none;
  }

  debug_data.msg_stream << "        " << __func__ << ":= " << stop_watch_.toc(__func__)
                        << " [ms]\n";
  return *traj_points;
}

boost::optional<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>
EBPathOptimizer::calculateTrajectory(
  const std::vector<geometry_msgs::msg::Point> & padded_interpolated_points,
  const std::vector<ConstrainRectangle> & constrain_rectangles, const int farthest_idx,
  DebugData & debug_data)
{
  stop_watch_.tic(__func__);

  // update constrain for QP based on constrain rectangles
  updateConstrain(padded_interpolated_points, constrain_rectangles);

  // solve QP and get optimized trajectory
  const auto result = solveQP();
  const auto optimized_points = result.first;
  const auto status = result.second;
  if (status != 1) {
    utils::logOSQPSolutionStatus(status, "EB: ");
    return boost::none;
  }

  const auto traj_points =
    convertOptimizedPointsToTrajectory(optimized_points, constrain_rectangles, farthest_idx);

  debug_data.msg_stream << "          " << __func__ << ":= " << stop_watch_.toc(__func__)
                        << " [ms]\n";
  return traj_points;
}

std::pair<std::vector<double>, int64_t> EBPathOptimizer::solveQP()
{
  osqp_solver_ptr_->updateEpsRel(qp_param_.eps_rel);
  osqp_solver_ptr_->updateEpsAbs(qp_param_.eps_abs);

  const auto result = osqp_solver_ptr_->optimize();
  const auto optimized_points = std::get<0>(result);
  const auto status = std::get<3>(result);

  utils::logOSQPSolutionStatus(std::get<3>(result), "EB: ");

  return std::make_pair(optimized_points, status);
}

std::vector<geometry_msgs::msg::Pose> EBPathOptimizer::getFixedPoints(
  const geometry_msgs::msg::Pose & ego_pose,
  [[maybe_unused]] const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
  const std::unique_ptr<Trajectories> & prev_trajs)
{
  /* use of prev_traj_points(fine resolution) instead of prev_opt_traj(coarse resolution)
     stabilize trajectory's yaw*/
  if (prev_trajs) {
    if (prev_trajs->smoothed_trajectory.empty()) {
      std::vector<geometry_msgs::msg::Pose> empty_points;
      return empty_points;
    }
    const size_t begin_idx = motion_utils::findFirstNearestIndexWithSoftConstraints(
      prev_trajs->smoothed_trajectory, ego_pose, traj_param_.ego_nearest_dist_threshold,
      traj_param_.ego_nearest_yaw_threshold);
    const int backward_fixing_idx = std::max(
      static_cast<int>(
        begin_idx -
        traj_param_.backward_fixing_distance / traj_param_.delta_arc_length_for_trajectory),
      0);

    // NOTE: Fixed length of EB has to be longer than that of MPT.
    constexpr double forward_fixed_length_margin = 5.0;
    const double forward_fixed_length = std::max(
      current_ego_vel_ * traj_param_.forward_fixing_min_time + forward_fixed_length_margin,
      traj_param_.forward_fixing_min_distance);

    const int forward_fixing_idx = std::min(
      static_cast<int>(
        begin_idx + forward_fixed_length / traj_param_.delta_arc_length_for_trajectory),
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

EBPathOptimizer::CandidatePoints EBPathOptimizer::getCandidatePoints(
  const geometry_msgs::msg::Pose & ego_pose,
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
  const std::unique_ptr<Trajectories> & prev_trajs, DebugData & debug_data)
{
  const std::vector<geometry_msgs::msg::Pose> fixed_points =
    getFixedPoints(ego_pose, path_points, prev_trajs);
  if (fixed_points.empty()) {
    CandidatePoints candidate_points = getDefaultCandidatePoints(path_points);
    return candidate_points;
  }

  // try to find non-fix points
  const auto opt_begin_idx = motion_utils::findNearestIndex(
    path_points, fixed_points.back(), std::numeric_limits<double>::max(),
    traj_param_.delta_yaw_threshold_for_closest_point);
  if (!opt_begin_idx) {
    CandidatePoints candidate_points;
    candidate_points.fixed_points = fixed_points;
    candidate_points.begin_path_idx = path_points.size();
    candidate_points.end_path_idx = path_points.size();
    return candidate_points;
  }

  const int begin_idx = std::min(
    static_cast<int>(opt_begin_idx.get()) + eb_param_.num_offset_for_begin_idx,
    static_cast<int>(path_points.size()) - 1);

  std::vector<geometry_msgs::msg::Pose> non_fixed_points;
  for (size_t i = begin_idx; i < path_points.size(); i++) {
    non_fixed_points.push_back(path_points[i].pose);
  }
  CandidatePoints candidate_points;
  candidate_points.fixed_points = fixed_points;
  candidate_points.non_fixed_points = non_fixed_points;
  candidate_points.begin_path_idx = begin_idx;
  candidate_points.end_path_idx = path_points.size() - 1;

  debug_data.fixed_points = candidate_points.fixed_points;
  debug_data.non_fixed_points = candidate_points.non_fixed_points;
  return candidate_points;
}

std::vector<geometry_msgs::msg::Point> EBPathOptimizer::getPaddedInterpolatedPoints(
  const std::vector<geometry_msgs::msg::Point> & interpolated_points, const int farthest_point_idx)
{
  std::vector<geometry_msgs::msg::Point> padded_interpolated_points;
  for (int i = 0; i < eb_param_.num_sampling_points_for_eb; i++) {
    if (i > farthest_point_idx) {
      padded_interpolated_points.push_back(interpolated_points[farthest_point_idx]);
    } else {
      padded_interpolated_points.push_back(interpolated_points[i]);
    }
  }
  return padded_interpolated_points;
}

EBPathOptimizer::CandidatePoints EBPathOptimizer::getDefaultCandidatePoints(
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points)
{
  double accum_arc_length = 0;
  int end_path_idx = 0;
  std::vector<geometry_msgs::msg::Pose> fixed_points;
  for (size_t i = 0; i < path_points.size(); i++) {
    if (i > 0) {
      accum_arc_length += tier4_autoware_utils::calcDistance2d(
        path_points[i].pose.position, path_points[i - 1].pose.position);
    }
    if (
      accum_arc_length > eb_param_.num_sampling_points_for_eb * eb_param_.delta_arc_length_for_eb) {
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

int EBPathOptimizer::getNumFixedPoints(
  const std::vector<geometry_msgs::msg::Pose> & fixed_points,
  const std::vector<geometry_msgs::msg::Point> & interpolated_points, const int farthest_idx)
{
  int num_fixed_points = 0;
  if (!fixed_points.empty() && !interpolated_points.empty()) {
    std::vector<geometry_msgs::msg::Point> interpolated_points =
      interpolation_utils::getInterpolatedPoints(fixed_points, eb_param_.delta_arc_length_for_eb);
    num_fixed_points = interpolated_points.size();
  }
  num_fixed_points = std::min(num_fixed_points, farthest_idx);
  return num_fixed_points;
}

std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>
EBPathOptimizer::convertOptimizedPointsToTrajectory(
  const std::vector<double> optimized_points,
  [[maybe_unused]] const std::vector<ConstrainRectangle> & constraints, const int farthest_idx)
{
  std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> traj_points;
  for (int i = 0; i <= farthest_idx; i++) {
    autoware_auto_planning_msgs::msg::TrajectoryPoint tmp_point;
    tmp_point.pose.position.x = optimized_points[i];
    tmp_point.pose.position.y = optimized_points[i + eb_param_.num_sampling_points_for_eb];
    traj_points.push_back(tmp_point);
  }
  for (size_t i = 0; i < traj_points.size(); i++) {
    if (i > 0) {
      traj_points[i].pose.orientation = geometry_utils::getQuaternionFromPoints(
        traj_points[i].pose.position, traj_points[i - 1].pose.position);
    } else if (i == 0 && traj_points.size() > 1) {
      traj_points[i].pose.orientation = geometry_utils::getQuaternionFromPoints(
        traj_points[i + 1].pose.position, traj_points[i].pose.position);
    }
  }
  return traj_points;
}

boost::optional<std::vector<ConstrainRectangle>> EBPathOptimizer::getConstrainRectangleVec(
  const autoware_auto_planning_msgs::msg::Path & path,
  const std::vector<geometry_msgs::msg::Point> & interpolated_points, const int num_fixed_points,
  const int farthest_point_idx, const int straight_idx)
{
  auto curvatures = points_utils::calcCurvature(interpolated_points, 10);

  std::vector<ConstrainRectangle> smooth_constrain_rects(eb_param_.num_sampling_points_for_eb);
  for (int i = 0; i < eb_param_.num_sampling_points_for_eb; i++) {
    // `Anchor` has pose + velocity
    const Anchor anchor = getAnchor(interpolated_points, i, path.points);

    // four types of rectangle for various types
    if (i == 0 || i == 1 || i >= farthest_point_idx - 1 || i < num_fixed_points - 1) {
      // rect has four points for a rectangle in map coordinate
      const auto rect = getConstrainRectangle(anchor, eb_param_.clearance_for_fixing);
      smooth_constrain_rects[i] = rect;
    } else if (  // NOLINT
      i >= num_fixed_points - eb_param_.num_joint_buffer_points &&
      i <= num_fixed_points + eb_param_.num_joint_buffer_points) {
      const auto rect = getConstrainRectangle(anchor, eb_param_.clearance_for_joint);
      smooth_constrain_rects[i] = rect;
    } else if (i >= straight_idx) {
      const auto rect = getConstrainRectangle(anchor, eb_param_.clearance_for_straight_line);
      smooth_constrain_rects[i] = rect;
    } else {
      const double min_x = -eb_param_.clearance_for_only_smoothing;
      const double max_x = eb_param_.clearance_for_only_smoothing;
      const double min_y = curvatures[i] > 0 ? 0 : -eb_param_.clearance_for_only_smoothing;
      const double max_y = curvatures[i] <= 0 ? 0 : eb_param_.clearance_for_only_smoothing;
      const auto rect = getConstrainRectangle(anchor, min_x, max_x, min_y, max_y);
      smooth_constrain_rects[i] = rect;
    }
  }
  return smooth_constrain_rects;
}

void EBPathOptimizer::updateConstrain(
  const std::vector<geometry_msgs::msg::Point> & interpolated_points,
  const std::vector<ConstrainRectangle> & rectangle_points)
{
  const int num_points = eb_param_.num_sampling_points_for_eb;

  Eigen::MatrixXd A = default_a_matrix_;
  std::vector<double> lower_bound(num_points * 2, 0.0);
  std::vector<double> upper_bound(num_points * 2, 0.0);
  for (int i = 0; i < num_points; ++i) {
    Constrain constrain =
      getConstrainFromConstrainRectangle(interpolated_points[i], rectangle_points[i]);
    A(i, i) = constrain.top_and_bottom.x_coef;
    A(i, i + num_points) = constrain.top_and_bottom.y_coef;
    A(i + num_points, i) = constrain.left_and_right.x_coef;
    A(i + num_points, i + num_points) = constrain.left_and_right.y_coef;
    lower_bound[i] = constrain.top_and_bottom.lower_bound;
    upper_bound[i] = constrain.top_and_bottom.upper_bound;
    lower_bound[i + num_points] = constrain.left_and_right.lower_bound;
    upper_bound[i + num_points] = constrain.left_and_right.upper_bound;
  }

  osqp_solver_ptr_->updateBounds(lower_bound, upper_bound);
  osqp_solver_ptr_->updateA(A);
}

EBPathOptimizer::Anchor EBPathOptimizer::getAnchor(
  const std::vector<geometry_msgs::msg::Point> & interpolated_points, const int interpolated_idx,
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points) const
{
  geometry_msgs::msg::Pose pose;
  pose.position = interpolated_points[interpolated_idx];
  if (interpolated_idx > 0) {
    pose.orientation = geometry_utils::getQuaternionFromPoints(
      interpolated_points[interpolated_idx], interpolated_points[interpolated_idx - 1]);
  } else if (interpolated_idx == 0 && interpolated_points.size() > 1) {
    pose.orientation = geometry_utils::getQuaternionFromPoints(
      interpolated_points[interpolated_idx + 1], interpolated_points[interpolated_idx]);
  }
  const auto opt_nearest_idx = motion_utils::findNearestIndex(
    path_points, pose, std::numeric_limits<double>::max(),
    traj_param_.delta_yaw_threshold_for_closest_point);
  const int nearest_idx = opt_nearest_idx ? *opt_nearest_idx : 0;

  const geometry_msgs::msg::Quaternion nearest_q = path_points[nearest_idx].pose.orientation;
  Anchor anchor;
  anchor.pose.position = interpolated_points[interpolated_idx];
  anchor.pose.orientation = nearest_q;
  return anchor;
}

int EBPathOptimizer::getStraightLineIdx(
  const std::vector<geometry_msgs::msg::Point> & interpolated_points, const int farthest_point_idx,
  std::vector<geometry_msgs::msg::Point> & debug_detected_straight_points)
{
  double prev_yaw = 0;
  int straight_line_idx = farthest_point_idx;
  for (int i = farthest_point_idx; i >= 0; i--) {
    if (i < farthest_point_idx) {
      const double yaw =
        tier4_autoware_utils::calcAzimuthAngle(interpolated_points[i], interpolated_points[i + 1]);
      const double delta_yaw = yaw - prev_yaw;
      const double norm_delta_yaw = tier4_autoware_utils::normalizeRadian(delta_yaw);
      if (std::fabs(norm_delta_yaw) > traj_param_.delta_yaw_threshold_for_straight) {
        break;
      }
      straight_line_idx = i;
      prev_yaw = yaw;
    } else if (i == farthest_point_idx && farthest_point_idx >= 1) {
      const double yaw =
        tier4_autoware_utils::calcAzimuthAngle(interpolated_points[i - 1], interpolated_points[i]);
      prev_yaw = yaw;
    }
  }
  for (int i = straight_line_idx; i <= farthest_point_idx; i++) {
    debug_detected_straight_points.push_back(interpolated_points[i]);
  }
  return straight_line_idx;
}

EBPathOptimizer::Constrain EBPathOptimizer::getConstrainFromConstrainRectangle(
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

EBPathOptimizer::ConstrainLines EBPathOptimizer::getConstrainLines(
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
  constrain_range.top_left = geometry_utils::transformToAbsoluteCoordinate2D(top_left, anchor.pose);
  geometry_msgs::msg::Point top_right;
  top_right.x = clearance;
  top_right.y = -1 * clearance;
  constrain_range.top_right =
    geometry_utils::transformToAbsoluteCoordinate2D(top_right, anchor.pose);
  geometry_msgs::msg::Point bottom_left;
  bottom_left.x = -1 * clearance;
  bottom_left.y = clearance;
  constrain_range.bottom_left =
    geometry_utils::transformToAbsoluteCoordinate2D(bottom_left, anchor.pose);
  geometry_msgs::msg::Point bottom_right;
  bottom_right.x = -1 * clearance;
  bottom_right.y = -1 * clearance;
  constrain_range.bottom_right =
    geometry_utils::transformToAbsoluteCoordinate2D(bottom_right, anchor.pose);
  return constrain_range;
}

ConstrainRectangle EBPathOptimizer::getConstrainRectangle(
  const Anchor & anchor, const double min_x, const double max_x, const double min_y,
  const double max_y) const
{
  ConstrainRectangle rect;
  rect.top_left = tier4_autoware_utils::calcOffsetPose(anchor.pose, max_x, max_y, 0.0).position;
  rect.top_right = tier4_autoware_utils::calcOffsetPose(anchor.pose, max_x, min_y, 0.0).position;
  rect.bottom_left = tier4_autoware_utils::calcOffsetPose(anchor.pose, min_x, max_y, 0.0).position;
  rect.bottom_right = tier4_autoware_utils::calcOffsetPose(anchor.pose, min_x, min_y, 0.0).position;
  return rect;
}
