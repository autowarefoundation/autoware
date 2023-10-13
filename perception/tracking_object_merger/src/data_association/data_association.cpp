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

#include "tracking_object_merger/data_association/data_association.hpp"

#include "object_recognition_utils/object_recognition_utils.hpp"
#include "tier4_autoware_utils/geometry/geometry.hpp"
#include "tracking_object_merger/data_association/solver/gnn_solver.hpp"
#include "tracking_object_merger/utils/utils.hpp"

#include <algorithm>
#include <fstream>
#include <list>
#include <memory>
#include <unordered_map>
#include <vector>
namespace
{
double getFormedYawAngle(
  const geometry_msgs::msg::Quaternion & quat0, const geometry_msgs::msg::Quaternion & quat1,
  const bool distinguish_front_or_back = true)
{
  const double yaw0 = tier4_autoware_utils::normalizeRadian(tf2::getYaw(quat0));
  const double yaw1 = tier4_autoware_utils::normalizeRadian(tf2::getYaw(quat1));
  const double angle_range = distinguish_front_or_back ? M_PI : M_PI_2;
  const double angle_step = distinguish_front_or_back ? 2.0 * M_PI : M_PI;
  // Fixed yaw0 to be in the range of +-90 or 180 degrees of yaw1
  double fixed_yaw0 = yaw0;
  while (angle_range <= yaw1 - fixed_yaw0) {
    fixed_yaw0 = fixed_yaw0 + angle_step;
  }
  while (angle_range <= fixed_yaw0 - yaw1) {
    fixed_yaw0 = fixed_yaw0 - angle_step;
  }
  return std::fabs(fixed_yaw0 - yaw1);
}
}  // namespace

DataAssociation::DataAssociation(
  std::vector<int> can_assign_vector, std::vector<double> max_dist_vector,
  std::vector<double> max_rad_vector, std::vector<double> min_iou_vector,
  std::vector<double> max_velocity_diff_vector)
: score_threshold_(0.01)
{
  {
    const int assign_label_num = static_cast<int>(std::sqrt(can_assign_vector.size()));
    Eigen::Map<Eigen::MatrixXi> can_assign_matrix_tmp(
      can_assign_vector.data(), assign_label_num, assign_label_num);
    can_assign_matrix_ = can_assign_matrix_tmp.transpose();
  }
  {
    const int max_dist_label_num = static_cast<int>(std::sqrt(max_dist_vector.size()));
    Eigen::Map<Eigen::MatrixXd> max_dist_matrix_tmp(
      max_dist_vector.data(), max_dist_label_num, max_dist_label_num);
    max_dist_matrix_ = max_dist_matrix_tmp.transpose();
  }
  {
    const int max_rad_label_num = static_cast<int>(std::sqrt(max_rad_vector.size()));
    Eigen::Map<Eigen::MatrixXd> max_rad_matrix_tmp(
      max_rad_vector.data(), max_rad_label_num, max_rad_label_num);
    max_rad_matrix_ = max_rad_matrix_tmp.transpose();
  }
  {
    const int min_iou_label_num = static_cast<int>(std::sqrt(min_iou_vector.size()));
    Eigen::Map<Eigen::MatrixXd> min_iou_matrix_tmp(
      min_iou_vector.data(), min_iou_label_num, min_iou_label_num);
    min_iou_matrix_ = min_iou_matrix_tmp.transpose();
  }
  {
    const int max_velocity_diff_label_num =
      static_cast<int>(std::sqrt(max_velocity_diff_vector.size()));
    Eigen::Map<Eigen::MatrixXd> max_velocity_diff_matrix_tmp(
      max_velocity_diff_vector.data(), max_velocity_diff_label_num, max_velocity_diff_label_num);
    max_velocity_diff_matrix_ = max_velocity_diff_matrix_tmp.transpose();
  }

  gnn_solver_ptr_ = std::make_unique<gnn_solver::MuSSP>();
}

void DataAssociation::assign(
  const Eigen::MatrixXd & src, std::unordered_map<int, int> & direct_assignment,
  std::unordered_map<int, int> & reverse_assignment)
{
  std::vector<std::vector<double>> score(src.rows());
  for (int row = 0; row < src.rows(); ++row) {
    score.at(row).resize(src.cols());
    for (int col = 0; col < src.cols(); ++col) {
      score.at(row).at(col) = src(row, col);
    }
  }
  // Solve
  gnn_solver_ptr_->maximizeLinearAssignment(score, &direct_assignment, &reverse_assignment);

  for (auto itr = direct_assignment.begin(); itr != direct_assignment.end();) {
    if (src(itr->first, itr->second) < score_threshold_) {
      itr = direct_assignment.erase(itr);
      continue;
    } else {
      ++itr;
    }
  }
  for (auto itr = reverse_assignment.begin(); itr != reverse_assignment.end();) {
    if (src(itr->second, itr->first) < score_threshold_) {
      itr = reverse_assignment.erase(itr);
      continue;
    } else {
      ++itr;
    }
  }
}

/**
 * @brief calc score matrix between two tracked objects
 *        This is used to associate input measurements
 *
 * @param objects0 : measurements
 * @param objects1 : base objects(tracker objects)
 * @return Eigen::MatrixXd
 */
Eigen::MatrixXd DataAssociation::calcScoreMatrix(
  const autoware_auto_perception_msgs::msg::TrackedObjects & objects0,
  const autoware_auto_perception_msgs::msg::TrackedObjects & objects1)
{
  Eigen::MatrixXd score_matrix =
    Eigen::MatrixXd::Zero(objects1.objects.size(), objects0.objects.size());
  for (size_t objects1_idx = 0; objects1_idx < objects1.objects.size(); ++objects1_idx) {
    const auto & object1 = objects1.objects.at(objects1_idx);
    for (size_t objects0_idx = 0; objects0_idx < objects0.objects.size(); ++objects0_idx) {
      const auto & object0 = objects0.objects.at(objects0_idx);
      const double score = calcScoreBetweenObjects(object0, object1);

      score_matrix(objects1_idx, objects0_idx) = score;
    }
  }
  return score_matrix;
}

/**
 * @brief calc score matrix between two tracked object and Tracker State
 *
 * @param objects0 : measurements
 * @param objects1 : tracker inner objects
 * @return Eigen::MatrixXd
 */
Eigen::MatrixXd DataAssociation::calcScoreMatrix(
  const autoware_auto_perception_msgs::msg::TrackedObjects & objects0,
  const std::vector<TrackerState> & trackers)
{
  Eigen::MatrixXd score_matrix = Eigen::MatrixXd::Zero(trackers.size(), objects0.objects.size());
  for (size_t trackers_idx = 0; trackers_idx < trackers.size(); ++trackers_idx) {
    const auto & object1 = trackers.at(trackers_idx).getObject();

    for (size_t objects0_idx = 0; objects0_idx < objects0.objects.size(); ++objects0_idx) {
      const auto & object0 = objects0.objects.at(objects0_idx);
      const double score = calcScoreBetweenObjects(object0, object1);

      score_matrix(trackers_idx, objects0_idx) = score;
    }
  }
  return score_matrix;
}

double DataAssociation::calcScoreBetweenObjects(
  const autoware_auto_perception_msgs::msg::TrackedObject & object0,
  const autoware_auto_perception_msgs::msg::TrackedObject & object1) const
{
  const std::uint8_t object1_label =
    object_recognition_utils::getHighestProbLabel(object1.classification);
  const std::uint8_t object0_label =
    object_recognition_utils::getHighestProbLabel(object0.classification);

  std::vector<double> tracker_pose = {
    object1.kinematics.pose_with_covariance.pose.position.x,
    object1.kinematics.pose_with_covariance.pose.position.y};
  std::vector<double> measurement_pose = {
    object0.kinematics.pose_with_covariance.pose.position.x,
    object0.kinematics.pose_with_covariance.pose.position.y};

  double score = 0.0;
  if (can_assign_matrix_(object1_label, object0_label)) {
    const double max_dist = max_dist_matrix_(object1_label, object0_label);
    const double dist = tier4_autoware_utils::calcDistance2d(
      object0.kinematics.pose_with_covariance.pose.position,
      object1.kinematics.pose_with_covariance.pose.position);

    bool passed_gate = true;
    // dist gate
    if (passed_gate) {
      if (max_dist < dist) passed_gate = false;
    }
    // angle gate
    if (passed_gate) {
      const double max_rad = max_rad_matrix_(object1_label, object0_label);
      const double angle = getFormedYawAngle(
        object0.kinematics.pose_with_covariance.pose.orientation,
        object1.kinematics.pose_with_covariance.pose.orientation, false);
      if (std::fabs(max_rad) < M_PI && std::fabs(max_rad) < std::fabs(angle)) passed_gate = false;
    }
    // 2d iou gate
    if (passed_gate) {
      const double min_iou = min_iou_matrix_(object1_label, object0_label);
      const double min_union_iou_area = 1e-2;
      const double iou = object_recognition_utils::get2dIoU(object0, object1, min_union_iou_area);
      if (iou < min_iou) passed_gate = false;
    }
    // max velocity diff gate
    if (passed_gate) {
      const double max_velocity_diff = max_velocity_diff_matrix_(object1_label, object0_label);
      // calc absolute velocity diff (only thinking about speed)
      const auto speed0 = std::hypot(
        object0.kinematics.twist_with_covariance.twist.linear.x,
        object0.kinematics.twist_with_covariance.twist.linear.y);
      const auto speed1 = std::hypot(
        object1.kinematics.twist_with_covariance.twist.linear.x,
        object1.kinematics.twist_with_covariance.twist.linear.y);
      const double velocity_diff = std::fabs(speed0 - speed1);
      if (max_velocity_diff < velocity_diff) passed_gate = false;
    }

    // all gate is passed
    if (passed_gate) {
      score = (max_dist - std::min(dist, max_dist)) / max_dist;
      if (score < score_threshold_) score = 0.0;
    }
  }
  return score;
}
