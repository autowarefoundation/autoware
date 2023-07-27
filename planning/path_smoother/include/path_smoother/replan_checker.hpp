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

#ifndef PATH_SMOOTHER__REPLAN_CHECKER_HPP_
#define PATH_SMOOTHER__REPLAN_CHECKER_HPP_

#include "path_smoother/common_structs.hpp"
#include "path_smoother/type_alias.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <vector>

namespace path_smoother
{
class ReplanChecker
{
public:
  explicit ReplanChecker(rclcpp::Node * node, const EgoNearestParam & ego_nearest_param);
  void onParam(const std::vector<rclcpp::Parameter> & parameters);

  bool isResetRequired(const PlannerData & planner_data) const;

  bool isReplanRequired(const PlannerData & planner_data, const rclcpp::Time & current_time) const;

  void updateData(
    const PlannerData & planner_data, const bool is_replan_required,
    const rclcpp::Time & current_time);

private:
  EgoNearestParam ego_nearest_param_;
  rclcpp::Logger logger_;

  // previous variables for isResetRequired
  std::shared_ptr<std::vector<TrajectoryPoint>> prev_traj_points_ptr_{nullptr};
  std::shared_ptr<geometry_msgs::msg::Pose> prev_ego_pose_ptr_{nullptr};

  // previous variable for isReplanRequired
  std::shared_ptr<rclcpp::Time> prev_replanned_time_ptr_{nullptr};

  // algorithm parameters
  bool enable_;
  double max_path_shape_around_ego_lat_dist_;
  double max_path_shape_forward_lat_dist_;
  double max_path_shape_forward_lon_dist_;
  double max_ego_moving_dist_;
  double max_goal_moving_dist_;
  double max_delta_time_sec_;

  bool isPathAroundEgoChanged(
    const PlannerData & planner_data, const std::vector<TrajectoryPoint> & prev_traj_points) const;
  bool isPathForwardChanged(
    const PlannerData & planner_data, const std::vector<TrajectoryPoint> & prev_traj_points) const;
  bool isPathGoalChanged(
    const PlannerData & planner_data, const std::vector<TrajectoryPoint> & prev_traj_points) const;
};
}  // namespace path_smoother

#endif  // PATH_SMOOTHER__REPLAN_CHECKER_HPP_
