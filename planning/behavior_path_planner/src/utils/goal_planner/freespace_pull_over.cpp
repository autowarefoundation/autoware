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

#include "behavior_path_planner/utils/goal_planner/freespace_pull_over.hpp"

#include "behavior_path_planner/utils/goal_planner/util.hpp"
#include "behavior_path_planner/utils/path_utils.hpp"

#include <memory>
#include <vector>

namespace behavior_path_planner
{
FreespacePullOver::FreespacePullOver(
  rclcpp::Node & node, const GoalPlannerParameters & parameters,
  const vehicle_info_util::VehicleInfo & vehicle_info)
: PullOverPlannerBase{node, parameters}, velocity_{parameters.freespace_parking_velocity}
{
  freespace_planning_algorithms::VehicleShape vehicle_shape(
    vehicle_info, parameters.vehicle_shape_margin);
  if (parameters.freespace_parking_algorithm == "astar") {
    use_back_ = parameters.astar_parameters.use_back;
    planner_ = std::make_unique<AstarSearch>(
      parameters.freespace_parking_common_parameters, vehicle_shape, parameters.astar_parameters);
  } else if (parameters.freespace_parking_algorithm == "rrtstar") {
    use_back_ = true;  // no option for disabling back in rrtstar
    planner_ = std::make_unique<RRTStar>(
      parameters.freespace_parking_common_parameters, vehicle_shape,
      parameters.rrt_star_parameters);
  }
}

boost::optional<PullOverPath> FreespacePullOver::plan(const Pose & goal_pose)
{
  const Pose & current_pose = planner_data_->self_odometry->pose.pose;

  planner_->setMap(*planner_data_->costmap);

  // offset goal pose to make straight path near goal for improving parking precision
  // todo: support straight path when using back
  constexpr double straight_distance = 1.0;
  const Pose end_pose =
    use_back_ ? goal_pose
              : tier4_autoware_utils::calcOffsetPose(goal_pose, -straight_distance, 0.0, 0.0);
  const bool found_path = planner_->makePlan(current_pose, end_pose);
  if (!found_path) {
    return {};
  }

  PathWithLaneId path =
    utils::convertWayPointsToPathWithLaneId(planner_->getWaypoints(), velocity_);
  const auto reverse_indices = utils::getReversingIndices(path);
  std::vector<PathWithLaneId> partial_paths = utils::dividePath(path, reverse_indices);

  // remove points which are near the goal
  PathWithLaneId & last_path = partial_paths.back();
  const double th_goal_distance = 1.0;
  for (auto it = last_path.points.begin(); it != last_path.points.end(); ++it) {
    size_t index = std::distance(last_path.points.begin(), it);
    if (index == 0) continue;
    const double distance =
      tier4_autoware_utils::calcDistance2d(end_pose.position, it->point.pose.position);
    if (distance < th_goal_distance) {
      last_path.points.erase(it, last_path.points.end());
      break;
    }
  }

  // add PathPointWithLaneId to last path
  auto addPose = [&last_path](const Pose & pose) {
    PathPointWithLaneId p = last_path.points.back();
    p.point.pose = pose;
    last_path.points.push_back(p);
  };

  if (use_back_) {
    addPose(end_pose);
  } else {
    // add interpolated poses
    auto addInterpolatedPoses = [&addPose](const Pose & pose1, const Pose & pose2) {
      constexpr double interval = 0.5;
      std::vector<Pose> interpolated_poses = utils::interpolatePose(pose1, pose2, interval);
      for (const auto & pose : interpolated_poses) {
        addPose(pose);
      }
    };
    addInterpolatedPoses(last_path.points.back().point.pose, end_pose);
    addPose(end_pose);
    addInterpolatedPoses(end_pose, goal_pose);
    addPose(goal_pose);
  }

  utils::correctDividedPathVelocity(partial_paths);

  for (auto & path : partial_paths) {
    const auto is_driving_forward = motion_utils::isDrivingForward(path.points);
    if (!is_driving_forward) {
      // path points is less than 2
      return {};
    }
  }

  PullOverPath pull_over_path{};
  pull_over_path.partial_paths = partial_paths;
  pull_over_path.start_pose = current_pose;
  pull_over_path.end_pose = goal_pose;
  pull_over_path.type = getPlannerType();

  return pull_over_path;
}
}  // namespace behavior_path_planner
