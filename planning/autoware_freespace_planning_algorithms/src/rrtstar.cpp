// Copyright 2021 Tier IV, Inc. All rights reserved.
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

#include "autoware/freespace_planning_algorithms/rrtstar.hpp"

#include "autoware/freespace_planning_algorithms/kinematic_bicycle_model.hpp"

namespace autoware::freespace_planning_algorithms
{
rrtstar_core::Pose poseMsgToPose(const geometry_msgs::msg::Pose & pose_msg)
{
  return rrtstar_core::Pose{
    pose_msg.position.x, pose_msg.position.y, tf2::getYaw(pose_msg.orientation)};
}

RRTStar::RRTStar(
  const PlannerCommonParam & planner_common_param, const VehicleShape & original_vehicle_shape,
  const RRTStarParam & rrtstar_param)
: AbstractPlanningAlgorithm(
    planner_common_param, VehicleShape(
                            original_vehicle_shape.length + 2 * rrtstar_param.margin,
                            original_vehicle_shape.width + 2 * rrtstar_param.margin,
                            original_vehicle_shape.base_length, original_vehicle_shape.max_steering,
                            original_vehicle_shape.base2back + rrtstar_param.margin)),
  rrtstar_param_(rrtstar_param),
  original_vehicle_shape_(original_vehicle_shape)
{
  if (rrtstar_param_.margin <= 0) {
    throw std::invalid_argument("rrt's collision margin must be greater than 0");
  }
}

bool RRTStar::makePlan(
  const geometry_msgs::msg::Pose & start_pose,
  const std::vector<geometry_msgs::msg::Pose> & goal_candidates)
{
  return makePlan(start_pose, goal_candidates.front());
}

bool RRTStar::makePlan(
  const geometry_msgs::msg::Pose & start_pose, const geometry_msgs::msg::Pose & goal_pose)
{
  const rclcpp::Time begin = rclcpp::Clock(RCL_ROS_TIME).now();

  start_pose_ = global2local(costmap_, start_pose);
  goal_pose_ = global2local(costmap_, goal_pose);

  const auto is_obstacle_free = [&](const rrtstar_core::Pose & pose) {
    const int index_x = std::round(pose.x / costmap_.info.resolution);
    const int index_y = std::round(pose.y / costmap_.info.resolution);
    const int index_theta = discretizeAngle(pose.yaw, planner_common_param_.theta_size);
    return !detectCollision(IndexXYT{index_x, index_y, index_theta});
  };

  const rrtstar_core::Pose lo{0, 0, 0};
  const rrtstar_core::Pose hi{
    costmap_.info.resolution * costmap_.info.width, costmap_.info.resolution * costmap_.info.height,
    M_PI};
  const double radius = kinematic_bicycle_model::getTurningRadius(
    collision_vehicle_shape_.base_length, collision_vehicle_shape_.max_steering);
  const auto cspace = rrtstar_core::CSpace(lo, hi, radius, is_obstacle_free);
  const auto x_start = poseMsgToPose(start_pose_);
  const auto x_goal = poseMsgToPose(goal_pose_);

  if (!is_obstacle_free(x_start)) {
    return false;
  }

  if (!is_obstacle_free(x_goal)) {
    return false;
  }

  const bool is_informed = rrtstar_param_.use_informed_sampling;  // always better
  const double collision_check_resolution = rrtstar_param_.margin * 2;
  auto algo = rrtstar_core::RRTStar(
    x_start, x_goal, rrtstar_param_.neighbor_radius, collision_check_resolution, is_informed,
    cspace);
  while (true) {
    const rclcpp::Time now = rclcpp::Clock(RCL_ROS_TIME).now();
    const double msec = (now - begin).seconds() * 1000.0;

    if (msec > planner_common_param_.time_limit) {
      // break regardless of solution find or not
      break;
    }

    if (algo.isSolutionFound()) {
      if (!rrtstar_param_.enable_update) {
        break;
      } else {
        if (msec > rrtstar_param_.max_planning_time) {
          break;
        }
      }
    }

    algo.extend();
  }

  if (!algo.isSolutionFound()) {
    return false;
  }
  const auto waypoints = algo.sampleSolutionWaypoints();
  setRRTPath(waypoints);
  return true;
}

bool RRTStar::hasObstacleOnTrajectory(const geometry_msgs::msg::PoseArray & trajectory) const
{
  for (const auto & pose : trajectory.poses) {
    const auto pose_local = global2local(costmap_, pose);
    if (detectCollision(pose_local)) {
      return true;
    }
  }
  return false;
}

void RRTStar::setRRTPath(const std::vector<rrtstar_core::Pose> & waypoints)
{
  std_msgs::msg::Header header;
  header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  header.frame_id = costmap_.header.frame_id;

  waypoints_.header = header;
  waypoints_.waypoints.clear();

  for (size_t i = 0; i < waypoints.size(); ++i) {
    auto & pt = waypoints.at(i);
    geometry_msgs::msg::Pose pose_local;
    pose_local.position.x = pt.x;
    pose_local.position.y = pt.y;
    pose_local.position.z = goal_pose_.position.z;
    tf2::Quaternion quat;
    quat.setRPY(0, 0, pt.yaw);
    tf2::convert(quat, pose_local.orientation);

    geometry_msgs::msg::PoseStamped pose;
    pose.pose = local2global(costmap_, pose_local);
    pose.header = header;
    PlannerWaypoint pw;
    if (0 == i) {
      const auto & pt_now = waypoints.at(i);
      const auto & pt_next = waypoints.at(i + 1);
      const double inner_product =
        cos(pt_now.yaw) * (pt_next.x - pt_now.x) + sin(pt_now.yaw) * (pt_next.y - pt_now.y);
      pw.is_back = (inner_product < 0.0);
    } else {
      const auto & pt_pre = waypoints.at(i - 1);
      const auto & pt_now = waypoints.at(i);
      const double inner_product =
        cos(pt_pre.yaw) * (pt_now.x - pt_pre.x) + sin(pt_pre.yaw) * (pt_now.y - pt_pre.y);
      pw.is_back = !(inner_product > 0.0);
    }
    pw.pose = pose;
    waypoints_.waypoints.push_back(pw);
  }
}

}  // namespace autoware::freespace_planning_algorithms
