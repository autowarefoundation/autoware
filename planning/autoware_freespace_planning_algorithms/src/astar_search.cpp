// Copyright 2015-2019 Autoware Foundation. All rights reserved.
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

#include "autoware/freespace_planning_algorithms/astar_search.hpp"

#include "autoware/freespace_planning_algorithms/kinematic_bicycle_model.hpp"

#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware/universe_utils/math/unit_conversion.hpp>

#include <tf2/utils.h>

#include <limits>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <algorithm>
#include <vector>

namespace autoware::freespace_planning_algorithms
{
using autoware::universe_utils::calcDistance2d;

double calcReedsSheppDistance(const Pose & p1, const Pose & p2, double radius)
{
  const auto rs_space = ReedsSheppStateSpace(radius);
  const ReedsSheppStateSpace::StateXYT pose0{
    p1.position.x, p1.position.y, tf2::getYaw(p1.orientation)};
  const ReedsSheppStateSpace::StateXYT pose1{
    p2.position.x, p2.position.y, tf2::getYaw(p2.orientation)};
  return rs_space.distance(pose0, pose1);
}

void setYaw(geometry_msgs::msg::Quaternion * orientation, const double yaw)
{
  *orientation = autoware::universe_utils::createQuaternionFromYaw(yaw);
}

Pose calcRelativePose(const Pose & base_pose, const Pose & pose)
{
  tf2::Transform tf_transform;
  tf2::convert(base_pose, tf_transform);

  geometry_msgs::msg::TransformStamped transform;
  transform.transform = tf2::toMsg(tf_transform.inverse());

  geometry_msgs::msg::PoseStamped transformed;
  geometry_msgs::msg::PoseStamped pose_orig;
  pose_orig.pose = pose;
  tf2::doTransform(pose_orig, transformed, transform);

  return transformed.pose;
}

AstarSearch::AstarSearch(
  const PlannerCommonParam & planner_common_param, const VehicleShape & collision_vehicle_shape,
  const AstarParam & astar_param)
: AbstractPlanningAlgorithm(planner_common_param, collision_vehicle_shape),
  astar_param_(astar_param),
  goal_node_(nullptr),
  use_reeds_shepp_(true)
{
  steering_resolution_ =
    collision_vehicle_shape_.max_steering / planner_common_param_.turning_steps;
  heading_resolution_ = 2.0 * M_PI / planner_common_param_.theta_size;

  const double avg_steering =
    steering_resolution_ + (collision_vehicle_shape_.max_steering - steering_resolution_) / 2.0;
  avg_turning_radius_ =
    kinematic_bicycle_model::getTurningRadius(collision_vehicle_shape_.base_length, avg_steering);

  is_backward_search_ = astar_param_.search_method == "backward";

  min_expansion_dist_ = astar_param_.expansion_distance;
  max_expansion_dist_ = collision_vehicle_shape_.base_length * base_length_max_expansion_factor_;
}

void AstarSearch::setMap(const nav_msgs::msg::OccupancyGrid & costmap)
{
  AbstractPlanningAlgorithm::setMap(costmap);

  // ensure minimum expansion distance is larger then grid cell diagonal length
  min_expansion_dist_ = std::max(astar_param_.expansion_distance, 1.5 * costmap_.info.resolution);
  max_expansion_dist_ = std::max(
    collision_vehicle_shape_.base_length * base_length_max_expansion_factor_, min_expansion_dist_);
}

bool AstarSearch::makePlan(const Pose & start_pose, const Pose & goal_pose)
{
  resetData();

  start_pose_ = global2local(costmap_, start_pose);
  goal_pose_ = global2local(costmap_, goal_pose);

  if (detectCollision(start_pose_) || detectCollision(goal_pose_)) {
    throw std::logic_error("Invalid start or goal pose");
  }

  if (is_backward_search_) std::swap(start_pose_, goal_pose_);

  setCollisionFreeDistanceMap();

  setStartNode();

  if (!search()) {
    throw std::logic_error("HA* failed to find path to goal");
  }

  return true;
}

void AstarSearch::resetData()
{
  // clearing openlist is necessary because otherwise remaining elements of openlist
  // point to deleted node.
  openlist_ = std::priority_queue<AstarNode *, std::vector<AstarNode *>, NodeComparison>();
  const int nb_of_grid_nodes = costmap_.info.width * costmap_.info.height;
  const int total_astar_node_count = nb_of_grid_nodes * planner_common_param_.theta_size;
  graph_ = std::vector<AstarNode>(total_astar_node_count);
  col_free_distance_map_ =
    std::vector<double>(nb_of_grid_nodes, std::numeric_limits<double>::max());
}

void AstarSearch::setCollisionFreeDistanceMap()
{
  using Entry = std::pair<IndexXY, double>;
  struct CompareEntry
  {
    bool operator()(const Entry & a, const Entry & b) const { return a.second > b.second; }
  };
  std::priority_queue<Entry, std::vector<Entry>, CompareEntry> heap;
  std::vector<bool> closed(col_free_distance_map_.size(), false);
  auto goal_index = pose2index(costmap_, goal_pose_, planner_common_param_.theta_size);
  col_free_distance_map_[indexToId(goal_index)] = 0.0;
  heap.push({IndexXY{goal_index.x, goal_index.y}, 0.0});

  Entry current;
  std::array<int, 3> offsets = {1, 0, -1};
  while (!heap.empty()) {
    current = heap.top();
    heap.pop();
    const int id = indexToId(current.first);
    if (closed[id]) continue;
    closed[id] = true;

    const auto & index = current.first;
    for (const auto & offset_x : offsets) {
      const int x = index.x + offset_x;
      for (const auto & offset_y : offsets) {
        const int y = index.y + offset_y;
        const IndexXY n_index{x, y};
        const double offset = std::abs(offset_x) + std::abs(offset_y);
        if (isOutOfRange(n_index) || isObs(n_index) || offset < 1) continue;
        if (getObstacleEDT(n_index) < 0.5 * collision_vehicle_shape_.width) continue;
        const int n_id = indexToId(n_index);
        const double dist = current.second + (sqrt(offset) * costmap_.info.resolution);
        if (closed[n_id] || col_free_distance_map_[n_id] < dist) continue;
        col_free_distance_map_[n_id] = dist;
        heap.push({n_index, dist});
      }
    }
  }
}

void AstarSearch::setStartNode()
{
  const auto index = pose2index(costmap_, start_pose_, planner_common_param_.theta_size);
  // Set start node
  AstarNode * start_node = &graph_[getKey(index)];
  start_node->set(start_pose_, 0.0, estimateCost(start_pose_, index), 0, false);
  start_node->dir_distance = 0.0;
  start_node->dist_to_goal = calcDistance2d(start_pose_, goal_pose_);
  start_node->dist_to_obs = getObstacleEDT(index);
  start_node->status = NodeStatus::Open;
  start_node->parent = nullptr;

  // Push start node to openlist
  openlist_.push(start_node);
}

double AstarSearch::estimateCost(const Pose & pose, const IndexXYT & index) const
{
  double total_cost = col_free_distance_map_[indexToId(index)];
  // Temporarily, until reeds_shepp gets stable.
  if (use_reeds_shepp_) {
    total_cost =
      std::max(total_cost, calcReedsSheppDistance(pose, goal_pose_, avg_turning_radius_));
  }
  return astar_param_.distance_heuristic_weight * total_cost;
}

bool AstarSearch::search()
{
  const rclcpp::Time begin = rclcpp::Clock(RCL_ROS_TIME).now();

  // Start A* search
  while (!openlist_.empty()) {
    // Check time and terminate if the search reaches the time limit
    const rclcpp::Time now = rclcpp::Clock(RCL_ROS_TIME).now();
    const double msec = (now - begin).seconds() * 1000.0;
    if (msec > planner_common_param_.time_limit) {
      return false;
    }

    // Expand minimum cost node
    AstarNode * current_node = openlist_.top();
    openlist_.pop();
    if (current_node->status == NodeStatus::Closed) continue;
    current_node->status = NodeStatus::Closed;

    if (isGoal(*current_node)) {
      goal_node_ = current_node;
      setPath(*current_node);
      return true;
    }

    expandNodes(*current_node);
    if (astar_param_.use_back) expandNodes(*current_node, true);
  }

  // Failed to find path
  return false;
}

void AstarSearch::expandNodes(AstarNode & current_node, const bool is_back)
{
  const auto current_pose = node2pose(current_node);
  const double direction = (is_back == is_backward_search_) ? 1.0 : -1.0;
  const double distance = getExpansionDistance(current_node) * direction;
  int steering_index = -1 * planner_common_param_.turning_steps;
  for (; steering_index <= planner_common_param_.turning_steps; ++steering_index) {
    // skip expansion back to parent
    if (
      current_node.parent != nullptr && is_back != current_node.is_back &&
      steering_index == current_node.steering_index) {
      continue;
    }

    const double steering = static_cast<double>(steering_index) * steering_resolution_;
    const auto next_pose = kinematic_bicycle_model::getPose(
      current_pose, collision_vehicle_shape_.base_length, steering, distance);
    const auto next_index = pose2index(costmap_, next_pose, planner_common_param_.theta_size);

    if (isOutOfRange(next_index) || isObs(next_index)) continue;

    AstarNode * next_node = &graph_[getKey(next_index)];
    if (next_node->status == NodeStatus::Closed || detectCollision(next_index)) continue;

    const double distance_to_obs = getObstacleEDT(next_index);
    const bool is_direction_switch =
      (current_node.parent != nullptr) && (is_back != current_node.is_back);

    double total_weight = 1.0;
    total_weight += getSteeringCost(steering_index);
    if (is_back) total_weight *= (1.0 + planner_common_param_.reverse_weight);

    double move_cost = current_node.gc + (total_weight * std::abs(distance));
    move_cost += getSteeringChangeCost(steering_index, current_node.steering_index);
    move_cost += getObsDistanceCost(distance_to_obs);
    if (is_direction_switch) move_cost += getDirectionChangeCost(current_node.dir_distance);

    double total_cost = move_cost + estimateCost(next_pose, next_index);
    // Compare cost
    if (next_node->status == NodeStatus::None || next_node->fc > total_cost) {
      next_node->status = NodeStatus::Open;
      next_node->set(next_pose, move_cost, total_cost, steering_index, is_back);
      next_node->dir_distance =
        std::abs(distance) + (is_direction_switch ? 0.0 : current_node.dir_distance);
      next_node->dist_to_goal = calcDistance2d(next_pose, goal_pose_);
      next_node->dist_to_obs = distance_to_obs;
      next_node->parent = &current_node;
      openlist_.push(next_node);
      continue;
    }
  }
}

double AstarSearch::getExpansionDistance(const AstarNode & current_node) const
{
  if (!astar_param_.adapt_expansion_distance || max_expansion_dist_ <= min_expansion_dist_) {
    return min_expansion_dist_;
  }
  double exp_dist = std::min(
    current_node.dist_to_goal * dist_to_goal_expansion_factor_,
    current_node.dist_to_obs * dist_to_obs_expansion_factor_);
  return std::clamp(exp_dist, min_expansion_dist_, max_expansion_dist_);
}

double AstarSearch::getSteeringCost(const int steering_index) const
{
  return planner_common_param_.curve_weight *
         (abs(steering_index) / planner_common_param_.turning_steps);
}

double AstarSearch::getSteeringChangeCost(
  const int steering_index, const int prev_steering_index) const
{
  double steering_index_diff = abs(steering_index - prev_steering_index);
  return astar_param_.smoothness_weight * steering_index_diff /
         (2.0 * planner_common_param_.turning_steps);
}

double AstarSearch::getDirectionChangeCost(const double dir_distance) const
{
  return planner_common_param_.direction_change_weight * (1.0 + (1.0 / (1.0 + dir_distance)));
}

double AstarSearch::getObsDistanceCost(const double obs_distance) const
{
  return astar_param_.obstacle_distance_weight *
         std::max(1.0 - (obs_distance / cost_free_obs_dist), 0.0);
}

void AstarSearch::setPath(const AstarNode & goal_node)
{
  std_msgs::msg::Header header;
  header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  header.frame_id = costmap_.header.frame_id;

  // From the goal node to the start node
  const AstarNode * node = &goal_node;

  std::vector<PlannerWaypoint> waypoints;

  geometry_msgs::msg::PoseStamped pose;
  pose.header = header;

  const auto interpolate = [this, &waypoints, &pose](const AstarNode & node) {
    if (node.parent == nullptr || !astar_param_.adapt_expansion_distance) return;
    const auto parent_pose = node2pose(*node.parent);
    const double distance_2d = calcDistance2d(node2pose(node), parent_pose);
    const int n = static_cast<int>(distance_2d / min_expansion_dist_);
    for (int i = 1; i < n; ++i) {
      const double dist =
        ((distance_2d * i) / n) * (node.is_back == is_backward_search_ ? 1.0 : -1.0);
      const double steering = node.steering_index * steering_resolution_;
      const auto local_pose = kinematic_bicycle_model::getPose(
        parent_pose, collision_vehicle_shape_.base_length, steering, dist);
      pose.pose = local2global(costmap_, local_pose);
      waypoints.push_back({pose, node.is_back});
    }
  };

  // push astar nodes poses
  while (node != nullptr) {
    pose.pose = local2global(costmap_, node2pose(*node));
    waypoints.push_back({pose, node->is_back});
    interpolate(*node);
    // To the next node
    node = node->parent;
  }

  if (waypoints.empty()) return;

  if (waypoints.size() > 1) waypoints.back().is_back = waypoints.rbegin()[1].is_back;

  if (!is_backward_search_) {
    // Reverse the vector to be start to goal order
    std::reverse(waypoints.begin(), waypoints.end());
  }

  waypoints_.header = header;
  waypoints_.waypoints.clear();

  for (size_t i = 0; i < waypoints.size() - 1; ++i) {
    const auto & current = waypoints[i];
    const auto & next = waypoints[i + 1];

    waypoints_.waypoints.push_back(current);

    if (current.is_back != next.is_back) {
      waypoints_.waypoints.push_back(
        {is_backward_search_ ? next.pose : current.pose,
         is_backward_search_ ? current.is_back : next.is_back});
    }
  }

  waypoints_.waypoints.push_back(waypoints.back());
}

bool AstarSearch::isGoal(const AstarNode & node) const
{
  const double lateral_goal_range = planner_common_param_.lateral_goal_range / 2.0;
  const double longitudinal_goal_range = planner_common_param_.longitudinal_goal_range / 2.0;
  const double goal_angle =
    autoware::universe_utils::deg2rad(planner_common_param_.angle_goal_range / 2.0);

  const auto relative_pose = calcRelativePose(goal_pose_, node2pose(node));

  // Check conditions
  if (astar_param_.only_behind_solutions && relative_pose.position.x > 0) {
    return false;
  }

  if (
    std::fabs(relative_pose.position.x) > longitudinal_goal_range ||
    std::fabs(relative_pose.position.y) > lateral_goal_range) {
    return false;
  }

  const auto angle_diff =
    autoware::universe_utils::normalizeRadian(tf2::getYaw(relative_pose.orientation));
  if (std::abs(angle_diff) > goal_angle) {
    return false;
  }

  return true;
}

Pose AstarSearch::node2pose(const AstarNode & node) const
{
  Pose pose_local;

  pose_local.position.x = node.x;
  pose_local.position.y = node.y;
  pose_local.position.z = goal_pose_.position.z;
  pose_local.orientation = autoware::universe_utils::createQuaternionFromYaw(node.theta);

  return pose_local;
}

}  // namespace autoware::freespace_planning_algorithms
