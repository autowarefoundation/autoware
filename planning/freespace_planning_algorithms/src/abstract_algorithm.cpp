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

#include "freespace_planning_algorithms/abstract_algorithm.hpp"

#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/math/normalization.hpp>

#include <vector>

namespace freespace_planning_algorithms
{
using tier4_autoware_utils::createQuaternionFromYaw;
using tier4_autoware_utils::normalizeRadian;

geometry_msgs::msg::Pose transformPose(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::TransformStamped & transform)
{
  geometry_msgs::msg::Pose transformed_pose;
  tf2::doTransform(pose, transformed_pose, transform);

  return transformed_pose;
}

int discretizeAngle(const double theta, const int theta_size)
{
  const double one_angle_range = 2.0 * M_PI / theta_size;
  return static_cast<int>(std::rint(normalizeRadian(theta, 0.0) / one_angle_range)) % theta_size;
}

IndexXYT pose2index(
  const nav_msgs::msg::OccupancyGrid & costmap, const geometry_msgs::msg::Pose & pose_local,
  const int theta_size)
{
  const int index_x = pose_local.position.x / costmap.info.resolution;
  const int index_y = pose_local.position.y / costmap.info.resolution;
  const int index_theta = discretizeAngle(tf2::getYaw(pose_local.orientation), theta_size);
  return {index_x, index_y, index_theta};
}

geometry_msgs::msg::Pose index2pose(
  const nav_msgs::msg::OccupancyGrid & costmap, const IndexXYT & index, const int theta_size)
{
  geometry_msgs::msg::Pose pose_local;

  pose_local.position.x = index.x * costmap.info.resolution;
  pose_local.position.y = index.y * costmap.info.resolution;

  const double one_angle_range = 2.0 * M_PI / theta_size;
  const double yaw = index.theta * one_angle_range;
  pose_local.orientation = createQuaternionFromYaw(yaw);

  return pose_local;
}

geometry_msgs::msg::Pose global2local(
  const nav_msgs::msg::OccupancyGrid & costmap, const geometry_msgs::msg::Pose & pose_global)
{
  tf2::Transform tf_origin;
  tf2::convert(costmap.info.origin, tf_origin);

  geometry_msgs::msg::TransformStamped transform;
  transform.transform = tf2::toMsg(tf_origin.inverse());

  return transformPose(pose_global, transform);
}

geometry_msgs::msg::Pose local2global(
  const nav_msgs::msg::OccupancyGrid & costmap, const geometry_msgs::msg::Pose & pose_local)
{
  tf2::Transform tf_origin;
  tf2::convert(costmap.info.origin, tf_origin);

  geometry_msgs::msg::TransformStamped transform;
  transform.transform = tf2::toMsg(tf_origin);

  return transformPose(pose_local, transform);
}

double PlannerWaypoints::compute_length() const
{
  if (waypoints.empty()) {
    std::runtime_error("cannot compute cost because waypoint has size 0");
  }
  double total_cost = 0.0;
  for (size_t i = 0; i < waypoints.size() - 1; ++i) {
    const auto pose_a = waypoints.at(i);
    const auto pose_b = waypoints.at(i + 1);
    total_cost += tier4_autoware_utils::calcDistance2d(pose_a.pose, pose_b.pose);
  }
  return total_cost;
}

void AbstractPlanningAlgorithm::setMap(const nav_msgs::msg::OccupancyGrid & costmap)
{
  costmap_ = costmap;
  const auto height = costmap_.info.height;
  const auto width = costmap_.info.width;

  // Initialize status
  std::vector<std::vector<bool>> is_obstacle_table;
  is_obstacle_table.resize(height);
  for (uint32_t i = 0; i < height; i++) {
    is_obstacle_table.at(i).resize(width);
    for (uint32_t j = 0; j < width; j++) {
      const int cost = costmap_.data[i * width + j];

      if (cost < 0 || planner_common_param_.obstacle_threshold <= cost) {
        is_obstacle_table[i][j] = true;
      }
    }
  }
  is_obstacle_table_ = is_obstacle_table;

  // construct collision indexes table
  if (is_collision_table_initialized == false) {
    for (int i = 0; i < planner_common_param_.theta_size; i++) {
      std::vector<IndexXY> indexes_2d, vertex_indexes_2d;
      computeCollisionIndexes(i, indexes_2d, vertex_indexes_2d);
      coll_indexes_table_.push_back(indexes_2d);
      vertex_indexes_table_.push_back(vertex_indexes_2d);
    }
    is_collision_table_initialized = true;
  }
}

void AbstractPlanningAlgorithm::computeCollisionIndexes(
  int theta_index, std::vector<IndexXY> & indexes_2d,
  std::vector<IndexXY> & vertex_indexes_2d) const
{
  IndexXYT base_index{0, 0, theta_index};
  const VehicleShape & vehicle_shape = collision_vehicle_shape_;

  // Define the robot as rectangle
  const double back = -1.0 * vehicle_shape.base2back;
  const double front = vehicle_shape.length - vehicle_shape.base2back;
  const double right = -1.0 * vehicle_shape.width / 2.0;
  const double left = vehicle_shape.width / 2.0;

  const auto base_pose = index2pose(costmap_, base_index, planner_common_param_.theta_size);
  const auto base_theta = tf2::getYaw(base_pose.orientation);

  // Convert each point to index and check if the node is Obstacle
  const auto addIndex2d = [&](
                            const double x, const double y, std::vector<IndexXY> & indexes_cache) {
    // Calculate offset in rotated frame
    const double offset_x = std::cos(base_theta) * x - std::sin(base_theta) * y;
    const double offset_y = std::sin(base_theta) * x + std::cos(base_theta) * y;

    geometry_msgs::msg::Pose pose_local;
    pose_local.position.x = base_pose.position.x + offset_x;
    pose_local.position.y = base_pose.position.y + offset_y;

    const auto index = pose2index(costmap_, pose_local, planner_common_param_.theta_size);
    const auto index_2d = IndexXY{index.x, index.y};
    indexes_cache.push_back(index_2d);
  };

  for (double x = back; x <= front; x += costmap_.info.resolution / 2) {
    for (double y = right; y <= left; y += costmap_.info.resolution / 2) {
      addIndex2d(x, y, indexes_2d);
    }
    addIndex2d(x, left, indexes_2d);
  }
  for (double y = right; y <= left; y += costmap_.info.resolution / 2) {
    addIndex2d(front, y, indexes_2d);
  }
  addIndex2d(front, left, indexes_2d);

  const auto compareIndex2d = [](const IndexXY & left, const IndexXY & right) {
    if (left.x != right.x) {
      return (left.x < right.x);
    } else {
      return (left.y < right.y);
    }
  };

  const auto equalIndex2d = [](const IndexXY & left, const IndexXY & right) {
    return ((left.x == right.x) && (left.y == right.y));
  };

  // remove duplicate indexes
  std::sort(indexes_2d.begin(), indexes_2d.end(), compareIndex2d);
  indexes_2d.erase(
    std::unique(indexes_2d.begin(), indexes_2d.end(), equalIndex2d), indexes_2d.end());

  addIndex2d(front, left, vertex_indexes_2d);
  addIndex2d(front, right, vertex_indexes_2d);
  addIndex2d(back, right, vertex_indexes_2d);
  addIndex2d(back, left, vertex_indexes_2d);
}

bool AbstractPlanningAlgorithm::detectCollision(const IndexXYT & base_index) const
{
  if (coll_indexes_table_.empty()) {
    std::cerr << "[abstract_algorithm] setMap has not yet been done." << std::endl;
    return false;
  }

  const auto & vertex_indexes_2d = vertex_indexes_table_[base_index.theta];
  for (const auto & vertex_index_2d : vertex_indexes_2d) {
    IndexXYT vertex_index{vertex_index_2d.x, vertex_index_2d.y, 0};
    // must slide to current base position
    vertex_index.x += base_index.x;
    vertex_index.y += base_index.y;
    if (isOutOfRange(vertex_index)) {
      return true;
    }
  }

  const auto & coll_indexes_2d = coll_indexes_table_[base_index.theta];
  for (const auto & coll_index_2d : coll_indexes_2d) {
    int idx_theta = 0;  // whatever. Yaw is nothing to do with collision detection between grids.
    IndexXYT coll_index{coll_index_2d.x, coll_index_2d.y, idx_theta};
    // must slide to current base position
    coll_index.x += base_index.x;
    coll_index.y += base_index.y;

    if (isObs(coll_index)) {
      return true;
    }
  }

  return false;
}

bool AbstractPlanningAlgorithm::hasObstacleOnTrajectory(
  const geometry_msgs::msg::PoseArray & trajectory) const
{
  for (const auto & pose : trajectory.poses) {
    const auto pose_local = global2local(costmap_, pose);
    const auto index = pose2index(costmap_, pose_local, planner_common_param_.theta_size);

    if (detectCollision(index)) {
      return true;
    }
  }

  return false;
}

}  // namespace freespace_planning_algorithms
