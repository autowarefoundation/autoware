// Copyright 2022 TIER IV, Inc. All rights reserved.
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

#include "behavior_path_planner_common/utils/occupancy_grid_based_collision_detector/occupancy_grid_based_collision_detector.hpp"

#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/math/normalization.hpp>

#include <vector>

namespace behavior_path_planner
{
using tier4_autoware_utils::createQuaternionFromYaw;
using tier4_autoware_utils::normalizeRadian;
using tier4_autoware_utils::transformPose;

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

void OccupancyGridBasedCollisionDetector::setMap(const nav_msgs::msg::OccupancyGrid & costmap)
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

      if (cost < 0 || param_.obstacle_threshold <= cost) {
        is_obstacle_table[i][j] = true;
      }
    }
  }
  is_obstacle_table_ = is_obstacle_table;

  // construct collision indexes table
  coll_indexes_table_.clear();
  for (int i = 0; i < param_.theta_size; i++) {
    std::vector<IndexXY> indexes_2d;
    computeCollisionIndexes(i, indexes_2d);
    coll_indexes_table_.push_back(indexes_2d);
  }
}

void OccupancyGridBasedCollisionDetector::computeCollisionIndexes(
  int theta_index, std::vector<IndexXY> & indexes_2d)
{
  IndexXYT base_index{0, 0, theta_index};
  const VehicleShape & vehicle_shape = param_.vehicle_shape;

  // Define the robot as rectangle
  const double back = -1.0 * vehicle_shape.base2back;
  const double front = vehicle_shape.length - vehicle_shape.base2back;
  const double right = -1.0 * vehicle_shape.width / 2.0;
  const double left = vehicle_shape.width / 2.0;

  const auto base_pose = index2pose(costmap_, base_index, param_.theta_size);
  const auto base_theta = tf2::getYaw(base_pose.orientation);

  // Convert each point to index and check if the node is Obstacle
  const auto addIndex2d = [&](const double x, const double y) {
    // Calculate offset in rotated frame
    const double offset_x = std::cos(base_theta) * x - std::sin(base_theta) * y;
    const double offset_y = std::sin(base_theta) * x + std::cos(base_theta) * y;

    geometry_msgs::msg::Pose pose_local;
    pose_local.position.x = base_pose.position.x + offset_x;
    pose_local.position.y = base_pose.position.y + offset_y;

    const auto index = pose2index(costmap_, pose_local, param_.theta_size);
    const auto index_2d = IndexXY{index.x, index.y};
    indexes_2d.push_back(index_2d);
  };

  for (double x = back; x <= front; x += costmap_.info.resolution / 2) {
    for (double y = right; y <= left; y += costmap_.info.resolution / 2) {
      addIndex2d(x, y);
    }
    addIndex2d(x, left);
  }
  for (double y = right; y <= left; y += costmap_.info.resolution / 2) {
    addIndex2d(front, y);
  }
  addIndex2d(front, left);
}

bool OccupancyGridBasedCollisionDetector::detectCollision(
  const IndexXYT & base_index, const bool check_out_of_range) const
{
  if (coll_indexes_table_.empty()) {
    std::cerr << "[occupancy_grid_based_collision_detector] setMap has not yet been done."
              << std::endl;
    return false;
  }
  const auto & coll_indexes_2d = coll_indexes_table_[base_index.theta];
  for (const auto & coll_index_2d : coll_indexes_2d) {
    int idx_theta = 0;  // whatever. Yaw is nothing to do with collision detection between grids.
    IndexXYT coll_index{coll_index_2d.x, coll_index_2d.y, idx_theta};
    // must slide to current base position
    coll_index.x += base_index.x;
    coll_index.y += base_index.y;
    if (check_out_of_range) {
      if (isOutOfRange(coll_index) || isObs(coll_index)) return true;
    } else {
      if (isOutOfRange(coll_index)) return false;
      if (isObs(coll_index)) return true;
    }
  }
  return false;
}

bool OccupancyGridBasedCollisionDetector::hasObstacleOnPath(
  const geometry_msgs::msg::PoseArray & path, const bool check_out_of_range) const
{
  for (const auto & pose : path.poses) {
    const auto pose_local = global2local(costmap_, pose);
    const auto index = pose2index(costmap_, pose_local, param_.theta_size);

    if (detectCollision(index, check_out_of_range)) {
      return true;
    }
  }

  return false;
}

bool OccupancyGridBasedCollisionDetector::hasObstacleOnPath(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
  const bool check_out_of_range) const
{
  for (const auto & p : path.points) {
    const auto pose_local = global2local(costmap_, p.point.pose);
    const auto index = pose2index(costmap_, pose_local, param_.theta_size);

    if (detectCollision(index, check_out_of_range)) {
      return true;
    }
  }

  return false;
}

}  // namespace behavior_path_planner
