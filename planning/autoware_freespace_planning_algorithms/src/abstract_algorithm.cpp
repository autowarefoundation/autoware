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

#include "autoware/freespace_planning_algorithms/abstract_algorithm.hpp"

#include "autoware/freespace_planning_algorithms/kinematic_bicycle_model.hpp"

#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware/universe_utils/math/normalization.hpp>

#include <limits>
#include <vector>

namespace autoware::freespace_planning_algorithms
{
using autoware::universe_utils::createQuaternionFromYaw;

geometry_msgs::msg::Pose transformPose(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::TransformStamped & transform)
{
  geometry_msgs::msg::Pose transformed_pose;
  tf2::doTransform(pose, transformed_pose, transform);

  return transformed_pose;
}

int discretizeAngle(const double theta, const int theta_size)
{
  const double angle_resolution = 2.0 * M_PI / theta_size;
  return static_cast<int>(std::round(normalizeRadian(theta, 0.0) / angle_resolution)) % theta_size;
}

IndexXYT pose2index(
  const nav_msgs::msg::OccupancyGrid & costmap, const geometry_msgs::msg::Pose & pose_local,
  const int theta_size)
{
  const int index_x = std::round(pose_local.position.x / costmap.info.resolution);
  const int index_y = std::round(pose_local.position.y / costmap.info.resolution);
  const int index_theta = discretizeAngle(tf2::getYaw(pose_local.orientation), theta_size);
  return {index_x, index_y, index_theta};
}

geometry_msgs::msg::Pose index2pose(
  const nav_msgs::msg::OccupancyGrid & costmap, const IndexXYT & index, const int theta_size)
{
  geometry_msgs::msg::Pose pose_local;

  pose_local.position.x = index.x * costmap.info.resolution;
  pose_local.position.y = index.y * costmap.info.resolution;

  const double angle_resolution = 2.0 * M_PI / theta_size;
  const double yaw = index.theta * angle_resolution;
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
    throw std::runtime_error("cannot compute cost because waypoint has size 0");
  }
  double total_cost = 0.0;
  for (size_t i = 0; i < waypoints.size() - 1; ++i) {
    const auto pose_a = waypoints.at(i);
    const auto pose_b = waypoints.at(i + 1);
    total_cost += autoware::universe_utils::calcDistance2d(pose_a.pose, pose_b.pose);
  }
  return total_cost;
}

void AbstractPlanningAlgorithm::setMap(const nav_msgs::msg::OccupancyGrid & costmap)
{
  costmap_ = costmap;

  const uint32_t nb_of_cells = costmap_.data.size();
  // Initialize status
  std::vector<bool> is_obstacle_table;
  is_obstacle_table.resize(nb_of_cells);
  for (uint32_t i = 0; i < nb_of_cells; ++i) {
    const int cost = costmap_.data[i];
    if (cost < 0 || planner_common_param_.obstacle_threshold <= cost) {
      is_obstacle_table[i] = true;
    }
  }
  is_obstacle_table_ = is_obstacle_table;

  computeEDTMap();

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

  const double base2front = collision_vehicle_shape_.length - collision_vehicle_shape_.base2back;
  nb_of_margin_cells_ = std::ceil(
    std::hypot(0.5 * collision_vehicle_shape_.width, base2front) / costmap_.info.resolution);
}

double AbstractPlanningAlgorithm::getDistanceToObstacle(const geometry_msgs::msg::Pose & pose) const
{
  const auto local_pose = global2local(costmap_, pose);
  const auto index = pose2index(costmap_, local_pose, planner_common_param_.theta_size);
  if (indexToId(index) >= static_cast<int>(edt_map_.size())) {
    return std::numeric_limits<double>::max();
  }
  return getObstacleEDT(index).distance;
}

void AbstractPlanningAlgorithm::computeEDTMap()
{
  edt_map_.clear();
  const int height = costmap_.info.height;
  const int width = costmap_.info.width;
  const double resolution_m = costmap_.info.resolution;
  std::vector<std::pair<double, geometry_msgs::msg::Point>> edt_map;
  edt_map.reserve(costmap_.data.size());

  std::vector<std::pair<double, geometry_msgs::msg::Point>> temporary_storage(width);
  // scan rows
  for (int i = 0; i < height; ++i) {
    double distance = resolution_m;
    bool found_obstacle = false;
    // forward scan
    for (int j = 0; j < width; ++j) {
      if (isObs(IndexXY{j, i})) {
        temporary_storage[j].first = 0.0;
        temporary_storage[j].second.x = 0.0;
        distance = resolution_m;
        found_obstacle = true;
      } else if (found_obstacle) {
        temporary_storage[j].first = distance;
        temporary_storage[j].second.x = -distance;
        distance += resolution_m;
      } else {
        temporary_storage[j].first = std::numeric_limits<double>::infinity();
      }
    }

    distance = resolution_m;
    found_obstacle = false;
    // backward scan
    for (int j = width - 1; j >= 0; --j) {
      if (isObs(IndexXY{j, i})) {
        distance = resolution_m;
        found_obstacle = true;
      } else if (found_obstacle && temporary_storage[j].first > distance) {
        temporary_storage[j].first = distance;
        temporary_storage[j].second.x = distance;
        distance += resolution_m;
      }
    }
    edt_map.insert(edt_map.end(), temporary_storage.begin(), temporary_storage.end());
  }

  temporary_storage.clear();
  temporary_storage.resize(height);
  // scan columns;
  for (int j = 0; j < width; ++j) {
    for (int i = 0; i < height; ++i) {
      int id = indexToId(IndexXY{j, i});
      double min_value = edt_map[id].first * edt_map[id].first;
      geometry_msgs::msg::Point rel_pos = edt_map[id].second;
      for (int k = 0; k < height; ++k) {
        id = indexToId(IndexXY{j, k});
        double dist = resolution_m * std::abs(static_cast<double>(i - k));
        double value = edt_map[id].first * edt_map[id].first + dist * dist;
        if (value < min_value) {
          min_value = value;
          rel_pos.x = edt_map[id].second.x;
          rel_pos.y = dist;
        }
      }
      temporary_storage[i].first = sqrt(min_value);
      temporary_storage[i].second = rel_pos;
    }
    for (int i = 0; i < height; ++i) {
      edt_map[indexToId(IndexXY{j, i})] = temporary_storage[i];
    }
  }
  for (const auto & edt : edt_map) {
    const double angle = std::atan2(edt.second.y, edt.second.x);
    edt_map_.push_back({edt.first, angle});
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

bool AbstractPlanningAlgorithm::detectBoundaryExit(const IndexXYT & base_index) const
{
  if (isWithinMargin(base_index)) return false;
  const auto & vertex_indexes_2d = vertex_indexes_table_[base_index.theta];
  for (const auto & vertex_index_2d : vertex_indexes_2d) {
    IndexXY vertex_index{vertex_index_2d.x, vertex_index_2d.y};
    // must slide to current base position
    vertex_index.x += base_index.x;
    vertex_index.y += base_index.y;
    if (isOutOfRange(vertex_index)) {
      return true;
    }
  }
  return false;
}

bool AbstractPlanningAlgorithm::detectCollision(const geometry_msgs::msg::Pose & base_pose) const
{
  const auto base_index = pose2index(costmap_, base_pose, planner_common_param_.theta_size);
  return detectCollision(base_index);
}

bool AbstractPlanningAlgorithm::detectCollision(const IndexXYT & base_index) const
{
  if (coll_indexes_table_.empty()) {
    std::cerr << "[abstract_algorithm] setMap has not yet been done." << std::endl;
    return false;
  }

  if (detectBoundaryExit(base_index)) return true;

  double obstacle_edt = getObstacleEDT(base_index).distance;

  // if nearest obstacle is further than largest dimension, no collision is guaranteed
  // if nearest obstacle is closer than smallest dimension, collision is guaranteed
  if (obstacle_edt > collision_vehicle_shape_.max_dimension) return false;
  if (obstacle_edt < collision_vehicle_shape_.min_dimension) return true;

  const auto & coll_indexes_2d = coll_indexes_table_[base_index.theta];
  for (const auto & coll_index_2d : coll_indexes_2d) {
    IndexXY coll_index{coll_index_2d.x, coll_index_2d.y};
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
    if (detectCollision(pose_local)) {
      return true;
    }
  }

  return false;
}

}  // namespace autoware::freespace_planning_algorithms
