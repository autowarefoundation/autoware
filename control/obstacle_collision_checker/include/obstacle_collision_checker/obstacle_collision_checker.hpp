// Copyright 2020 Tier IV, Inc. All rights reserved.
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

#ifndef OBSTACLE_COLLISION_CHECKER__OBSTACLE_COLLISION_CHECKER_HPP_
#define OBSTACLE_COLLISION_CHECKER__OBSTACLE_COLLISION_CHECKER_HPP_

#include <tier4_autoware_utils/geometry/boost_geometry.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>

#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <boost/optional.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <map>
#include <string>
#include <vector>

namespace obstacle_collision_checker
{
using tier4_autoware_utils::LinearRing2d;

struct Param
{
  double delay_time;
  double footprint_margin;
  double max_deceleration;
  double resample_interval;
  double search_radius;
};

struct Input
{
  geometry_msgs::msg::PoseStamped::ConstSharedPtr current_pose;
  geometry_msgs::msg::Twist::ConstSharedPtr current_twist;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr obstacle_pointcloud;
  geometry_msgs::msg::TransformStamped::ConstSharedPtr obstacle_transform;
  autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr reference_trajectory;
  autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr predicted_trajectory;
};

struct Output
{
  std::map<std::string, double> processing_time_map;
  bool will_collide;
  autoware_auto_planning_msgs::msg::Trajectory resampled_trajectory;
  std::vector<LinearRing2d> vehicle_footprints;
  std::vector<LinearRing2d> vehicle_passing_areas;
};

class ObstacleCollisionChecker
{
public:
  explicit ObstacleCollisionChecker(rclcpp::Node & node);
  Output update(const Input & input);

  void setParam(const Param & param) { param_ = param; }

private:
  Param param_;
  vehicle_info_util::VehicleInfo vehicle_info_;

  //! This function assumes the input trajectory is sampled dense enough
  static autoware_auto_planning_msgs::msg::Trajectory resampleTrajectory(
    const autoware_auto_planning_msgs::msg::Trajectory & trajectory, const double interval);

  static autoware_auto_planning_msgs::msg::Trajectory cutTrajectory(
    const autoware_auto_planning_msgs::msg::Trajectory & trajectory, const double length);

  static std::vector<LinearRing2d> createVehicleFootprints(
    const autoware_auto_planning_msgs::msg::Trajectory & trajectory, const Param & param,
    const vehicle_info_util::VehicleInfo & vehicle_info);

  static std::vector<LinearRing2d> createVehiclePassingAreas(
    const std::vector<LinearRing2d> & vehicle_footprints);

  static LinearRing2d createHullFromFootprints(
    const LinearRing2d & area1, const LinearRing2d & area2);

  static bool willCollide(
    const pcl::PointCloud<pcl::PointXYZ> & obstacle_pointcloud,
    const std::vector<LinearRing2d> & vehicle_footprints);

  static bool hasCollision(
    const pcl::PointCloud<pcl::PointXYZ> & obstacle_pointcloud,
    const LinearRing2d & vehicle_footprint);
};
}  // namespace obstacle_collision_checker

#endif  // OBSTACLE_COLLISION_CHECKER__OBSTACLE_COLLISION_CHECKER_HPP_
