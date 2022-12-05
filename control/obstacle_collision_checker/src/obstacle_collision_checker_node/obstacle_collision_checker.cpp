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

#include "obstacle_collision_checker/obstacle_collision_checker.hpp"

#include <pcl_ros/transforms.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/math/normalization.hpp>
#include <tier4_autoware_utils/math/unit_conversion.hpp>
#include <tier4_autoware_utils/system/stop_watch.hpp>

#include <boost/geometry.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <tf2/utils.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#endif

#include <iostream>
#include <vector>

namespace
{
pcl::PointCloud<pcl::PointXYZ> getTransformedPointCloud(
  const sensor_msgs::msg::PointCloud2 & pointcloud_msg,
  const geometry_msgs::msg::Transform & transform)
{
  const Eigen::Matrix4f transform_matrix = tf2::transformToEigen(transform).matrix().cast<float>();

  sensor_msgs::msg::PointCloud2 transformed_msg;
  pcl_ros::transformPointCloud(transform_matrix, pointcloud_msg, transformed_msg);

  pcl::PointCloud<pcl::PointXYZ> transformed_pointcloud;
  pcl::fromROSMsg(transformed_msg, transformed_pointcloud);

  return transformed_pointcloud;
}

pcl::PointCloud<pcl::PointXYZ> filterPointCloudByTrajectory(
  const pcl::PointCloud<pcl::PointXYZ> & pointcloud,
  const autoware_auto_planning_msgs::msg::Trajectory & trajectory, const double radius)
{
  pcl::PointCloud<pcl::PointXYZ> filtered_pointcloud;
  for (const auto & point : pointcloud.points) {
    for (const auto & trajectory_point : trajectory.points) {
      const double dx = trajectory_point.pose.position.x - point.x;
      const double dy = trajectory_point.pose.position.y - point.y;
      if (std::hypot(dx, dy) < radius) {
        filtered_pointcloud.points.push_back(point);
        break;
      }
    }
  }
  return filtered_pointcloud;
}

double calcBrakingDistance(
  const double abs_velocity, const double max_deceleration, const double delay_time)
{
  const double idling_distance = abs_velocity * delay_time;
  const double braking_distance = (abs_velocity * abs_velocity) / (2.0 * max_deceleration);
  return idling_distance + braking_distance;
}

}  // namespace

namespace obstacle_collision_checker
{
ObstacleCollisionChecker::ObstacleCollisionChecker(rclcpp::Node & node)
: vehicle_info_(vehicle_info_util::VehicleInfoUtil(node).getVehicleInfo())
{
}

Output ObstacleCollisionChecker::update(const Input & input)
{
  Output output;
  tier4_autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch;

  // resample trajectory by braking distance
  constexpr double min_velocity = 0.01;
  const auto & raw_abs_velocity = std::abs(input.current_twist->linear.x);
  const auto abs_velocity = raw_abs_velocity < min_velocity ? 0.0 : raw_abs_velocity;
  const auto braking_distance =
    calcBrakingDistance(abs_velocity, param_.max_deceleration, param_.delay_time);
  output.resampled_trajectory = cutTrajectory(
    resampleTrajectory(*input.predicted_trajectory, param_.resample_interval), braking_distance);
  output.processing_time_map["resampleTrajectory"] = stop_watch.toc(true);

  // resample pointcloud
  const auto obstacle_pointcloud =
    getTransformedPointCloud(*input.obstacle_pointcloud, input.obstacle_transform->transform);
  const auto filtered_obstacle_pointcloud = filterPointCloudByTrajectory(
    obstacle_pointcloud, output.resampled_trajectory, param_.search_radius);

  output.vehicle_footprints =
    createVehicleFootprints(output.resampled_trajectory, param_, vehicle_info_);
  output.processing_time_map["createVehicleFootprints"] = stop_watch.toc(true);

  output.vehicle_passing_areas = createVehiclePassingAreas(output.vehicle_footprints);
  output.processing_time_map["createVehiclePassingAreas"] = stop_watch.toc(true);

  output.will_collide = willCollide(filtered_obstacle_pointcloud, output.vehicle_passing_areas);
  output.processing_time_map["willCollide"] = stop_watch.toc(true);

  return output;
}

autoware_auto_planning_msgs::msg::Trajectory ObstacleCollisionChecker::resampleTrajectory(
  const autoware_auto_planning_msgs::msg::Trajectory & trajectory, const double interval)
{
  autoware_auto_planning_msgs::msg::Trajectory resampled;
  resampled.header = trajectory.header;

  resampled.points.push_back(trajectory.points.front());
  for (size_t i = 1; i < trajectory.points.size() - 1; ++i) {
    const auto & point = trajectory.points.at(i);

    const auto p1 = tier4_autoware_utils::fromMsg(resampled.points.back().pose.position).to_2d();
    const auto p2 = tier4_autoware_utils::fromMsg(point.pose.position).to_2d();

    if (boost::geometry::distance(p1, p2) > interval) {
      resampled.points.push_back(point);
    }
  }
  resampled.points.push_back(trajectory.points.back());

  return resampled;
}

autoware_auto_planning_msgs::msg::Trajectory ObstacleCollisionChecker::cutTrajectory(
  const autoware_auto_planning_msgs::msg::Trajectory & trajectory, const double length)
{
  autoware_auto_planning_msgs::msg::Trajectory cut;
  cut.header = trajectory.header;

  double total_length = 0.0;
  cut.points.push_back(trajectory.points.front());
  for (size_t i = 1; i < trajectory.points.size(); ++i) {
    const auto & point = trajectory.points.at(i);

    const auto p1 = tier4_autoware_utils::fromMsg(cut.points.back().pose.position);
    const auto p2 = tier4_autoware_utils::fromMsg(point.pose.position);
    const auto points_distance = boost::geometry::distance(p1.to_2d(), p2.to_2d());

    const auto remain_distance = length - total_length;

    // Over length
    if (remain_distance <= 0.0) {
      break;
    }

    // Require interpolation
    if (remain_distance <= points_distance) {
      const Eigen::Vector3d p_interpolated = p1 + remain_distance * (p2 - p1).normalized();

      autoware_auto_planning_msgs::msg::TrajectoryPoint p;
      p.pose.position.x = p_interpolated.x();
      p.pose.position.y = p_interpolated.y();
      p.pose.position.z = p_interpolated.z();
      p.pose.orientation = point.pose.orientation;

      cut.points.push_back(p);
      break;
    }

    cut.points.push_back(point);
    total_length += points_distance;
  }

  return cut;
}

std::vector<LinearRing2d> ObstacleCollisionChecker::createVehicleFootprints(
  const autoware_auto_planning_msgs::msg::Trajectory & trajectory, const Param & param,
  const vehicle_info_util::VehicleInfo & vehicle_info)
{
  // Create vehicle footprint in base_link coordinate
  const auto local_vehicle_footprint = vehicle_info.createFootprint(param.footprint_margin);

  // Create vehicle footprint on each TrajectoryPoint
  std::vector<LinearRing2d> vehicle_footprints;
  for (const auto & p : trajectory.points) {
    vehicle_footprints.push_back(
      tier4_autoware_utils::transformVector<tier4_autoware_utils::LinearRing2d>(
        local_vehicle_footprint, tier4_autoware_utils::pose2transform(p.pose)));
  }

  return vehicle_footprints;
}

std::vector<LinearRing2d> ObstacleCollisionChecker::createVehiclePassingAreas(
  const std::vector<LinearRing2d> & vehicle_footprints)
{
  // Create hull from two adjacent vehicle footprints
  std::vector<LinearRing2d> areas;
  for (size_t i = 0; i < vehicle_footprints.size() - 1; ++i) {
    const auto & footprint1 = vehicle_footprints.at(i);
    const auto & footprint2 = vehicle_footprints.at(i + 1);
    areas.push_back(createHullFromFootprints(footprint1, footprint2));
  }

  return areas;
}

LinearRing2d ObstacleCollisionChecker::createHullFromFootprints(
  const LinearRing2d & area1, const LinearRing2d & area2)
{
  tier4_autoware_utils::MultiPoint2d combined;
  for (const auto & p : area1) {
    combined.push_back(p);
  }
  for (const auto & p : area2) {
    combined.push_back(p);
  }
  LinearRing2d hull;
  boost::geometry::convex_hull(combined, hull);
  return hull;
}

bool ObstacleCollisionChecker::willCollide(
  const pcl::PointCloud<pcl::PointXYZ> & obstacle_pointcloud,
  const std::vector<LinearRing2d> & vehicle_footprints)
{
  for (size_t i = 1; i < vehicle_footprints.size(); i++) {
    // skip first footprint because surround obstacle checker handle it
    const auto & vehicle_footprint = vehicle_footprints.at(i);
    if (hasCollision(obstacle_pointcloud, vehicle_footprint)) {
      RCLCPP_WARN(
        rclcpp::get_logger("obstacle_collision_checker"), "ObstacleCollisionChecker::willCollide");
      return true;
    }
  }

  return false;
}

bool ObstacleCollisionChecker::hasCollision(
  const pcl::PointCloud<pcl::PointXYZ> & obstacle_pointcloud,
  const LinearRing2d & vehicle_footprint)
{
  for (const auto & point : obstacle_pointcloud.points) {
    if (boost::geometry::within(
          tier4_autoware_utils::Point2d{point.x, point.y}, vehicle_footprint)) {
      RCLCPP_WARN(
        rclcpp::get_logger("obstacle_collision_checker"),
        "[ObstacleCollisionChecker] Collide to Point x: %f y: %f", point.x, point.y);
      return true;
    }
  }

  return false;
}
}  // namespace obstacle_collision_checker
