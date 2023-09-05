// Copyright 2022 TIER IV, Inc.
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

#ifndef FRONT_VEHICLE_VELOCITY_ESTIMATOR__FRONT_VEHICLE_VELOCITY_ESTIMATOR_HPP_
#define FRONT_VEHICLE_VELOCITY_ESTIMATOR__FRONT_VEHICLE_VELOCITY_ESTIMATOR_HPP_

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/logger.hpp"
#include "tier4_autoware_utils/geometry/boost_geometry.hpp"

#include "autoware_auto_perception_msgs/msg/detected_objects.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <deque>
#include <memory>
#include <string>

namespace front_vehicle_velocity_estimator
{
using autoware_auto_perception_msgs::msg::DetectedObject;
using autoware_auto_perception_msgs::msg::DetectedObjectKinematics;
using autoware_auto_perception_msgs::msg::DetectedObjects;
using autoware_auto_perception_msgs::msg::ObjectClassification;
using nav_msgs::msg::Odometry;
using sensor_msgs::msg::PointCloud2;
using tier4_autoware_utils::LinearRing2d;
using tier4_autoware_utils::Point2d;

class FrontVehicleVelocityEstimator
{
public:
  explicit FrontVehicleVelocityEstimator(const rclcpp::Logger & logger) : logger_(logger) {}

  struct Input
  {
    PointCloud2::ConstSharedPtr pointcloud{};
    DetectedObjects::ConstSharedPtr objects{};
    Odometry::ConstSharedPtr odometry{};
  };

  struct Output
  {
    DetectedObjects objects{};
    PointCloud2 nearest_neighbor_pointcloud{};
  };

  struct Param
  {
    int moving_average_num{};
    float threshold_pointcloud_z_high{};
    float threshold_pointcloud_z_low{};
    double threshold_relative_velocity{};
    double threshold_absolute_velocity{};
  };

  void setParam(const Param & param) { param_ = param; }
  Output update(const Input & input);

private:
  struct ObjectsWithFrontVehicle
  {
    DetectedObjects::SharedPtr objects_without_front_vehicle{};
    DetectedObject front_vehicle{};
    bool is_front_vehicle = false;
  };

  rclcpp::Logger logger_;

  // Buffer data
  Param param_{};
  std::deque<double> velocity_queue_{};
  rclcpp::Time prev_time_{};
  pcl::PointXYZ prev_point_{};

  // Function
  LinearRing2d createBoxArea(const Point2d size);
  LinearRing2d createObjectArea(const DetectedObject & object);
  ObjectsWithFrontVehicle filterFrontVehicle(
    DetectedObjects::ConstSharedPtr objects, const LinearRing2d & front_area);
  pcl::PointXYZ getNearestNeighborPoint(
    const DetectedObject & object, PointCloud2::ConstSharedPtr pointcloud,
    const Point2d & front_size);
  double estimateRelativeVelocity(const pcl::PointXYZ & point, const rclcpp::Time & header_time);
  double estimateAbsoluteVelocity(
    const double relative_velocity, Odometry::ConstSharedPtr odometry);
  bool isFrontVehicle(const DetectedObject & object, const LinearRing2d & front_area);
  bool isWithinVehicle(
    const DetectedObject & object, const pcl::PointXYZ & point, const Point2d & front_size);
};

}  // namespace front_vehicle_velocity_estimator

#endif  // FRONT_VEHICLE_VELOCITY_ESTIMATOR__FRONT_VEHICLE_VELOCITY_ESTIMATOR_HPP_
