// Copyright 2022 Tier IV, Inc.
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

// NOLINTNEXTLINE(whitespace/line_length)
#ifndef DETECTED_OBJECT_VALIDATION__OBSTACLE_POINTCLOUD_BASED_VALIDATOR__OBSTACLE_POINTCLOUD_BASED_VALIDATOR_HPP_
// NOLINTNEXTLINE(whitespace/line_length)
#define DETECTED_OBJECT_VALIDATION__OBSTACLE_POINTCLOUD_BASED_VALIDATOR__OBSTACLE_POINTCLOUD_BASED_VALIDATOR_HPP_

#include "detected_object_validation/obstacle_pointcloud_based_validator/debugger.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/pcl_search.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <optional>
#include <vector>
namespace obstacle_pointcloud_based_validator
{

struct PointsNumThresholdParam
{
  std::vector<int64_t> min_points_num;
  std::vector<int64_t> max_points_num;
  std::vector<double> min_points_and_distance_ratio;
};

class Validator
{
private:
  PointsNumThresholdParam points_num_threshold_param_;

protected:
  pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_pointcloud_;

public:
  explicit Validator(PointsNumThresholdParam & points_num_threshold_param);
  inline pcl::PointCloud<pcl::PointXYZ>::Ptr getDebugPointCloudWithinObject()
  {
    return cropped_pointcloud_;
  }

  virtual bool setKdtreeInputCloud(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & input_pointcloud) = 0;
  virtual bool validate_object(
    const autoware_auto_perception_msgs::msg::DetectedObject & transformed_object) = 0;
  virtual std::optional<float> getMaxRadius(
    const autoware_auto_perception_msgs::msg::DetectedObject & object) = 0;
  size_t getThresholdPointCloud(const autoware_auto_perception_msgs::msg::DetectedObject & object);
  virtual pcl::PointCloud<pcl::PointXYZ>::Ptr getDebugNeighborPointCloud() = 0;
};

class Validator2D : public Validator
{
private:
  pcl::PointCloud<pcl::PointXY>::Ptr obstacle_pointcloud_;
  pcl::PointCloud<pcl::PointXY>::Ptr neighbor_pointcloud_;
  pcl::search::Search<pcl::PointXY>::Ptr kdtree_;

public:
  explicit Validator2D(PointsNumThresholdParam & points_num_threshold_param);

  pcl::PointCloud<pcl::PointXYZ>::Ptr convertToXYZ(
    const pcl::PointCloud<pcl::PointXY>::Ptr & pointcloud_xy);
  inline pcl::PointCloud<pcl::PointXYZ>::Ptr getDebugNeighborPointCloud()
  {
    return convertToXYZ(neighbor_pointcloud_);
  }

  bool setKdtreeInputCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & input_cloud);
  bool validate_object(
    const autoware_auto_perception_msgs::msg::DetectedObject & transformed_object);
  std::optional<float> getMaxRadius(
    const autoware_auto_perception_msgs::msg::DetectedObject & object);
  std::optional<size_t> getPointCloudWithinObject(
    const autoware_auto_perception_msgs::msg::DetectedObject & object,
    const pcl::PointCloud<pcl::PointXY>::Ptr neighbor_pointcloud);
};
class Validator3D : public Validator
{
private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_pointcloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr neighbor_pointcloud_;
  pcl::search::Search<pcl::PointXYZ>::Ptr kdtree_;

public:
  explicit Validator3D(PointsNumThresholdParam & points_num_threshold_param);
  inline pcl::PointCloud<pcl::PointXYZ>::Ptr getDebugNeighborPointCloud()
  {
    return neighbor_pointcloud_;
  }
  bool setKdtreeInputCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & input_cloud);
  bool validate_object(
    const autoware_auto_perception_msgs::msg::DetectedObject & transformed_object);
  std::optional<float> getMaxRadius(
    const autoware_auto_perception_msgs::msg::DetectedObject & object);
  std::optional<size_t> getPointCloudWithinObject(
    const autoware_auto_perception_msgs::msg::DetectedObject & object,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr neighbor_pointcloud);
};

class ObstaclePointCloudBasedValidator : public rclcpp::Node
{
public:
  explicit ObstaclePointCloudBasedValidator(const rclcpp::NodeOptions & node_options);

private:
  rclcpp::Publisher<autoware_auto_perception_msgs::msg::DetectedObjects>::SharedPtr objects_pub_;
  message_filters::Subscriber<autoware_auto_perception_msgs::msg::DetectedObjects> objects_sub_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> obstacle_pointcloud_sub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  typedef message_filters::sync_policies::ApproximateTime<
    autoware_auto_perception_msgs::msg::DetectedObjects, sensor_msgs::msg::PointCloud2>
    SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Sync;
  Sync sync_;
  PointsNumThresholdParam points_num_threshold_param_;

  std::shared_ptr<Debugger> debugger_;
  bool using_2d_validator_;
  std::unique_ptr<Validator> validator_;

private:
  void onObjectsAndObstaclePointCloud(
    const autoware_auto_perception_msgs::msg::DetectedObjects::ConstSharedPtr & input_objects,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & input_obstacle_pointcloud);
};
}  // namespace obstacle_pointcloud_based_validator

// NOLINTNEXTLINE(whitespace/line_length)
#endif  // DETECTED_OBJECT_VALIDATION__OBSTACLE_POINTCLOUD_BASED_VALIDATOR__OBSTACLE_POINTCLOUD_BASED_VALIDATOR_HPP_
