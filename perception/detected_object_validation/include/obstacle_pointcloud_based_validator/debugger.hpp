// Copyright 2021 Tier IV, Inc.
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

#ifndef OBSTACLE_POINTCLOUD_BASED_VALIDATOR__DEBUGGER_HPP_
#define OBSTACLE_POINTCLOUD_BASED_VALIDATOR__DEBUGGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <memory>
#include <vector>

namespace obstacle_pointcloud_based_validator
{
class Debugger
{
public:
  explicit Debugger(rclcpp::Node * node)
  : neighbor_pointcloud_(new pcl::PointCloud<pcl::PointXYZ>),
    pointcloud_within_polygon_(new pcl::PointCloud<pcl::PointXYZ>)
  {
    removed_objects_pub_ =
      node->create_publisher<autoware_auto_perception_msgs::msg::DetectedObjects>(
        "~/debug/removed_objects", 1);
    neighbor_pointcloud_pub_ =
      node->create_publisher<sensor_msgs::msg::PointCloud2>("~/debug/neighbor_pointcloud", 1);
    pointcloud_within_polygon_pub_ =
      node->create_publisher<sensor_msgs::msg::PointCloud2>("~/debug/pointcloud_within_polygon", 1);
  }

  ~Debugger() {}
  void publishRemovedObjects(const autoware_auto_perception_msgs::msg::DetectedObjects & input)
  {
    removed_objects_pub_->publish(input);
  }
  void publishNeighborPointcloud(const std_msgs::msg::Header & header)
  {
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*neighbor_pointcloud_, output);
    output.header = header;
    neighbor_pointcloud_pub_->publish(output);
    neighbor_pointcloud_->clear();
  }

  void publishPointcloudWithinPolygon(const std_msgs::msg::Header & header)
  {
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*pointcloud_within_polygon_, output);
    output.header = header;
    pointcloud_within_polygon_pub_->publish(output);
    pointcloud_within_polygon_->clear();
  }

  void addNeighborPointcloud(const pcl::PointCloud<pcl::PointXY>::Ptr & input)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_xyz = toXYZ(input);
    for (const auto & point : *input_xyz) {
      neighbor_pointcloud_->push_back(point);
    }
  }

  void addPointcloudWithinPolygon(const pcl::PointCloud<pcl::PointXYZ>::Ptr & input)
  {
    // pcl::PointCloud<pcl::PointXYZ>::Ptr input_xyz = toXYZ(input);
    for (const auto & point : *input) {
      pointcloud_within_polygon_->push_back(point);
    }
  }

private:
  rclcpp::Publisher<autoware_auto_perception_msgs::msg::DetectedObjects>::SharedPtr
    removed_objects_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr neighbor_pointcloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_within_polygon_pub_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr neighbor_pointcloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_within_polygon_;

private:
  inline pcl::PointCloud<pcl::PointXYZ>::Ptr toXYZ(
    const pcl::PointCloud<pcl::PointXY>::Ptr & pointcloud)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    pointcloud_xyz->reserve(pointcloud->size());
    for (const auto & point : *pointcloud) {
      pointcloud_xyz->push_back(pcl::PointXYZ(point.x, point.y, 0.0));
    }
    return pointcloud_xyz;
  }
};
}  // namespace obstacle_pointcloud_based_validator

#endif  // OBSTACLE_POINTCLOUD_BASED_VALIDATOR__DEBUGGER_HPP_
