// Copyright 2020 Tier IV, Inc.
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

#ifndef DUMMY_PERCEPTION_PUBLISHER__NODE_HPP_
#define DUMMY_PERCEPTION_PUBLISHER__NODE_HPP_

#include "dummy_perception_publisher/msg/object.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tier4_perception_msgs/msg/detected_objects_with_feature.hpp>

#include <pcl/common/distances.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <random>
#include <vector>

struct ObjectInfo
{
  ObjectInfo(
    const dummy_perception_publisher::msg::Object & object, const rclcpp::Time & current_time);
  double length;
  double width;
  double height;
  double std_dev_x;
  double std_dev_y;
  double std_dev_z;
  double std_dev_yaw;
  tf2::Transform tf_map2moved_object;
};

class PointCloudCreator
{
public:
  virtual ~PointCloudCreator() {}

  virtual std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> create_pointclouds(
    const std::vector<ObjectInfo> & obj_infos, const tf2::Transform & tf_base_link2map,
    std::mt19937 & random_generator,
    pcl::PointCloud<pcl::PointXYZ>::Ptr & merged_pointcloud) const = 0;
};

class ObjectCentricPointCloudCreator : public PointCloudCreator
{
public:
  explicit ObjectCentricPointCloudCreator(bool enable_ray_tracing)
  : enable_ray_tracing_(enable_ray_tracing)
  {
  }

  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> create_pointclouds(
    const std::vector<ObjectInfo> & obj_infos, const tf2::Transform & tf_base_link2map,
    std::mt19937 & random_generator,
    pcl::PointCloud<pcl::PointXYZ>::Ptr & merged_pointcloud) const override;

private:
  void create_object_pointcloud(
    const ObjectInfo & obj_info, const tf2::Transform & tf_base_link2map,
    std::mt19937 & random_generator, pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud) const;

  bool enable_ray_tracing_;
};

class EgoCentricPointCloudCreator : public PointCloudCreator
{
public:
  explicit EgoCentricPointCloudCreator(double visible_range) : visible_range_(visible_range) {}
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> create_pointclouds(
    const std::vector<ObjectInfo> & obj_infos, const tf2::Transform & tf_base_link2map,
    std::mt19937 & random_generator,
    pcl::PointCloud<pcl::PointXYZ>::Ptr & merged_pointcloud) const override;

private:
  double visible_range_;
};

class DummyPerceptionPublisherNode : public rclcpp::Node
{
private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::Publisher<tier4_perception_msgs::msg::DetectedObjectsWithFeature>::SharedPtr
    detected_object_with_feature_pub_;
  rclcpp::Subscription<dummy_perception_publisher::msg::Object>::SharedPtr object_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::vector<dummy_perception_publisher::msg::Object> objects_;
  double visible_range_;
  double detection_successful_rate_;
  bool enable_ray_tracing_;
  bool use_object_recognition_;
  bool use_real_param_;
  std::unique_ptr<PointCloudCreator> pointcloud_creator_;

  double angle_increment_;

  std::mt19937 random_generator_;
  void timerCallback();
  void objectCallback(const dummy_perception_publisher::msg::Object::ConstSharedPtr msg);

public:
  DummyPerceptionPublisherNode();
  ~DummyPerceptionPublisherNode() {}
};

#endif  // DUMMY_PERCEPTION_PUBLISHER__NODE_HPP_
