// Copyright 2020 TierIV
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

#pragma once

#include "cluster2d.hpp"
#include "feature_generator.hpp"
#include "lidar_apollo_instance_segmentation/node.hpp"

#include <TrtNet.hpp>

#include <pcl/common/transforms.h>
#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#endif
#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>

class LidarApolloInstanceSegmentation : public LidarInstanceSegmentationInterface
{
public:
  explicit LidarApolloInstanceSegmentation(rclcpp::Node * node);
  ~LidarApolloInstanceSegmentation() {}
  bool detectDynamicObjects(
    const sensor_msgs::msg::PointCloud2 & input,
    tier4_perception_msgs::msg::DetectedObjectsWithFeature & output) override;

private:
  bool transformCloud(
    const sensor_msgs::msg::PointCloud2 & input, sensor_msgs::msg::PointCloud2 & transformed_cloud,
    float z_offset);

  rclcpp::Node * node_;
  std::unique_ptr<Tn::trtNet> net_ptr_;
  std::shared_ptr<Cluster2D> cluster2d_;
  std::shared_ptr<FeatureGenerator> feature_generator_;
  float score_threshold_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::string target_frame_;
  float z_offset_;
};
