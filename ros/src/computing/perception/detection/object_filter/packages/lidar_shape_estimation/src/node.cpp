/*
 * Copyright 2018 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 *
 * v1.0 Yukihiro Saito
 */

#include "lidar_shape_estimation/node.hpp"
#include "lidar_shape_estimation/shape_estimator.hpp"

ShapeEstimationNode::ShapeEstimationNode() : nh_(""), pnh_("~")
{
  sub_ = nh_.subscribe("input", 1, &ShapeEstimationNode::callback, this);
  pub_ = nh_.advertise<autoware_msgs::DetectedObjectArray>("objects", 1, true);
}

void ShapeEstimationNode::callback(const autoware_msgs::DetectedObjectArray::ConstPtr& input_msg)
{
  // Guard
  if (pub_.getNumSubscribers() < 1)
    return;

  // Create output msg
  auto output_msg = *input_msg;

  // Estimate shape for each object and pack msg
  for (auto& object : output_msg.objects)
  {
    // convert ros to pcl
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(object.pointcloud, *cluster);
    // estimate shape and pose
    estimator_.getShapeAndPose(object.label, *cluster, object);
  }

  // Publish
  pub_.publish(output_msg);
  return;
}
