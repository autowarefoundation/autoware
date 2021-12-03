// Copyright 2018 Autoware Foundation. All rights reserved.
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

#ifndef SHAPE_ESTIMATION__SHAPE_ESTIMATOR_HPP_
#define SHAPE_ESTIMATION__SHAPE_ESTIMATOR_HPP_

#include <autoware_auto_perception_msgs/msg/object_classification.hpp>
#include <autoware_auto_perception_msgs/msg/shape.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <boost/optional.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <string>

class ShapeEstimator
{
private:
  bool estimateShape(
    const uint8_t label, const pcl::PointCloud<pcl::PointXYZ> & cluster,
    const boost::optional<float> & yaw, autoware_auto_perception_msgs::msg::Shape & shape_output,
    geometry_msgs::msg::Pose & pose_output);
  bool applyFilter(
    const uint8_t label, const autoware_auto_perception_msgs::msg::Shape & shape_output,
    const geometry_msgs::msg::Pose & pose_output);
  bool applyCorrector(
    const uint8_t label, const bool use_reference_yaw,
    autoware_auto_perception_msgs::msg::Shape & shape_output,
    geometry_msgs::msg::Pose & pose_output);

  bool use_corrector_;
  bool use_filter_;

public:
  ShapeEstimator(bool use_corrector, bool use_filter);

  virtual ~ShapeEstimator() = default;

  virtual bool estimateShapeAndPose(
    const uint8_t label, const pcl::PointCloud<pcl::PointXYZ> & cluster,
    const boost::optional<float> & yaw, autoware_auto_perception_msgs::msg::Shape & shape_output,
    geometry_msgs::msg::Pose & pose_output);
};

#endif  // SHAPE_ESTIMATION__SHAPE_ESTIMATOR_HPP_
