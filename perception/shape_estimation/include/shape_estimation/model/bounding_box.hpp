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

#ifndef SHAPE_ESTIMATION__MODEL__BOUNDING_BOX_HPP_
#define SHAPE_ESTIMATION__MODEL__BOUNDING_BOX_HPP_

#include "shape_estimation/model/model_interface.hpp"
#include "shape_estimation/shape_estimator.hpp"

#include <vector>

class BoundingBoxShapeModel : public ShapeEstimationModelInterface
{
private:
  bool fitLShape(
    const pcl::PointCloud<pcl::PointXYZ> & cluster, const float min_angle, const float max_angle,
    autoware_auto_perception_msgs::msg::Shape & shape_output,
    geometry_msgs::msg::Pose & pose_output);
  float calcClosenessCriterion(const std::vector<float> & C_1, const std::vector<float> & C_2);
  float optimize(
    const pcl::PointCloud<pcl::PointXYZ> & cluster, const float min_angle, const float max_angle);
  float boostOptimize(
    const pcl::PointCloud<pcl::PointXYZ> & cluster, const float min_angle, const float max_angle);

public:
  BoundingBoxShapeModel();
  explicit BoundingBoxShapeModel(
    const boost::optional<ReferenceYawInfo> & ref_yaw_info, bool use_boost_bbox_optimizer = false);
  boost::optional<ReferenceYawInfo> ref_yaw_info_;
  bool use_boost_bbox_optimizer_;

  ~BoundingBoxShapeModel() {}

  bool estimate(
    const pcl::PointCloud<pcl::PointXYZ> & cluster,
    autoware_auto_perception_msgs::msg::Shape & shape_output,
    geometry_msgs::msg::Pose & pose_output) override;
};

#endif  // SHAPE_ESTIMATION__MODEL__BOUNDING_BOX_HPP_
