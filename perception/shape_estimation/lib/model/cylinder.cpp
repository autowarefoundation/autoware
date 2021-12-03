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

#include "shape_estimation/model/cylinder.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <autoware_auto_perception_msgs/msg/shape.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>

bool CylinderShapeModel::estimate(
  const pcl::PointCloud<pcl::PointXYZ> & cluster,
  autoware_auto_perception_msgs::msg::Shape & shape_output, geometry_msgs::msg::Pose & pose_output)
{
  // calc min and max z for cylinder length
  float min_z = cluster.empty() ? 0.0 : cluster.at(0).z;
  float max_z = cluster.empty() ? 0.0 : cluster.at(0).z;
  for (const auto & point : cluster) {
    min_z = std::min(point.z, min_z);
    max_z = std::max(point.z, max_z);
  }

  // calc circumscribed circle on x-y plane
  cv::Mat_<float> cv_points(static_cast<int>(cluster.size()), 2);
  for (size_t i = 0; i < cluster.size(); ++i) {
    cv_points(i, 0) = cluster.at(i).x;  // x
    cv_points(i, 1) = cluster.at(i).y;  // y
  }
  cv::Point2f center;
  float radius;
  cv::minEnclosingCircle(cv::Mat(cv_points).reshape(2), center, radius);
  constexpr float ep = 0.001;
  radius = std::max(radius, static_cast<float>(ep));

  shape_output.type = autoware_auto_perception_msgs::msg::Shape::CYLINDER;
  shape_output.dimensions.x = static_cast<float>(radius) * 2.0;
  shape_output.dimensions.y = static_cast<float>(radius) * 2.0;
  shape_output.dimensions.z = std::max((max_z - min_z), ep);
  pose_output.position.x = center.x;
  pose_output.position.y = center.y;
  pose_output.position.z = min_z + shape_output.dimensions.z * 0.5;
  pose_output.orientation.x = 0;
  pose_output.orientation.y = 0;
  pose_output.orientation.z = 0;
  pose_output.orientation.w = 1;
  return true;
}
