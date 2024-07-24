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

#ifndef AUTOWARE__IMAGE_PROJECTION_BASED_FUSION__UTILS__GEOMETRY_HPP_
#define AUTOWARE__IMAGE_PROJECTION_BASED_FUSION__UTILS__GEOMETRY_HPP_

#define EIGEN_MPL2_ONLY

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <autoware_perception_msgs/msg/shape.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/region_of_interest.hpp>

#include <vector>

namespace autoware::image_projection_based_fusion
{

using autoware_perception_msgs::msg::Shape;
using geometry_msgs::msg::Pose;

double calcIoU(
  const sensor_msgs::msg::RegionOfInterest & roi_1,
  const sensor_msgs::msg::RegionOfInterest & roi_2);

double calcIoUX(
  const sensor_msgs::msg::RegionOfInterest & roi_1,
  const sensor_msgs::msg::RegionOfInterest & roi_2);

double calcIoUY(
  const sensor_msgs::msg::RegionOfInterest & roi_1,
  const sensor_msgs::msg::RegionOfInterest & roi_2);

void objectToVertices(
  const Pose & pose, const Shape & shape, std::vector<Eigen::Vector3d> & vertices);

void boundingBoxToVertices(
  const Pose & pose, const Shape & shape, std::vector<Eigen::Vector3d> & vertices);

void cylinderToVertices(
  const Pose & pose, const Shape & shape, std::vector<Eigen::Vector3d> & vertices);

void polygonToVertices(
  const Pose & pose, const Shape & shape, std::vector<Eigen::Vector3d> & vertices);

void transformPoints(
  const std::vector<Eigen::Vector3d> & input_points, const Eigen::Affine3d & affine_transform,
  std::vector<Eigen::Vector3d> & output_points);

bool is_inside(
  const sensor_msgs::msg::RegionOfInterest & outer,
  const sensor_msgs::msg::RegionOfInterest & inner, const double outer_offset_scale = 1.1);

void sanitizeROI(sensor_msgs::msg::RegionOfInterest & roi, const int width, const int height);

}  // namespace autoware::image_projection_based_fusion

#endif  // AUTOWARE__IMAGE_PROJECTION_BASED_FUSION__UTILS__GEOMETRY_HPP_
