// Copyright 2020 TIER IV, Inc.
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

#include "image_projection_based_fusion/utils/geometry.hpp"

#include <rclcpp/rclcpp.hpp>

namespace image_projection_based_fusion
{

double calcIoU(
  const sensor_msgs::msg::RegionOfInterest & roi_1,
  const sensor_msgs::msg::RegionOfInterest & roi_2)
{
  double s_1, s_2;
  s_1 = static_cast<double>(roi_1.width * static_cast<double>(roi_1.height));
  s_2 = static_cast<double>(roi_2.width * static_cast<double>(roi_2.height));

  double overlap_s;
  double overlap_max_x, overlap_max_y, overlap_min_x, overlap_min_y;
  overlap_min_x = roi_1.x_offset < roi_2.x_offset ? roi_2.x_offset : roi_1.x_offset;
  overlap_min_y = roi_1.y_offset < roi_2.y_offset ? roi_2.y_offset : roi_1.y_offset;
  overlap_max_x = roi_1.x_offset + roi_1.width < roi_2.x_offset + roi_2.width
                    ? roi_1.x_offset + roi_1.width
                    : roi_2.x_offset + roi_2.width;
  overlap_max_y = roi_1.y_offset + roi_1.height < roi_2.y_offset + roi_2.height
                    ? roi_1.y_offset + roi_1.height
                    : roi_2.y_offset + roi_2.height;
  overlap_s = (overlap_max_x - overlap_min_x) * (overlap_max_y - overlap_min_y);
  if (overlap_max_x < overlap_min_x || overlap_max_y < overlap_min_y) {
    return 0.0;
  }
  return overlap_s / (s_1 + s_2 - overlap_s);
}
double calcIoUX(
  const sensor_msgs::msg::RegionOfInterest & roi_1,
  const sensor_msgs::msg::RegionOfInterest & roi_2)
{
  double s_1, s_2;
  s_1 = static_cast<double>(roi_1.width);
  s_2 = static_cast<double>(roi_2.width);
  double overlap_s;
  double overlap_max_x, overlap_max_y, overlap_min_x, overlap_min_y;
  overlap_min_x = roi_1.x_offset < roi_2.x_offset ? roi_2.x_offset : roi_1.x_offset;
  overlap_min_y = roi_1.y_offset < roi_2.y_offset ? roi_2.y_offset : roi_1.y_offset;
  overlap_max_x = roi_1.x_offset + roi_1.width < roi_2.x_offset + roi_2.width
                    ? roi_1.x_offset + roi_1.width
                    : roi_2.x_offset + roi_2.width;
  overlap_max_y = roi_1.y_offset + roi_1.height < roi_2.y_offset + roi_2.height
                    ? roi_1.y_offset + roi_1.height
                    : roi_2.y_offset + roi_2.height;
  overlap_s = (overlap_max_x - overlap_min_x);
  if (overlap_max_x < overlap_min_x || overlap_max_y < overlap_min_y) {
    return 0.0;
  }
  return overlap_s / (s_1 + s_2 - overlap_s);
}
double calcIoUY(
  const sensor_msgs::msg::RegionOfInterest & roi_1,
  const sensor_msgs::msg::RegionOfInterest & roi_2)
{
  double s_1, s_2;
  s_1 = static_cast<double>(roi_1.height);
  s_2 = static_cast<double>(roi_2.height);
  double overlap_s;
  double overlap_max_x, overlap_max_y, overlap_min_x, overlap_min_y;
  overlap_min_x = roi_1.x_offset < roi_2.x_offset ? roi_2.x_offset : roi_1.x_offset;
  overlap_min_y = roi_1.y_offset < roi_2.y_offset ? roi_2.y_offset : roi_1.y_offset;
  overlap_max_x = roi_1.x_offset + roi_1.width < roi_2.x_offset + roi_2.width
                    ? roi_1.x_offset + roi_1.width
                    : roi_2.x_offset + roi_2.width;
  overlap_max_y = roi_1.y_offset + roi_1.height < roi_2.y_offset + roi_2.height
                    ? roi_1.y_offset + roi_1.height
                    : roi_2.y_offset + roi_2.height;
  overlap_s = (overlap_max_y - overlap_min_y);
  if (overlap_max_x < overlap_min_x || overlap_max_y < overlap_min_y) {
    return 0.0;
  }
  return overlap_s / (s_1 + s_2 - overlap_s);
}

void objectToVertices(
  const Pose & pose, const Shape & shape, std::vector<Eigen::Vector3d> & vertices)
{
  if (shape.type == Shape::BOUNDING_BOX) {
    boundingBoxToVertices(pose, shape, vertices);
  } else if (shape.type == Shape::CYLINDER) {
    cylinderToVertices(pose, shape, vertices);
  } else if (shape.type == Shape::POLYGON) {
    // polygonToVertices(pose, shape, vertices);
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("image_projection_based_fusion"), "POLYGON is not supported");
  }
}

void boundingBoxToVertices(
  const Pose & pose, const Shape & shape, std::vector<Eigen::Vector3d> & vertices)
{
  const std::vector<std::vector<double>> corners_template = {
    // down surface
    {1, 1, -1},
    {1, -1, -1},
    {-1, -1, -1},
    {-1, 1, -1},
    // up surface
    {1, 1, 1},
    {1, -1, 1},
    {-1, -1, 1},
    {-1, 1, 1},
  };

  const auto position = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  const auto orientation = Eigen::Quaterniond(
    pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);

  for (const auto & corner : corners_template) {
    Eigen::Vector3d corner_point(
      shape.dimensions.x * corner.at(0) / 2.0, shape.dimensions.y * corner.at(1) / 2.0,
      shape.dimensions.z * corner.at(2) / 2.0);
    vertices.push_back(orientation * corner_point + position);
  }
}

void cylinderToVertices(
  const Pose & pose, const Shape & shape, std::vector<Eigen::Vector3d> & vertices)
{
  const auto & center = pose.position;
  const auto & radius = shape.dimensions.x * 0.5;
  constexpr std::size_t n = 6;
  vertices.reserve(n * 2);
  for (std::size_t i = 0; i < n; ++i) {
    Eigen::Vector3d vertex;
    const double theta = (static_cast<double>(i) / static_cast<double>(n)) * 2.0 * M_PI +
                         M_PI / static_cast<double>(n);
    vertex.x() = std::cos(theta) * radius + center.x;
    vertex.y() = std::sin(theta) * radius + center.y;
    vertex.z() = shape.dimensions.z * 0.5 + center.z;
    vertices.push_back(vertex);
    vertex.z() = -shape.dimensions.z * 0.5 + center.z;
    vertices.push_back(vertex);
  }
}

void transformPoints(
  const std::vector<Eigen::Vector3d> & input_points, const Eigen::Affine3d & affine_transform,
  std::vector<Eigen::Vector3d> & output_points)
{
  output_points.reserve(input_points.size());
  for (const auto & point : input_points) {
    output_points.push_back(affine_transform * point);
  }
}

}  // namespace image_projection_based_fusion
