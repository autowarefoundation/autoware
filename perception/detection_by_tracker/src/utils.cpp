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

#include "detection_by_tracker/utils.hpp"

#include <boost/geometry.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include <algorithm>
#include <vector>

namespace utils
{
void toPolygon2d(
  const autoware_auto_perception_msgs::msg::DetectedObject & object,
  tier4_autoware_utils::Polygon2d & output);
bool isClockWise(const tier4_autoware_utils::Polygon2d & polygon);
tier4_autoware_utils::Polygon2d inverseClockWise(const tier4_autoware_utils::Polygon2d & polygon);

double getArea(const autoware_auto_perception_msgs::msg::Shape & shape)
{
  double area = 0.0;
  if (shape.type == autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX) {
    area = getRectangleArea(shape.dimensions);
  } else if (shape.type == autoware_auto_perception_msgs::msg::Shape::CYLINDER) {
    area = getCircleArea(shape.dimensions);
  } else if (shape.type == autoware_auto_perception_msgs::msg::Shape::POLYGON) {
    area = getPolygonArea(shape.footprint);
  }
  return area;
}

double getPolygonArea(const geometry_msgs::msg::Polygon & footprint)
{
  double area = 0.0;

  for (size_t i = 0; i < footprint.points.size(); ++i) {
    size_t j = (i + 1) % footprint.points.size();
    area += 0.5 * (footprint.points.at(i).x * footprint.points.at(j).y -
                   footprint.points.at(j).x * footprint.points.at(i).y);
  }

  return area;
}

double getRectangleArea(const geometry_msgs::msg::Vector3 & dimensions)
{
  return static_cast<double>(dimensions.x * dimensions.y);
}

double getCircleArea(const geometry_msgs::msg::Vector3 & dimensions)
{
  return static_cast<double>((dimensions.x / 2.0) * (dimensions.x / 2.0) * M_PI);
}

double get2dIoU(
  const autoware_auto_perception_msgs::msg::DetectedObject & object1,
  const autoware_auto_perception_msgs::msg::DetectedObject & object2)
{
  tier4_autoware_utils::Polygon2d polygon1, polygon2;
  toPolygon2d(object1, polygon1);
  toPolygon2d(object2, polygon2);

  std::vector<tier4_autoware_utils::Polygon2d> union_polygons;
  std::vector<tier4_autoware_utils::Polygon2d> intersection_polygons;
  boost::geometry::union_(polygon1, polygon2, union_polygons);
  boost::geometry::intersection(polygon1, polygon2, intersection_polygons);

  double union_area = 0.0;
  double intersection_area = 0.0;
  for (const auto & union_polygon : union_polygons) {
    union_area += boost::geometry::area(union_polygon);
  }
  for (const auto & intersection_polygon : intersection_polygons) {
    intersection_area += boost::geometry::area(intersection_polygon);
  }
  const double iou = union_area < 0.01 ? 0.0 : std::min(1.0, intersection_area / union_area);
  return iou;
}

double get2dPrecision(
  const autoware_auto_perception_msgs::msg::DetectedObject & source_object,
  const autoware_auto_perception_msgs::msg::DetectedObject & target_object)
{
  tier4_autoware_utils::Polygon2d source_polygon, target_polygon;
  toPolygon2d(source_object, source_polygon);
  toPolygon2d(target_object, target_polygon);

  std::vector<tier4_autoware_utils::Polygon2d> intersection_polygons;
  boost::geometry::intersection(source_polygon, target_polygon, intersection_polygons);

  double intersection_area = 0.0;
  double source_area = 0.0;
  for (const auto & intersection_polygon : intersection_polygons) {
    intersection_area += boost::geometry::area(intersection_polygon);
  }
  source_area = boost::geometry::area(source_polygon);
  const double precision = std::min(1.0, intersection_area / source_area);
  return precision;
}

double get2dRecall(
  const autoware_auto_perception_msgs::msg::DetectedObject & source_object,
  const autoware_auto_perception_msgs::msg::DetectedObject & target_object)
{
  tier4_autoware_utils::Polygon2d source_polygon, target_polygon;
  toPolygon2d(source_object, source_polygon);
  toPolygon2d(target_object, target_polygon);

  std::vector<tier4_autoware_utils::Polygon2d> intersection_polygons;
  boost::geometry::union_(source_polygon, target_polygon, intersection_polygons);

  double intersection_area = 0.0;
  double target_area = 0.0;
  for (const auto & intersection_polygon : intersection_polygons) {
    intersection_area += boost::geometry::area(intersection_polygon);
  }
  target_area += boost::geometry::area(target_polygon);
  const double recall = std::min(1.0, intersection_area / target_area);
  return recall;
}

tier4_autoware_utils::Polygon2d inverseClockWise(const tier4_autoware_utils::Polygon2d & polygon)
{
  tier4_autoware_utils::Polygon2d inverted_polygon;
  for (int i = polygon.outer().size() - 1; 0 <= i; --i) {
    inverted_polygon.outer().push_back(polygon.outer().at(i));
  }
  return inverted_polygon;
}

bool isClockWise(const tier4_autoware_utils::Polygon2d & polygon)
{
  const int n = polygon.outer().size();
  const double x_offset = polygon.outer().at(0).x();
  const double y_offset = polygon.outer().at(0).y();
  double sum = 0.0;
  for (std::size_t i = 0; i < polygon.outer().size(); ++i) {
    sum +=
      (polygon.outer().at(i).x() - x_offset) * (polygon.outer().at((i + 1) % n).y() - y_offset) -
      (polygon.outer().at(i).y() - y_offset) * (polygon.outer().at((i + 1) % n).x() - x_offset);
  }

  return sum < 0.0;
}

void toPolygon2d(
  const autoware_auto_perception_msgs::msg::DetectedObject & object,
  tier4_autoware_utils::Polygon2d & output)
{
  if (object.shape.type == autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX) {
    const auto & pose = object.kinematics.pose_with_covariance.pose;
    const double yaw = tier4_autoware_utils::normalizeRadian(tf2::getYaw(pose.orientation));
    Eigen::Matrix2d rotation;
    rotation << std::cos(yaw), -std::sin(yaw), std::sin(yaw), std::cos(yaw);
    Eigen::Vector2d offset0, offset1, offset2, offset3;
    offset0 = rotation *
              Eigen::Vector2d(object.shape.dimensions.x * 0.5f, object.shape.dimensions.y * 0.5f);
    offset1 = rotation *
              Eigen::Vector2d(object.shape.dimensions.x * 0.5f, -object.shape.dimensions.y * 0.5f);
    offset2 = rotation *
              Eigen::Vector2d(-object.shape.dimensions.x * 0.5f, -object.shape.dimensions.y * 0.5f);
    offset3 = rotation *
              Eigen::Vector2d(-object.shape.dimensions.x * 0.5f, object.shape.dimensions.y * 0.5f);
    output.outer().push_back(boost::geometry::make<tier4_autoware_utils::Point2d>(
      pose.position.x + offset0.x(), pose.position.y + offset0.y()));
    output.outer().push_back(boost::geometry::make<tier4_autoware_utils::Point2d>(
      pose.position.x + offset1.x(), pose.position.y + offset1.y()));
    output.outer().push_back(boost::geometry::make<tier4_autoware_utils::Point2d>(
      pose.position.x + offset2.x(), pose.position.y + offset2.y()));
    output.outer().push_back(boost::geometry::make<tier4_autoware_utils::Point2d>(
      pose.position.x + offset3.x(), pose.position.y + offset3.y()));
    output.outer().push_back(output.outer().front());
  } else if (object.shape.type == autoware_auto_perception_msgs::msg::Shape::CYLINDER) {
    const auto & center = object.kinematics.pose_with_covariance.pose.position;
    const auto & radius = object.shape.dimensions.x * 0.5;
    constexpr int n = 6;
    for (int i = 0; i < n; ++i) {
      Eigen::Vector2d point;
      point.x() = std::cos(
                    (static_cast<double>(i) / static_cast<double>(n)) * 2.0 * M_PI +
                    M_PI / static_cast<double>(n)) *
                    radius +
                  center.x;
      point.y() = std::sin(
                    (static_cast<double>(i) / static_cast<double>(n)) * 2.0 * M_PI +
                    M_PI / static_cast<double>(n)) *
                    radius +
                  center.y;
      output.outer().push_back(
        boost::geometry::make<tier4_autoware_utils::Point2d>(point.x(), point.y()));
    }
    output.outer().push_back(output.outer().front());
  } else if (object.shape.type == autoware_auto_perception_msgs::msg::Shape::POLYGON) {
    const auto & pose = object.kinematics.pose_with_covariance.pose;
    for (const auto & point : object.shape.footprint.points) {
      output.outer().push_back(boost::geometry::make<tier4_autoware_utils::Point2d>(
        pose.position.x + point.x, pose.position.y + point.y));
    }
    output.outer().push_back(output.outer().front());
  }
  output = isClockWise(output) ? output : inverseClockWise(output);
}

}  // namespace utils
