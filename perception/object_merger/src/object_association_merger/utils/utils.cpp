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

#include <object_association_merger/utils/utils.hpp>

namespace utils
{
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

  for (int i = 0; i < static_cast<int>(footprint.points.size()); ++i) {
    int j = (i + 1) % static_cast<int>(footprint.points.size());
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
}  // namespace utils
