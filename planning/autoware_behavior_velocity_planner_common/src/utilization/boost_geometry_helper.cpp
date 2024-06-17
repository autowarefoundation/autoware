// Copyright 2023 TIER IV, Inc.
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

#include <autoware/behavior_velocity_planner_common/utilization/boost_geometry_helper.hpp>

#include <boost/geometry/algorithms/correct.hpp>

#include <algorithm>
#include <vector>
namespace autoware::behavior_velocity_planner
{

Polygon2d lines2polygon(const LineString2d & left_line, const LineString2d & right_line)
{
  Polygon2d polygon;

  polygon.outer().push_back(left_line.front());

  for (auto itr = right_line.begin(); itr != right_line.end(); ++itr) {
    polygon.outer().push_back(*itr);
  }

  for (auto itr = left_line.rbegin(); itr != left_line.rend(); ++itr) {
    polygon.outer().push_back(*itr);
  }

  bg::correct(polygon);
  return polygon;
}

Polygon2d upScalePolygon(
  const geometry_msgs::msg::Point & position, const Polygon2d & polygon, const double scale)
{
  Polygon2d transformed_polygon;
  // upscale
  for (size_t i = 0; i < polygon.outer().size(); i++) {
    const double upscale_x = (polygon.outer().at(i).x() - position.x) * scale + position.x;
    const double upscale_y = (polygon.outer().at(i).y() - position.y) * scale + position.y;
    transformed_polygon.outer().emplace_back(Point2d(upscale_x, upscale_y));
  }
  return transformed_polygon;
}

geometry_msgs::msg::Polygon toGeomPoly(const Polygon2d & polygon)
{
  geometry_msgs::msg::Polygon polygon_msg;
  geometry_msgs::msg::Point32 point_msg;
  for (const auto & p : polygon.outer()) {
    point_msg.x = p.x();
    point_msg.y = p.y();
    polygon_msg.points.push_back(point_msg);
  }
  return polygon_msg;
}
}  // namespace autoware::behavior_velocity_planner
