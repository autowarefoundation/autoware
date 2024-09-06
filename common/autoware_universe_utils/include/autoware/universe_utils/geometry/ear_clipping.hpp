// Copyright 2024 TIER IV, Inc.
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

#ifndef AUTOWARE__UNIVERSE_UTILS__GEOMETRY__EAR_CLIPPING_HPP_
#define AUTOWARE__UNIVERSE_UTILS__GEOMETRY__EAR_CLIPPING_HPP_

#include "autoware/universe_utils/geometry/boost_geometry.hpp"

#include <utility>
#include <vector>

namespace autoware::universe_utils
{

class EarClipping
{
public:
  std::vector<std::size_t> indices;
  std::size_t vertices = 0;
  using Polygon2d = autoware::universe_utils::Polygon2d;
  using Point2d = autoware::universe_utils::Point2d;
  using LinearRing2d = autoware::universe_utils::LinearRing2d;

  void operator()(const Polygon2d & polygon);

  ~EarClipping()
  {
    for (auto * p : points_) {
      delete p;
    }
  }

private:
  struct Point
  {
    Point(const std::size_t index, Point2d point) : i(index), pt(std::move(point)) {}

    const std::size_t i;  // Index of the point in the original polygon
    const Point2d pt;     // The Point2d object representing the coordinates

    // Previous and next vertices (Points) in the polygon ring
    Point * prev = nullptr;
    Point * next = nullptr;
    bool steiner = false;

    [[nodiscard]] double x() const { return pt.x(); }
    [[nodiscard]] double y() const { return pt.y(); }
  };

  std::vector<Point *> points_;

  Point * linked_list(const LinearRing2d & points, bool clockwise);
  static Point * filter_points(Point * start, Point * end = nullptr);
  Point * cure_local_intersections(Point * start);
  static Point * get_leftmost(Point * start);
  Point * split_polygon(Point * a, Point * b);
  Point * insert_point(std::size_t i, const Point2d & p, Point * last);
  Point * eliminate_holes(
    const std::vector<LinearRing2d> & inners, EarClipping::Point * outer_point);
  Point * eliminate_hole(Point * hole, Point * outer_point);
  static Point * find_hole_bridge(Point * hole, Point * outer_point);
  void ear_clipping_linked(Point * ear, int pass = 0);
  void split_ear_clipping(Point * start);
  static void remove_point(Point * p);
  static bool is_ear(Point * ear);
  static bool sector_contains_sector(const Point * m, const Point * p);
  [[nodiscard]] static bool point_in_triangle(
    double ax, double ay, double bx, double by, double cx, double cy, double px, double py);
  static bool is_valid_diagonal(Point * a, Point * b);
  static bool equals(const Point * p1, const Point * p2);
  static bool intersects(const Point * p1, const Point * q1, const Point * p2, const Point * q2);
  static bool on_segment(const Point * p, const Point * q, const Point * r);
  static bool intersects_polygon(const Point * a, const Point * b);
  static bool locally_inside(const Point * a, const Point * b);
  static bool middle_inside(const Point * a, const Point * b);
  static int sign(double val);
  static double area(const Point * p, const Point * q, const Point * r);

  // Function to construct a new Point object
  EarClipping::Point * construct_point(std::size_t index, const Point2d & point)
  {
    auto * new_point = new Point(index, point);
    points_.push_back(new_point);
    return new_point;
  }
};

/// @brief Triangulate based on ear clipping algorithm
/// @param polygon concave/convex polygon with/without holes
/// @details algorithm based on https://github.com/mapbox/earclipping with modification
std::vector<autoware::universe_utils::Polygon2d> triangulate(
  const autoware::universe_utils::Polygon2d & poly);

}  // namespace autoware::universe_utils

#endif  // AUTOWARE__UNIVERSE_UTILS__GEOMETRY__EAR_CLIPPING_HPP_
