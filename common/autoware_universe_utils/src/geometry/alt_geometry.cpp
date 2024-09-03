// Copyright 2023-2024 TIER IV, Inc.
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

#include "autoware/universe_utils/geometry/alt_geometry.hpp"

namespace autoware::universe_utils
{
// Alternatives for Boost.Geometry ----------------------------------------------------------------
namespace alt
{
Point2d from_boost(const autoware::universe_utils::Point2d & point)
{
  return {point.x(), point.y()};
}

ConvexPolygon2d from_boost(const autoware::universe_utils::Polygon2d & polygon)
{
  Points2d points;
  for (const auto & point : polygon.outer()) {
    points.push_back(from_boost(point));
  }

  ConvexPolygon2d _polygon(points);
  correct(_polygon);
  return _polygon;
}

autoware::universe_utils::Point2d to_boost(const Point2d & point)
{
  return {point.x(), point.y()};
}

autoware::universe_utils::Polygon2d to_boost(const ConvexPolygon2d & polygon)
{
  autoware::universe_utils::Polygon2d _polygon;
  for (const auto & vertex : polygon.vertices()) {
    _polygon.outer().push_back(to_boost(vertex));
  }
  return _polygon;
}
}  // namespace alt

double area(const alt::ConvexPolygon2d & poly)
{
  const auto & vertices = poly.vertices();

  double area = 0.;
  for (size_t i = 1; i < vertices.size() - 1; ++i) {
    area += (vertices[i + 1] - vertices.front()).cross(vertices[i] - vertices.front()) / 2;
  }

  return area;
}

alt::ConvexPolygon2d convex_hull(const alt::Points2d & points)
{
  if (points.size() < 3) {
    throw std::invalid_argument("At least 3 points are required for calculating convex hull.");
  }

  // QuickHull algorithm

  const auto p_minmax_itr = std::minmax_element(
    points.begin(), points.end(), [](const auto & a, const auto & b) { return a.x() < b.x(); });
  const auto & p_min = *p_minmax_itr.first;
  const auto & p_max = *p_minmax_itr.second;

  alt::Points2d vertices;

  if (points.size() == 3) {
    std::rotate_copy(
      points.begin(), p_minmax_itr.first, points.end(), std::back_inserter(vertices));
  } else {
    auto make_hull = [&vertices](
                       auto self, const alt::Point2d & p1, const alt::Point2d & p2,
                       const alt::Points2d & points) {
      if (points.empty()) {
        return;
      }

      const auto farthest =
        *std::max_element(points.begin(), points.end(), [&](const auto & a, const auto & b) {
          const auto seg_vec = p2 - p1;
          return seg_vec.cross(a - p1) < seg_vec.cross(b - p1);
        });

      alt::Points2d subset1, subset2;
      for (const auto & point : points) {
        if (is_above(point, p1, farthest)) {
          subset1.push_back(point);
        } else if (is_above(point, farthest, p2)) {
          subset2.push_back(point);
        }
      }

      self(self, p1, farthest, subset1);
      vertices.push_back(farthest);
      self(self, farthest, p2, subset2);
    };

    alt::Points2d above_points, below_points;
    for (const auto & point : points) {
      if (is_above(point, p_min, p_max)) {
        above_points.push_back(point);
      } else if (is_above(point, p_max, p_min)) {
        below_points.push_back(point);
      }
    }

    vertices.push_back(p_min);
    make_hull(make_hull, p_min, p_max, above_points);
    vertices.push_back(p_max);
    make_hull(make_hull, p_max, p_min, below_points);
  }

  alt::ConvexPolygon2d hull(vertices);
  correct(hull);

  return hull;
}

void correct(alt::ConvexPolygon2d & poly)
{
  auto & vertices = poly.vertices();

  // sort points in clockwise order with respect to the first point
  std::sort(vertices.begin() + 1, vertices.end(), [&](const auto & a, const auto & b) {
    return (a - vertices.front()).cross(b - vertices.front()) < 0;
  });

  if (equals(vertices.front(), vertices.back())) {
    vertices.pop_back();
  }
}

bool covered_by(const alt::Point2d & point, const alt::ConvexPolygon2d & poly)
{
  constexpr double epsilon = 1e-6;

  const auto & vertices = poly.vertices();
  const auto num_of_vertices = vertices.size();
  int64_t winding_number = 0;

  const auto [y_min_vertex, y_max_vertex] = std::minmax_element(
    vertices.begin(), vertices.end(), [](const auto & a, const auto & b) { return a.y() < b.y(); });
  if (point.y() < y_min_vertex->y() || point.y() > y_max_vertex->y()) {
    return false;
  }

  double cross;
  for (size_t i = 0; i < num_of_vertices; ++i) {
    const auto & p1 = vertices[i];
    const auto & p2 = vertices[(i + 1) % num_of_vertices];

    if (p1.y() <= point.y() && p2.y() >= point.y()) {  // upward edge
      cross = (p2 - p1).cross(point - p1);
      if (cross > 0) {  // point is to the left of edge
        winding_number++;
        continue;
      }
    } else if (p1.y() >= point.y() && p2.y() <= point.y()) {  // downward edge
      cross = (p2 - p1).cross(point - p1);
      if (cross < 0) {  // point is to the left of edge
        winding_number--;
        continue;
      }
    } else {
      continue;
    }

    if (std::abs(cross) < epsilon) {  // point is on edge
      return true;
    }
  }

  return winding_number != 0;
}

bool disjoint(const alt::ConvexPolygon2d & poly1, const alt::ConvexPolygon2d & poly2)
{
  if (equals(poly1, poly2)) {
    return false;
  }

  if (intersects(poly1, poly2)) {
    return false;
  }

  for (const auto & vertex : poly1.vertices()) {
    if (touches(vertex, poly2)) {
      return false;
    }
  }

  return true;
}

double distance(
  const alt::Point2d & point, const alt::Point2d & seg_start, const alt::Point2d & seg_end)
{
  constexpr double epsilon = 1e-3;

  const auto seg_vec = seg_end - seg_start;
  const auto point_vec = point - seg_start;

  const double seg_vec_norm = seg_vec.norm();
  const double seg_point_dot = seg_vec.dot(point_vec);

  if (seg_vec_norm < epsilon || seg_point_dot < 0) {
    return point_vec.norm();
  } else if (seg_point_dot > std::pow(seg_vec_norm, 2)) {
    return (point - seg_end).norm();
  } else {
    return std::abs(seg_vec.cross(point_vec)) / seg_vec_norm;
  }
}

double distance(const alt::Point2d & point, const alt::ConvexPolygon2d & poly)
{
  if (covered_by(point, poly)) {
    return 0.0;
  }

  // TODO(mitukou1109): Use plane sweep method to improve performance?
  const auto & vertices = poly.vertices();
  double min_distance = std::numeric_limits<double>::max();
  for (size_t i = 0; i < vertices.size(); ++i) {
    min_distance =
      std::min(min_distance, distance(point, vertices[i], vertices[(i + 1) % vertices.size()]));
  }

  return min_distance;
}

bool equals(const alt::Point2d & point1, const alt::Point2d & point2)
{
  constexpr double epsilon = 1e-3;
  return std::abs(point1.x() - point2.x()) < epsilon && std::abs(point1.y() - point2.y()) < epsilon;
}

bool equals(const alt::ConvexPolygon2d & poly1, const alt::ConvexPolygon2d & poly2)
{
  return std::all_of(poly1.vertices().begin(), poly1.vertices().end(), [&](const auto & a) {
    return std::any_of(poly2.vertices().begin(), poly2.vertices().end(), [&](const auto & b) {
      return equals(a, b);
    });
  });
}

bool intersects(
  const alt::Point2d & seg1_start, const alt::Point2d & seg1_end, const alt::Point2d & seg2_start,
  const alt::Point2d & seg2_end)
{
  constexpr double epsilon = 1e-6;

  const auto v1 = seg1_end - seg1_start;
  const auto v2 = seg2_end - seg2_start;

  const auto det = v1.cross(v2);
  if (std::abs(det) < epsilon) {
    return false;
  }

  const auto v12 = seg2_end - seg1_end;
  const double t = v2.cross(v12) / det;
  const double s = v1.cross(v12) / det;
  if (t < 0 || 1 < t || s < 0 || 1 < s) {
    return false;
  }

  return true;
}

bool intersects(const alt::ConvexPolygon2d & poly1, const alt::ConvexPolygon2d & poly2)
{
  if (equals(poly1, poly2)) {
    return true;
  }

  // GJK algorithm

  auto find_support_vector = [](
                               const alt::ConvexPolygon2d & poly1,
                               const alt::ConvexPolygon2d & poly2,
                               const alt::Vector2d & direction) {
    auto find_farthest_vertex =
      [](const alt::ConvexPolygon2d & poly, const alt::Vector2d & direction) {
        return std::max_element(
          poly.vertices().begin(), poly.vertices().end(),
          [&](const auto & a, const auto & b) { return direction.dot(a) <= direction.dot(b); });
      };
    return *find_farthest_vertex(poly1, direction) - *find_farthest_vertex(poly2, -direction);
  };

  alt::Vector2d direction = {1.0, 0.0};
  auto a = find_support_vector(poly1, poly2, direction);
  direction = -a;
  auto b = find_support_vector(poly1, poly2, direction);
  if (b.dot(direction) <= 0.0) {
    return false;
  }

  direction = (b - a).vector_triple(-a, b - a);
  while (true) {
    auto c = find_support_vector(poly1, poly2, direction);
    if (c.dot(direction) <= 0.0) {
      return false;
    }

    auto n_ca = (b - c).vector_triple(a - c, a - c);
    if (n_ca.dot(-c) > 0.0) {
      b = c;
      direction = n_ca;
    } else {
      auto n_cb = (a - c).vector_triple(b - c, b - c);
      if (n_cb.dot(-c) > 0.0) {
        a = c;
        direction = n_cb;
      } else {
        break;
      }
    }
  }

  return true;
}

bool is_above(
  const alt::Point2d & point, const alt::Point2d & seg_start, const alt::Point2d & seg_end)
{
  return (seg_end - seg_start).cross(point - seg_start) > 0;
}

bool is_clockwise(const alt::ConvexPolygon2d & poly)
{
  return area(poly) > 0;
}

bool touches(
  const alt::Point2d & point, const alt::Point2d & seg_start, const alt::Point2d & seg_end)
{
  constexpr double epsilon = 1e-6;

  // if the cross product of the vectors from the start point and the end point to the point is 0
  // and the vectors opposite each other, the point is on the segment
  const auto start_vec = point - seg_start;
  const auto end_vec = point - seg_end;
  return std::abs(start_vec.cross(end_vec)) < epsilon && start_vec.dot(end_vec) <= 0;
}

bool touches(const alt::Point2d & point, const alt::ConvexPolygon2d & poly)
{
  const auto & vertices = poly.vertices();
  const auto num_of_vertices = vertices.size();

  const auto [y_min_vertex, y_max_vertex] = std::minmax_element(
    vertices.begin(), vertices.end(), [](const auto & a, const auto & b) { return a.y() < b.y(); });
  if (point.y() < y_min_vertex->y() || point.y() > y_max_vertex->y()) {
    return false;
  }

  for (size_t i = 0; i < num_of_vertices; ++i) {
    // check if the point is on each edge of the polygon
    if (touches(point, vertices[i], vertices[(i + 1) % num_of_vertices])) {
      return true;
    }
  }

  return false;
}

bool within(const alt::Point2d & point, const alt::ConvexPolygon2d & poly)
{
  constexpr double epsilon = 1e-6;

  const auto & vertices = poly.vertices();
  const auto num_of_vertices = vertices.size();
  int64_t winding_number = 0;

  const auto [y_min_vertex, y_max_vertex] = std::minmax_element(
    vertices.begin(), vertices.end(), [](const auto & a, const auto & b) { return a.y() < b.y(); });
  if (point.y() <= y_min_vertex->y() || point.y() >= y_max_vertex->y()) {
    return false;
  }

  double cross;
  for (size_t i = 0; i < num_of_vertices; ++i) {
    const auto & p1 = vertices[i];
    const auto & p2 = vertices[(i + 1) % num_of_vertices];

    if (p1.y() < point.y() && p2.y() > point.y()) {  // upward edge
      cross = (p2 - p1).cross(point - p1);
      if (cross > 0) {  // point is to the left of edge
        winding_number++;
        continue;
      }
    } else if (p1.y() > point.y() && p2.y() < point.y()) {  // downward edge
      cross = (p2 - p1).cross(point - p1);
      if (cross < 0) {  // point is to the left of edge
        winding_number--;
        continue;
      }
    } else {
      continue;
    }

    if (std::abs(cross) < epsilon) {  // point is on edge
      return false;
    }
  }

  return winding_number != 0;
}

bool within(
  const alt::ConvexPolygon2d & poly_contained, const alt::ConvexPolygon2d & poly_containing)
{
  if (equals(poly_contained, poly_containing)) {
    return true;
  }

  // check if all points of poly_contained are within poly_containing
  for (const auto & vertex : poly_contained.vertices()) {
    if (!within(vertex, poly_containing)) {
      return false;
    }
  }

  return true;
}
}  // namespace autoware::universe_utils
