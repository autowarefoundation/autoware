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

#include "autoware/universe_utils/geometry/random_concave_polygon.hpp"

#include "autoware/universe_utils/geometry/boost_geometry.hpp"

#include <boost/geometry/algorithms/convex_hull.hpp>
#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/intersects.hpp>
#include <boost/geometry/strategies/agnostic/hull_graham_andrew.hpp>

#include <random>

namespace autoware::universe_utils
{
namespace
{
/// @brief define Edge as a pair of Points
struct Edge
{
  Point2d first;
  Point2d second;
  bool valid = true;
  bool operator==(const Edge & other) const
  {
    return (first == other.first && second == other.second) ||
           (first == other.second && second == other.first);
  }
};

/// @brief structure to hold a polygon and its edges
struct PolygonWithEdges
{
  Polygon2d polygon;
  std::vector<Edge> edges;
};

/// @brief prepares coordinate vectors for a given number of vertices
std::vector<double> prepare_coordinate_vectors(
  const size_t nb_vertices,
  std::uniform_real_distribution<double> &
    random_double,                                   // cppcheck-suppress constParameterReference
  std::uniform_int_distribution<int> & random_bool,  // cppcheck-suppress constParameterReference
  std::default_random_engine & random_engine)
{
  std::vector<double> v;
  v.reserve(nb_vertices);
  for (auto i = 0UL; i < nb_vertices; ++i) {
    v.push_back(random_double(random_engine));
  }
  std::sort(v.begin(), v.end());
  const auto min_v = v.front();
  const auto max_v = v.back();
  std::vector<double> v1;
  v1.push_back(min_v);
  std::vector<double> v2;
  v2.push_back(min_v);
  for (auto i = 1UL; i + 1 < v.size(); ++i) {
    if (random_bool(random_engine) == 0) {
      v1.push_back((v[i]));
    } else {
      v2.push_back((v[i]));
    }
  }
  v1.push_back(max_v);
  v2.push_back(max_v);
  std::vector<double> diffs;
  for (auto i = 0UL; i + 1 < v1.size(); ++i) {
    diffs.push_back(v1[i + 1] - v1[i]);
  }
  for (auto i = 0UL; i + 1 < v2.size(); ++i) {
    diffs.push_back(v2[i] - v2[i + 1]);
  }
  std::vector<double> vectors;
  vectors = diffs;
  return vectors;
}

/// @brief calculates the distance from a point to an edge
double dist(const Edge & edge, const Point2d & point)
{
  double x = point.x();
  double y = point.y();
  double x1 = edge.first.x();
  double y1 = edge.first.y();
  double x2 = edge.second.x();
  double y2 = edge.second.y();

  double dx = x2 - x1;
  double dy = y2 - y1;

  if (dx == 0.0 && dy == 0.0) {
    dx = x - x1;
    dy = y - y1;
    return std::sqrt(dx * dx + dy * dy);
  }

  double t = ((x - x1) * dx + (y - y1) * dy) / (dx * dx + dy * dy);
  t = std::max(0.0, std::min(1.0, t));
  dx = x - (x1 + t * dx);
  dy = y - (y1 + t * dy);
  return std::sqrt(dx * dx + dy * dy);
}

/// @brief checks if an edge intersects with a polygon
bool intersecting(const Edge & e, const Polygon2d & polygon)
{
  Segment2d edge_segment{e.first, e.second};
  if (boost::geometry::intersects(edge_segment, polygon)) {
    // additional check: ensure it's not just a single point intersection
    for (size_t i = 0; i < polygon.outer().size(); ++i) {
      const Point2d & p1 = polygon.outer()[i];
      const Point2d & p2 = polygon.outer()[(i + 1) % polygon.outer().size()];
      Segment2d poly_segment{p1, p2};

      if (boost::geometry::intersects(edge_segment, poly_segment)) {
        if (!(e.first == p1 || e.first == p2 || e.second == p1 || e.second == p2)) {
          return true;
        }
      }
    }
    return false;
  }

  return false;
}

/// @brief checks if an edge is valid for a given polygon and set of points
bool is_valid(const Edge & e, const Polygon2d & P, const std::vector<Point2d> & Q)
{
  bool valid = false;
  size_t i = 0;

  while (!valid && i < Q.size()) {
    const Point2d & q = Q[i];
    Edge e1 = {e.first, q};
    Edge e2 = {q, e.second};
    bool intersects_e1 = intersecting(e1, P);
    bool intersects_e2 = intersecting(e2, P);

    if (!intersects_e1 && !intersects_e2) {
      valid = true;
    }

    ++i;
  }

  return valid;
}

/// @brief finds the nearest node from a set of points to an edge
Point2d get_nearest_node(const std::vector<Point2d> & Q, const Edge & e)
{
  double min_distance = std::numeric_limits<double>::max();
  Point2d nearest_node(0, 0);

  for (const auto & node : Q) {
    double distance = dist(e, node);

    if (distance < min_distance) {
      min_distance = distance;
      nearest_node = node;
    }
  }
  return nearest_node;
}

/// @brief finds the edge that is closest to the given set of points
Edge get_breaking_edge(const PolygonWithEdges & polygon_with_edges, const std::vector<Point2d> & Q)
{
  double min_distance = std::numeric_limits<double>::max();
  Edge e_breaking;
  e_breaking.valid = false;

  for (const auto & edge : polygon_with_edges.edges) {
    if (is_valid(edge, polygon_with_edges.polygon, Q)) {
      Point2d nearest_node = get_nearest_node(Q, edge);
      double distance = dist(edge, nearest_node);
      if (distance < min_distance) {
        min_distance = distance;
        e_breaking = edge;
      }
    }
  }
  return e_breaking;
}

/// @brief updates the polygon's outer ring based on its edges
void update_polygon_from_edges(PolygonWithEdges & polygon_with_edges)
{
  LinearRing2d new_outer_ring;

  for (const auto & edge : polygon_with_edges.edges) {
    if (edge.valid) {
      new_outer_ring.push_back(edge.first);
    }
  }

  polygon_with_edges.polygon.outer() = new_outer_ring;
  boost::geometry::correct(polygon_with_edges.polygon);
}

/// @brief inserts a node into the polygon's edges
void insert_node(PolygonWithEdges & polygon_with_edges, const Point2d & w, const Edge & e)
{
  std::vector<Edge> new_edges;
  for (const Edge & current_edge : polygon_with_edges.edges) {
    if (current_edge == e) {
      new_edges.push_back({e.first, w});
      new_edges.push_back({w, e.second});
    } else {
      new_edges.push_back(current_edge);
    }
  }

  polygon_with_edges.edges = std::move(new_edges);
}

/// @brief removes a node from a set of points
void remove_node(std::vector<Point2d> & Q, const Point2d & w)
{
  const double epsilon = 1e-9;

  Q.erase(
    std::remove_if(
      Q.begin(), Q.end(),
      [&](const Point2d & p) {
        return std::abs(p.x() - w.x()) < epsilon && std::abs(p.y() - w.y()) < epsilon;
      }),
    Q.end());
}

/// @brief marks edges as valid if they are valid according to the polygon and points
void mark_valid_edges(PolygonWithEdges & polygon_with_edges, const std::vector<Point2d> & Q)
{
  for (auto & edge : polygon_with_edges.edges) {
    if (is_valid(edge, polygon_with_edges.polygon, Q)) {
      edge.valid = true;
    }
  }
}

/// @brief performs inward denting on a linear ring to create a new polygon
Polygon2d inward_denting(LinearRing2d & ring)
{
  LinearRing2d convex_ring;
  std::vector<Point2d> q;
  q.reserve(ring.size());
  boost::geometry::strategy::convex_hull::graham_andrew<LinearRing2d, Point2d> strategy;
  boost::geometry::convex_hull(ring, convex_ring, strategy);
  PolygonWithEdges polygon_with_edges;
  polygon_with_edges.polygon.outer() = convex_ring;
  polygon_with_edges.edges.resize(polygon_with_edges.polygon.outer().size());

  for (const auto & point : ring) {
    if (boost::geometry::within(point, polygon_with_edges.polygon)) {
      q.push_back(point);
    }
  }
  for (size_t i = 0; i < polygon_with_edges.edges.size(); ++i) {
    polygon_with_edges.edges[i] = {
      polygon_with_edges.polygon.outer()[i],
      polygon_with_edges.polygon.outer()[(i + 1) % polygon_with_edges.polygon.outer().size()]};
  }
  while (!q.empty()) {
    Edge e = get_breaking_edge(polygon_with_edges, q);
    Point2d w = get_nearest_node(q, e);
    insert_node(polygon_with_edges, w, e);
    remove_node(q, w);
    mark_valid_edges(polygon_with_edges, q);
    update_polygon_from_edges(polygon_with_edges);
  }

  return polygon_with_edges.polygon;
}

}  // namespace

/// @brief checks if a polygon is convex
bool is_convex(const autoware::universe_utils::Polygon2d & polygon)
{
  const auto & outer_ring = polygon.outer();
  size_t num_points = outer_ring.size();

  if (num_points < 4) {
    return true;
  }

  bool is_positive = false;
  bool is_negative = false;

  for (size_t i = 0; i < num_points; ++i) {
    auto p1 = outer_ring[i];
    auto p2 = outer_ring[(i + 1) % num_points];
    auto p3 = outer_ring[(i + 2) % num_points];

    double cross_product =
      (p2.x() - p1.x()) * (p3.y() - p2.y()) - (p2.y() - p1.y()) * (p3.x() - p2.x());

    if (cross_product > 0) {
      is_positive = true;
    } else if (cross_product < 0) {
      is_negative = true;
    }

    if (is_positive && is_negative) {
      return false;
    }
  }

  return true;
}

/// @brief checks for collisions between two vectors of convex polygons using a specified collision
/// detection algorithm
bool test_intersection(
  const std::vector<autoware::universe_utils::Polygon2d> & polygons1,
  const std::vector<autoware::universe_utils::Polygon2d> & polygons2,
  const std::function<bool(
    const autoware::universe_utils::Polygon2d &, const autoware::universe_utils::Polygon2d &)> &
    intersection_func)
{
  for (const auto & poly1 : polygons1) {
    for (const auto & poly2 : polygons2) {
      if (intersection_func(poly1, poly2)) {
        return true;
      }
    }
  }
  return false;
}

Polygon2d random_concave_polygon(const size_t vertices, const double max)
{
  if (vertices < 4) {
    return Polygon2d();
  }

  std::random_device r;
  std::default_random_engine random_engine(r());
  std::uniform_real_distribution<double> uniform_dist(-max, max);
  std::uniform_int_distribution<int> random_bool(0, 1);

  Polygon2d poly;
  bool is_non_convex = false;
  int max_attempts = 100;
  int attempt = 0;

  while (!is_non_convex && attempt < max_attempts) {
    auto xs = prepare_coordinate_vectors(vertices, uniform_dist, random_bool, random_engine);
    auto ys = prepare_coordinate_vectors(vertices, uniform_dist, random_bool, random_engine);

    std::shuffle(ys.begin(), ys.end(), random_engine);

    LinearRing2d vectors;
    for (size_t i = 0; i < xs.size(); ++i) {
      vectors.emplace_back(xs[i], ys[i]);
    }

    LinearRing2d points;
    for (const auto & p : vectors) {
      points.emplace_back(p.x(), p.y());
    }
    // apply inward denting algorithm
    poly = inward_denting(points);
    // check for convexity
    if (!is_convex(poly)) {
      is_non_convex = true;
    }
    LinearRing2d poly_outer = poly.outer();
    poly.outer() = poly_outer;

    // shift polygon to ensure all coordinates are positive
    double min_x = std::numeric_limits<double>::max();
    double min_y = std::numeric_limits<double>::max();
    for (const auto & point : poly.outer()) {
      if (point.x() < min_x) {
        min_x = point.x();
      }
      if (point.y() < min_y) {
        min_y = point.y();
      }
    }

    double shift_x = -min_x + std::abs(min_x - (-max));
    double shift_y = -min_y + std::abs(min_y - (-max));

    for (auto & point : poly.outer()) {
      point.x() += shift_x;
      point.y() += shift_y;
    }

    boost::geometry::correct(poly);
    ++attempt;
  }
  return poly;
}
}  // namespace autoware::universe_utils
