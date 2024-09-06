// Copyright 2024 Tier IV, Inc.
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

#include "autoware/universe_utils/geometry/random_convex_polygon.hpp"

#include <boost/geometry/algorithms/correct.hpp>

#include <algorithm>
#include <random>
#include <vector>

namespace autoware::universe_utils
{
namespace
{
struct VectorsWithMin
{
  std::vector<double> vectors;
  double min{};
};

VectorsWithMin prepare_coordinate_vectors(
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
  VectorsWithMin vectors;
  vectors.vectors = diffs;
  vectors.min = min_v;
  return vectors;
}
}  // namespace
Polygon2d random_convex_polygon(const size_t vertices, const double max)
{
  std::random_device r;
  std::default_random_engine random_engine(r());
  std::uniform_real_distribution<double> uniform_dist(-max, max);
  std::uniform_int_distribution random_bool(0, 1);
  auto xs = prepare_coordinate_vectors(vertices, uniform_dist, random_bool, random_engine);
  auto ys = prepare_coordinate_vectors(vertices, uniform_dist, random_bool, random_engine);
  std::shuffle(ys.vectors.begin(), ys.vectors.end(), random_engine);
  LinearRing2d vectors;
  for (auto i = 0UL; i < xs.vectors.size(); ++i) {
    vectors.emplace_back(xs.vectors[i], ys.vectors[i]);
  }
  std::sort(vectors.begin(), vectors.end(), [](const Point2d & p1, const Point2d & p2) {
    return std::atan2(p1.y(), p1.x()) < std::atan2(p2.y(), p2.x());
  });
  auto min_x = max;
  auto min_y = max;
  auto x = 0.0;
  auto y = 0.0;
  LinearRing2d points;
  for (const auto & p : vectors) {
    points.emplace_back(x, y);
    x += p.x();
    y += p.y();
    min_x = std::min(p.x(), min_x);
    min_y = std::min(p.y(), min_y);
  }
  const auto shift_x = min_x - xs.min;
  const auto shift_y = min_y - ys.min;
  for (auto & p : points) {
    p.x() += shift_x;
    p.y() += shift_y;
  }
  Polygon2d poly;
  poly.outer() = points;
  boost::geometry::correct(poly);
  return poly;
}
}  // namespace autoware::universe_utils
