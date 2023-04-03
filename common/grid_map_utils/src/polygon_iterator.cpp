// Copyright 2022 Tier IV, Inc.
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

#include "grid_map_utils/polygon_iterator.hpp"

#include "grid_map_core/GridMap.hpp"
#include "grid_map_core/Polygon.hpp"
#include "grid_map_core/TypeDefs.hpp"

#include <algorithm>
#include <functional>
#include <utility>

namespace grid_map_utils
{

std::vector<Edge> PolygonIterator::calculateSortedEdges(const grid_map::Polygon & polygon)
{
  std::vector<Edge> edges;
  edges.reserve(polygon.nVertices());
  const auto & vertices = polygon.getVertices();
  for (auto vertex = vertices.cbegin(); std::next(vertex) != vertices.cend(); ++vertex) {
    // order pair by decreasing x and ignore horizontal edges (when x is equal)
    if (vertex->x() > std::next(vertex)->x())
      edges.emplace_back(*vertex, *std::next(vertex));
    else if (vertex->x() < std::next(vertex)->x())
      edges.emplace_back(*std::next(vertex), *vertex);
  }
  std::sort(edges.begin(), edges.end());
  edges.shrink_to_fit();
  return edges;
}

std::vector<std::vector<double>> PolygonIterator::calculateIntersectionsPerLine(
  const std::vector<Edge> & edges, const std::pair<int, int> from_to_row,
  const grid_map::Position & origin, const grid_map::GridMap & grid_map)
{
  const auto from_row = from_to_row.first;
  const auto to_row = from_to_row.second;
  // calculate for each line the y value intersecting with the polygon in decreasing order
  std::vector<std::vector<double>> y_intersections_per_line;
  y_intersections_per_line.reserve(to_row - from_row + 1);
  for (auto row = from_row; row <= to_row; ++row) {
    std::vector<double> y_intersections;
    const auto line_x = origin.x() - grid_map.getResolution() * row;
    for (const auto & edge : edges) {
      // special case when exactly touching a vertex: only count edge for its lowest x
      // up-down edge (\/) case: count the vertex twice
      // down-down edge case: count the vertex only once
      if (edge.second.x() == line_x) {
        y_intersections.push_back(edge.second.y());
      } else if (edge.first.x() >= line_x && edge.second.x() < line_x) {
        const auto diff = edge.first - edge.second;
        const auto y = edge.second.y() + (line_x - edge.second.x()) * diff.y() / diff.x();
        y_intersections.push_back(y);
      } else if (edge.first.x() < line_x) {  // edge below the line
        break;
      }
    }
    std::sort(y_intersections.begin(), y_intersections.end(), std::greater());
    // remove pairs outside of map
    auto iter = y_intersections.cbegin();
    while (iter != y_intersections.cend() && std::next(iter) != y_intersections.cend() &&
           *iter >= origin.y() && *std::next(iter) >= origin.y()) {
      iter = y_intersections.erase(iter);
      iter = y_intersections.erase(iter);
    }
    iter = std::lower_bound(
      y_intersections.cbegin(), y_intersections.cend(),
      origin.y() - (grid_map.getSize()(1) - 1) * grid_map.getResolution(), std::greater());
    while (iter != y_intersections.cend() && std::next(iter) != y_intersections.cend()) {
      iter = y_intersections.erase(iter);
      iter = y_intersections.erase(iter);
    }
    y_intersections_per_line.push_back(y_intersections);
  }
  return y_intersections_per_line;
}

std::pair<int, int> PolygonIterator::calculateRowRange(
  const std::vector<Edge> & edges, const grid_map::Position & origin,
  const grid_map::GridMap & grid_map)
{
  const auto min_vertex_x =
    std::min_element(edges.cbegin(), edges.cend(), [](const Edge & e1, const Edge & e2) {
      return e1.second.x() < e2.second.x();
    })->second.x();
  const auto max_vertex_x = edges.front().first.x();

  const auto dist_min_to_origin = origin.x() - min_vertex_x + grid_map.getResolution();
  const auto dist_max_to_origin = origin.x() - max_vertex_x + grid_map.getResolution();
  const auto min_row = std::clamp(
    static_cast<int>(dist_max_to_origin / grid_map.getResolution()), 0, grid_map.getSize()(0) - 1);
  const auto max_row = std::clamp(
    static_cast<int>(dist_min_to_origin / grid_map.getResolution()), 0, grid_map.getSize()(0) - 1);

  return {min_row, max_row};
}

PolygonIterator::PolygonIterator(
  const grid_map::GridMap & grid_map, const grid_map::Polygon & polygon)
{
  auto poly = polygon;
  if (poly.nVertices() < 3) return;
  // repeat the first vertex to get the last edge [last vertex, first vertex]
  if (poly.getVertex(0) != poly.getVertex(poly.nVertices() - 1)) poly.addVertex(poly.getVertex(0));

  map_start_idx_ = grid_map.getStartIndex();
  map_resolution_ = grid_map.getResolution();
  map_size_ = grid_map.getSize();
  const auto origin = [&]() {
    grid_map::Position origin;
    grid_map.getPosition(map_start_idx_, origin);
    return origin;
  }();
  map_origin_y_ = origin.y();

  // We make line scan left -> right / up -> down *in the index frame* (idx[0,0] is pos[up, left]).
  // In the position frame, this corresponds to high -> low Y values and high -> low X values.
  const std::vector<Edge> edges = calculateSortedEdges(poly);
  if (edges.empty()) return;
  const auto from_to_row = calculateRowRange(edges, origin, grid_map);
  intersections_per_line_ = calculateIntersectionsPerLine(edges, from_to_row, origin, grid_map);

  row_of_first_line_ = from_to_row.first;
  current_col_ = 0;
  current_to_col_ = -1;
  current_line_ = -1;  // goToNextLine() increments the line so assign -1 to start from 0

  // Initialize iterator to the first (row,column) inside the Polygon
  if (!intersections_per_line_.empty()) {
    goToNextLine();
    if (!isPastEnd()) {
      calculateColumnIndexes();
      calculateIndex();
    }
  }
}

bool PolygonIterator::operator!=(const PolygonIterator & other) const
{
  return current_line_ != other.current_line_ || current_col_ != other.current_col_;
}

const grid_map::Index & PolygonIterator::operator*() const
{
  return current_index_;
}

void PolygonIterator::goToNextLine()
{
  ++current_line_;
  while (current_line_ < intersections_per_line_.size() &&
         intersections_per_line_[current_line_].size() < 2)
    ++current_line_;
  if (!isPastEnd()) intersection_iter_ = intersections_per_line_[current_line_].cbegin();
}

void PolygonIterator::calculateColumnIndexes()
{
  const auto dist_from_origin = map_origin_y_ - *intersection_iter_ + map_resolution_;
  current_col_ =
    std::clamp(static_cast<int>(dist_from_origin / map_resolution_), 0, map_size_(1) - 1);
  ++intersection_iter_;
  const auto dist_to_origin = map_origin_y_ - *intersection_iter_;
  current_to_col_ =
    std::clamp(static_cast<int>(dist_to_origin / map_resolution_), 0, map_size_(1) - 1);
  // Case where intersections do not encompass the center of a cell: iterate again
  if (current_to_col_ < current_col_) {
    operator++();
  }
}

void PolygonIterator::calculateIndex()
{
  current_index_(0) = map_start_idx_(0) + row_of_first_line_ + static_cast<int>(current_line_);
  grid_map::wrapIndexToRange(current_index_(0), map_size_(0));
  current_index_(1) = map_start_idx_(1) + current_col_;
  grid_map::wrapIndexToRange(current_index_(1), map_size_(1));
}

PolygonIterator & PolygonIterator::operator++()
{
  ++current_col_;
  if (current_col_ > current_to_col_) {
    ++intersection_iter_;
    if (
      intersection_iter_ == intersections_per_line_[current_line_].cend() ||
      std::next(intersection_iter_) == intersections_per_line_[current_line_].cend()) {
      goToNextLine();
    }
    if (!isPastEnd()) {
      calculateColumnIndexes();
    }
  }
  if (!isPastEnd()) {
    calculateIndex();
  }
  return *this;
}

[[nodiscard]] bool PolygonIterator::isPastEnd() const
{
  return current_line_ >= intersections_per_line_.size();
}
}  // namespace grid_map_utils
