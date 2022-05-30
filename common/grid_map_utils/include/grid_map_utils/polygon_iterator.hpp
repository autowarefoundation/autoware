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

#ifndef GRID_MAP_UTILS__POLYGON_ITERATOR_HPP_
#define GRID_MAP_UTILS__POLYGON_ITERATOR_HPP_

#include "grid_map_core/TypeDefs.hpp"

#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/GridMapMath.hpp>
#include <grid_map_core/Polygon.hpp>

#include <utility>
#include <vector>

namespace grid_map_utils
{

/// @brief Representation of a polygon edge made of 2 vertices
struct Edge
{
  grid_map::Position first;
  grid_map::Position second;

  Edge(grid_map::Position f, grid_map::Position s) : first(std::move(f)), second(std::move(s)) {}

  /// @brief Sorting operator resulting in edges sorted from highest to lowest x values
  bool operator<(const Edge & e)
  {
    return first.x() > e.first.x() || (first.x() == e.first.x() && second.x() > e.second.x());
  }
};

/** @brief A polygon iterator for grid_map::GridMap based on the scan line algorithm.
    @details This iterator allows to iterate over all cells whose center is inside a polygon. \
             This reproduces the behavior of the original grid_map::PolygonIterator which uses\
             a "point in polygon" check for each cell of the gridmap, making it very expensive\
             to run on large maps. In comparison, the scan line algorithm implemented here is \
             much more scalable.
*/
class PolygonIterator
{
public:
  /// @brief Constructor.
  /// @details Calculate the indexes of the gridmap that are inside the polygon using the scan line
  /// algorithm.
  /// @param grid_map the grid map to iterate on.
  /// @param polygon the polygonal area to iterate on.
  PolygonIterator(const grid_map::GridMap & grid_map, const grid_map::Polygon & polygon);
  /// @brief Compare to another iterator.
  /// @param other other iterator.
  /// @return whether the current iterator points to a different address than the other one.
  bool operator!=(const PolygonIterator & other) const;
  /// @brief Dereference the iterator with const.
  /// @return the value to which the iterator is pointing.
  const grid_map::Index & operator*() const;
  /// @brief Increase the iterator to the next element.
  /// @return a reference to the updated iterator.
  PolygonIterator & operator++();
  /// @brief Indicates if iterator is past end.
  /// @return true if iterator is out of scope, false if end has not been reached.
  [[nodiscard]] bool isPastEnd() const;

private:
  /** @brief Calculate sorted edges of the given polygon.
      @details Vertices in an edge are ordered from higher to lower x.
              Edges are sorted in reverse lexicographical order of x.
      @param polygon Polygon for which edges are calculated.
      @return Sorted edges of the polygon.
  */
  static std::vector<Edge> calculateSortedEdges(const grid_map::Polygon & polygon);

  /// @brief Calculates intersections between lines (i.e., center of rows) and the polygon edges.
  /// @param edges Edges of the polygon.
  /// @param from_to_row ranges of lines to use for intersection.
  /// @param origin Position of the top-left cell in the grid map.
  /// @param grid_map grid map.
  /// @return for each row the vector of y values with an intersection.
  static std::vector<std::vector<double>> calculateIntersectionsPerLine(
    const std::vector<Edge> & edges, const std::pair<int, int> from_to_row,
    const grid_map::Position & origin, const grid_map::GridMap & grid_map);

  /// @brief Calculates the range of rows covering the given edges.
  /// @details The rows are calculated without any shift that might exist in the grid map.
  /// @param edges Edges of the polygon.
  /// @param origin Position of the top-left cell in the grid map.
  /// @param grid_map grid map.
  /// @return the range of rows as a pair {first row, last row}.
  static std::pair<int, int> calculateRowRange(
    const std::vector<Edge> & edges, const grid_map::Position & origin,
    const grid_map::GridMap & grid_map);

  // Helper functions
  /// @brief Increment the current_line_ to the line with intersections
  void goToNextLine();
  /// @brief Calculate the initial current_col_ and the current_to_col_ for the current intersection
  void calculateColumnIndexes();
  /// @brief Calculate the current_index_ from the current_line_ and current_col_
  void calculateIndex();

  /// Gridmap info
  int row_of_first_line_;
  grid_map::Index map_start_idx_;
  grid_map::Size map_size_;
  double map_resolution_;
  double map_origin_y_;
  /// Intersections between scan lines and the polygon
  std::vector<std::vector<double>> intersections_per_line_;
  std::vector<double>::const_iterator intersection_iter_;
  /// current indexes
  grid_map::Index current_index_;
  size_t current_line_;
  int current_col_;
  int current_to_col_;
};
}  // namespace grid_map_utils

#endif  // GRID_MAP_UTILS__POLYGON_ITERATOR_HPP_
