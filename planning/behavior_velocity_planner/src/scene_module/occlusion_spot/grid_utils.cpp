// Copyright 2021 Tier IV, Inc.
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

#include <grid_map_ros/GridMapRosConverter.hpp>
#include <scene_module/occlusion_spot/grid_utils.hpp>

#include <algorithm>
#include <vector>

namespace behavior_velocity_planner
{
namespace grid_utils
{
bool isOcclusionSpotSquare(
  OcclusionSpotSquare & occlusion_spot, const grid_map::Matrix & grid_data,
  const grid_map::Index & cell, int side_size, const grid_map::Size & grid_size)
{
  // Calculate ranges to check
  int min_x;
  int max_x;
  int min_y;
  int max_y;
  // No occlusion_spot with size 0
  if (side_size == 0) {
    return false;
  }
  /**
   * @brief
   *   (min_x,min_y)...(max_x,min_y)
   *        .               .
   *   (min_x,max_y)...(max_x,max_y)
   */
  const int offset = side_size - 1;
  min_x = cell.x();
  max_x = cell.x() + offset;
  min_y = cell.y() - offset;
  max_y = cell.y();
  // Ensure we stay inside the grid
  min_x = std::max(0, min_x);
  max_x = std::min(grid_size.x() - 1, max_x);
  min_y = std::max(0, min_y);
  max_y = std::min(grid_size.y() - 1, max_y);
  for (int x = min_x; x <= max_x; ++x) {
    for (int y = min_y; y <= max_y; ++y) {
      // if the value is not unknown value return false
      if (grid_data(x, y) != grid_utils::occlusion_cost_value::UNKNOWN) {
        return false;
      }
    }
  }
  occlusion_spot.side_size = side_size;
  occlusion_spot.index = cell;
  return true;
}

void findOcclusionSpots(
  std::vector<grid_map::Position> & occlusion_spot_positions, const grid_map::GridMap & grid,
  const lanelet::BasicPolygon2d & polygon, double min_size)
{
  const grid_map::Matrix & grid_data = grid["layer"];
  const int min_occlusion_spot_size = std::max(0.0, std::floor(min_size / grid.getResolution()));
  grid_map::Polygon grid_polygon;
  for (const auto & point : polygon) {
    grid_polygon.addVertex({point.x(), point.y()});
  }
  for (grid_map::PolygonIterator iterator(grid, grid_polygon); !iterator.isPastEnd(); ++iterator) {
    OcclusionSpotSquare occlusion_spot_square;
    if (isOcclusionSpotSquare(
          occlusion_spot_square, grid_data, *iterator, min_occlusion_spot_size, grid.getSize())) {
      if (!grid.getPosition(occlusion_spot_square.index, occlusion_spot_square.position)) {
        continue;
      }
      std::vector<grid_map::Position> corner_positions;
      getCornerPositions(corner_positions, grid, occlusion_spot_square);
      for (const grid_map::Position & corner : corner_positions) {
        occlusion_spot_positions.emplace_back(corner);
      }
    }
  }
}

bool isCollisionFree(
  const grid_map::GridMap & grid, const grid_map::Position & p1, const grid_map::Position & p2)
{
  const grid_map::Matrix & grid_data = grid["layer"];
  try {
    for (grid_map::LineIterator iterator(grid, p1, p2); !iterator.isPastEnd(); ++iterator) {
      const grid_map::Index & index = *iterator;
      if (grid_data(index.x(), index.y()) == grid_utils::occlusion_cost_value::OCCUPIED) {
        return false;
      }
    }
  } catch (const std::invalid_argument & e) {
    std::cerr << e.what() << std::endl;
  }
  return true;
}

void getCornerPositions(
  std::vector<grid_map::Position> & corner_positions, const grid_map::GridMap & grid,
  const OcclusionSpotSquare & occlusion_spot_square)
{
  // Special case with size = 1: only one cell
  if (occlusion_spot_square.side_size == 1) {
    corner_positions.emplace_back(occlusion_spot_square.position);
    return;
  }
  std::vector<grid_map::Index> corner_indexes;
  const int offset = (occlusion_spot_square.side_size - 1) / 2;
  /**
   * @brief relation of each grid position
   *    bl br
   *    tl tr
   */
  corner_indexes = {// bl
                    grid_map::Index(
                      std::max(0, occlusion_spot_square.index.x() - offset),
                      std::max(0, occlusion_spot_square.index.y() - offset)),
                    // br
                    grid_map::Index(
                      std::min(grid.getSize().x() - 1, occlusion_spot_square.index.x() + offset),
                      std::max(0, occlusion_spot_square.index.y() - offset)),
                    // tl
                    grid_map::Index(
                      std::max(0, occlusion_spot_square.index.x() - offset),
                      std::min(grid.getSize().y() - 1, occlusion_spot_square.index.y() + offset)),
                    // tr
                    grid_map::Index(
                      std::min(grid.getSize().x() - 1, occlusion_spot_square.index.x() + offset),
                      std::min(grid.getSize().y() - 1, occlusion_spot_square.index.y() + offset))};
  for (const grid_map::Index & corner_index : corner_indexes) {
    grid_map::Position corner_position;
    grid.getPosition(corner_index, corner_position);
    corner_positions.emplace_back(corner_position);
  }
}
void imageToOccupancyGrid(const cv::Mat & cv_image, nav_msgs::msg::OccupancyGrid * occupancy_grid)
{
  const int width = cv_image.cols;
  const int height = cv_image.rows;
  occupancy_grid->data.clear();
  occupancy_grid->data.resize(width * height);
  for (int x = width - 1; x >= 0; x--) {
    for (int y = height - 1; y >= 0; y--) {
      const int idx = (height - 1 - y) + (width - 1 - x) * height;
      const unsigned char intensity = cv_image.at<unsigned char>(y, x);
      occupancy_grid->data.at(idx) = intensity;
    }
  }
}
void toQuantizedImage(
  const nav_msgs::msg::OccupancyGrid & occupancy_grid, cv::Mat * cv_image, const GridParam & param)
{
  const int width = cv_image->cols;
  const int height = cv_image->rows;
  for (int x = width - 1; x >= 0; x--) {
    for (int y = height - 1; y >= 0; y--) {
      const int idx = (height - 1 - y) + (width - 1 - x) * height;
      int8_t intensity = occupancy_grid.data.at(idx);
      if (0 <= intensity && intensity < param.free_space_max) {
        intensity = grid_utils::occlusion_cost_value::FREE_SPACE;
      } else if (  // NOLINT
        intensity == occlusion_cost_value::NO_INFORMATION || intensity < param.occupied_min) {
        intensity = grid_utils::occlusion_cost_value::UNKNOWN;
      } else {
        intensity = grid_utils::occlusion_cost_value::OCCUPIED;
      }
      cv_image->at<unsigned char>(y, x) = intensity;
    }
  }
}
void denoiseOccupancyGridCV(
  nav_msgs::msg::OccupancyGrid & occupancy_grid, grid_map::GridMap & grid_map,
  const GridParam & param)
{
  cv::Mat cv_image(
    occupancy_grid.info.width, occupancy_grid.info.height, CV_8UC1,
    cv::Scalar(grid_utils::occlusion_cost_value::OCCUPIED));
  toQuantizedImage(occupancy_grid, &cv_image, param);
  constexpr int num_iter = 2;
  //!< @brief opening & closing to remove noise in occupancy grid
  cv::dilate(cv_image, cv_image, cv::Mat(), cv::Point(-1, -1), num_iter);
  cv::erode(cv_image, cv_image, cv::Mat(), cv::Point(-1, -1), num_iter);
  imageToOccupancyGrid(cv_image, &occupancy_grid);
  grid_map::GridMapRosConverter::fromOccupancyGrid(occupancy_grid, "layer", grid_map);
}
}  // namespace grid_utils
}  // namespace behavior_velocity_planner
