// Copyright 2022 TIER IV, Inc.
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

#include "obstacle_velocity_limiter/occupancy_grid_utils.hpp"

#include "obstacle_velocity_limiter/types.hpp"

#include <grid_map_core/Polygon.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <grid_map_cv/GridMapCvConverter.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_utils/polygon_iterator.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

namespace obstacle_velocity_limiter
{
void maskPolygons(grid_map::GridMap & grid_map, const ObstacleMasks & obstacle_masks)
{
  constexpr auto convert = [](const polygon_t & in) {
    grid_map::Polygon out;
    for (const auto & p : in.outer()) out.addVertex(p);
    return out;
  };

  auto & layer = grid_map["layer"];

  if (!obstacle_masks.positive_mask.outer().empty()) {
    const auto layer_copy = grid_map["layer"];
    layer.setConstant(0.0);
    grid_map::Position position;
    for (grid_map_utils::PolygonIterator iterator(grid_map, convert(obstacle_masks.positive_mask));
         !iterator.isPastEnd(); ++iterator)
      layer((*iterator)(0), (*iterator)(1)) = layer_copy((*iterator)(0), (*iterator)(1));
  }

  for (const auto & negative_mask : obstacle_masks.negative_masks)
    for (grid_map_utils::PolygonIterator iterator(grid_map, convert(negative_mask));
         !iterator.isPastEnd(); ++iterator)
      layer((*iterator)(0), (*iterator)(1)) = 0.0;
}

void threshold(grid_map::GridMap & grid_map, const float threshold)
{
  for (grid_map::GridMapIterator iter(grid_map); !iter.isPastEnd(); ++iter) {
    auto & val = grid_map.at("layer", *iter);
    if (val < threshold)
      val = 0.0;
    else
      val = 127;
  }
}

grid_map::GridMap convertToGridMap(const OccupancyGrid & occupancy_grid)
{
  grid_map::GridMap grid_map;
  grid_map::GridMapRosConverter::fromOccupancyGrid(occupancy_grid, "layer", grid_map);
  return grid_map;
}

multilinestring_t extractObstacles(
  const grid_map::GridMap & grid_map, const OccupancyGrid & occupancy_grid)
{
  cv::Mat cv_image;
  grid_map::GridMapCvConverter::toImage<unsigned char, 1>(grid_map, "layer", CV_8UC1, cv_image);
  cv::dilate(cv_image, cv_image, cv::Mat(), cv::Point(-1, -1), 2);
  cv::erode(cv_image, cv_image, cv::Mat(), cv::Point(-1, -1), 2);
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(cv_image, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
  multilinestring_t obstacles;
  const auto & info = occupancy_grid.info;
  for (const auto & contour : contours) {
    linestring_t line;
    for (const auto & point : contour) {
      line.emplace_back(
        (info.width - 1.0 - point.y) * info.resolution + info.origin.position.x,
        (info.height - 1.0 - point.x) * info.resolution + info.origin.position.y);
    }
    obstacles.emplace_back(line);
  }
  return obstacles;
}
}  // namespace obstacle_velocity_limiter
