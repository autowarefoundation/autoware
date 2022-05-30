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

#include "grid_map_core/TypeDefs.hpp"
#include "grid_map_cv/GridMapCvConverter.hpp"
#include "grid_map_cv/GridMapCvProcessing.hpp"
#include "grid_map_utils/polygon_iterator.hpp"

#include <grid_map_core/iterators/PolygonIterator.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <tier4_autoware_utils/system/stop_watch.hpp>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <random>
#include <string>
#include <vector>

int main(int argc, char * argv[])
{
  bool visualize = false;
  for (int i = 1; i < argc; ++i) {
    const auto arg = std::string(argv[i]);
    if (arg == "-v" || arg == "--visualize") {
      visualize = true;
    }
  }
  std::ofstream result_file;
  result_file.open("benchmark_results.csv");
  result_file
    << "#Size PolygonVertices PolygonIndexes grid_map_utils_constructor grid_map_utils_iteration "
       "grid_map_constructor grid_map_iteration\n";
  tier4_autoware_utils::StopWatch<std::chrono::milliseconds> stopwatch;

  constexpr auto nb_iterations = 10;
  constexpr auto polygon_side_vertices =
    25;  // number of vertex per side of the square base_polygon
  const auto grid_map_length = grid_map::Length(10.0, 10.0);
  std::random_device r;
  std::default_random_engine engine(0);
  // TODO(Maxime CLEMENT): moving breaks the polygon visualization
  std::uniform_real_distribution move_dist(-2.0, 2.0);
  std::uniform_real_distribution poly_x_offset(-2.0, 2.0);
  std::uniform_real_distribution poly_y_offset(-2.0, 2.0);

  grid_map::Polygon base_polygon;
  const auto top_left = grid_map::Position(-grid_map_length.x() / 2, grid_map_length.y() / 2);
  const auto top_right = grid_map::Position(grid_map_length.x() / 2, grid_map_length.y() / 2);
  const auto bot_right = grid_map::Position(grid_map_length.x() / 2, -grid_map_length.y() / 2);
  const auto bot_left = grid_map::Position(-grid_map_length.x() / 2, -grid_map_length.y() / 2);
  const auto top_vector = top_right - top_left;
  for (double i = 0; i < polygon_side_vertices; ++i) {
    const auto factor = i / polygon_side_vertices;
    base_polygon.addVertex(top_left + factor * top_vector);
  }
  const auto right_vector = bot_right - top_right;
  for (double i = 0; i < polygon_side_vertices; ++i) {
    const auto factor = i / polygon_side_vertices;
    base_polygon.addVertex(top_right + factor * right_vector);
  }
  const auto bot_vector = bot_left - bot_right;
  for (double i = 0; i < polygon_side_vertices; ++i) {
    const auto factor = i / polygon_side_vertices;
    base_polygon.addVertex(bot_right + factor * bot_vector);
  }
  const auto left_vector = top_left - bot_left;
  for (double i = 0; i < polygon_side_vertices; ++i) {
    const auto factor = i / polygon_side_vertices;
    base_polygon.addVertex(bot_left + factor * left_vector);
  }

  for (auto grid_map_size = 100; grid_map_size <= 1000; grid_map_size += 100) {
    std::cout << "Map of size " << grid_map_size << " by " << grid_map_size << std::endl;
    const auto resolution = grid_map_length(0) / grid_map_size;
    grid_map::GridMap map({"layer"});
    map.setGeometry(grid_map_length, resolution);
    for (auto vertices = 3ul; vertices <= base_polygon.nVertices(); ++vertices) {
      auto polygon_indexes = 0.0;
      std::cout << "\tPolygon with " << vertices << " vertices" << std::endl;

      double grid_map_utils_constructor_duration{};
      double grid_map_constructor_duration{};
      double grid_map_utils_iteration_duration{};
      double grid_map_iteration_duration{};

      for (auto iteration = 0; iteration < nb_iterations; ++iteration) {
        map.setGeometry(grid_map::Length(10.0, 10.0), resolution, grid_map::Position(0.0, 0.0));
        const auto move = grid_map::Position(move_dist(engine), move_dist(engine));
        map.move(move);

        // generate random sub-polygon of base_polygon with some noise
        grid_map::Polygon polygon;
        std::vector<size_t> indexes(base_polygon.nVertices());
        for (size_t i = 0; i <= base_polygon.nVertices(); ++i) indexes[i] = i;
        std::shuffle(indexes.begin(), indexes.end(), std::default_random_engine(iteration));
        indexes.resize(vertices);
        std::sort(indexes.begin(), indexes.end());
        for (const auto idx : indexes) {
          const auto offset = grid_map::Position(poly_x_offset(engine), poly_y_offset(engine));
          polygon.addVertex(base_polygon.getVertex(idx) + offset);
        }
        stopwatch.tic("gmu_ctor");
        grid_map_utils::PolygonIterator grid_map_utils_iterator(map, polygon);
        grid_map_utils_constructor_duration += stopwatch.toc("gmu_ctor");
        stopwatch.tic("gm_ctor");
        grid_map::PolygonIterator grid_map_iterator(map, polygon);
        grid_map_constructor_duration += stopwatch.toc("gm_ctor");
        bool diff = false;
        while (!grid_map_utils_iterator.isPastEnd() && !grid_map_iterator.isPastEnd()) {
          stopwatch.tic("gmu_iter");
          const auto gmu_idx = *grid_map_utils_iterator;
          ++grid_map_utils_iterator;
          grid_map_utils_iteration_duration += stopwatch.toc("gmu_iter");
          stopwatch.tic("gm_iter");
          const auto gm_idx = *grid_map_iterator;
          ++grid_map_iterator;
          grid_map_iteration_duration += stopwatch.toc("gm_iter");
          ++polygon_indexes;

          if (gmu_idx.x() != gm_idx.x() || gmu_idx.y() != gm_idx.y()) {
            diff = true;
          }
        }
        if (grid_map_iterator.isPastEnd() != grid_map_utils_iterator.isPastEnd()) {
          diff = true;
        }
        if (diff || visualize) {
          // Prepare images of the cells selected by the two PolygonIterators
          auto gridmap = map;
          for (grid_map_utils::PolygonIterator iterator(map, polygon); !iterator.isPastEnd();
               ++iterator)
            map.at("layer", *iterator) = 100;
          for (grid_map::PolygonIterator iterator(gridmap, polygon); !iterator.isPastEnd();
               ++iterator)
            gridmap.at("layer", *iterator) = 100;

          cv::Mat img;
          cv::Mat custom_img;
          cv::Mat gm_img;
          cv::Mat diff_img;

          grid_map::GridMapCvConverter::toImage<unsigned char, 1>(
            map, "layer", CV_8UC1, 0.0, 100, img);
          cv::resize(img, custom_img, cv::Size(500, 500), cv::INTER_LINEAR);
          grid_map::GridMapCvConverter::toImage<unsigned char, 1>(
            gridmap, "layer", CV_8UC1, 0.0, 100, img);
          cv::resize(img, gm_img, cv::Size(500, 500), cv::INTER_LINEAR);
          cv::compare(custom_img, gm_img, diff_img, cv::CMP_EQ);

          cv::imshow("custom", custom_img);
          cv::imshow("grid_map", gm_img);
          cv::imshow("diff", diff_img);
          cv::waitKey(0);
          cv::destroyAllWindows();
        }
      }
      // print results to file
      result_file << grid_map_size << " " << vertices << " " << polygon_indexes / nb_iterations
                  << " " << grid_map_utils_constructor_duration / nb_iterations << " "
                  << grid_map_utils_iteration_duration / nb_iterations << " "
                  << grid_map_constructor_duration / nb_iterations << " "
                  << grid_map_iteration_duration / nb_iterations << "\n";
    }
  }
  result_file.close();
}
