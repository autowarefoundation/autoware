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

#ifndef UPDATER__OCCUPANCY_GRID_MAP_BINARY_BAYES_FILTER_UPDATER_HPP_
#define UPDATER__OCCUPANCY_GRID_MAP_BINARY_BAYES_FILTER_UPDATER_HPP_

#include "updater/occupancy_grid_map_updater_interface.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace costmap_2d
{
class OccupancyGridMapBBFUpdater : public OccupancyGridMapUpdaterInterface
{
public:
  enum Index : size_t { OCCUPIED = 0U, FREE = 1U };
  OccupancyGridMapBBFUpdater(
    const unsigned int cells_size_x, const unsigned int cells_size_y, const float resolution);
  bool update(const Costmap2D & single_frame_occupancy_grid_map) override;
  void initRosParam(rclcpp::Node & node) override;

private:
  inline unsigned char applyBBF(const unsigned char & z, const unsigned char & o);
  Eigen::Matrix2f probability_matrix_;
  double v_ratio_;
};

}  // namespace costmap_2d

#endif  // UPDATER__OCCUPANCY_GRID_MAP_BINARY_BAYES_FILTER_UPDATER_HPP_
