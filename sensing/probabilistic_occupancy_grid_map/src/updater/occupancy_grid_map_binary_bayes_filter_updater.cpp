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

#include "updater/occupancy_grid_map_binary_bayes_filter_updater.hpp"

#include "cost_value.hpp"

#include <algorithm>

namespace costmap_2d
{
inline unsigned char OccupancyGridMapBBFUpdater::applyBBF(
  const unsigned char & z, const unsigned char & o)
{
  constexpr float cost2p = 1.f / 255.f;
  const float po = o * cost2p;
  float pz{};
  float not_pz{};
  float po_hat{};
  if (z == occupancy_cost_value::LETHAL_OBSTACLE) {
    pz = probability_matrix_(Index::OCCUPIED, Index::OCCUPIED);
    not_pz = probability_matrix_(Index::FREE, Index::OCCUPIED);
    po_hat = ((po * pz) / ((po * pz) + ((1.f - po) * not_pz)));
  } else if (z == occupancy_cost_value::FREE_SPACE) {
    pz = 1.f - probability_matrix_(Index::FREE, Index::FREE);
    not_pz = 1.f - probability_matrix_(Index::OCCUPIED, Index::FREE);
    po_hat = ((po * pz) / ((po * pz) + ((1.f - po) * not_pz)));
  } else if (z == occupancy_cost_value::NO_INFORMATION) {
    constexpr float inv_v_ratio = 1.f / 10.f;
    po_hat = ((po + (0.5f * inv_v_ratio)) / ((1.f * inv_v_ratio) + 1.f));
  }
  return std::min(
    std::max(static_cast<unsigned char>(po_hat * 255.f + 0.5f), static_cast<unsigned char>(1)),
    static_cast<unsigned char>(254));
}

bool OccupancyGridMapBBFUpdater::update(const Costmap2D & single_frame_occupancy_grid_map)
{
  updateOrigin(
    single_frame_occupancy_grid_map.getOriginX(), single_frame_occupancy_grid_map.getOriginY());
  for (unsigned int x = 0; x < getSizeInCellsX(); x++) {
    for (unsigned int y = 0; y < getSizeInCellsY(); y++) {
      unsigned int index = getIndex(x, y);
      costmap_[index] = applyBBF(single_frame_occupancy_grid_map.getCost(x, y), costmap_[index]);
    }
  }
  return true;
}
}  // namespace costmap_2d
