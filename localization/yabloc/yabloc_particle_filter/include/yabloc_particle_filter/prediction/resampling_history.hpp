// Copyright 2023 TIER IV, Inc.
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

#ifndef YABLOC_PARTICLE_FILTER__PREDICTION__RESAMPLING_HISTORY_HPP_
#define YABLOC_PARTICLE_FILTER__PREDICTION__RESAMPLING_HISTORY_HPP_

#include <algorithm>
#include <numeric>
#include <vector>

namespace yabloc::modularized_particle_filter
{
class ResamplingHistory
{
public:
  ResamplingHistory(int max_history_num, int number_of_particles)
  : max_history_num_(max_history_num), number_of_particles_(number_of_particles)
  {
    resampling_history_.resize(max_history_num);

    for (auto & generation : resampling_history_) {
      generation.resize(number_of_particles);
      std::iota(generation.begin(), generation.end(), 0);
    }
  }

  [[nodiscard]] bool check_history_validity() const
  {
    for (auto & generation : resampling_history_) {
      bool result = std::any_of(generation.begin(), generation.end(), [this](int x) {
        return x < 0 || x >= number_of_particles_;
      });

      if (result) {
        return false;
      }
    }
    return true;
  }

  std::vector<int> & operator[](int generation_id)
  {
    return resampling_history_.at(generation_id % max_history_num_);
  }

  const std::vector<int> & operator[](int generation_id) const
  {
    return resampling_history_.at(generation_id % max_history_num_);
  }

private:
  // Number of updates to keep resampling history.
  // Resampling records prior to this will not be kept.
  const int max_history_num_;
  //
  const int number_of_particles_;
  //
  std::vector<std::vector<int>> resampling_history_;
};

}  // namespace yabloc::modularized_particle_filter

#endif  // YABLOC_PARTICLE_FILTER__PREDICTION__RESAMPLING_HISTORY_HPP_
