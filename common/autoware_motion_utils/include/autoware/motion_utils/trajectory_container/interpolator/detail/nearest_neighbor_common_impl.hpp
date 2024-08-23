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

// clang-format off
#ifndef AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__INTERPOLATOR__DETAIL__NEAREST_NEIGHBOR_COMMON_IMPL_HPP_  // NOLINT
#define AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__INTERPOLATOR__DETAIL__NEAREST_NEIGHBOR_COMMON_IMPL_HPP_  // NOLINT
// clang-format on

#include "autoware/motion_utils/trajectory_container/interpolator/interpolator.hpp"

#include <vector>

namespace autoware::motion_utils::trajectory_container::interpolator::detail
{
/**
 * @brief Common Implementation of nearest neighbor.
 *
 * This class implements the core functionality for nearest neighbor interpolation.
 *
 * @tparam T The type of the values being interpolated.
 */
template <typename T>
class NearestNeighborCommonImpl : public Interpolator<T>
{
protected:
  std::vector<T> values_;  ///< Interpolation values.

  /**
   * @brief Compute the interpolated value at the given point.
   *
   * @param s The point at which to compute the interpolated value.
   * @return The interpolated value.
   */
  [[nodiscard]] T compute_impl(const double & s) const override
  {
    const int32_t idx = this->get_index(s);
    return (std::abs(s - this->axis_[idx]) <= std::abs(s - this->axis_[idx + 1]))
             ? this->values_.at(idx)
             : this->values_.at(idx + 1);
  }

  /**
   * @brief Build the interpolator with the given values.
   *
   * @param axis The axis values.
   * @param values The values to interpolate.
   * @return True if the interpolator was built successfully, false otherwise.
   */
  void build_impl(
    const Eigen::Ref<const Eigen::VectorXd> & axis, const std::vector<T> & values) override
  {
    this->axis_ = axis;
    this->values_ = values;
  }

public:
  /**
   * @brief Get the minimum number of required points for the interpolator.
   *
   * @return The minimum number of required points.
   */
  [[nodiscard]] size_t minimum_required_points() const override { return 1; }
};

}  // namespace autoware::motion_utils::trajectory_container::interpolator::detail

// clang-format off
#endif  // AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__INTERPOLATOR__DETAIL__NEAREST_NEIGHBOR_COMMON_IMPL_HPP_  // NOLINT
// clang-format on
