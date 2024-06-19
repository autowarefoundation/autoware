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

#ifndef YABLOC_PARTICLE_FILTER__GNSS_CORRECTOR__WEIGHT_MANAGER_HPP_
#define YABLOC_PARTICLE_FILTER__GNSS_CORRECTOR__WEIGHT_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>

namespace yabloc::modularized_particle_filter
{
struct WeightManager
{
  struct Parameter
  {
    float flat_radius_;
    float max_radius_;
    float max_weight_;
    float min_weight_;

    float coeff_;
    void compute_coeff()
    {
      coeff_ = -std::log(min_weight_ / max_weight_) / (max_radius_ * max_radius_);
    }
  };

  Parameter for_fixed_{};
  Parameter for_not_fixed_{};

  explicit WeightManager(rclcpp::Node * node)
  {
    for_fixed_.flat_radius_ =
      static_cast<float>(node->declare_parameter<float>("for_fixed/flat_radius"));
    for_fixed_.max_radius_ =
      static_cast<float>(node->declare_parameter<float>("for_fixed/max_radius"));
    for_fixed_.min_weight_ =
      static_cast<float>(node->declare_parameter<float>("for_fixed/min_weight"));
    for_fixed_.max_weight_ =
      static_cast<float>(node->declare_parameter<float>("for_fixed/max_weight"));
    for_fixed_.compute_coeff();

    for_not_fixed_.flat_radius_ =
      static_cast<float>(node->declare_parameter<float>("for_not_fixed/flat_radius"));
    for_not_fixed_.max_radius_ =
      static_cast<float>(node->declare_parameter<float>("for_not_fixed/max_radius"));
    for_not_fixed_.min_weight_ =
      static_cast<float>(node->declare_parameter<float>("for_not_fixed/min_weight"));
    for_not_fixed_.max_weight_ =
      static_cast<float>(node->declare_parameter<float>("for_not_fixed/max_weight"));
    for_not_fixed_.compute_coeff();
  }

  [[nodiscard]] static float normal_pdf(float distance, const Parameter & param)
  {
    // NOTE: This is not exact normal distribution because of no scale factor depending on sigma
    float d = std::clamp(std::abs(distance) - param.flat_radius_, 0.f, param.max_radius_);
    return param.max_weight_ * std::exp(-param.coeff_ * d * d);
  }

  [[nodiscard]] float normal_pdf(float distance, bool is_rtk_fixed) const
  {
    if (is_rtk_fixed) {
      return normal_pdf(distance, for_fixed_);
    }
    return normal_pdf(distance, for_not_fixed_);
  }

  [[nodiscard]] static float inverse_normal_pdf(float prob, const Parameter & param)
  {
    prob = (param.max_weight_ - param.min_weight_) * prob + param.min_weight_;

    if (prob > param.max_radius_) return param.max_radius_;
    if (prob < param.min_weight_) return param.flat_radius_ + param.max_radius_;
    return param.flat_radius_ + std::sqrt(-std::log(prob / param.max_weight_) / param.coeff_);
  }

  [[nodiscard]] float inverse_normal_pdf(float prob, bool is_rtk_fixed) const
  {
    if (is_rtk_fixed) {
      return inverse_normal_pdf(prob, for_fixed_);
    }
    return inverse_normal_pdf(prob, for_not_fixed_);
  }
};
}  // namespace yabloc::modularized_particle_filter

#endif  // YABLOC_PARTICLE_FILTER__GNSS_CORRECTOR__WEIGHT_MANAGER_HPP_
