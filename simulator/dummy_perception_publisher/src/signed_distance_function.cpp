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

#include "dummy_perception_publisher/signed_distance_function.hpp"

#include <tf2/LinearMath/Vector3.h>

#include <iostream>
#include <limits>

namespace signed_distance_function
{

double AbstractSignedDistanceFunction::getSphereTracingDist(
  double x_start, double y_start, double angle, double max_dist, double eps) const
{
  // https://computergraphics.stackexchange.com/questions/161/what-is-ray-marching-is-sphere-tracing-the-same-thing/163
  tf2::Vector3 direction(cos(angle), sin(angle), 0.0);

  const size_t max_iter = 40;

  const auto pos_start = tf2::Vector3(x_start, y_start, 0.0);

  auto ray_tip = pos_start;
  for (size_t itr = 0; itr < max_iter; ++itr) {
    const auto dist = this->operator()(ray_tip.getX(), ray_tip.getY());
    if (dist > max_dist) {
      return std::numeric_limits<double>::infinity();
    }
    ray_tip = ray_tip + dist * direction;
    bool almost_on_surface = std::abs(dist) < eps;
    if (almost_on_surface) {
      return tf2::tf2Distance(ray_tip, pos_start);
    }
  }
  // ray did not hit the surface.
  return std::numeric_limits<double>::infinity();
}

double BoxSDF::operator()(double x, double y) const
{
  const auto && vec_global = tf2::Vector3(x, y, 0.0);
  const auto vec_local = tf_local_to_global_(vec_global);

  // As for signed distance field for a box, please refere:
  // https://www.iquilezles.org/www/articles/distfunctions/distfunctions.htm
  const auto sd_val_x = std::abs(vec_local.getX()) - 0.5 * length_;
  const auto sd_val_y = std::abs(vec_local.getY()) - 0.5 * width_;
  const auto positive_dist_x = std::max(sd_val_x, 0.0);
  const auto positive_dist_y = std::max(sd_val_y, 0.0);

  const auto positive_dist = std::hypot(positive_dist_x, positive_dist_y);
  if (positive_dist > 0.0) {
    return positive_dist;
  }
  const auto negative_dist = std::min(std::max(sd_val_x, sd_val_y), 0.0);
  return negative_dist;
}

double CompositeSDF::operator()(double x, double y) const
{
  const size_t nearest_idx = nearest_sdf_index(x, y);
  return sdf_ptrs_.at(nearest_idx)->operator()(x, y);
}

size_t CompositeSDF::nearest_sdf_index(double x, double y) const
{
  double min_value = std::numeric_limits<double>::infinity();
  size_t idx_min{};
  for (size_t i = 0; i < sdf_ptrs_.size(); ++i) {
    const auto value = sdf_ptrs_.at(i)->operator()(x, y);
    if (value < min_value) {
      min_value = value;
      idx_min = i;
    }
  }
  return idx_min;
}

}  // namespace signed_distance_function
