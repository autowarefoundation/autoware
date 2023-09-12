// Copyright 2023 Tier IV, Inc.
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

#include "path_sampler/prepare_inputs.hpp"

#include "frenet_planner/structures.hpp"
#include "path_sampler/utils/geometry_utils.hpp"
#include "sampler_common/structures.hpp"
#include "sampler_common/transform/spline_transform.hpp"
#include "tier4_autoware_utils/geometry/boost_polygon_utils.hpp"

#include <eigen3/unsupported/Eigen/Splines>

#include <boost/geometry/geometry.hpp>

#include <algorithm>
#include <list>
#include <memory>
#include <vector>

namespace path_sampler
{

void prepareConstraints(
  sampler_common::Constraints & constraints, const PredictedObjects & predicted_objects,
  const std::vector<geometry_msgs::msg::Point> & left_bound,
  const std::vector<geometry_msgs::msg::Point> & right_bound)
{
  constraints.obstacle_polygons = sampler_common::MultiPolygon2d();
  for (const auto & o : predicted_objects.objects)
    if (o.kinematics.initial_twist_with_covariance.twist.linear.x < 0.5)  // TODO(Maxime): param
      constraints.obstacle_polygons.push_back(tier4_autoware_utils::toPolygon2d(o));
  constraints.dynamic_obstacles = {};  // TODO(Maxime): not implemented

  // TODO(Maxime): directly use lines instead of polygon
  sampler_common::Polygon2d drivable_area_polygon;
  for (const auto & p : right_bound) drivable_area_polygon.outer().emplace_back(p.x, p.y);
  for (auto it = left_bound.rbegin(); it != left_bound.rend(); ++it)
    drivable_area_polygon.outer().emplace_back(it->x, it->y);
  drivable_area_polygon.outer().push_back(drivable_area_polygon.outer().front());
  constraints.drivable_polygons = {drivable_area_polygon};
}

frenet_planner::SamplingParameters prepareSamplingParameters(
  const sampler_common::State & initial_state, const double base_length,
  const sampler_common::transform::Spline2D & path_spline, const Parameters & params)
{
  // calculate target lateral positions
  std::vector<double> target_lateral_positions;
  if (params.sampling.nb_target_lateral_positions > 1) {
    target_lateral_positions = {0.0, initial_state.frenet.d};
    double min_d = 0.0;
    double max_d = 0.0;
    double min_d_s = std::numeric_limits<double>::max();
    double max_d_s = std::numeric_limits<double>::max();
    for (const auto & drivable_poly : params.constraints.drivable_polygons) {
      for (const auto & p : drivable_poly.outer()) {
        const auto frenet_coordinates = path_spline.frenet(p);
        const auto d_s = std::abs(frenet_coordinates.s - initial_state.frenet.s);
        if (d_s < min_d_s && frenet_coordinates.d < 0.0) {
          min_d_s = d_s;
          min_d = frenet_coordinates.d;
        }
        if (d_s < max_d_s && frenet_coordinates.d > 0.0) {
          max_d_s = d_s;
          max_d = frenet_coordinates.d;
        }
      }
    }
    min_d += params.constraints.ego_width / 2.0;
    max_d -= params.constraints.ego_width / 2.0;
    if (min_d < max_d) {
      for (auto r = 0.0; r <= 1.0; r += 1.0 / (params.sampling.nb_target_lateral_positions - 1))
        target_lateral_positions.push_back(interpolation::lerp(min_d, max_d, r));
    }
  } else {
    target_lateral_positions = params.sampling.target_lateral_positions;
  }
  frenet_planner::SamplingParameters sampling_parameters;
  sampling_parameters.resolution = params.sampling.resolution;
  const auto max_s = path_spline.lastS();
  frenet_planner::SamplingParameter p;
  for (const auto target_length : params.sampling.target_lengths) {
    p.target_state.position.s = std::min(
      max_s, path_spline.frenet(initial_state.pose).s + std::max(0.0, target_length - base_length));
    for (const auto target_lateral_position : target_lateral_positions) {
      p.target_state.position.d = target_lateral_position;
      for (const auto target_lat_vel : params.sampling.frenet.target_lateral_velocities) {
        p.target_state.lateral_velocity = target_lat_vel;
        for (const auto target_lat_acc : params.sampling.frenet.target_lateral_accelerations) {
          p.target_state.lateral_acceleration = target_lat_acc;
          sampling_parameters.parameters.push_back(p);
        }
      }
    }
    if (p.target_state.position.s == max_s) break;
  }
  return sampling_parameters;
}

sampler_common::transform::Spline2D preparePathSpline(
  const std::vector<TrajectoryPoint> & path, const bool smooth_path)
{
  std::vector<double> x;
  std::vector<double> y;
  if (smooth_path) {
    // TODO(Maxime CLEMENT): this version using Eigen::Spline is unreliable and sometimes crashes
    constexpr auto spline_dim = 3;
    Eigen::MatrixXd control_points(path.size(), 2);
    for (auto i = 0lu; i < path.size(); ++i) {
      const auto & point = path[i];
      control_points(i, 0) = point.pose.position.x;
      control_points(i, 1) = point.pose.position.y;
    }
    control_points.transposeInPlace();
    const auto nb_knots = path.size() + spline_dim + 3;
    Eigen::RowVectorXd knots(nb_knots);
    constexpr auto repeat_end_knots = 3lu;
    const auto knot_step = 1.0 / static_cast<double>(nb_knots - 2 * repeat_end_knots);
    auto i = 0lu;
    for (; i < repeat_end_knots; ++i) knots[i] = 0.0;
    for (; i < nb_knots - repeat_end_knots; ++i) knots[i] = knots[i - 1] + knot_step;
    for (; i < nb_knots; ++i) knots[i] = 1.0;
    const auto spline = Eigen::Spline<double, 2, spline_dim>(knots, control_points);
    x.reserve(path.size());
    y.reserve(path.size());
    const auto t_step = 1 / static_cast<double>(path.size());
    for (auto t = 0.0; t < 1.0; t += t_step) {
      const auto p = spline(t);
      x.push_back(p.x());
      y.push_back(p.y());
    }
  } else {
    x.reserve(path.size());
    y.reserve(path.size());
    for (const auto & point : path) {
      x.push_back(point.pose.position.x);
      y.push_back(point.pose.position.y);
    }
  }
  return {x, y};
}
}  // namespace path_sampler
