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

#include <bezier_sampler/bezier.hpp>

#include <iostream>

namespace bezier_sampler
{
Bezier::Bezier(Eigen::Matrix<double, 6, 2> control_points)
: control_points_(std::move(control_points))
{
}
Bezier::Bezier(const std::vector<Eigen::Vector2d> & control_points)
{
  if (control_points.size() != 6) {
    std::cerr << "Trying to initialize a quintic bezier curve with " << control_points.size()
              << " (!= 6) control points." << std::endl;
  } else {
    control_points_ << control_points[0], control_points[1], control_points[2], control_points[3],
      control_points[4], control_points[5];
  }
}

const Eigen::Matrix<double, 6, 2> & Bezier::getControlPoints() const
{
  return control_points_;
}

Eigen::Vector2d Bezier::value(const double t) const
{
  Eigen::Vector2d point = {0.0, 0.0};
  // sum( binomial(i in 5) * (1 - t)^(5-i) * t^i * control_points_[i] )
  point += std::pow((1 - t), 5) * control_points_.row(0);
  point += 5 * std::pow((1 - t), 4) * t * control_points_.row(1);
  point += 10 * std::pow((1 - t), 3) * t * t * control_points_.row(2);
  point += 10 * std::pow((1 - t), 2) * t * t * t * control_points_.row(3);
  point += 5 * (1 - t) * t * t * t * t * control_points_.row(4);
  point += t * t * t * t * t * control_points_.row(5);
  return point;
}

Eigen::Vector2d Bezier::valueM(const double t) const
{
  Eigen::Matrix<double, 1, 6> ts;
  ts << 1, t, t * t, t * t * t, t * t * t * t, t * t * t * t * t;
  return ts * quintic_bezier_coefficients * control_points_;
}

std::vector<Eigen::Vector2d> Bezier::cartesian(const int nb_points) const
{
  std::vector<Eigen::Vector2d> points;
  points.reserve(nb_points);
  const double step = 1.0 / (nb_points - 1);
  for (double t = 0.0; t <= 1.0; t += step) points.push_back(valueM(t));
  return points;
}

std::vector<Eigen::Vector2d> Bezier::cartesian(const double resolution) const
{
  std::vector<Eigen::Vector2d> points;
  points.reserve(static_cast<int>(1 / resolution));
  for (double t = 0.0; t <= 1.0; t += resolution) points.push_back(valueM(t));
  return points;
}

std::vector<Eigen::Vector3d> Bezier::cartesianWithHeading(const int nb_points) const
{
  std::vector<Eigen::Vector3d> points;
  points.reserve(nb_points);
  const double step = 1.0 / (nb_points - 1);
  for (double t = 0.0; t <= 1.0; t += step) {
    Eigen::Vector2d point = valueM(t);
    points.emplace_back(point.x(), point.y(), heading(t));
  }
  return points;
}

Eigen::Vector2d Bezier::velocity(const double t) const
{
  Eigen::Matrix<double, 1, 5> ts;
  ts << 1, t, t * t, t * t * t, t * t * t * t;
  return ts * quintic_bezier_velocity_coefficients * control_points_;
}

Eigen::Vector2d Bezier::acceleration(const double t) const
{
  Eigen::Matrix<double, 1, 4> ts;
  ts << 1, t, t * t, t * t * t;
  return ts * quintic_bezier_acceleration_coefficients * control_points_;
}

double Bezier::curvature(const double t) const
{
  const Eigen::Vector2d vel = velocity(t);
  const Eigen::Vector2d accel = acceleration(t);
  double curvature = std::numeric_limits<double>::infinity();
  const double denominator = std::pow(vel.x() * vel.x() + vel.y() * vel.y(), 3.0 / 2.0);
  if (denominator != 0) curvature = (vel.x() * accel.y() - accel.x() * vel.y()) / denominator;
  return curvature;
}

double Bezier::heading(const double t) const
{
  const Eigen::Vector2d vel = velocity(t);
  return std::atan2(vel.y(), vel.x());
}

}  // namespace bezier_sampler
