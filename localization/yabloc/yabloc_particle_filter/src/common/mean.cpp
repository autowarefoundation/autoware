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

#include "yabloc_particle_filter/common/mean.hpp"

#include <Eigen/Geometry>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <yabloc_common/pose_conversions.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <numeric>

namespace yabloc::modularized_particle_filter
{
namespace
{
double mean_radian(const std::vector<double> & angles, const std::vector<double> & weights)
{
  std::complex<double> c{};
  for (int i{0}; i < static_cast<int>(angles.size()); i++) {
    c += weights[i] * std::polar(1.0, angles[i]);
  }
  std::complex<double> cw{std::accumulate(weights.begin(), weights.end(), 0.0)};
  return std::arg(c / cw);
}
}  // namespace

geometry_msgs::msg::Pose get_mean_pose(
  const yabloc_particle_filter::msg::ParticleArray & particle_array)
{
  using Pose = geometry_msgs::msg::Pose;
  using Particle = yabloc_particle_filter::msg::Particle;

  Pose mean_pose;

  double sum_weight{std::accumulate(
    particle_array.particles.begin(), particle_array.particles.end(), 0.0,
    [](double weight, const Particle & particle) { return weight + particle.weight; })};

  if (std::isinf(sum_weight)) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("meanPose"), "sum_weight: " << sum_weight);
  }

  std::vector<double> rolls{};
  std::vector<double> pitches{};
  std::vector<double> yaws{};
  std::vector<double> normalized_weights{};
  for (const Particle & particle : particle_array.particles) {
    double normalized_weight = particle.weight / sum_weight;

    mean_pose.position.x += particle.pose.position.x * normalized_weight;
    mean_pose.position.y += particle.pose.position.y * normalized_weight;
    mean_pose.position.z += particle.pose.position.z * normalized_weight;

    double yaw{0.0};
    double pitch{0.0};
    double roll{0.0};
    tf2::getEulerYPR(particle.pose.orientation, yaw, pitch, roll);

    rolls.push_back(roll);
    pitches.push_back(pitch);
    yaws.push_back(yaw);
    normalized_weights.push_back(normalized_weight);
  }

  const double mean_roll{mean_radian(rolls, normalized_weights)};
  const double mean_pitch{mean_radian(pitches, normalized_weights)};
  const double mean_yaw{mean_radian(yaws, normalized_weights)};

  tf2::Quaternion q;
  q.setRPY(mean_roll, mean_pitch, mean_yaw);
  mean_pose.orientation = tf2::toMsg(q);
  return mean_pose;
}

Eigen::Matrix3f std_of_distribution(const yabloc_particle_filter::msg::ParticleArray & array)
{
  using Particle = yabloc_particle_filter::msg::Particle;
  auto ori = get_mean_pose(array).orientation;
  Eigen::Quaternionf orientation(
    static_cast<float>(ori.w), static_cast<float>(ori.x), static_cast<float>(ori.y),
    static_cast<float>(ori.z));
  float inv_n = 1.f / static_cast<float>(array.particles.size());
  Eigen::Vector3f mean = Eigen::Vector3f::Zero();
  for (const Particle & p : array.particles) {
    Eigen::Affine3f affine = common::pose_to_affine(p.pose);
    mean += affine.translation();
  }
  mean *= inv_n;

  Eigen::Matrix3f sigma = Eigen::Matrix3f::Zero();
  for (const Particle & p : array.particles) {
    Eigen::Affine3f affine = common::pose_to_affine(p.pose);
    Eigen::Vector3f d = affine.translation() - mean;
    d = orientation.conjugate() * d;
    sigma += (d * d.transpose()) * inv_n;
  }

  return sigma;
}
}  // namespace yabloc::modularized_particle_filter
