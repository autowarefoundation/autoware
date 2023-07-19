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

#ifndef YABLOC_PARTICLE_FILTER__GNSS_CORRECTOR__GNSS_PARTICLE_CORRECTOR_HPP_
#define YABLOC_PARTICLE_FILTER__GNSS_CORRECTOR__GNSS_PARTICLE_CORRECTOR_HPP_

#include "yabloc_particle_filter/gnss_corrector/weight_manager.hpp"

#include <Eigen/Core>
#include <yabloc_particle_filter/correction/abstract_corrector.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float32.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace yabloc::modularized_particle_filter
{
class GnssParticleCorrector : public AbstractCorrector
{
public:
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using PoseCovStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
  using NavSatFix = sensor_msgs::msg::NavSatFix;
  using Marker = visualization_msgs::msg::Marker;
  using MarkerArray = visualization_msgs::msg::MarkerArray;
  using Float32 = std_msgs::msg::Float32;

  GnssParticleCorrector();

private:
  const float mahalanobis_distance_threshold_;
  const WeightManager weight_manager_;

  rclcpp::Subscription<Float32>::SharedPtr height_sub_;
  rclcpp::Subscription<PoseCovStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr marker_pub_;

  Float32 latest_height_;
  Eigen::Vector3f last_mean_position_;

  void on_pose(const PoseCovStamped::ConstSharedPtr pose_msg);

  void process(
    const Eigen::Vector3f & gnss_position, const rclcpp::Time & stamp, const bool is_rtk_fixed);

  bool is_gnss_observation_valid(
    const Eigen::Matrix3f & sigma, const Eigen::Vector3f & mean_position,
    const Eigen::Vector3f & gnss_position);

  ParticleArray weight_particles(
    const ParticleArray & predicted_particles, const Eigen::Vector3f & pose, bool is_rtk_fixed);

  // unstable feature
  void add_weight_by_orientation(
    ParticleArray & weighted_particles, const Eigen::Vector3f & velocity);

  void publish_marker(const Eigen::Vector3f & position, bool fixed);
};
}  // namespace yabloc::modularized_particle_filter

#endif  // YABLOC_PARTICLE_FILTER__GNSS_CORRECTOR__GNSS_PARTICLE_CORRECTOR_HPP_
