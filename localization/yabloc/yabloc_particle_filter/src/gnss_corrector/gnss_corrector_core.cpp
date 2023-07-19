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

#include "yabloc_particle_filter/gnss_corrector/gnss_particle_corrector.hpp"

#include <yabloc_common/color.hpp>
#include <yabloc_common/pose_conversions.hpp>

namespace yabloc::modularized_particle_filter
{
GnssParticleCorrector::GnssParticleCorrector()
: AbstractCorrector("gnss_particle_corrector"),
  mahalanobis_distance_threshold_(declare_parameter<float>("mahalanobis_distance_threshold")),
  weight_manager_(this)
{
  using std::placeholders::_1;

  // Subscriber
  auto on_pose = std::bind(&GnssParticleCorrector::on_pose, this, _1);
  auto on_height = [this](const Float32 & height) { this->latest_height_ = height; };
  pose_sub_ = create_subscription<PoseCovStamped>("~/input/pose_with_covariance", 10, on_pose);
  height_sub_ = create_subscription<Float32>("~/input/height", 10, on_height);

  // Publisher
  marker_pub_ = create_publisher<MarkerArray>("~/debug/gnss_range_marker", 10);
}

bool GnssParticleCorrector::is_gnss_observation_valid(
  const Eigen::Matrix3f & sigma, const Eigen::Vector3f & mean_position,
  const Eigen::Vector3f & gnss_position)
{
  Eigen::Matrix3f inv_sigma = sigma.completeOrthogonalDecomposition().pseudoInverse();
  Eigen::Vector3f diff = gnss_position - mean_position;
  diff.z() = 0;

  float mahalanobis_distance = std::sqrt(diff.dot(inv_sigma * diff));
  RCLCPP_INFO_STREAM(get_logger(), "mahalanobis: " << mahalanobis_distance);

  if (mahalanobis_distance > mahalanobis_distance_threshold_) {
    RCLCPP_WARN_STREAM(
      get_logger(), "Mahalanobis distance is too large: " << mahalanobis_distance << ">"
                                                          << mahalanobis_distance_threshold_);
    return false;
  }
  return true;
}

void GnssParticleCorrector::on_pose(const PoseCovStamped::ConstSharedPtr pose_msg)
{
  const rclcpp::Time stamp = pose_msg->header.stamp;
  const auto & position = pose_msg->pose.pose.position;
  Eigen::Vector3f gnss_position;
  gnss_position << position.x, position.y, position.z;

  constexpr bool is_rtk_fixed = false;
  process(gnss_position, stamp, is_rtk_fixed);
}

void GnssParticleCorrector::process(
  const Eigen::Vector3f & gnss_position, const rclcpp::Time & stamp, const bool is_rtk_fixed)
{
  publish_marker(gnss_position, is_rtk_fixed);

  std::optional<ParticleArray> opt_particles = get_synchronized_particle_array(stamp);

  if (!opt_particles.has_value()) return;
  auto dt = (stamp - rclcpp::Time(opt_particles->header.stamp));
  if (std::abs(dt.seconds()) > 0.1) {
    RCLCPP_WARN_STREAM(
      get_logger(), "Timestamp gap between gnss and particles is too large: " << dt.seconds());
  }

  const Eigen::Matrix3f sigma = modularized_particle_filter::std_of_distribution(*opt_particles);
  const geometry_msgs::msg::Pose mean_pose = get_mean_pose(opt_particles.value());
  const Eigen::Vector3f mean_position = common::pose_to_affine(mean_pose).translation();

  // Check validity of GNSS measurement by mahalanobis distance
  if (!is_gnss_observation_valid(sigma, mean_position, gnss_position)) {
    return;
  }

  ParticleArray weighted_particles =
    weight_particles(opt_particles.value(), gnss_position, is_rtk_fixed);

  // Compute travel distance from last update position
  // If the distance is too short, skip weighting
  {
    Eigen::Vector3f mean_position = common::pose_to_affine(mean_pose).translation();
    if ((mean_position - last_mean_position_).squaredNorm() > 1) {
      this->set_weighted_particle_array(weighted_particles);
      last_mean_position_ = mean_position;
    } else {
      RCLCPP_WARN_STREAM_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Skip particle weighting due to almost the same position");
    }
  }
}

void GnssParticleCorrector::publish_marker(const Eigen::Vector3f & position, bool is_rtk_fixed)
{
  using namespace std::literals::chrono_literals;
  using Point = geometry_msgs::msg::Point;

  auto drawCircle = [](std::vector<Point> & points, float radius) -> void {
    const int N = 10;
    for (int theta = 0; theta < 2 * N + 1; theta++) {
      geometry_msgs::msg::Point pt;
      pt.x = radius * std::cos(theta * M_PI / N);
      pt.y = radius * std::sin(theta * M_PI / N);
      points.push_back(pt);
    }
  };

  MarkerArray array_msg;
  for (int i = 0; i < 5; i++) {
    Marker marker;
    marker.header.stamp = get_clock()->now();
    marker.header.frame_id = "/map";
    marker.id = i;
    marker.type = Marker::LINE_STRIP;
    marker.lifetime = rclcpp::Duration(500ms);
    marker.pose.position.x = position.x();
    marker.pose.position.y = position.y();
    marker.pose.position.z = latest_height_.data;

    float prob = i / 4.0f;
    marker.color = common::color_scale::rainbow(prob);
    marker.color.a = 0.5f;
    marker.scale.x = 0.1;
    drawCircle(marker.points, weight_manager_.inverse_normal_pdf(prob, is_rtk_fixed));
    array_msg.markers.push_back(marker);
  }
  marker_pub_->publish(array_msg);
}

GnssParticleCorrector::ParticleArray GnssParticleCorrector::weight_particles(
  const ParticleArray & predicted_particles, const Eigen::Vector3f & pose, bool is_rtk_fixed)
{
  ParticleArray weighted_particles{predicted_particles};

  for (auto & particle : weighted_particles.particles) {
    float distance = static_cast<float>(
      std::hypot(particle.pose.position.x - pose.x(), particle.pose.position.y - pose.y()));
    particle.weight = weight_manager_.normal_pdf(distance, is_rtk_fixed);
  }

  return weighted_particles;
}

}  // namespace yabloc::modularized_particle_filter
