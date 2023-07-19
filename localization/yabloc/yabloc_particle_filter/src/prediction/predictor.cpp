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

#include "yabloc_particle_filter/prediction/predictor.hpp"

#include "yabloc_particle_filter/common/mean.hpp"
#include "yabloc_particle_filter/common/prediction_util.hpp"
#include "yabloc_particle_filter/prediction/resampler.hpp"

#include <Eigen/Core>
#include <sophus/geometry.hpp>
#include <yabloc_common/pose_conversions.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/utils.h>

#include <numeric>

namespace yabloc::modularized_particle_filter
{

Predictor::Predictor()
: Node("predictor"),
  number_of_particles_(declare_parameter<int>("num_of_particles")),
  resampling_interval_seconds_(declare_parameter<float>("resampling_interval_seconds")),
  static_linear_covariance_(declare_parameter<float>("static_linear_covariance")),
  static_angular_covariance_(declare_parameter<float>("static_angular_covariance")),
  cov_xx_yy_{this->template declare_parameter<std::vector<double>>("cov_xx_yy")}
{
  tf2_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Publishers
  predicted_particles_pub_ = create_publisher<ParticleArray>("~/output/predicted_particles", 10);
  pose_pub_ = create_publisher<PoseStamped>("~/output/pose", 10);
  pose_cov_pub_ = create_publisher<PoseCovStamped>("~/output/pose_with_covariance", 10);
  marker_pub_ = create_publisher<Marker>("~/debug/init_marker", 10);

  // Subscribers
  using std::placeholders::_1;
  auto on_initial = std::bind(&Predictor::on_initial_pose, this, _1);
  auto on_twist_cov = std::bind(&Predictor::on_twist_cov, this, _1);
  auto on_particle = std::bind(&Predictor::on_weighted_particles, this, _1);
  auto on_height = [this](std_msgs::msg::Float32 m) -> void { this->ground_height_ = m.data; };

  initialpose_sub_ = create_subscription<PoseCovStamped>("~/input/initialpose", 1, on_initial);
  particles_sub_ =
    create_subscription<ParticleArray>("~/input/weighted_particles", 10, on_particle);
  height_sub_ = create_subscription<std_msgs::msg::Float32>("~/input/height", 10, on_height);
  twist_cov_sub_ =
    create_subscription<TwistCovStamped>("~/input/twist_with_covariance", 10, on_twist_cov);

  // Timer callback
  const double prediction_rate = declare_parameter<double>("prediction_rate");
  auto on_timer = std::bind(&Predictor::on_timer, this);
  timer_ = rclcpp::create_timer(
    this, this->get_clock(), rclcpp::Rate(prediction_rate).period(), std::move(on_timer));

  // Optional modules
  if (declare_parameter<bool>("visualize", false)) {
    visualizer_ptr_ = std::make_unique<ParticleVisualizer>(*this);
  }
}

void Predictor::on_initial_pose(const PoseCovStamped::ConstSharedPtr initialpose)
{
  // Publish initial pose marker
  auto position = initialpose->pose.pose.position;
  Eigen::Vector3f pos_vec3f;
  pos_vec3f << position.x, position.y, position.z;

  auto orientation = initialpose->pose.pose.orientation;
  float theta = 2 * std::atan2(orientation.z, orientation.w);
  Eigen::Vector3f tangent;
  tangent << std::cos(theta), std::sin(theta), 0;

  publish_range_marker(pos_vec3f, tangent);

  // Rectify initial pose
  auto initialpose_rectified = rectify_initial_pose(pos_vec3f, tangent, *initialpose);

  // Initialize particles given the initial pose and its covariance
  initialize_particles(initialpose_rectified);
}

void Predictor::initialize_particles(const PoseCovStamped & initialpose)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "initialize_particles");
  yabloc_particle_filter::msg::ParticleArray particle_array{};
  particle_array.header = initialpose.header;
  particle_array.id = 0;
  particle_array.particles.resize(number_of_particles_);

  Eigen::Matrix2d cov;
  cov(0, 0) = initialpose.pose.covariance[6 * 0 + 0];
  cov(0, 1) = initialpose.pose.covariance[6 * 0 + 1];
  cov(1, 0) = initialpose.pose.covariance[6 * 1 + 0];
  cov(1, 1) = initialpose.pose.covariance[6 * 1 + 1];

  const double yaw = tf2::getYaw(initialpose.pose.pose.orientation);
  const double yaw_std = std::sqrt(initialpose.pose.covariance[6 * 5 + 5]);

  for (auto & particle : particle_array.particles) {
    geometry_msgs::msg::Pose pose = initialpose.pose.pose;
    const Eigen::Vector2d noise = util::nrand_2d(cov);
    pose.position.x += noise.x();
    pose.position.y += noise.y();

    float noised_yaw = util::normalize_radian(yaw + util::nrand(yaw_std));
    pose.orientation.w = std::cos(noised_yaw / 2.0);
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = std::sin(noised_yaw / 2.0);

    particle.pose = pose;
    particle.weight = 1.0;
  }
  particle_array_opt_ = particle_array;

  // We have to initialize resampler every particles initialization,
  // because resampler has particles resampling history and it will be outdate.
  resampler_ptr_ = std::make_unique<RetroactiveResampler>(number_of_particles_, 100);
}

void Predictor::on_twist_cov(const TwistCovStamped::ConstSharedPtr twist_cov)
{
  const auto twist = twist_cov->twist;
  TwistCovStamped twist_covariance;
  twist_covariance.header = twist_cov->header;
  twist_covariance.twist.twist = twist.twist;
  twist_covariance.twist.covariance.at(0) = static_linear_covariance_;
  twist_covariance.twist.covariance.at(7) = 1e4;
  twist_covariance.twist.covariance.at(14) = 1e4;
  twist_covariance.twist.covariance.at(21) = 1e4;
  twist_covariance.twist.covariance.at(28) = 1e4;
  twist_covariance.twist.covariance.at(35) = static_angular_covariance_;
  latest_twist_opt_ = twist_covariance;
}

void Predictor::update_with_dynamic_noise(
  ParticleArray & particle_array, const TwistCovStamped & twist, double dt)
{
  // linear & angular velocity
  const float linear_x = twist.twist.twist.linear.x;
  const float angular_z = twist.twist.twist.angular.z;
  // standard deviation of linear & angular velocity
  const float std_linear_x = std::sqrt(twist.twist.covariance[6 * 0 + 0]);
  const float std_angular_z = std::sqrt(twist.twist.covariance[6 * 5 + 5]);
  // 1[rad/s] = 60[deg/s]
  // 1[m/s] = 3.6[km/h]
  const float truncated_angular_std =
    std_angular_z * std::clamp(std::sqrt(std::abs(linear_x)), 0.1f, 1.0f);
  const float truncated_linear_std = std::clamp(std_linear_x * linear_x, 0.1f, 2.0f);

  for (auto & particle : particle_array.particles) {
    Sophus::SE3f se3_pose = common::pose_to_se3(particle.pose);
    Eigen::Matrix<float, 6, 1> noised_xi;
    noised_xi.setZero();
    noised_xi(0) = linear_x + util::nrand(truncated_linear_std);
    noised_xi(5) = angular_z + util::nrand(truncated_angular_std);
    se3_pose *= Sophus::SE3f::exp(noised_xi * dt);

    geometry_msgs::msg::Pose pose = common::se3_to_pose(se3_pose);
    pose.position.z = ground_height_;
    particle.pose = pose;
  }
}

void Predictor::on_timer()
{
  // ==========================================================================
  // Pre-check section
  // Return if particle_array is not initialized yet
  if (!particle_array_opt_.has_value()) {
    return;
  }
  // Return if twist is not subscribed yet
  if (!latest_twist_opt_.has_value()) {
    return;
  }
  //
  ParticleArray particle_array = particle_array_opt_.value();
  const rclcpp::Time current_time = this->now();
  const rclcpp::Time msg_time = particle_array.header.stamp;
  const double dt = (current_time - msg_time).seconds();
  particle_array.header.stamp = current_time;

  // ==========================================================================
  // Prediction section
  // NOTE: Sometimes particle_array.header.stamp is ancient due to lagged pose_initializer
  if (dt < 0.0 || dt > 1.0) {
    RCLCPP_WARN_STREAM(get_logger(), "time stamp is wrong? " << dt);
    particle_array_opt_->header.stamp = current_time;
    return;
  }

  update_with_dynamic_noise(particle_array, latest_twist_opt_.value(), dt);

  // ==========================================================================
  // Post-process section
  //
  predicted_particles_pub_->publish(particle_array);
  //
  publish_mean_pose(get_mean_pose(particle_array), this->now());
  // If visualizer exists,
  if (visualizer_ptr_) {
    visualizer_ptr_->publish(particle_array);
  }

  particle_array_opt_ = particle_array;
}

void Predictor::on_weighted_particles(const ParticleArray::ConstSharedPtr weighted_particles_ptr)
{
  // NOTE: **We need not to check particle_array_opt.has_value().**
  // Since the weighted_particles is generated from messages published from this node,
  // the particle_array must have an entity in this function.
  ParticleArray particle_array = particle_array_opt_.value();

  // ==========================================================================
  // From here, weighting section
  try {
    particle_array =
      resampler_ptr_->add_weight_retroactively(particle_array, *weighted_particles_ptr);
  } catch (const resampling_skip_exception & e) {
    // Do nothing (just skipping the resample())
    RCLCPP_INFO_STREAM(this->get_logger(), "skipped resampling");
  }

  // ==========================================================================
  // From here, resampling section
  const double current_time = rclcpp::Time(particle_array.header.stamp).seconds();
  try {
    // Exit if previous resampling time is not valid.
    if (!previous_resampling_time_opt_.has_value()) {
      previous_resampling_time_opt_ = current_time;
      throw resampling_skip_exception("previous resampling time is not valid");
    }

    if (current_time - previous_resampling_time_opt_.value() <= resampling_interval_seconds_) {
      throw resampling_skip_exception("it is not time to resample");
    }

    particle_array = resampler_ptr_->resample(particle_array);
    previous_resampling_time_opt_ = current_time;
  } catch (const resampling_skip_exception & e) {
    void();
    // Do nothing (just skipping the resample())
  }

  // ==========================================================================
  particle_array_opt_ = particle_array;
}

void Predictor::publish_mean_pose(
  const geometry_msgs::msg::Pose & mean_pose, const rclcpp::Time & stamp)
{
  // Publish pose
  {
    PoseStamped pose_stamped;
    pose_stamped.header.stamp = stamp;
    pose_stamped.header.frame_id = "map";
    pose_stamped.pose = mean_pose;
    pose_pub_->publish(pose_stamped);
  }

  // Publish pose with covariance
  {
    // TODO(KYabuuchi) Use particle distribution someday
    PoseCovStamped pose_cov_stamped;
    pose_cov_stamped.header.stamp = stamp;
    pose_cov_stamped.header.frame_id = "map";
    pose_cov_stamped.pose.pose = mean_pose;
    pose_cov_stamped.pose.covariance[6 * 0 + 0] = 0.255;
    pose_cov_stamped.pose.covariance[6 * 1 + 1] = 0.255;
    pose_cov_stamped.pose.covariance[6 * 2 + 2] = 0.255;
    pose_cov_stamped.pose.covariance[6 * 3 + 3] = 0.00625;
    pose_cov_stamped.pose.covariance[6 * 4 + 4] = 0.00625;
    pose_cov_stamped.pose.covariance[6 * 5 + 5] = 0.00625;

    pose_cov_pub_->publish(pose_cov_stamped);
  }

  // Publish TF
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = particle_array_opt_->header.stamp;
    transform.header.frame_id = "map";
    transform.child_frame_id = "particle_filter";
    transform.transform.translation.x = mean_pose.position.x;
    transform.transform.translation.y = mean_pose.position.y;
    transform.transform.translation.z = mean_pose.position.z;
    transform.transform.rotation = mean_pose.orientation;
    tf2_broadcaster_->sendTransform(transform);
  }
}

void Predictor::publish_range_marker(const Eigen::Vector3f & pos, const Eigen::Vector3f & tangent)
{
  Marker msg;
  msg.type = Marker::LINE_STRIP;
  msg.header.stamp = get_clock()->now();
  msg.header.frame_id = "map";
  msg.scale.x = 0.2;
  msg.scale.y = 0.2;

  msg.color.r = 0;
  msg.color.g = 1;
  msg.color.b = 0;
  msg.color.a = 1;

  auto cast2gp = [](const Eigen::Vector3f & vec3f) -> geometry_msgs::msg::Point {
    geometry_msgs::msg::Point p;
    p.x = vec3f.x();
    p.y = vec3f.y();
    p.z = vec3f.z();
    return p;
  };

  Eigen::Vector3f binormal;
  binormal << -tangent.y(), tangent.x(), tangent.z();

  msg.points.push_back(cast2gp(pos + tangent + binormal));
  msg.points.push_back(cast2gp(pos + tangent - binormal));
  msg.points.push_back(cast2gp(pos - tangent - binormal));
  msg.points.push_back(cast2gp(pos - tangent + binormal));
  msg.points.push_back(cast2gp(pos + tangent + binormal));

  marker_pub_->publish(msg);
}

Predictor::PoseCovStamped Predictor::rectify_initial_pose(
  const Eigen::Vector3f & pos, const Eigen::Vector3f & tangent,
  const PoseCovStamped & raw_initialpose) const
{
  PoseCovStamped msg = raw_initialpose;
  msg.header.frame_id = "map";
  msg.pose.pose.position.x = pos.x();
  msg.pose.pose.position.y = pos.y();
  msg.pose.pose.position.z = pos.z();

  float theta = std::atan2(tangent.y(), tangent.x());

  msg.pose.pose.orientation.w = std::cos(theta / 2);
  msg.pose.pose.orientation.x = 0;
  msg.pose.pose.orientation.y = 0;
  msg.pose.pose.orientation.z = std::sin(theta / 2);

  Eigen::Matrix2f cov;
  cov << cov_xx_yy_.at(0), 0, 0, cov_xx_yy_.at(1);
  Eigen::Rotation2D r(theta);
  cov = r * cov * r.inverse();

  msg.pose.covariance.at(6 * 0 + 0) = cov(0, 0);
  msg.pose.covariance.at(6 * 0 + 1) = cov(0, 1);
  msg.pose.covariance.at(6 * 1 + 0) = cov(1, 0);
  msg.pose.covariance.at(6 * 1 + 1) = cov(1, 1);
  msg.pose.covariance.at(6 * 5 + 5) = 0.0076;  // 0.0076 = (5deg)^2

  return msg;
}

}  // namespace yabloc::modularized_particle_filter
