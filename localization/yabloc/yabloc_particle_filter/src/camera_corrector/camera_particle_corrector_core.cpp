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

#include "yabloc_particle_filter/camera_corrector/camera_particle_corrector.hpp"
#include "yabloc_particle_filter/camera_corrector/logit.hpp"

#include <autoware/universe_utils/math/trigonometry.hpp>
#include <autoware/universe_utils/system/stop_watch.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <yabloc_common/color.hpp>
#include <yabloc_common/pose_conversions.hpp>
#include <yabloc_common/pub_sub.hpp>
#include <yabloc_common/transform_line_segments.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <cmath>

namespace yabloc::modularized_particle_filter
{

CameraParticleCorrector::CameraParticleCorrector(const rclcpp::NodeOptions & options)
: AbstractCorrector("camera_particle_corrector", options),
  min_prob_(static_cast<float>(declare_parameter<float>("min_prob"))),
  far_weight_gain_(static_cast<float>(declare_parameter<float>("far_weight_gain"))),
  cost_map_(this)
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  enable_switch_ = declare_parameter<bool>("enabled_at_first");

  // Publication
  pub_image_ = create_publisher<Image>("~/debug/match_image", 10);
  pub_map_image_ = create_publisher<Image>("~/debug/cost_map_image", 10);
  pub_marker_ = create_publisher<MarkerArray>("~/debug/cost_map_range", 10);
  pub_string_ = create_publisher<String>("~/debug/state_string", 10);
  pub_scored_cloud_ = create_publisher<PointCloud2>("~/debug/scored_cloud", 10);
  pub_scored_posteriori_cloud_ = create_publisher<PointCloud2>("~/debug/scored_post_cloud", 10);

  // Subscription
  auto on_line_segments = std::bind(&CameraParticleCorrector::on_line_segments, this, _1);
  auto on_ll2 = std::bind(&CameraParticleCorrector::on_ll2, this, _1);
  auto on_bounding_box = std::bind(&CameraParticleCorrector::on_bounding_box, this, _1);
  auto on_pose = std::bind(&CameraParticleCorrector::on_pose, this, _1);
  sub_line_segments_cloud_ =
    create_subscription<PointCloud2>("~/input/line_segments_cloud", 10, on_line_segments);
  sub_ll2_ = create_subscription<PointCloud2>("~/input/ll2_road_marking", 10, on_ll2);
  sub_bounding_box_ =
    create_subscription<PointCloud2>("~/input/ll2_bounding_box", 10, on_bounding_box);
  sub_pose_ = create_subscription<PoseStamped>("~/input/pose", 10, on_pose);

  auto on_service = std::bind(&CameraParticleCorrector::on_service, this, _1, _2);
  switch_service_ = create_service<SetBool>("~/switch_srv", on_service);

  // Timer callback
  auto on_timer = std::bind(&CameraParticleCorrector::on_timer, this);
  timer_ =
    rclcpp::create_timer(this, this->get_clock(), rclcpp::Rate(1).period(), std::move(on_timer));
}

void CameraParticleCorrector::on_pose(const PoseStamped & msg)
{
  latest_pose_ = msg;
}

void CameraParticleCorrector::on_service(
  SetBool::Request::ConstSharedPtr request, SetBool::Response::SharedPtr response)
{
  enable_switch_ = request->data;
  response->success = true;
  if (enable_switch_)
    RCLCPP_INFO_STREAM(get_logger(), "camera_corrector is enabled");
  else
    RCLCPP_INFO_STREAM(get_logger(), "camera_corrector is disabled");
}

void CameraParticleCorrector::on_bounding_box(const PointCloud2 & msg)
{
  // NOTE: Under construction
  pcl::PointCloud<pcl::PointXYZL> ll2_bounding_box;
  pcl::fromROSMsg(msg, ll2_bounding_box);
  cost_map_.set_bounding_box(ll2_bounding_box);
  RCLCPP_INFO_STREAM(get_logger(), "Set bounding box into cost map");
}

std::pair<CameraParticleCorrector::LineSegments, CameraParticleCorrector::LineSegments>
CameraParticleCorrector::split_line_segments(const PointCloud2 & msg)
{
  LineSegments all_line_segments_cloud;
  pcl::fromROSMsg(msg, all_line_segments_cloud);
  LineSegments reliable_cloud;
  LineSegments iffy_cloud;
  {
    for (const auto & p : all_line_segments_cloud) {
      if (p.label == 0)
        iffy_cloud.push_back(p);
      else
        reliable_cloud.push_back(p);
    }
  }

  auto [good_cloud, bad_cloud] = filt(iffy_cloud);
  {
    cv::Mat debug_image = cv::Mat::zeros(800, 800, CV_8UC3);
    auto draw = [&debug_image](const LineSegments & cloud, const cv::Scalar & color) -> void {
      for (const auto & line : cloud) {
        const Eigen::Vector3f p1 = line.getVector3fMap();
        const Eigen::Vector3f p2 = line.getNormalVector3fMap();
        cv::line(debug_image, cv2pt(p1), cv2pt(p2), color, 2);
      }
    };

    draw(reliable_cloud, cv::Scalar(0, 0, 255));
    draw(good_cloud, cv::Scalar(0, 255, 0));
    draw(bad_cloud, cv::Scalar(100, 100, 100));
    common::publish_image(*pub_image_, debug_image, msg.header.stamp);
  }

  return {reliable_cloud, good_cloud};
}

void CameraParticleCorrector::on_line_segments(const PointCloud2 & line_segments_msg)
{
  autoware::universe_utils::StopWatch stop_watch;
  const rclcpp::Time stamp = line_segments_msg.header.stamp;
  std::optional<ParticleArray> opt_array = this->get_synchronized_particle_array(stamp);
  if (!opt_array.has_value()) {
    return;
  }

  const rclcpp::Duration dt = (stamp - opt_array->header.stamp);
  if (std::abs(dt.seconds()) > 0.1) {
    const std::string text = "Timestamp gap between image and particles is LARGE ";
    RCLCPP_WARN_STREAM(get_logger(), text << dt.seconds());
  }

  auto [line_segments_cloud, iffy_line_segments_cloud] = split_line_segments(line_segments_msg);
  ParticleArray weighted_particles = opt_array.value();

  bool publish_weighted_particles = true;
  const Pose mean_pose = get_mean_pose(weighted_particles);
  {
    // Check travel distance and publish weights if it is enough long
    Eigen::Vector3f mean_position = common::pose_to_affine(mean_pose).translation();
    if ((mean_position - last_mean_position_).squaredNorm() > 1) {
      last_mean_position_ = mean_position;
    } else {
      using namespace std::literals::chrono_literals;
      publish_weighted_particles = false;
      RCLCPP_WARN_STREAM_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Skip particle weighting due to almost the same position");
    }
  }

  cost_map_.set_height(static_cast<float>(mean_pose.position.z));

  if (publish_weighted_particles) {
    for (auto & particle : weighted_particles.particles) {
      Sophus::SE3f transform = common::pose_to_se3(particle.pose);
      LineSegments transformed_line_segments =
        common::transform_line_segments(line_segments_cloud, transform);
      LineSegments transformed_iffy_line_segments =
        common::transform_line_segments(iffy_line_segments_cloud, transform);
      transformed_line_segments += transformed_iffy_line_segments;

      float logit = compute_logit(transformed_line_segments, transform.translation());
      particle.weight = logit_to_prob(logit, 0.01f);
    }

    if (enable_switch_) {
      this->set_weighted_particle_array(weighted_particles);
    }
  }

  cost_map_.erase_obsolete();  // NOTE:
  pub_marker_->publish(cost_map_.show_map_range());

  // DEBUG: just visualization
  {
    Sophus::SE3f transform = common::pose_to_se3(get_mean_pose(weighted_particles));

    pcl::PointCloud<pcl::PointXYZI> cloud = evaluate_cloud(
      common::transform_line_segments(line_segments_cloud, transform), transform.translation());
    pcl::PointCloud<pcl::PointXYZI> iffy_cloud = evaluate_cloud(
      common::transform_line_segments(iffy_line_segments_cloud, transform),
      transform.translation());

    pcl::PointCloud<pcl::PointXYZRGB> rgb_cloud;
    pcl::PointCloud<pcl::PointXYZRGB> rgb_iffy_cloud;

    float max_score = std::accumulate(
      cloud.begin(), cloud.end(), 0.0f,
      [](float max_score, const auto & p) { return std::max(max_score, std::abs(p.intensity)); });
    for (const auto p : cloud) {
      pcl::PointXYZRGB rgb;
      rgb.getVector3fMap() = p.getVector3fMap();
      rgb.rgba = static_cast<uint32_t>(common::color_scale::blue_red(p.intensity / max_score));
      rgb_cloud.push_back(rgb);
    }
    for (const auto p : iffy_cloud) {
      pcl::PointXYZRGB rgb;
      rgb.getVector3fMap() = p.getVector3fMap();
      rgb.rgba = static_cast<uint32_t>(common::color_scale::blue_red(p.intensity / max_score));
      rgb_iffy_cloud.push_back(rgb);
    }

    common::publish_cloud(*pub_scored_cloud_, rgb_cloud, line_segments_msg.header.stamp);
    common::publish_cloud(
      *pub_scored_posteriori_cloud_, rgb_iffy_cloud, line_segments_msg.header.stamp);
  }

  if (stop_watch.toc() > 0.080) {
    RCLCPP_WARN_STREAM(get_logger(), "on_line_segments: " << stop_watch.toc() * 1000.0 << "[ms]");
  } else {
    RCLCPP_INFO_STREAM(get_logger(), "on_line_segments: " << stop_watch.toc() * 1000.0 << "[ms]");
  }

  // Publish status as string
  {
    String msg;
    std::stringstream ss;
    ss << "-- Camera particle corrector --" << std::endl;
    ss << (enable_switch_ ? "ENABLED" : "disabled") << std::endl;
    ss << "time: " << stop_watch.toc() << std::endl;
    msg.data = ss.str();
    pub_string_->publish(msg);
  }
}

void CameraParticleCorrector::on_timer()
{
  if (latest_pose_.has_value())
    common::publish_image(
      *pub_map_image_, cost_map_.get_map_image(latest_pose_->pose), latest_pose_->header.stamp);
}

void CameraParticleCorrector::on_ll2(const PointCloud2 & ll2_msg)
{
  pcl::PointCloud<pcl::PointNormal> ll2_cloud;
  pcl::fromROSMsg(ll2_msg, ll2_cloud);
  cost_map_.set_cloud(ll2_cloud);
  RCLCPP_INFO_STREAM(get_logger(), "Set LL2 cloud into Hierarchical cost map");
}

float abs_cos(const Eigen::Vector3f & t, float deg)
{
  const auto radian = static_cast<float>(deg * M_PI / 180.0);
  Eigen::Vector2f x(t.x(), t.y());
  Eigen::Vector2f y(autoware::universe_utils::cos(radian), autoware::universe_utils::sin(radian));
  x.normalize();
  return std::abs(x.dot(y));
}

float CameraParticleCorrector::compute_logit(
  const LineSegments & line_segments_cloud, const Eigen::Vector3f & self_position)
{
  float logit = 0;
  for (const LineSegment & pn : line_segments_cloud) {
    const Eigen::Vector3f tangent = (pn.getNormalVector3fMap() - pn.getVector3fMap()).normalized();
    const float length = (pn.getVector3fMap() - pn.getNormalVector3fMap()).norm();

    for (float distance = 0; distance < length; distance += 0.1f) {
      Eigen::Vector3f p = pn.getVector3fMap() + tangent * distance;

      // NOTE: Close points are prioritized
      float squared_norm = (p - self_position).topRows(2).squaredNorm();
      float gain = std::exp(-far_weight_gain_ * squared_norm);  // 0 < gain < 1

      const CostMapValue v3 = cost_map_.at(p.topRows(2));

      if (v3.unmapped) {
        // logit does not change if target pixel is unmapped
        continue;
      }
      if (pn.label == 0) {  // posteriori
        logit +=
          0.2f * gain * (abs_cos(tangent, static_cast<float>(v3.angle)) * v3.intensity - 0.5f);
      } else {  // apriori
        logit += gain * (abs_cos(tangent, static_cast<float>(v3.angle)) * v3.intensity - 0.5f);
      }
    }
  }
  return logit;
}

pcl::PointCloud<pcl::PointXYZI> CameraParticleCorrector::evaluate_cloud(
  const LineSegments & line_segments_cloud, const Eigen::Vector3f & self_position)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  for (const LineSegment & pn : line_segments_cloud) {
    Eigen::Vector3f tangent = (pn.getNormalVector3fMap() - pn.getVector3fMap()).normalized();
    float length = (pn.getVector3fMap() - pn.getNormalVector3fMap()).norm();

    for (float distance = 0; distance < length; distance += 0.1f) {
      Eigen::Vector3f p = pn.getVector3fMap() + tangent * distance;

      // NOTE: Close points are prioritized
      float squared_norm = (p - self_position).topRows(2).squaredNorm();

      CostMapValue v3 = cost_map_.at(p.topRows(2));
      float logit = 0;
      if (!v3.unmapped) {
        float gain = std::exp(-far_weight_gain_ * squared_norm);

        logit = gain * (abs_cos(tangent, static_cast<float>(v3.angle)) * v3.intensity - 0.5f);
      }

      pcl::PointXYZI xyzi(logit_to_prob(logit, 10.f));
      xyzi.getVector3fMap() = p;
      cloud.push_back(xyzi);
    }
  }
  return cloud;
}
}  // namespace yabloc::modularized_particle_filter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(yabloc::modularized_particle_filter::CameraParticleCorrector)
