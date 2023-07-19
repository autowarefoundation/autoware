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

#include "yabloc_image_processing/segment_filter/segment_filter.hpp"

#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <yabloc_common/cv_decompress.hpp>
#include <yabloc_common/pub_sub.hpp>

#include <pcl_conversions/pcl_conversions.h>

namespace yabloc::segment_filter
{
SegmentFilter::SegmentFilter()
: Node("segment_filter"),
  image_size_(declare_parameter<int>("image_size")),
  max_range_(declare_parameter<float>("max_range")),
  min_segment_length_(declare_parameter<float>("min_segment_length")),
  max_segment_distance_(declare_parameter<float>("max_segment_distance")),
  max_lateral_distance_(declare_parameter<float>("max_lateral_distance")),
  info_(this),
  synchro_subscriber_(this, "~/input/line_segments_cloud", "~/input/graph_segmented"),
  tf_subscriber_(this->get_clock())
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  auto cb = std::bind(&SegmentFilter::execute, this, _1, _2);
  synchro_subscriber_.set_callback(std::move(cb));

  pub_projected_cloud_ =
    create_publisher<PointCloud2>("~/output/projected_line_segments_cloud", 10);
  pub_debug_cloud_ = create_publisher<PointCloud2>("~/debug/debug_line_segments", 10);
  pub_image_ = create_publisher<Image>("~/debug/projected_image", 10);
}

cv::Point2i SegmentFilter::to_cv_point(const Eigen::Vector3f & v) const
{
  cv::Point pt;
  pt.x = -v.y() / max_range_ * image_size_ * 0.5f + image_size_ / 2;
  pt.y = -v.x() / max_range_ * image_size_ * 0.5f + image_size_;
  return pt;
}

bool SegmentFilter::define_project_func()
{
  if (project_func_) return true;

  if (info_.is_camera_info_nullopt()) return false;
  Eigen::Matrix3f intrinsic_inv = info_.intrinsic().inverse();

  std::optional<Eigen::Affine3f> camera_extrinsic =
    tf_subscriber_(info_.get_frame_id(), "base_link");
  if (!camera_extrinsic.has_value()) return false;

  const Eigen::Vector3f t = camera_extrinsic->translation();
  const Eigen::Quaternionf q(camera_extrinsic->rotation());

  // TODO(KYabuuchi) This will take into account ground tilt and camera vibration someday.
  project_func_ = [intrinsic_inv, q,
                   t](const Eigen::Vector3f & u) -> std::optional<Eigen::Vector3f> {
    Eigen::Vector3f u3(u.x(), u.y(), 1);
    Eigen::Vector3f u_bearing = (q * intrinsic_inv * u3).normalized();
    if (u_bearing.z() > -0.01) return std::nullopt;
    float u_distance = -t.z() / u_bearing.z();
    Eigen::Vector3f v;
    v.x() = t.x() + u_bearing.x() * u_distance;
    v.y() = t.y() + u_bearing.y() * u_distance;
    v.z() = 0;
    return v;
  };
  return true;
}

void SegmentFilter::execute(const PointCloud2 & line_segments_msg, const Image & segment_msg)
{
  if (!define_project_func()) {
    using namespace std::literals::chrono_literals;
    RCLCPP_INFO_STREAM_THROTTLE(
      get_logger(), *get_clock(), (1000ms).count(), "project_func cannot be defined");
    return;
  }

  const rclcpp::Time stamp = line_segments_msg.header.stamp;

  pcl::PointCloud<pcl::PointNormal>::Ptr line_segments_cloud{
    new pcl::PointCloud<pcl::PointNormal>()};
  cv::Mat mask_image = common::decompress_to_cv_mat(segment_msg);
  pcl::fromROSMsg(line_segments_msg, *line_segments_cloud);

  const std::set<int> indices = filter_by_mask(mask_image, *line_segments_cloud);

  pcl::PointCloud<pcl::PointNormal> valid_edges = project_lines(*line_segments_cloud, indices);
  pcl::PointCloud<pcl::PointNormal> invalid_edges =
    project_lines(*line_segments_cloud, indices, true);

  // Projected line segments
  {
    pcl::PointCloud<pcl::PointXYZLNormal> combined_edges;
    for (const auto & pn : valid_edges) {
      pcl::PointXYZLNormal pln;
      pln.getVector3fMap() = pn.getVector3fMap();
      pln.getNormalVector3fMap() = pn.getNormalVector3fMap();
      pln.label = 255;
      combined_edges.push_back(pln);
    }
    for (const auto & pn : invalid_edges) {
      pcl::PointXYZLNormal pln;
      pln.getVector3fMap() = pn.getVector3fMap();
      pln.getNormalVector3fMap() = pn.getNormalVector3fMap();
      pln.label = 0;
      combined_edges.push_back(pln);
    }
    common::publish_cloud(*pub_projected_cloud_, combined_edges, stamp);
  }

  // Image
  {
    cv::Mat projected_image = cv::Mat::zeros(cv::Size{image_size_, image_size_}, CV_8UC3);
    for (auto & pn : valid_edges) {
      cv::Point2i p1 = to_cv_point(pn.getVector3fMap());
      cv::Point2i p2 = to_cv_point(pn.getNormalVector3fMap());
      cv::line(projected_image, p1, p2, cv::Scalar(100, 100, 255), 4, cv::LineTypes::LINE_8);
    }
    for (auto & pn : invalid_edges) {
      cv::Point2i p1 = to_cv_point(pn.getVector3fMap());
      cv::Point2i p2 = to_cv_point(pn.getNormalVector3fMap());
      cv::line(projected_image, p1, p2, cv::Scalar(200, 200, 200), 3, cv::LineTypes::LINE_8);
    }
    common::publish_image(*pub_image_, projected_image, stamp);
  }

  // Line segments for debug
  {
    pcl::PointCloud<pcl::PointXYZLNormal> combined_debug_edges;
    for (size_t index = 0; index < line_segments_cloud->size(); ++index) {
      const pcl::PointNormal & pn = line_segments_cloud->at(index);
      pcl::PointXYZLNormal pln;
      pln.getVector3fMap() = pn.getVector3fMap();
      pln.getNormalVector3fMap() = pn.getNormalVector3fMap();
      if (indices.count(index) > 0)
        pln.label = 255;
      else
        pln.label = 0;
      combined_debug_edges.push_back(pln);
    }
    common::publish_cloud(*pub_debug_cloud_, combined_debug_edges, stamp);
  }
}

bool SegmentFilter::is_near_element(
  const pcl::PointNormal & pn, pcl::PointNormal & truncated_pn) const
{
  float min_distance = std::min(pn.x, pn.normal_x);
  float max_distance = std::max(pn.x, pn.normal_x);
  if (min_distance > max_segment_distance_) return false;
  if (max_distance < max_segment_distance_) {
    truncated_pn = pn;
    return true;
  }

  truncated_pn = pn;
  Eigen::Vector3f t = pn.getVector3fMap() - pn.getNormalVector3fMap();
  float not_zero_tx = t.x() > 0 ? std::max(t.x(), 1e-3f) : std::min(t.x(), -1e-3f);
  float lambda = (max_segment_distance_ - pn.x) / not_zero_tx;
  Eigen::Vector3f m = pn.getVector3fMap() + lambda * t;
  if (pn.x > pn.normal_x)
    truncated_pn.getVector3fMap() = m;
  else
    truncated_pn.getNormalVector3fMap() = m;
  return true;
}

std::set<ushort> get_unique_pixel_value(cv::Mat & image)
{
  // `image` is a set of ushort.
  // The purpose is to find the unique set of values contained in `image`.
  // For example, if `image` is {0,1,2,0,1,2,3}, this function returns {0,1,2,3}.

  if (image.depth() != CV_16U) throw std::runtime_error("image's depth must be ushort");

  auto begin = image.begin<ushort>();
  auto last = std::unique(begin, image.end<ushort>());
  std::sort(begin, last);
  last = std::unique(begin, last);
  return std::set<ushort>(begin, last);
}

pcl::PointCloud<pcl::PointNormal> SegmentFilter::project_lines(
  const pcl::PointCloud<pcl::PointNormal> & points, const std::set<int> & indices,
  bool negative) const
{
  pcl::PointCloud<pcl::PointNormal> projected_points;
  for (size_t index = 0; index < points.size(); ++index) {
    if (negative) {
      if (indices.count(index) > 0) continue;
    } else {
      if (indices.count(index) == 0) continue;
    }

    pcl::PointNormal truncated_pn = points.at(index);

    std::optional<Eigen::Vector3f> opt1 = project_func_(truncated_pn.getVector3fMap());
    std::optional<Eigen::Vector3f> opt2 = project_func_(truncated_pn.getNormalVector3fMap());
    if (!opt1.has_value()) continue;
    if (!opt2.has_value()) continue;

    // If line segment has shorter length than config, it is excluded
    if (min_segment_length_ > 0) {
      float length = (opt1.value() - opt2.value()).norm();
      if (length < min_segment_length_) continue;
    }
    if (max_lateral_distance_ > 0) {
      float abs_lateral1 = std::abs(opt1.value().y());
      float abs_lateral2 = std::abs(opt2.value().y());
      if (std::min(abs_lateral1, abs_lateral2) > max_lateral_distance_) continue;
    }

    pcl::PointNormal xyz;
    xyz.x = opt1->x();
    xyz.y = opt1->y();
    xyz.z = opt1->z();
    xyz.normal_x = opt2->x();
    xyz.normal_y = opt2->y();
    xyz.normal_z = opt2->z();

    //
    pcl::PointNormal truncated_xyz = xyz;
    if (max_segment_distance_ > 0)
      if (!is_near_element(xyz, truncated_xyz)) continue;

    projected_points.push_back(truncated_xyz);
  }
  return projected_points;
}

std::set<int> SegmentFilter::filter_by_mask(
  const cv::Mat & mask, const pcl::PointCloud<pcl::PointNormal> & edges)
{
  // Create line image and assign different color to each segment.
  cv::Mat line_image = cv::Mat::zeros(mask.size(), CV_16UC1);
  for (size_t i = 0; i < edges.size(); i++) {
    auto & pn = edges.at(i);
    Eigen::Vector3f p1 = pn.getVector3fMap();
    Eigen::Vector3f p2 = pn.getNormalVector3fMap();
    cv::Scalar color = cv::Scalar::all(i + 1);
    cv::line(
      line_image, cv::Point2i(p1.x(), p1.y()), cv::Point2i(p2.x(), p2.y()), color, 1,
      cv::LineTypes::LINE_4);
  }

  cv::Mat mask_image;
  mask.convertTo(mask_image, CV_16UC1);
  cv::threshold(mask_image, mask_image, 1, std::numeric_limits<ushort>::max(), cv::THRESH_BINARY);

  // TODO(KYabuuchi) Using boost::geometry is more intuitive.
  // https://boostjp.github.io/tips/geometry.html#disjoint

  // And operator
  cv::Mat masked_line;
  cv::bitwise_and(mask_image, line_image, masked_line);
  std::set<ushort> pixel_values = get_unique_pixel_value(masked_line);

  // Extract edges within masks
  std::set<int> reliable_indices;
  for (size_t i = 0; i < edges.size(); i++) {
    if (pixel_values.count(i + 1) != 0) reliable_indices.insert(i);
  }

  return reliable_indices;
}

}  // namespace yabloc::segment_filter
