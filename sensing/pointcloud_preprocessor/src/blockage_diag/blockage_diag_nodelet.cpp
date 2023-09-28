// Copyright 2022 TIER IV, Inc.
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

#include "pointcloud_preprocessor/blockage_diag/blockage_diag_nodelet.hpp"

#include "autoware_point_types/types.hpp"

#include <boost/circular_buffer.hpp>

#include <algorithm>
#include <numeric>

namespace pointcloud_preprocessor
{
using autoware_point_types::PointXYZIRADRT;
using diagnostic_msgs::msg::DiagnosticStatus;

BlockageDiagComponent::BlockageDiagComponent(const rclcpp::NodeOptions & options)
: Filter("BlockageDiag", options)
{
  {
    // initialize params:
    horizontal_ring_id_ = declare_parameter<int>("horizontal_ring_id");
    blockage_ratio_threshold_ = declare_parameter<float>("blockage_ratio_threshold");
    vertical_bins_ = declare_parameter<int>("vertical_bins");
    angle_range_deg_ = declare_parameter<std::vector<double>>("angle_range");
    lidar_model_ = declare_parameter<std::string>("model");
    blockage_count_threshold_ = declare_parameter<int>("blockage_count_threshold");
    blockage_buffering_frames_ = declare_parameter<int>("blockage_buffering_frames");
    blockage_buffering_interval_ = declare_parameter<int>("blockage_buffering_interval");
    dust_ratio_threshold_ = declare_parameter<float>("dust_ratio_threshold");
    dust_count_threshold_ = declare_parameter<int>("dust_count_threshold");
    dust_kernel_size_ = declare_parameter<int>("dust_kernel_size");
    dust_buffering_frames_ = declare_parameter<int>("dust_buffering_frames");
    dust_buffering_interval_ = declare_parameter<int>("dust_buffering_interval");
  }

  updater_.setHardwareID("blockage_diag");
  updater_.add(
    std::string(this->get_namespace()) + ": blockage_validation", this,
    &BlockageDiagComponent::onBlockageChecker);
  updater_.add(
    std::string(this->get_namespace()) + ": dust_validation", this,
    &BlockageDiagComponent::dustChecker);
  updater_.setPeriod(0.1);
  single_frame_dust_mask_pub =
    image_transport::create_publisher(this, "blockage_diag/debug/single_frame_dust_mask_image");
  multi_frame_dust_mask_pub =
    image_transport::create_publisher(this, "blockage_diag/debug/multi_frame_dust_mask_image");
  lidar_depth_map_pub_ =
    image_transport::create_publisher(this, "blockage_diag/debug/lidar_depth_map");
  blockage_mask_pub_ =
    image_transport::create_publisher(this, "blockage_diag/debug/blockage_mask_image");
  blockage_dust_merged_pub =
    image_transport::create_publisher(this, "blockage_diag/debug/blockage_dust_merged_image");
  ground_blockage_ratio_pub_ = create_publisher<tier4_debug_msgs::msg::Float32Stamped>(
    "blockage_diag/debug/ground_blockage_ratio", rclcpp::SensorDataQoS());
  sky_blockage_ratio_pub_ = create_publisher<tier4_debug_msgs::msg::Float32Stamped>(
    "blockage_diag/debug/sky_blockage_ratio", rclcpp::SensorDataQoS());
  ground_dust_ratio_pub_ = create_publisher<tier4_debug_msgs::msg::Float32Stamped>(
    "blockage_diag/debug/ground_dust_ratio", rclcpp::SensorDataQoS());
  using std::placeholders::_1;
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&BlockageDiagComponent::paramCallback, this, _1));
}

void BlockageDiagComponent::onBlockageChecker(DiagnosticStatusWrapper & stat)
{
  stat.add("ground_blockage_ratio", std::to_string(ground_blockage_ratio_));
  stat.add("ground_blockage_count", std::to_string(ground_blockage_count_));
  stat.add(
    "ground_blockage_range_deg", "[" + std::to_string(ground_blockage_range_deg_[0]) + "," +
                                   std::to_string(ground_blockage_range_deg_[1]) + "]");
  stat.add("sky_blockage_ratio", std::to_string(sky_blockage_ratio_));
  stat.add("sky_blockage_count", std::to_string(sky_blockage_count_));
  stat.add(
    "sky_blockage_range_deg", "[" + std::to_string(sky_blockage_range_deg_[0]) + "," +
                                std::to_string(sky_blockage_range_deg_[1]) + "]");
  // TODO(badai-nguyen): consider sky_blockage_ratio_ for DiagnosticsStatus." [todo]

  auto level = DiagnosticStatus::OK;
  std::string msg;
  if (ground_blockage_ratio_ < 0) {
    level = DiagnosticStatus::STALE;
    msg = "STALE";
  } else if (
    (ground_blockage_ratio_ > blockage_ratio_threshold_) &&
    (ground_blockage_count_ > blockage_count_threshold_)) {
    level = DiagnosticStatus::ERROR;
    msg = "ERROR";
  } else if (ground_blockage_ratio_ > 0.0f) {
    level = DiagnosticStatus::WARN;
    msg = "WARN";
  } else {
    level = DiagnosticStatus::OK;
    msg = "OK";
  }

  if ((ground_blockage_ratio_ > 0.0f) && (sky_blockage_ratio_ > 0.0f)) {
    msg = msg + ": LIDAR both blockage";
  } else if (ground_blockage_ratio_ > 0.0f) {
    msg = msg + ": LIDAR ground blockage";
  } else if (sky_blockage_ratio_ > 0.0f) {
    msg = msg + ": LIDAR sky blockage";
  }
  stat.summary(level, msg);
}

void BlockageDiagComponent::dustChecker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  stat.add("ground_dust_ratio", std::to_string(ground_dust_ratio_));
  auto level = DiagnosticStatus::OK;
  std::string msg;
  if (ground_dust_ratio_ < 0.0f) {
    level = DiagnosticStatus::STALE;
    msg = "STALE";
  } else if (
    (ground_dust_ratio_ > dust_ratio_threshold_) && (dust_frame_count_ > dust_count_threshold_)) {
    level = DiagnosticStatus::ERROR;
    msg = "ERROR";
  } else if (ground_dust_ratio_ > 0.0f) {
    level = DiagnosticStatus::WARN;
    msg = "WARN";
  } else {
    level = DiagnosticStatus::OK;
    msg = "OK";
  }

  if (ground_dust_ratio_ > 0.0f) {
    msg = msg + ": LIDAR ground dust";
  }
  stat.summary(level, msg);
}

void BlockageDiagComponent::filter(
  const PointCloud2ConstPtr & input, [[maybe_unused]] const IndicesPtr & indices,
  PointCloud2 & output)
{
  std::scoped_lock lock(mutex_);
  int vertical_bins = vertical_bins_;
  int ideal_horizontal_bins;
  float distance_coefficient = 327.67f;
  float horizontal_resolution_ = 0.4f;
  if (lidar_model_ == "Pandar40P") {
    distance_coefficient = 327.67f;
    horizontal_resolution_ = 0.4f;
  } else if (lidar_model_ == "PandarQT") {
    distance_coefficient = 3276.75f;
    horizontal_resolution_ = 0.6f;
  }
  ideal_horizontal_bins =
    static_cast<uint>((angle_range_deg_[1] - angle_range_deg_[0]) / horizontal_resolution_);
  pcl::PointCloud<PointXYZIRADRT>::Ptr pcl_input(new pcl::PointCloud<PointXYZIRADRT>);
  pcl::fromROSMsg(*input, *pcl_input);
  std::vector<float> horizontal_bin_reference(ideal_horizontal_bins);
  std::vector<pcl::PointCloud<PointXYZIRADRT>> each_ring_pointcloud(vertical_bins);
  cv::Mat full_size_depth_map(
    cv::Size(ideal_horizontal_bins, vertical_bins), CV_16UC1, cv::Scalar(0));
  cv::Mat lidar_depth_map(cv::Size(ideal_horizontal_bins, vertical_bins), CV_16UC1, cv::Scalar(0));
  cv::Mat lidar_depth_map_8u(
    cv::Size(ideal_horizontal_bins, vertical_bins), CV_8UC1, cv::Scalar(0));
  if (pcl_input->points.empty()) {
    ground_blockage_ratio_ = 1.0f;
    sky_blockage_ratio_ = 1.0f;
    if (ground_blockage_count_ <= 2 * blockage_count_threshold_) {
      ground_blockage_count_ += 1;
    }
    if (sky_blockage_count_ <= 2 * blockage_count_threshold_) {
      sky_blockage_count_ += 1;
    }
    ground_blockage_range_deg_[0] = angle_range_deg_[0];
    ground_blockage_range_deg_[1] = angle_range_deg_[1];
    sky_blockage_range_deg_[0] = angle_range_deg_[0];
    sky_blockage_range_deg_[1] = angle_range_deg_[1];
  } else {
    for (int i = 0; i < ideal_horizontal_bins; ++i) {
      horizontal_bin_reference.at(i) = angle_range_deg_[0] + i * horizontal_resolution_;
    }
    for (const auto p : pcl_input->points) {
      for (int horizontal_bin = 0;
           horizontal_bin < static_cast<int>(horizontal_bin_reference.size()); horizontal_bin++) {
        if (
          (p.azimuth / 100 >
           (horizontal_bin_reference.at(horizontal_bin) - horizontal_resolution_ / 2)) &&
          (p.azimuth / 100 <=
           (horizontal_bin_reference.at(horizontal_bin) + horizontal_resolution_ / 2))) {
          if (lidar_model_ == "Pandar40P") {
            full_size_depth_map.at<uint16_t>(p.ring, horizontal_bin) =
              UINT16_MAX - distance_coefficient * p.distance;
          } else if (lidar_model_ == "PandarQT") {
            full_size_depth_map.at<uint16_t>(vertical_bins - p.ring - 1, horizontal_bin) =
              UINT16_MAX - distance_coefficient * p.distance;
          }
        }
      }
    }
  }
  full_size_depth_map.convertTo(lidar_depth_map_8u, CV_8UC1, 1.0 / 300);
  cv::Mat no_return_mask(cv::Size(ideal_horizontal_bins, vertical_bins), CV_8UC1, cv::Scalar(0));
  cv::inRange(lidar_depth_map_8u, 0, 1, no_return_mask);
  cv::Mat erosion_dst;
  cv::Mat blockage_element = cv::getStructuringElement(
    cv::MORPH_RECT, cv::Size(2 * blockage_kernel_ + 1, 2 * blockage_kernel_ + 1),
    cv::Point(blockage_kernel_, blockage_kernel_));
  cv::erode(no_return_mask, erosion_dst, blockage_element);
  cv::dilate(erosion_dst, no_return_mask, blockage_element);
  static boost::circular_buffer<cv::Mat> no_return_mask_buffer(blockage_buffering_frames_);
  cv::Mat time_series_blockage_result(
    cv::Size(ideal_horizontal_bins, vertical_bins), CV_8UC1, cv::Scalar(0));
  cv::Mat time_series_blockage_mask(
    cv::Size(ideal_horizontal_bins, vertical_bins), CV_8UC1, cv::Scalar(0));
  cv::Mat no_return_mask_binarized(
    cv::Size(ideal_horizontal_bins, vertical_bins), CV_8UC1, cv::Scalar(0));

  if (blockage_buffering_interval_ == 0) {
    no_return_mask.copyTo(time_series_blockage_result);
  } else {
    no_return_mask_binarized = no_return_mask / 255;
    if (blockage_frame_count_ >= blockage_buffering_interval_) {
      no_return_mask_buffer.push_back(no_return_mask_binarized);
      blockage_frame_count_ = 0;
    } else {
      blockage_frame_count_++;
    }
    for (const auto & binary_mask : no_return_mask_buffer) {
      time_series_blockage_mask += binary_mask;
    }
    cv::inRange(
      time_series_blockage_mask, no_return_mask_buffer.size() - 1, no_return_mask_buffer.size(),
      time_series_blockage_result);
  }
  cv::Mat ground_no_return_mask;
  cv::Mat sky_no_return_mask;
  no_return_mask(cv::Rect(0, 0, ideal_horizontal_bins, horizontal_ring_id_))
    .copyTo(sky_no_return_mask);
  no_return_mask(
    cv::Rect(0, horizontal_ring_id_, ideal_horizontal_bins, vertical_bins - horizontal_ring_id_))
    .copyTo(ground_no_return_mask);
  ground_blockage_ratio_ =
    static_cast<float>(cv::countNonZero(ground_no_return_mask)) /
    static_cast<float>(ideal_horizontal_bins * (vertical_bins - horizontal_ring_id_));
  sky_blockage_ratio_ = static_cast<float>(cv::countNonZero(sky_no_return_mask)) /
                        static_cast<float>(ideal_horizontal_bins * horizontal_ring_id_);

  if (ground_blockage_ratio_ > blockage_ratio_threshold_) {
    cv::Rect ground_blockage_bb = cv::boundingRect(ground_no_return_mask);
    ground_blockage_range_deg_[0] = static_cast<float>(ground_blockage_bb.x) + angle_range_deg_[0];
    ground_blockage_range_deg_[1] =
      static_cast<float>(ground_blockage_bb.x + ground_blockage_bb.width) + angle_range_deg_[0];
    if (ground_blockage_count_ <= 2 * blockage_count_threshold_) {
      ground_blockage_count_ += 1;
    }
  } else {
    ground_blockage_count_ = 0;
  }
  if (sky_blockage_ratio_ > blockage_ratio_threshold_) {
    cv::Rect sky_blockage_bx = cv::boundingRect(sky_no_return_mask);
    sky_blockage_range_deg_[0] = static_cast<float>(sky_blockage_bx.x) + angle_range_deg_[0];
    sky_blockage_range_deg_[1] =
      static_cast<float>(sky_blockage_bx.x + sky_blockage_bx.width) + angle_range_deg_[0];
    if (sky_blockage_count_ <= 2 * blockage_count_threshold_) {
      sky_blockage_count_ += 1;
    }
  } else {
    sky_blockage_count_ = 0;
  }
  // dust
  cv::Mat ground_depth_map = lidar_depth_map_8u(
    cv::Rect(0, horizontal_ring_id_, ideal_horizontal_bins, vertical_bins - horizontal_ring_id_));
  cv::Mat sky_blank(horizontal_ring_id_, ideal_horizontal_bins, CV_8UC1, cv::Scalar(0));
  cv::Mat ground_blank(
    vertical_bins - horizontal_ring_id_, ideal_horizontal_bins, CV_8UC1, cv::Scalar(0));
  cv::Mat single_dust_img(
    cv::Size(lidar_depth_map_8u.rows, lidar_depth_map_8u.cols), CV_8UC1, cv::Scalar(0));
  cv::Mat single_dust_ground_img = ground_depth_map.clone();
  cv::inRange(single_dust_ground_img, 0, 1, single_dust_ground_img);
  cv::Mat dust_element = getStructuringElement(
    cv::MORPH_RECT, cv::Size(2 * dust_kernel_size_ + 1, 2 * dust_kernel_size_ + 1),
    cv::Point(-1, -1));
  cv::dilate(single_dust_ground_img, single_dust_ground_img, dust_element);
  cv::erode(single_dust_ground_img, single_dust_ground_img, dust_element);
  cv::inRange(single_dust_ground_img, 254, 255, single_dust_ground_img);
  cv::Mat ground_mask(cv::Size(ideal_horizontal_bins, horizontal_ring_id_), CV_8UC1);
  cv::vconcat(sky_blank, single_dust_ground_img, single_dust_img);
  static boost::circular_buffer<cv::Mat> dust_mask_buffer(dust_buffering_frames_);
  cv::Mat binarized_dust_mask_(
    cv::Size(ideal_horizontal_bins, vertical_bins), CV_8UC1, cv::Scalar(0));
  cv::Mat multi_frame_dust_mask(
    cv::Size(ideal_horizontal_bins, vertical_bins), CV_8UC1, cv::Scalar(0));
  cv::Mat multi_frame_ground_dust_result(
    cv::Size(ideal_horizontal_bins, vertical_bins), CV_8UC1, cv::Scalar(0));

  if (dust_buffering_interval_ == 0) {
    single_dust_img.copyTo(multi_frame_ground_dust_result);
    dust_buffering_frame_counter_ = 0;
  } else {
    binarized_dust_mask_ = single_dust_img / 255;
    if (dust_buffering_frame_counter_ >= dust_buffering_interval_) {
      dust_mask_buffer.push_back(binarized_dust_mask_);
      dust_buffering_frame_counter_ = 0;
    } else {
      dust_buffering_frame_counter_++;
    }
    for (const auto & binarized_dust_mask : dust_mask_buffer) {
      multi_frame_dust_mask += binarized_dust_mask;
    }
    cv::inRange(
      multi_frame_dust_mask, dust_mask_buffer.size() - 1, dust_mask_buffer.size(),
      multi_frame_ground_dust_result);
  }
  cv::Mat single_frame_ground_dust_colorized(
    cv::Size(ideal_horizontal_bins, vertical_bins), CV_8UC3, cv::Scalar(0, 0, 0));
  cv::applyColorMap(single_dust_img, single_frame_ground_dust_colorized, cv::COLORMAP_JET);
  cv::Mat multi_frame_ground_dust_colorized;
  cv::Mat blockage_dust_merged_img(
    cv::Size(ideal_horizontal_bins, vertical_bins), CV_8UC3, cv::Scalar(0, 0, 0));
  cv::Mat blockage_dust_merged_mask(
    cv::Size(ideal_horizontal_bins, vertical_bins), CV_8UC1, cv::Scalar(0));
  blockage_dust_merged_img.setTo(
    cv::Vec3b(0, 0, 255), time_series_blockage_result);  // red:blockage
  blockage_dust_merged_img.setTo(
    cv::Vec3b(0, 255, 255), multi_frame_ground_dust_result);  // yellow:dust
  sensor_msgs::msg::Image::SharedPtr single_frame_dust_mask_msg =
    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", single_frame_ground_dust_colorized)
      .toImageMsg();
  single_frame_dust_mask_pub.publish(single_frame_dust_mask_msg);
  sensor_msgs::msg::Image::SharedPtr multi_frame_dust_mask_msg =
    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", multi_frame_ground_dust_colorized)
      .toImageMsg();
  multi_frame_dust_mask_pub.publish(multi_frame_dust_mask_msg);
  sensor_msgs::msg::Image::SharedPtr lidar_depth_map_msg =
    cv_bridge::CvImage(std_msgs::msg::Header(), "mono16", full_size_depth_map).toImageMsg();
  lidar_depth_map_msg->header = input->header;
  lidar_depth_map_pub_.publish(lidar_depth_map_msg);
  cv::Mat blockage_mask_colorized;
  cv::applyColorMap(time_series_blockage_result, blockage_mask_colorized, cv::COLORMAP_JET);
  sensor_msgs::msg::Image::SharedPtr blockage_mask_msg =
    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", blockage_mask_colorized).toImageMsg();
  blockage_mask_msg->header = input->header;
  blockage_mask_pub_.publish(blockage_mask_msg);
  cv::Mat blockage_dust_merged_colorized(
    cv::Size(ideal_horizontal_bins, vertical_bins), CV_8UC3, cv::Scalar(0, 0, 0));
  blockage_dust_merged_img.copyTo(blockage_dust_merged_colorized);
  sensor_msgs::msg::Image::SharedPtr blockage_dust_merged_msg =
    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", blockage_dust_merged_colorized)
      .toImageMsg();
  if (ground_dust_ratio_ > dust_ratio_threshold_) {
    if (dust_frame_count_ < 2 * dust_count_threshold_) {
      dust_frame_count_++;
    }
  } else {
    dust_frame_count_ = 0;
  }
  blockage_dust_merged_msg->header = input->header;
  blockage_dust_merged_pub.publish(blockage_dust_merged_msg);

  tier4_debug_msgs::msg::Float32Stamped ground_blockage_ratio_msg;
  ground_blockage_ratio_msg.data = ground_blockage_ratio_;
  ground_blockage_ratio_msg.stamp = now();
  ground_blockage_ratio_pub_->publish(ground_blockage_ratio_msg);

  tier4_debug_msgs::msg::Float32Stamped sky_blockage_ratio_msg;
  sky_blockage_ratio_msg.data = sky_blockage_ratio_;
  sky_blockage_ratio_msg.stamp = now();
  sky_blockage_ratio_pub_->publish(sky_blockage_ratio_msg);
  tier4_debug_msgs::msg::Float32Stamped blockage_ratio_msg;
  tier4_debug_msgs::msg::Float32Stamped ground_dust_ratio_msg;

  ground_dust_ratio_ = static_cast<float>(cv::countNonZero(single_dust_ground_img)) /
                       (single_dust_ground_img.cols * single_dust_ground_img.rows);
  ground_dust_ratio_msg.data = ground_dust_ratio_;
  ground_dust_ratio_msg.stamp = now();
  ground_dust_ratio_pub_->publish(ground_dust_ratio_msg);
  pcl::toROSMsg(*pcl_input, output);
  output.header = input->header;
}
rcl_interfaces::msg::SetParametersResult BlockageDiagComponent::paramCallback(
  const std::vector<rclcpp::Parameter> & p)
{
  std::scoped_lock lock(mutex_);
  if (get_param(p, "blockage_ratio_threshold", blockage_ratio_threshold_)) {
    RCLCPP_DEBUG(
      get_logger(), "Setting new blockage_ratio_threshold to: %f.", blockage_ratio_threshold_);
  }
  if (get_param(p, "horizontal_ring_id", horizontal_ring_id_)) {
    RCLCPP_DEBUG(get_logger(), "Setting new horizontal_ring_id to: %d.", horizontal_ring_id_);
  }
  if (get_param(p, "vertical_bins", vertical_bins_)) {
    RCLCPP_DEBUG(get_logger(), "Setting new vertical_bins to: %d.", vertical_bins_);
  }
  if (get_param(p, "blockage_count_threshold", blockage_count_threshold_)) {
    RCLCPP_DEBUG(
      get_logger(), "Setting new blockage_count_threshold to: %d.", blockage_count_threshold_);
  }
  if (get_param(p, "model", lidar_model_)) {
    RCLCPP_DEBUG(get_logger(), "Setting new lidar model to: %s. ", lidar_model_.c_str());
  }
  if (get_param(p, "angle_range", angle_range_deg_)) {
    RCLCPP_DEBUG(
      get_logger(), " Setting new angle_range to: [%f , %f].", angle_range_deg_[0],
      angle_range_deg_[1]);
  }
  if (get_param(p, "blockage_buffering_frames", blockage_buffering_frames_)) {
    RCLCPP_DEBUG(
      get_logger(), "Setting new blockage_buffering_frames_ to: %d.", blockage_buffering_frames_);
  }
  if (get_param(p, "blockage_buffering_interval", blockage_buffering_interval_)) {
    RCLCPP_DEBUG(
      get_logger(), "Setting new blockage_buffering_interval_ to: %d.",
      blockage_buffering_interval_);
  }
  if (get_param(p, "dust_kernel_size", dust_kernel_size_)) {
    RCLCPP_DEBUG(get_logger(), "Setting new dust_kernel_size_ to: %d.", dust_kernel_size_);
  }
  if (get_param(p, "dust_buffering_frames", dust_buffering_frames_)) {
    RCLCPP_DEBUG(
      get_logger(), "Setting new dust_buffering_frames_ to: %d.", dust_buffering_frames_);
    // note:NOT affects to actual variable.
    // if you want change this param/variable, change the parameter called at launch this
    // node(aip_launcher).
  }
  if (get_param(p, "dust_buffering_interval", dust_buffering_interval_)) {
    RCLCPP_DEBUG(
      get_logger(), "Setting new dust_buffering_interval_ to: %d.", dust_buffering_interval_);
    dust_buffering_frame_counter_ = 0;
  }
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}
}  // namespace pointcloud_preprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_preprocessor::BlockageDiagComponent)
