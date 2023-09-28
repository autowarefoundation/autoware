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

#ifndef POINTCLOUD_PREPROCESSOR__BLOCKAGE_DIAG__BLOCKAGE_DIAG_NODELET_HPP_
#define POINTCLOUD_PREPROCESSOR__BLOCKAGE_DIAG__BLOCKAGE_DIAG_NODELET_HPP_

#include "pointcloud_preprocessor/filter.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <rclcpp/rclcpp.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <tier4_debug_msgs/msg/float32_stamped.hpp>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include <string>
#include <vector>

namespace pointcloud_preprocessor
{
using diagnostic_updater::DiagnosticStatusWrapper;
using diagnostic_updater::Updater;

class BlockageDiagComponent : public pointcloud_preprocessor::Filter
{
protected:
  virtual void filter(
    const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output);
  /** \brief Parameter service callback result : needed to be hold */
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  /** \brief Parameter service callback */
  rcl_interfaces::msg::SetParametersResult paramCallback(const std::vector<rclcpp::Parameter> & p);
  image_transport::Publisher lidar_depth_map_pub_;
  image_transport::Publisher blockage_mask_pub_;
  image_transport::Publisher single_frame_dust_mask_pub;
  image_transport::Publisher multi_frame_dust_mask_pub;
  image_transport::Publisher blockage_dust_merged_pub;
  rclcpp::Publisher<tier4_debug_msgs::msg::Float32Stamped>::SharedPtr ground_blockage_ratio_pub_;
  rclcpp::Publisher<tier4_debug_msgs::msg::Float32Stamped>::SharedPtr sky_blockage_ratio_pub_;
  rclcpp::Publisher<tier4_debug_msgs::msg::Float32Stamped>::SharedPtr ground_dust_ratio_pub_;
  rclcpp::Publisher<tier4_debug_msgs::msg::StringStamped>::SharedPtr blockage_type_pub_;

private:
  void onBlockageChecker(DiagnosticStatusWrapper & stat);
  void dustChecker(DiagnosticStatusWrapper & stat);
  Updater updater_{this};
  int vertical_bins_;
  std::vector<double> angle_range_deg_;
  int horizontal_ring_id_;
  float blockage_ratio_threshold_;
  float dust_ratio_threshold_;
  float ground_blockage_ratio_ = -1.0f;
  float sky_blockage_ratio_ = -1.0f;
  float ground_dust_ratio_ = -1.0f;
  std::vector<float> ground_blockage_range_deg_ = {0.0f, 0.0f};
  std::vector<float> sky_blockage_range_deg_ = {0.0f, 0.0f};
  int blockage_kernel_ = 10;
  int blockage_frame_count_ = 0;
  int ground_blockage_count_ = 0;
  int sky_blockage_count_ = 0;
  int blockage_count_threshold_;
  std::string lidar_model_;
  int blockage_buffering_frames_;
  int blockage_buffering_interval_;
  int dust_kernel_size_;
  int dust_buffering_frames_;
  int dust_buffering_interval_;
  int dust_buffering_frame_counter_ = 0;
  int dust_count_threshold_;
  int dust_frame_count_ = 0;

public:
  PCL_MAKE_ALIGNED_OPERATOR_NEW
  explicit BlockageDiagComponent(const rclcpp::NodeOptions & options);
};

}  // namespace pointcloud_preprocessor

#endif  // POINTCLOUD_PREPROCESSOR__BLOCKAGE_DIAG__BLOCKAGE_DIAG_NODELET_HPP_
