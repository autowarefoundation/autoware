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

#include <cv_bridge/cv_bridge.h>

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
    const PointCloud2ConstPtr & input, [[maybe_unused]] const IndicesPtr & indices,
    PointCloud2 & output);
  /** \brief Parameter service callback result : needed to be hold */
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  /** \brief Parameter service callback */
  rcl_interfaces::msg::SetParametersResult paramCallback(const std::vector<rclcpp::Parameter> & p);
  image_transport::Publisher lidar_depth_map_pub_;
  image_transport::Publisher blockage_mask_pub_;
  rclcpp::Publisher<tier4_debug_msgs::msg::Float32Stamped>::SharedPtr ground_blockage_ratio_pub_;
  rclcpp::Publisher<tier4_debug_msgs::msg::Float32Stamped>::SharedPtr sky_blockage_ratio_pub_;

private:
  void onBlockageChecker(DiagnosticStatusWrapper & stat);
  Updater updater_{this};
  uint vertical_bins_;
  std::vector<double> angle_range_deg_;
  uint horizontal_ring_id_ = 12;
  float blockage_ratio_threshold_;
  float ground_blockage_ratio_ = -1.0f;
  float sky_blockage_ratio_ = -1.0f;
  std::vector<float> ground_blockage_range_deg_ = {0.0f, 0.0f};
  std::vector<float> sky_blockage_range_deg_ = {0.0f, 0.0f};
  uint erode_kernel_ = 10;
  uint ground_blockage_count_ = 0;
  uint sky_blockage_count_ = 0;
  uint blockage_count_threshold_;
  std::string lidar_model_;
  uint buffering_frames_ = 100;
  uint buffering_interval_ = 5;

public:
  PCL_MAKE_ALIGNED_OPERATOR_NEW
  explicit BlockageDiagComponent(const rclcpp::NodeOptions & options);
};

}  // namespace pointcloud_preprocessor

#endif  // POINTCLOUD_PREPROCESSOR__BLOCKAGE_DIAG__BLOCKAGE_DIAG_NODELET_HPP_
