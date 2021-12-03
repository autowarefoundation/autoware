// Copyright 2021 Tier IV, Inc.
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

#ifndef POINTCLOUD_PREPROCESSOR__OUTLIER_FILTER__DUAL_RETURN_OUTLIER_FILTER_NODELET_HPP_
#define POINTCLOUD_PREPROCESSOR__OUTLIER_FILTER__DUAL_RETURN_OUTLIER_FILTER_NODELET_HPP_

#include "pointcloud_preprocessor/filter.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_debug_msgs/msg/float32_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cv_bridge/cv_bridge.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/pcl_search.h>

#include <vector>

namespace pointcloud_preprocessor
{
using diagnostic_updater::DiagnosticStatusWrapper;
using diagnostic_updater::Updater;

enum ReturnType : uint8_t {
  INVALID = 0,
  SINGLE_STRONGEST,
  SINGLE_LAST,
  DUAL_STRONGEST_FIRST,
  DUAL_STRONGEST_LAST,
  DUAL_WEAK_FIRST,
  DUAL_WEAK_LAST,
  DUAL_ONLY,
};

class DualReturnOutlierFilterComponent : public pointcloud_preprocessor::Filter
{
protected:
  virtual void filter(
    const PointCloud2ConstPtr & input, [[maybe_unused]] const IndicesPtr & indices,
    PointCloud2 & output);
  /** \brief Parameter service callback result : needed to be hold */
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  /** \brief Parameter service callback */
  rcl_interfaces::msg::SetParametersResult paramCallback(const std::vector<rclcpp::Parameter> & p);
  image_transport::Publisher image_pub_;
  rclcpp::Publisher<autoware_debug_msgs::msg::Float32Stamped>::SharedPtr visibility_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr noise_cloud_pub_;

private:
  void onVisibilityChecker(DiagnosticStatusWrapper & stat);
  Updater updater_{this};
  double visibility_ = 1.f;
  double weak_first_distance_ratio_;
  double general_distance_ratio_;
  int weak_first_local_noise_threshold_;
  double visibility_threshold_;
  int vertical_bins_;
  float max_azimuth_diff_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit DualReturnOutlierFilterComponent(const rclcpp::NodeOptions & options);
};

}  // namespace pointcloud_preprocessor

namespace return_type_cloud
{
struct PointXYZIRADT
{
  PCL_ADD_POINT4D;
  float intensity;
  std::uint16_t ring;
  float azimuth;
  float distance;
  std::uint8_t return_type;
  double time_stamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

}  // namespace return_type_cloud

POINT_CLOUD_REGISTER_POINT_STRUCT(
  return_type_cloud::PointXYZIRADT,
  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint16_t, ring, ring)(
    float, azimuth, azimuth)(float, distance, distance)(std::uint8_t, return_type, return_type)(
    double, time_stamp, time_stamp))

#endif  // POINTCLOUD_PREPROCESSOR__OUTLIER_FILTER__DUAL_RETURN_OUTLIER_FILTER_NODELET_HPP_
