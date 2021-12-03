// Copyright 2020 Tier IV, Inc.
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

#include "pointcloud_preprocessor/outlier_filter/ring_outlier_filter_nodelet.hpp"

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/segment_differences.h>

#include <algorithm>
#include <vector>

namespace pointcloud_preprocessor
{
RingOutlierFilterComponent::RingOutlierFilterComponent(const rclcpp::NodeOptions & options)
: Filter("RingOutlierFilter", options)
{
  // set initial parameters
  {
    distance_ratio_ = static_cast<double>(declare_parameter("distance_ratio", 1.03));
    object_length_threshold_ =
      static_cast<double>(declare_parameter("object_length_threshold", 0.1));
    num_points_threshold_ = static_cast<int>(declare_parameter("num_points_threshold", 4));
  }

  using std::placeholders::_1;
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&RingOutlierFilterComponent::paramCallback, this, _1));
}

void RingOutlierFilterComponent::filter(
  const PointCloud2ConstPtr & input, [[maybe_unused]] const IndicesPtr & indices,
  PointCloud2 & output)
{
  boost::mutex::scoped_lock lock(mutex_);
  pcl::PointCloud<custom_pcl::PointXYZIRADT>::Ptr pcl_input(
    new pcl::PointCloud<custom_pcl::PointXYZIRADT>);
  pcl::fromROSMsg(*input, *pcl_input);

  if (pcl_input->points.empty()) {
    return;
  }
  std::vector<pcl::PointCloud<custom_pcl::PointXYZIRADT>> pcl_input_ring_array;
  pcl_input_ring_array.resize(128);  // TODO(Yamato Ando)
  for (const auto & p : pcl_input->points) {
    pcl_input_ring_array.at(p.ring).push_back(p);
  }

  pcl::PointCloud<custom_pcl::PointXYZI>::Ptr pcl_output(
    new pcl::PointCloud<custom_pcl::PointXYZI>);
  pcl_output->points.reserve(pcl_input->points.size());

  pcl::PointCloud<custom_pcl::PointXYZI> pcl_tmp;
  custom_pcl::PointXYZI p{};
  for (const auto & ring_pointcloud : pcl_input_ring_array) {
    if (ring_pointcloud.points.size() < 2) {
      continue;
    }

    for (auto iter = std::begin(ring_pointcloud.points);
         iter != std::end(ring_pointcloud.points) - 1; ++iter) {
      p.x = iter->x;
      p.y = iter->y;
      p.z = iter->z;
      p.intensity = iter->intensity;
      pcl_tmp.points.push_back(p);
      // if(std::abs(iter->distance - (iter+1)->distance) <= std::sqrt(iter->distance) * 0.08) {
      const float min_dist = std::min(iter->distance, (iter + 1)->distance);
      const float max_dist = std::max(iter->distance, (iter + 1)->distance);
      float azimuth_diff = (iter + 1)->azimuth - iter->azimuth;
      azimuth_diff = azimuth_diff < 0.f ? azimuth_diff + 36000.f : azimuth_diff;

      if (max_dist < min_dist * distance_ratio_ && azimuth_diff < 100.f) {
        continue;
      }
      // same code
      if (
        static_cast<int>(pcl_tmp.points.size()) > num_points_threshold_ ||
        (pcl_tmp.points.front().x - pcl_tmp.points.back().x) *
              (pcl_tmp.points.front().x - pcl_tmp.points.back().x) +
            (pcl_tmp.points.front().y - pcl_tmp.points.back().y) *
              (pcl_tmp.points.front().y - pcl_tmp.points.back().y) +
            (pcl_tmp.points.front().z - pcl_tmp.points.back().z) *
              (pcl_tmp.points.front().z - pcl_tmp.points.back().z) >=
          object_length_threshold_ * object_length_threshold_) {
        for (const auto & tmp_p : pcl_tmp.points) {
          pcl_output->points.push_back(tmp_p);
        }
      }
      pcl_tmp.points.clear();
    }

    // same code
    if (
      static_cast<int>(pcl_tmp.points.size()) > num_points_threshold_ ||
      (pcl_tmp.points.front().x - pcl_tmp.points.back().x) *
            (pcl_tmp.points.front().x - pcl_tmp.points.back().x) +
          (pcl_tmp.points.front().y - pcl_tmp.points.back().y) *
            (pcl_tmp.points.front().y - pcl_tmp.points.back().y) +
          (pcl_tmp.points.front().z - pcl_tmp.points.back().z) *
            (pcl_tmp.points.front().z - pcl_tmp.points.back().z) >=
        object_length_threshold_ * object_length_threshold_) {
      for (const auto & tmp_p : pcl_tmp.points) {
        pcl_output->points.push_back(tmp_p);
      }
    }
    pcl_tmp.points.clear();
  }

  pcl::toROSMsg(*pcl_output, output);
  output.header = input->header;
}

rcl_interfaces::msg::SetParametersResult RingOutlierFilterComponent::paramCallback(
  const std::vector<rclcpp::Parameter> & p)
{
  boost::mutex::scoped_lock lock(mutex_);

  if (get_param(p, "distance_ratio", distance_ratio_)) {
    RCLCPP_DEBUG(get_logger(), "Setting new distance ratio to: %f.", distance_ratio_);
  }
  if (get_param(p, "object_length_threshold", object_length_threshold_)) {
    RCLCPP_DEBUG(
      get_logger(), "Setting new object length threshold to: %f.", object_length_threshold_);
  }
  if (get_param(p, "num_points_threshold", num_points_threshold_)) {
    RCLCPP_DEBUG(get_logger(), "Setting new num_points_threshold to: %d.", num_points_threshold_);
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  return result;
}
}  // namespace pointcloud_preprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_preprocessor::RingOutlierFilterComponent)
