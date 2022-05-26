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

#include <algorithm>
#include <vector>
namespace pointcloud_preprocessor
{
RingOutlierFilterComponent::RingOutlierFilterComponent(const rclcpp::NodeOptions & options)
: Filter("RingOutlierFilter", options)
{
  // initialize debug tool
  {
    using tier4_autoware_utils::DebugPublisher;
    using tier4_autoware_utils::StopWatch;
    stop_watch_ptr_ = std::make_unique<StopWatch<std::chrono::milliseconds>>();
    debug_publisher_ = std::make_unique<DebugPublisher>(this, "ring_outlier_filter");
    stop_watch_ptr_->tic("cyclic_time");
    stop_watch_ptr_->tic("processing_time");
  }

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
  std::scoped_lock lock(mutex_);
  stop_watch_ptr_->toc("processing_time", true);
  std::unordered_map<uint16_t, std::vector<std::size_t>> input_ring_map;
  input_ring_map.reserve(128);
  sensor_msgs::msg::PointCloud2::SharedPtr input_ptr =
    std::make_shared<sensor_msgs::msg::PointCloud2>(*input);

  const auto ring_offset =
    input->fields.at(static_cast<size_t>(autoware_point_types::PointIndex::Ring)).offset;
  for (std::size_t idx = 0U; idx < input_ptr->data.size(); idx += input_ptr->point_step) {
    input_ring_map[*reinterpret_cast<uint16_t *>(&input_ptr->data[idx + ring_offset])].push_back(
      idx);
  }

  PointCloud2Modifier<PointXYZI> output_modifier{output, input->header.frame_id};
  output_modifier.reserve(input->width);

  std::vector<std::size_t> tmp_indices;
  tmp_indices.reserve(input->width);

  const auto azimuth_offset =
    input->fields.at(static_cast<size_t>(autoware_point_types::PointIndex::Azimuth)).offset;
  const auto distance_offset =
    input->fields.at(static_cast<size_t>(autoware_point_types::PointIndex::Distance)).offset;
  for (const auto & ring_indices : input_ring_map) {
    if (ring_indices.second.size() < 2) {
      continue;
    }

    for (size_t idx = 0U; idx < ring_indices.second.size() - 1; ++idx) {
      const auto & current_idx = ring_indices.second.at(idx);
      const auto & next_idx = ring_indices.second.at(idx + 1);
      tmp_indices.emplace_back(current_idx);

      // if(std::abs(iter->distance - (iter+1)->distance) <= std::sqrt(iter->distance) * 0.08)
      const auto current_pt_azimuth =
        *reinterpret_cast<float *>(&input_ptr->data[current_idx + azimuth_offset]);
      const auto next_pt_azimuth =
        *reinterpret_cast<float *>(&input_ptr->data[next_idx + azimuth_offset]);
      float azimuth_diff = next_pt_azimuth - current_pt_azimuth;
      azimuth_diff = azimuth_diff < 0.f ? azimuth_diff + 36000.f : azimuth_diff;

      const auto current_pt_distance =
        *reinterpret_cast<float *>(&input_ptr->data[current_idx + distance_offset]);
      const auto next_pt_distance =
        *reinterpret_cast<float *>(&input_ptr->data[next_idx + distance_offset]);

      if (
        std::max(current_pt_distance, next_pt_distance) <
          std::min(current_pt_distance, next_pt_distance) * distance_ratio_ &&
        azimuth_diff < 100.f) {
        continue;
      }
      if (isCluster(input_ptr, tmp_indices)) {
        for (const auto & tmp_idx : tmp_indices) {
          output_modifier.push_back(
            std::move(*reinterpret_cast<PointXYZI *>(&input_ptr->data[tmp_idx])));
        }
      }
      tmp_indices.clear();
    }
    if (tmp_indices.empty()) {
      continue;
    }
    if (isCluster(input_ptr, tmp_indices)) {
      for (const auto & tmp_idx : tmp_indices) {
        output_modifier.push_back(
          std::move(*reinterpret_cast<PointXYZI *>(&input_ptr->data[tmp_idx])));
      }
    }
    tmp_indices.clear();
  }
  // add processing time for debug
  if (debug_publisher_) {
    const double cyclic_time_ms = stop_watch_ptr_->toc("cyclic_time", true);
    const double processing_time_ms = stop_watch_ptr_->toc("processing_time", true);
    debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/cyclic_time_ms", cyclic_time_ms);
    debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/processing_time_ms", processing_time_ms);
  }
}

rcl_interfaces::msg::SetParametersResult RingOutlierFilterComponent::paramCallback(
  const std::vector<rclcpp::Parameter> & p)
{
  std::scoped_lock lock(mutex_);

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
