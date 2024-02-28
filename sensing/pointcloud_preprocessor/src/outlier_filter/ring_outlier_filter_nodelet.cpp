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

#include "autoware_auto_geometry/common_3d.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <pcl/search/pcl_search.h>

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
    excluded_points_publisher_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("debug/ring_outlier_filter", 1);
    stop_watch_ptr_->tic("cyclic_time");
    stop_watch_ptr_->tic("processing_time");
  }

  // set initial parameters
  {
    distance_ratio_ = static_cast<double>(declare_parameter("distance_ratio", 1.03));
    object_length_threshold_ =
      static_cast<double>(declare_parameter("object_length_threshold", 0.1));
    num_points_threshold_ = static_cast<int>(declare_parameter("num_points_threshold", 4));
    max_rings_num_ = static_cast<uint16_t>(declare_parameter("max_rings_num", 128));
    max_points_num_per_ring_ =
      static_cast<size_t>(declare_parameter("max_points_num_per_ring", 4000));
    publish_excluded_points_ =
      static_cast<bool>(declare_parameter("publish_excluded_points", false));
  }

  using std::placeholders::_1;
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&RingOutlierFilterComponent::paramCallback, this, _1));
}

// TODO(sykwer): Temporary Implementation: Rename this function to `filter()` when all the filter
// nodes conform to new API. Then delete the old `filter()` defined below.
void RingOutlierFilterComponent::faster_filter(
  const PointCloud2ConstPtr & input, const IndicesPtr & unused_indices, PointCloud2 & output,
  const TransformInfo & transform_info)
{
  std::scoped_lock lock(mutex_);
  if (unused_indices) {
    RCLCPP_WARN(get_logger(), "Indices are not supported and will be ignored");
  }
  stop_watch_ptr_->toc("processing_time", true);

  output.point_step = sizeof(PointXYZI);
  output.data.resize(output.point_step * input->width);
  size_t output_size = 0;

  const auto ring_offset =
    input->fields.at(static_cast<size_t>(autoware_point_types::PointIndex::Ring)).offset;
  const auto azimuth_offset =
    input->fields.at(static_cast<size_t>(autoware_point_types::PointIndex::Azimuth)).offset;
  const auto distance_offset =
    input->fields.at(static_cast<size_t>(autoware_point_types::PointIndex::Distance)).offset;
  const auto intensity_offset =
    input->fields.at(static_cast<size_t>(autoware_point_types::PointIndex::Intensity)).offset;

  std::vector<std::vector<size_t>> ring2indices;
  ring2indices.reserve(max_rings_num_);

  for (uint16_t i = 0; i < max_rings_num_; i++) {
    ring2indices.push_back(std::vector<size_t>());
    ring2indices.back().reserve(max_points_num_per_ring_);
  }

  for (size_t data_idx = 0; data_idx < input->data.size(); data_idx += input->point_step) {
    const uint16_t ring = *reinterpret_cast<const uint16_t *>(&input->data[data_idx + ring_offset]);
    ring2indices[ring].push_back(data_idx);
  }

  // walk range: [walk_first_idx, walk_last_idx]
  int walk_first_idx = 0;
  int walk_last_idx = -1;

  for (const auto & indices : ring2indices) {
    if (indices.size() < 2) continue;

    walk_first_idx = 0;
    walk_last_idx = -1;

    for (size_t idx = 0U; idx < indices.size() - 1; ++idx) {
      const size_t & current_data_idx = indices[idx];
      const size_t & next_data_idx = indices[idx + 1];
      walk_last_idx = idx;

      // if(std::abs(iter->distance - (iter+1)->distance) <= std::sqrt(iter->distance) * 0.08)

      const float & current_azimuth =
        *reinterpret_cast<const float *>(&input->data[current_data_idx + azimuth_offset]);
      const float & next_azimuth =
        *reinterpret_cast<const float *>(&input->data[next_data_idx + azimuth_offset]);
      float azimuth_diff = next_azimuth - current_azimuth;
      azimuth_diff = azimuth_diff < 0.f ? azimuth_diff + 36000.f : azimuth_diff;

      const float & current_distance =
        *reinterpret_cast<const float *>(&input->data[current_data_idx + distance_offset]);
      const float & next_distance =
        *reinterpret_cast<const float *>(&input->data[next_data_idx + distance_offset]);

      if (
        std::max(current_distance, next_distance) <
          std::min(current_distance, next_distance) * distance_ratio_ &&
        azimuth_diff < 100.f) {
        continue;  // Determined to be included in the same walk
      }

      if (isCluster(
            input, std::make_pair(indices[walk_first_idx], indices[walk_last_idx]),
            walk_last_idx - walk_first_idx + 1)) {
        for (int i = walk_first_idx; i <= walk_last_idx; i++) {
          auto output_ptr = reinterpret_cast<PointXYZI *>(&output.data[output_size]);
          auto input_ptr = reinterpret_cast<const PointXYZI *>(&input->data[indices[i]]);

          if (transform_info.need_transform) {
            Eigen::Vector4f p(input_ptr->x, input_ptr->y, input_ptr->z, 1);
            p = transform_info.eigen_transform * p;
            output_ptr->x = p[0];
            output_ptr->y = p[1];
            output_ptr->z = p[2];
          } else {
            *output_ptr = *input_ptr;
          }
          const float & intensity =
            *reinterpret_cast<const float *>(&input->data[indices[i] + intensity_offset]);
          output_ptr->intensity = intensity;

          output_size += output.point_step;
        }
      }

      walk_first_idx = idx + 1;
    }

    if (walk_first_idx > walk_last_idx) continue;

    if (isCluster(
          input, std::make_pair(indices[walk_first_idx], indices[walk_last_idx]),
          walk_last_idx - walk_first_idx + 1)) {
      for (int i = walk_first_idx; i <= walk_last_idx; i++) {
        auto output_ptr = reinterpret_cast<PointXYZI *>(&output.data[output_size]);
        auto input_ptr = reinterpret_cast<const PointXYZI *>(&input->data[indices[i]]);

        if (transform_info.need_transform) {
          Eigen::Vector4f p(input_ptr->x, input_ptr->y, input_ptr->z, 1);
          p = transform_info.eigen_transform * p;
          output_ptr->x = p[0];
          output_ptr->y = p[1];
          output_ptr->z = p[2];
        } else {
          *output_ptr = *input_ptr;
        }
        const float & intensity =
          *reinterpret_cast<const float *>(&input->data[indices[i] + intensity_offset]);
        output_ptr->intensity = intensity;

        output_size += output.point_step;
      }
    }
  }

  output.data.resize(output_size);

  // Note that `input->header.frame_id` is data before converted when `transform_info.need_transform
  // == true`
  output.header.frame_id = !tf_input_frame_.empty() ? tf_input_frame_ : tf_input_orig_frame_;

  output.height = 1;
  output.width = static_cast<uint32_t>(output.data.size() / output.height / output.point_step);
  output.is_bigendian = input->is_bigendian;
  output.is_dense = input->is_dense;

  // set fields
  sensor_msgs::PointCloud2Modifier pcd_modifier(output);
  constexpr int num_fields = 4;
  pcd_modifier.setPointCloud2Fields(
    num_fields, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
    sensor_msgs::msg::PointField::FLOAT32, "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);

  if (publish_excluded_points_) {
    auto excluded_points = extractExcludedPoints(*input, output, 0.01);
    // set fields
    sensor_msgs::PointCloud2Modifier excluded_pcd_modifier(excluded_points);
    excluded_pcd_modifier.setPointCloud2Fields(
      num_fields, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
      sensor_msgs::msg::PointField::FLOAT32, "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
    excluded_points_publisher_->publish(excluded_points);
  }

  // add processing time for debug
  if (debug_publisher_) {
    const double cyclic_time_ms = stop_watch_ptr_->toc("cyclic_time", true);
    const double processing_time_ms = stop_watch_ptr_->toc("processing_time", true);
    debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/cyclic_time_ms", cyclic_time_ms);
    debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/processing_time_ms", processing_time_ms);

    auto pipeline_latency_ms =
      std::chrono::duration<double, std::milli>(
        std::chrono::nanoseconds((this->get_clock()->now() - input->header.stamp).nanoseconds()))
        .count();

    debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/pipeline_latency_ms", pipeline_latency_ms);
  }
}

// TODO(sykwer): Temporary Implementation: Delete this function definition when all the filter nodes
// conform to new API
void RingOutlierFilterComponent::filter(
  const PointCloud2ConstPtr & input, [[maybe_unused]] const IndicesPtr & indices,
  PointCloud2 & output)
{
  (void)input;
  (void)indices;
  (void)output;
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
  if (get_param(p, "publish_excluded_points", publish_excluded_points_)) {
    RCLCPP_DEBUG(
      get_logger(), "Setting new publish_excluded_points to: %d.", publish_excluded_points_);
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  return result;
}

sensor_msgs::msg::PointCloud2 RingOutlierFilterComponent::extractExcludedPoints(
  const sensor_msgs::msg::PointCloud2 & input, const sensor_msgs::msg::PointCloud2 & output,
  float epsilon)
{
  // Convert ROS PointCloud2 message to PCL point cloud for easier manipulation
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(input, *input_cloud);
  pcl::fromROSMsg(output, *output_cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr excluded_points(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::search::Search<pcl::PointXYZ>::Ptr tree;
  if (output_cloud->isOrganized()) {
    tree.reset(new pcl::search::OrganizedNeighbor<pcl::PointXYZ>());
  } else {
    tree.reset(new pcl::search::KdTree<pcl::PointXYZ>(false));
  }
  tree->setInputCloud(output_cloud);
  std::vector<int> nn_indices(1);
  std::vector<float> nn_distances(1);
  for (const auto & point : input_cloud->points) {
    if (!tree->nearestKSearch(point, 1, nn_indices, nn_distances)) {
      continue;
    }
    if (nn_distances[0] > epsilon) {
      excluded_points->points.push_back(point);
    }
  }

  sensor_msgs::msg::PointCloud2 excluded_points_msg;
  pcl::toROSMsg(*excluded_points, excluded_points_msg);

  // Set the metadata for the excluded points message based on the input cloud
  excluded_points_msg.height = 1;
  excluded_points_msg.width =
    static_cast<uint32_t>(output.data.size() / output.height / output.point_step);
  excluded_points_msg.row_step = static_cast<uint32_t>(output.data.size() / output.height);
  excluded_points_msg.is_bigendian = input.is_bigendian;
  excluded_points_msg.is_dense = input.is_dense;
  excluded_points_msg.header = input.header;
  excluded_points_msg.header.frame_id =
    !tf_input_frame_.empty() ? tf_input_frame_ : tf_input_orig_frame_;

  return excluded_points_msg;
}

}  // namespace pointcloud_preprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_preprocessor::RingOutlierFilterComponent)
