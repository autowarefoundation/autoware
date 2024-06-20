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

#include "autoware_point_types/types.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <algorithm>
#include <vector>
namespace pointcloud_preprocessor
{
using autoware_point_types::PointXYZIRADRT;

RingOutlierFilterComponent::RingOutlierFilterComponent(const rclcpp::NodeOptions & options)
: Filter("RingOutlierFilter", options)
{
  // initialize debug tool
  {
    using autoware::universe_utils::DebugPublisher;
    using autoware::universe_utils::StopWatch;
    stop_watch_ptr_ = std::make_unique<StopWatch<std::chrono::milliseconds>>();
    debug_publisher_ = std::make_unique<DebugPublisher>(this, "ring_outlier_filter");
    outlier_pointcloud_publisher_ =
      this->create_publisher<PointCloud2>("debug/ring_outlier_filter", 1);
    visibility_pub_ = create_publisher<tier4_debug_msgs::msg::Float32Stamped>(
      "ring_outlier_filter/debug/visibility", rclcpp::SensorDataQoS());
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
    publish_outlier_pointcloud_ =
      static_cast<bool>(declare_parameter("publish_outlier_pointcloud", false));

    min_azimuth_deg_ = static_cast<float>(declare_parameter("min_azimuth_deg", 0.0));
    max_azimuth_deg_ = static_cast<float>(declare_parameter("max_azimuth_deg", 360.0));
    max_distance_ = static_cast<float>(declare_parameter("max_distance", 12.0));
    vertical_bins_ = static_cast<int>(declare_parameter("vertical_bins", 128));
    horizontal_bins_ = static_cast<int>(declare_parameter("horizontal_bins", 36));
    noise_threshold_ = static_cast<int>(declare_parameter("noise_threshold", 2));
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

  pcl::PointCloud<PointXYZIRADRT>::Ptr outlier_pcl(new pcl::PointCloud<PointXYZIRADRT>);

  const auto ring_offset =
    input->fields.at(static_cast<size_t>(autoware_point_types::PointIndex::Ring)).offset;
  const auto azimuth_offset =
    input->fields.at(static_cast<size_t>(autoware_point_types::PointIndex::Azimuth)).offset;
  const auto distance_offset =
    input->fields.at(static_cast<size_t>(autoware_point_types::PointIndex::Distance)).offset;
  const auto intensity_offset =
    input->fields.at(static_cast<size_t>(autoware_point_types::PointIndex::Intensity)).offset;
  const auto return_type_offset =
    input->fields.at(static_cast<size_t>(autoware_point_types::PointIndex::ReturnType)).offset;
  const auto time_stamp_offset =
    input->fields.at(static_cast<size_t>(autoware_point_types::PointIndex::TimeStamp)).offset;

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
      } else if (publish_outlier_pointcloud_) {
        for (int i = walk_first_idx; i <= walk_last_idx; i++) {
          PointXYZIRADRT outlier_point;
          auto input_ptr =
            reinterpret_cast<const PointXYZIRADRT *>(&input->data[indices[walk_first_idx]]);
          if (transform_info.need_transform) {
            Eigen::Vector4f p(input_ptr->x, input_ptr->y, input_ptr->z, 1);
            p = transform_info.eigen_transform * p;
            outlier_point.x = p[0];
            outlier_point.y = p[1];
            outlier_point.z = p[2];
          } else {
            outlier_point = *input_ptr;
          }

          outlier_point.intensity = *reinterpret_cast<const float *>(
            &input->data[indices[walk_first_idx] + intensity_offset]);
          outlier_point.ring = *reinterpret_cast<const uint16_t *>(
            &input->data[indices[walk_first_idx] + ring_offset]);
          outlier_point.azimuth = *reinterpret_cast<const float *>(
            &input->data[indices[walk_first_idx] + azimuth_offset]);
          outlier_point.distance = *reinterpret_cast<const float *>(
            &input->data[indices[walk_first_idx] + distance_offset]);
          outlier_point.return_type = *reinterpret_cast<const uint8_t *>(
            &input->data[indices[walk_first_idx] + return_type_offset]);
          outlier_point.time_stamp = *reinterpret_cast<const float *>(
            &input->data[indices[walk_first_idx] + time_stamp_offset]);
          outlier_pcl->push_back(outlier_point);
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
    } else if (publish_outlier_pointcloud_) {
      for (int i = walk_first_idx; i < walk_last_idx; i++) {
        PointXYZIRADRT outlier_point;

        auto input_ptr = reinterpret_cast<const PointXYZIRADRT *>(&input->data[indices[i]]);
        if (transform_info.need_transform) {
          Eigen::Vector4f p(input_ptr->x, input_ptr->y, input_ptr->z, 1);
          p = transform_info.eigen_transform * p;
          outlier_point.x = p[0];
          outlier_point.y = p[1];
          outlier_point.z = p[2];
        } else {
          outlier_point = *input_ptr;
        }
        outlier_point.intensity =
          *reinterpret_cast<const float *>(&input->data[indices[i] + intensity_offset]);
        outlier_point.ring =
          *reinterpret_cast<const uint16_t *>(&input->data[indices[i] + ring_offset]);
        outlier_point.azimuth =
          *reinterpret_cast<const float *>(&input->data[indices[i] + azimuth_offset]);
        outlier_point.distance =
          *reinterpret_cast<const float *>(&input->data[indices[i] + distance_offset]);
        outlier_point.return_type =
          *reinterpret_cast<const uint8_t *>(&input->data[indices[i] + return_type_offset]);
        outlier_point.time_stamp =
          *reinterpret_cast<const float *>(&input->data[indices[i] + time_stamp_offset]);
        outlier_pcl->push_back(outlier_point);
      }
    }
  }

  setUpPointCloudFormat(input, output, output_size, /*num_fields=*/4);

  if (publish_outlier_pointcloud_) {
    PointCloud2 outlier;
    pcl::toROSMsg(*outlier_pcl, outlier);
    outlier.header = input->header;
    outlier_pointcloud_publisher_->publish(outlier);

    tier4_debug_msgs::msg::Float32Stamped visibility_msg;
    visibility_msg.data = calculateVisibilityScore(outlier);
    visibility_msg.stamp = input->header.stamp;
    visibility_pub_->publish(visibility_msg);
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
  if (get_param(p, "publish_outlier_pointcloud", publish_outlier_pointcloud_)) {
    RCLCPP_DEBUG(
      get_logger(), "Setting new publish_outlier_pointcloud to: %d.", publish_outlier_pointcloud_);
  }
  if (get_param(p, "vertical_bins", vertical_bins_)) {
    RCLCPP_DEBUG(get_logger(), "Setting new vertical_bins to: %d.", vertical_bins_);
  }
  if (get_param(p, "horizontal_bins", horizontal_bins_)) {
    RCLCPP_DEBUG(get_logger(), "Setting new horizontal_bins to: %d.", horizontal_bins_);
  }
  if (get_param(p, "noise_threshold", noise_threshold_)) {
    RCLCPP_DEBUG(get_logger(), "Setting new noise_threshold to: %d.", noise_threshold_);
  }
  if (get_param(p, "max_azimuth_deg", max_azimuth_deg_)) {
    RCLCPP_DEBUG(get_logger(), "Setting new max_azimuth_deg to: %f.", max_azimuth_deg_);
  }
  if (get_param(p, "min_azimuth_deg", min_azimuth_deg_)) {
    RCLCPP_DEBUG(get_logger(), "Setting new min_azimuth_deg to: %f.", min_azimuth_deg_);
  }
  if (get_param(p, "max_distance", max_distance_)) {
    RCLCPP_DEBUG(get_logger(), "Setting new max_distance to: %f.", max_distance_);
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  return result;
}

void RingOutlierFilterComponent::setUpPointCloudFormat(
  const PointCloud2ConstPtr & input, PointCloud2 & formatted_points, size_t points_size,
  size_t num_fields)
{
  formatted_points.data.resize(points_size);
  // Note that `input->header.frame_id` is data before converted when `transform_info.need_transform
  // == true`
  formatted_points.header.frame_id =
    !tf_input_frame_.empty() ? tf_input_frame_ : tf_input_orig_frame_;
  formatted_points.data.resize(formatted_points.point_step * input->width);
  formatted_points.height = 1;
  formatted_points.width =
    static_cast<uint32_t>(formatted_points.data.size() / formatted_points.point_step);
  formatted_points.is_bigendian = input->is_bigendian;
  formatted_points.is_dense = input->is_dense;

  sensor_msgs::PointCloud2Modifier pcd_modifier(formatted_points);
  pcd_modifier.setPointCloud2Fields(
    num_fields, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
    sensor_msgs::msg::PointField::FLOAT32, "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
}

float RingOutlierFilterComponent::calculateVisibilityScore(
  const sensor_msgs::msg::PointCloud2 & input)
{
  pcl::PointCloud<PointXYZIRADRT>::Ptr input_cloud(new pcl::PointCloud<PointXYZIRADRT>);
  pcl::fromROSMsg(input, *input_cloud);

  const uint32_t vertical_bins = vertical_bins_;
  const uint32_t horizontal_bins = horizontal_bins_;
  const float max_azimuth = max_azimuth_deg_ * 100.0;
  const float min_azimuth = min_azimuth_deg_ * 100.0;

  const uint32_t horizontal_resolution =
    static_cast<uint32_t>((max_azimuth - min_azimuth) / horizontal_bins);

  std::vector<pcl::PointCloud<PointXYZIRADRT>> ring_point_clouds(vertical_bins);
  cv::Mat frequency_image(cv::Size(horizontal_bins, vertical_bins), CV_8UC1, cv::Scalar(0));

  // Split points into rings
  for (const auto & point : input_cloud->points) {
    ring_point_clouds.at(point.ring).push_back(point);
  }

  // Calculate frequency for each bin in each ring
  for (const auto & ring_points : ring_point_clouds) {
    if (ring_points.empty()) continue;

    const uint ring_id = ring_points.front().ring;
    std::vector<int> frequency_in_ring(horizontal_bins, 0);

    for (const auto & point : ring_points.points) {
      if (point.azimuth < min_azimuth || point.azimuth >= max_azimuth) continue;
      if (point.distance >= max_distance_) continue;

      const uint bin_index =
        static_cast<uint>((point.azimuth - min_azimuth) / horizontal_resolution);

      frequency_in_ring[bin_index]++;
      frequency_in_ring[bin_index] =
        std::min(frequency_in_ring[bin_index], 255);  // Ensure value is within uchar range

      frequency_image.at<uchar>(ring_id, bin_index) =
        static_cast<uchar>(frequency_in_ring[bin_index]);
    }
  }

  cv::Mat binary_image;
  cv::inRange(frequency_image, noise_threshold_, 255, binary_image);

  const int num_pixels = cv::countNonZero(frequency_image);
  const float num_filled_pixels =
    static_cast<float>(num_pixels) / static_cast<float>(vertical_bins * horizontal_bins);

  return 1.0f - num_filled_pixels;
}

}  // namespace pointcloud_preprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_preprocessor::RingOutlierFilterComponent)
