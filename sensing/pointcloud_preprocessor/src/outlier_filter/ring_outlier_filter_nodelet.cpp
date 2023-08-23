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

#include "pointcloud_preprocessor/utility/utilities.hpp"

#include <range/v3/view/chunk.hpp>
#include <range/v3/view/transform.hpp>
#include <range/v3/view/zip.hpp>

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <algorithm>
#include <cstddef>
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
    max_rings_num_ = static_cast<uint16_t>(declare_parameter("max_rings_num", 128));
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

  // The ring_outlier_filter specifies the expected input point cloud format,
  // however, we want to verify the input is correct and make failures explicit.
  auto getFieldOffsetSafely = [=](
                                const std::string & field_name,
                                const pcl::PCLPointField::PointFieldTypes expected_type) -> size_t {
    const auto field_index = pcl::getFieldIndex(*input, field_name);
    if (field_index == -1) {
      RCLCPP_ERROR(get_logger(), "Field %s not found in input point cloud", field_name.c_str());
      return -1UL;
    }

    auto field = (*input).fields.at(field_index);
    if (field.datatype != expected_type) {
      RCLCPP_ERROR(
        get_logger(), "Field %s has unexpected type %d (expected %d)", field_name.c_str(),
        field.datatype, expected_type);
      return -1UL;
    }

    return static_cast<size_t>(field.offset);
  };

  // as per the specification of this node, these fields must be present in the input
  const auto ring_offset = getFieldOffsetSafely("ring", pcl::PCLPointField::UINT16);
  const auto azimuth_offset = getFieldOffsetSafely("azimuth", pcl::PCLPointField::FLOAT32);
  const auto distance_offset = getFieldOffsetSafely("distance", pcl::PCLPointField::FLOAT32);
  const auto intensity_offset = getFieldOffsetSafely("intensity", pcl::PCLPointField::FLOAT32);

  if (
    ring_offset == -1UL || azimuth_offset == -1UL || distance_offset == -1UL ||
    intensity_offset == -1UL) {
    RCLCPP_ERROR(get_logger(), "One or more required fields are missing in input point cloud");
    return;
  }

  // The initial implementation of ring outlier filter looked like this:
  //   1. Iterate over the input cloud and group point indices by ring
  //   2. For each ring:
  //   2.1. iterate over the ring points, and group points belonging to the same "walk"
  //   2.2. when a walk is closed, copy indexed points to the output cloud if the walk is long
  //   enough.
  //
  // Because LIDAR data is naturally "azimuth-major order" and not "ring-major order", such
  // implementation is not cache friendly at all, and has negative impact on all the other nodes.
  //
  // To tackle this issue, the algorithm has been rewritten so that points would be accessed in
  // order. To do so, all rings are being processing simultaneously instead of separately. The
  // overall logic is still the same.

  // ad-hoc struct to store finished walks information (for isCluster())
  struct WalkInfo
  {
    size_t id;
    int num_points;
    float first_point_distance;
    float first_point_azimuth;
    float last_point_distance;
    float last_point_azimuth;
  };

  // ad-hoc struct to keep track of each ring current walk
  struct RingWalkInfo
  {
    WalkInfo first_walk;
    WalkInfo current_walk;
  };

  // helper functions

  // check if walk is a valid cluster
  const float object_length_threshold2 = object_length_threshold_ * object_length_threshold_;
  auto isCluster = [=](const WalkInfo & walk_info) {
    // A cluster is a walk which has many points or is long enough
    if (walk_info.num_points > num_points_threshold_) return true;

    // Using the law of cosines, the distance between 2 points can be written as:
    //   |p2-p1|^2 = d1^2 + d2^2 - 2*d1*d2*cos(a)
    // where d1 and d2 are the distance attribute of p1 and p2, and 'a' the azimuth diff between
    // the 2 points.
    const float dist2 =
      walk_info.first_point_distance * walk_info.first_point_distance +
      walk_info.last_point_distance * walk_info.last_point_distance -
      2 * walk_info.first_point_distance * walk_info.last_point_distance *
        std::cos((walk_info.last_point_azimuth - walk_info.first_point_azimuth) * (M_PI / 18000.0));
    return dist2 > object_length_threshold2;
  };

  // check if 2 points belong to the same walk
  auto isSameWalk =
    [=](float curr_distance, float curr_azimuth, float prev_distance, float prev_azimuth) {
      float azimuth_diff = curr_azimuth - prev_azimuth;
      azimuth_diff = azimuth_diff < 0.f ? azimuth_diff + 36000.f : azimuth_diff;
      return std::max(curr_distance, prev_distance) <
               std::min(curr_distance, prev_distance) * distance_ratio_ &&
             azimuth_diff < 100.f;
    };

  // tmp vectors to keep track of walk/ring state while processing points in order (cache efficient)
  std::vector<RingWalkInfo> rings;     // info for each LiDAR ring
  std::vector<size_t> points_walk_id;  // for each input point, the walk index associated with it
  std::vector<bool>
    walks_cluster_status;  // for each generated walk, stores whether it is a cluster

  size_t latest_walk_id = -1UL;  // ID given to the latest walk created

  // initialize each ring with two empty walks (first and current walk)
  rings.resize(max_rings_num_, RingWalkInfo{{-1UL, 0, 0, 0, 0, 0}, {-1UL, 0, 0, 0, 0, 0}});
  // points are initially associated to no walk (-1UL)
  points_walk_id.resize(input->width * input->height, -1UL);
  walks_cluster_status.reserve(
    max_rings_num_ * 2);  // In the worst case, this could grow to the number of input points

  int invalid_ring_count = 0;

  // Build walks and classify points
  for (const auto & [raw_p, point_walk_id] :
       ranges::views::zip(input->data | ranges::views::chunk(input->point_step), points_walk_id)) {
    uint16_t ring_idx{};
    float curr_azimuth{};
    float curr_distance{};
    std::memcpy(&ring_idx, &raw_p.data()[ring_offset], sizeof(ring_idx));
    std::memcpy(&curr_azimuth, &raw_p.data()[azimuth_offset], sizeof(curr_azimuth));
    std::memcpy(&curr_distance, &raw_p.data()[distance_offset], sizeof(curr_distance));

    if (ring_idx >= max_rings_num_) {
      // Either the data is corrupted or max_rings_num_ is not set correctly
      // Note: point_walk_id == -1 so the point will be filtered out
      ++invalid_ring_count;
      continue;
    }

    auto & ring = rings[ring_idx];
    if (ring.current_walk.id == -1UL) {
      // first walk ever for this ring. It is both the first and current walk of the ring.
      ring.first_walk =
        WalkInfo{++latest_walk_id, 1, curr_distance, curr_azimuth, curr_distance, curr_azimuth};
      ring.current_walk = ring.first_walk;
      point_walk_id = latest_walk_id;
      continue;
    }

    auto & walk = ring.current_walk;
    if (isSameWalk(
          curr_distance, curr_azimuth, walk.last_point_distance, walk.last_point_azimuth)) {
      // current point is part of previous walk
      walk.num_points += 1;
      walk.last_point_distance = curr_distance;
      walk.last_point_azimuth = curr_azimuth;
      point_walk_id = walk.id;
    } else {
      // previous walk is finished, start a new one

      // check and store whether the previous walks is a cluster
      if (walk.id >= walks_cluster_status.size()) walks_cluster_status.resize(walk.id + 1, false);
      walks_cluster_status.at(walk.id) = isCluster(walk);

      ring.current_walk =
        WalkInfo{++latest_walk_id, 1, curr_distance, curr_azimuth, curr_distance, curr_azimuth};
      point_walk_id = latest_walk_id;
    }
  }

  // So far, we have processed ring points as if rings were not circular. Of course, the last and
  // first points of a ring could totally be part of the same walk. When such thing happens, we need
  // to merge the two walks
  for (auto & ring : rings) {
    if (ring.current_walk.id == -1UL) {
      continue;
    }

    const auto & walk = ring.current_walk;
    if (walk.id >= walks_cluster_status.size()) walks_cluster_status.resize(walk.id + 1, false);
    walks_cluster_status.at(walk.id) = isCluster(walk);

    if (ring.first_walk.id == ring.current_walk.id) {
      continue;
    }

    auto & first_walk = ring.first_walk;
    auto & last_walk = ring.current_walk;

    // check if the two walks are connected
    if (isSameWalk(
          first_walk.first_point_distance, first_walk.first_point_azimuth,
          last_walk.last_point_distance, last_walk.last_point_azimuth)) {
      // merge
      auto combined_num_points = first_walk.num_points + last_walk.num_points;
      first_walk.first_point_distance = last_walk.first_point_distance;
      first_walk.first_point_azimuth = last_walk.first_point_azimuth;
      first_walk.num_points = combined_num_points;
      last_walk.last_point_distance = first_walk.last_point_distance;
      last_walk.last_point_azimuth = first_walk.last_point_azimuth;
      last_walk.num_points = combined_num_points;

      walks_cluster_status.at(first_walk.id) = isCluster(first_walk);
      walks_cluster_status.at(last_walk.id) = isCluster(last_walk);
    }
  }

  // finally copy points
  output.point_step = sizeof(PointXYZI);
  output.data.resize(output.point_step * input->width * input->height);
  size_t output_size = 0;
  if (transform_info.need_transform) {
    for (const auto & [raw_p, point_walk_id] : ranges::views::zip(
           input->data | ranges::views::chunk(input->point_step), points_walk_id)) {
      // Filter out points on invalid rings and points not in a cluster
      if (point_walk_id == -1UL || !walks_cluster_status.at(point_walk_id)) {
        continue;
      }

      // assume binary representation of input point is compatible with PointXYZ*
      PointXYZI out_point;
      std::memcpy(&out_point, raw_p.data(), sizeof(PointXYZI));

      Eigen::Vector4f p(out_point.x, out_point.y, out_point.z, 1);
      p = transform_info.eigen_transform * p;
      out_point.x = p[0];
      out_point.y = p[1];
      out_point.z = p[2];
      // FIXME(VRichardJP) tmp fix because input data does not have the same layout than PointXYZI
      std::memcpy(
        &out_point.intensity, &raw_p.data()[intensity_offset], sizeof(out_point.intensity));

      std::memcpy(&output.data[output_size], &out_point, sizeof(PointXYZI));
      output_size += sizeof(PointXYZI);
    }
  } else {
    for (const auto & [raw_p, point_walk_id] : ranges::views::zip(
           input->data | ranges::views::chunk(input->point_step), points_walk_id)) {
      // Filter out points on invalid rings and points not in a cluster
      if (point_walk_id == -1UL || !walks_cluster_status.at(point_walk_id)) {
        continue;
      }

      PointXYZI out_point;
      std::memcpy(&out_point, raw_p.data(), sizeof(PointXYZI));
      // FIXME(VRichardJP) tmp fix because input data does not have the same layout than PointXYZI
      std::memcpy(
        &out_point.intensity, &raw_p.data()[intensity_offset], sizeof(out_point.intensity));

      std::memcpy(&output.data[output_size], &out_point, sizeof(PointXYZI));

      output_size += sizeof(PointXYZI);
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

  if (invalid_ring_count > 0) {
    RCLCPP_WARN(
      get_logger(), "%d points had ring index over max_rings_num (%d) and have been ignored.",
      invalid_ring_count, max_rings_num_);
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

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  return result;
}
}  // namespace pointcloud_preprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_preprocessor::RingOutlierFilterComponent)
