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

#include "pointcloud_preprocessor/downsample_filter/faster_voxel_grid_downsample_filter.hpp"

namespace pointcloud_preprocessor
{

FasterVoxelGridDownsampleFilter::FasterVoxelGridDownsampleFilter()
{
  pcl::for_each_type<typename pcl::traits::fieldList<pcl::PointXYZ>::type>(
    pcl::detail::FieldAdder<pcl::PointXYZ>(xyz_fields_));
  offset_initialized_ = false;
}

void FasterVoxelGridDownsampleFilter::set_voxel_size(
  float voxel_size_x, float voxel_size_y, float voxel_size_z)
{
  inverse_voxel_size_ =
    Eigen::Array3f::Ones() / Eigen::Array3f(voxel_size_x, voxel_size_y, voxel_size_z);
}

void FasterVoxelGridDownsampleFilter::set_field_offsets(const PointCloud2ConstPtr & input)
{
  x_offset_ = input->fields[pcl::getFieldIndex(*input, "x")].offset;
  y_offset_ = input->fields[pcl::getFieldIndex(*input, "y")].offset;
  z_offset_ = input->fields[pcl::getFieldIndex(*input, "z")].offset;
  int intensity_index = pcl::getFieldIndex(*input, "intensity");
  if (intensity_index != -1) {
    intensity_offset_ = input->fields[intensity_index].offset;
  } else {
    intensity_offset_ = z_offset_ + sizeof(float);
  }
  offset_initialized_ = true;
}

void FasterVoxelGridDownsampleFilter::filter(
  const PointCloud2ConstPtr & input, PointCloud2 & output, const TransformInfo & transform_info,
  const rclcpp::Logger & logger)
{
  // Check if the field offset has been set
  if (!offset_initialized_) {
    set_field_offsets(input);
  }

  // Compute the minimum and maximum voxel coordinates
  Eigen::Vector3i min_voxel, max_voxel;
  if (!get_min_max_voxel(input, min_voxel, max_voxel)) {
    RCLCPP_ERROR(
      logger,
      "Voxel size is too small for the input dataset. "
      "Integer indices would overflow.");
    output = *input;
    return;
  }

  // Storage for mapping voxel coordinates to centroids
  auto voxel_centroid_map = calc_centroids_each_voxel(input, max_voxel, min_voxel);

  // Initialize the output
  output.row_step = voxel_centroid_map.size() * input->point_step;
  output.data.resize(output.row_step);
  output.width = voxel_centroid_map.size();
  pcl_conversions::fromPCL(xyz_fields_, output.fields);
  output.is_dense = true;  // we filter out invalid points
  output.height = input->height;
  output.is_bigendian = input->is_bigendian;
  output.point_step = input->point_step;
  output.header = input->header;

  // Copy the centroids to the output
  copy_centroids_to_output(voxel_centroid_map, output, transform_info);
}

Eigen::Vector3f FasterVoxelGridDownsampleFilter::get_point_from_global_offset(
  const PointCloud2ConstPtr & input, size_t global_offset)
{
  Eigen::Vector3f point(
    *reinterpret_cast<const float *>(&input->data[global_offset + x_offset_]),
    *reinterpret_cast<const float *>(&input->data[global_offset + y_offset_]),
    *reinterpret_cast<const float *>(&input->data[global_offset + z_offset_]));
  return point;
}

bool FasterVoxelGridDownsampleFilter::get_min_max_voxel(
  const PointCloud2ConstPtr & input, Eigen::Vector3i & min_voxel, Eigen::Vector3i & max_voxel)
{
  // Compute the minimum and maximum point coordinates
  Eigen::Vector3f min_point, max_point;
  min_point.setConstant(FLT_MAX);
  max_point.setConstant(-FLT_MAX);
  for (size_t global_offset = 0; global_offset + input->point_step <= input->data.size();
       global_offset += input->point_step) {
    Eigen::Vector3f point = get_point_from_global_offset(input, global_offset);
    if (std::isfinite(point[0]) && std::isfinite(point[1]), std::isfinite(point[2])) {
      min_point = min_point.cwiseMin(point);
      max_point = max_point.cwiseMax(point);
    }
  }

  // Check that the voxel size is not too small, given the size of the data
  if (
    ((static_cast<std::int64_t>((max_point[0] - min_point[0]) * inverse_voxel_size_[0]) + 1) *
     (static_cast<std::int64_t>((max_point[1] - min_point[1]) * inverse_voxel_size_[1]) + 1) *
     (static_cast<std::int64_t>((max_point[2] - min_point[2]) * inverse_voxel_size_[2]) + 1)) >
    static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::max())) {
    return false;
  }

  // Compute the minimum and maximum voxel coordinates
  min_voxel[0] = static_cast<int>(std::floor(min_point[0] * inverse_voxel_size_[0]));
  min_voxel[1] = static_cast<int>(std::floor(min_point[1] * inverse_voxel_size_[1]));
  min_voxel[2] = static_cast<int>(std::floor(min_point[2] * inverse_voxel_size_[2]));
  max_voxel[0] = static_cast<int>(std::floor(max_point[0] * inverse_voxel_size_[0]));
  max_voxel[1] = static_cast<int>(std::floor(max_point[1] * inverse_voxel_size_[1]));
  max_voxel[2] = static_cast<int>(std::floor(max_point[2] * inverse_voxel_size_[2]));

  return true;
}

std::unordered_map<uint32_t, FasterVoxelGridDownsampleFilter::Centroid>
FasterVoxelGridDownsampleFilter::calc_centroids_each_voxel(
  const PointCloud2ConstPtr & input, const Eigen::Vector3i & max_voxel,
  const Eigen::Vector3i & min_voxel)
{
  std::unordered_map<uint32_t, Centroid> voxel_centroid_map;
  // Compute the number of divisions needed along all axis
  Eigen::Vector3i div_b = max_voxel - min_voxel + Eigen::Vector3i::Ones();
  // Set up the division multiplier
  Eigen::Vector3i div_b_mul(1, div_b[0], div_b[0] * div_b[1]);

  for (size_t global_offset = 0; global_offset + input->point_step <= input->data.size();
       global_offset += input->point_step) {
    Eigen::Vector3f point = get_point_from_global_offset(input, global_offset);
    if (std::isfinite(point[0]) && std::isfinite(point[1]), std::isfinite(point[2])) {
      // Calculate the voxel index to which the point belongs
      int ijk0 = static_cast<int>(std::floor(point[0] * inverse_voxel_size_[0]) - min_voxel[0]);
      int ijk1 = static_cast<int>(std::floor(point[1] * inverse_voxel_size_[1]) - min_voxel[1]);
      int ijk2 = static_cast<int>(std::floor(point[2] * inverse_voxel_size_[2]) - min_voxel[2]);
      uint32_t voxel_id = ijk0 * div_b_mul[0] + ijk1 * div_b_mul[1] + ijk2 * div_b_mul[2];

      // Add the point to the corresponding centroid
      if (voxel_centroid_map.find(voxel_id) == voxel_centroid_map.end()) {
        voxel_centroid_map[voxel_id] = Centroid(point[0], point[1], point[2]);
      } else {
        voxel_centroid_map[voxel_id].add_point(point[0], point[1], point[2]);
      }
    }
  }

  return voxel_centroid_map;
}

void FasterVoxelGridDownsampleFilter::copy_centroids_to_output(
  std::unordered_map<uint32_t, Centroid> & voxel_centroid_map, PointCloud2 & output,
  const TransformInfo & transform_info)
{
  size_t output_data_size = 0;
  for (const auto & pair : voxel_centroid_map) {
    Eigen::Vector4f centroid = pair.second.calc_centroid();
    if (transform_info.need_transform) {
      centroid = transform_info.eigen_transform * centroid;
    }
    *reinterpret_cast<float *>(&output.data[output_data_size + x_offset_]) = centroid[0];
    *reinterpret_cast<float *>(&output.data[output_data_size + y_offset_]) = centroid[1];
    *reinterpret_cast<float *>(&output.data[output_data_size + z_offset_]) = centroid[2];
    *reinterpret_cast<float *>(&output.data[output_data_size + intensity_offset_]) = centroid[3];
    output_data_size += output.point_step;
  }
}

}  // namespace pointcloud_preprocessor
