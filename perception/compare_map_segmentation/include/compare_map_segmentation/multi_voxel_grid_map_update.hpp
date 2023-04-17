// Copyright 2023 Autoware Foundation
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

#ifndef COMPARE_MAP_SEGMENTATION__MULTI_VOXEL_GRID_MAP_UPDATE_HPP_
#define COMPARE_MAP_SEGMENTATION__MULTI_VOXEL_GRID_MAP_UPDATE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <boost/sort/spreadsort/integer_sort.hpp>

#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>

#include <cfloat>  // for FLT_MAX
#include <iostream>
#include <limits>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace compare_map_segmentation
{
template <typename PointT>
// TODO(badai-nguyen): when map loader I/F is updated, remove this class since
// boundary point calculation become unnecessary
class MultiVoxelGrid : public pcl::VoxelGrid<PointT>
{
  typedef std::map<std::string, pcl::PointCloud<PointT>> MapCellDict;
  struct cloud_point_index_idx
  {
    unsigned int idx;
    unsigned int cloud_point_index;

    cloud_point_index_idx() = default;
    cloud_point_index_idx(unsigned int idx_, unsigned int cloud_point_index_)
    : idx(idx_), cloud_point_index(cloud_point_index_)
    {
    }
    bool operator<(const cloud_point_index_idx & p) const { return (idx < p.idx); }
  };

protected:
  using pcl::VoxelGrid<PointT>::filter_name_;
  using pcl::VoxelGrid<PointT>::downsample_all_data_;
  using pcl::VoxelGrid<PointT>::input_;
  using pcl::VoxelGrid<PointT>::save_leaf_layout_;
  using pcl::VoxelGrid<PointT>::min_b_;
  using pcl::VoxelGrid<PointT>::max_b_;
  using pcl::VoxelGrid<PointT>::divb_mul_;
  using pcl::VoxelGrid<PointT>::div_b_;
  using pcl::VoxelGrid<PointT>::inverse_leaf_size_;

  using pcl::VoxelGrid<PointT>::filter_limit_negative_;
  using pcl::VoxelGrid<PointT>::filter_limit_max_;
  using pcl::VoxelGrid<PointT>::filter_limit_min_;
  using pcl::VoxelGrid<PointT>::indices_;
  using pcl::VoxelGrid<PointT>::min_points_per_voxel_;
  using pcl::VoxelGrid<PointT>::filter_field_name_;

  using PointCloud = typename pcl::Filter<PointT>::PointCloud;
  using PointCloudPtr = typename PointCloud::Ptr;
  using PointCloudConstPtr = typename PointCloud::ConstPtr;

public:
  using pcl::VoxelGrid<PointT>::leaf_layout_;

  inline void set_voxel_grid(
    std::vector<int> * leaf_layout, const Eigen::Vector4i & min_b, const Eigen::Vector4i & max_b,
    const Eigen::Vector4i & div_b, const Eigen::Vector4i & divb_mul,
    const Eigen::Array4f & inverse_leaf_size)
  {
    leaf_layout_ = std::move(*leaf_layout);
    min_b_ = min_b;
    max_b_ = max_b;
    div_b_ = div_b;
    divb_mul_ = divb_mul;
    inverse_leaf_size_ = inverse_leaf_size;
  }
  inline Eigen::Vector4f get_min_p() const { return min_p; }
  inline Eigen::Vector4f get_max_p() const { return max_p; }
  inline Eigen::Vector4i get_min_b() const { return min_b_; }
  inline Eigen::Vector4i get_divb_mul() const { return divb_mul_; }
  inline Eigen::Vector4i get_max_b() const { return max_b_; }
  inline Eigen::Vector4i get_div_b() const { return div_b_; }
  inline Eigen::Array4f get_inverse_leaf_size() const { return inverse_leaf_size_; }
  inline std::vector<int> getLeafLayout() { return (leaf_layout_); }

  // TODO(badai-nguyen): when map loader I/F is updated, use default Voxel applyFilter since
  // boundary point calculation become unnecessary
  inline void applyFilter(PointCloud & output) override
  {
    // Has the input dataset been set already?
    if (!input_) {
      // pcl::PCL_WARN ("[pcl::applyFilter] No input dataset given!\n");
      output.width = output.height = 0;
      output.clear();
      return;
    }

    // Copy the header (and thus the frame_id) + allocate enough space for points
    output.height = 1;       // downsampling breaks the organized structure
    output.is_dense = true;  // we filter out invalid points

    // Get the minimum and maximum dimensions
    if (!filter_field_name_.empty())  // If we don't want to process the entire cloud...
      pcl::getMinMax3D<PointT>(
        input_, *indices_, filter_field_name_, static_cast<float>(filter_limit_min_),
        static_cast<float>(filter_limit_max_), min_p, max_p, filter_limit_negative_);
    else
      pcl::getMinMax3D<PointT>(*input_, *indices_, min_p, max_p);

    // Check that the leaf size is not too small, given the size of the data
    std::int64_t dx = static_cast<std::int64_t>((max_p[0] - min_p[0]) * inverse_leaf_size_[0]) + 1;
    std::int64_t dy = static_cast<std::int64_t>((max_p[1] - min_p[1]) * inverse_leaf_size_[1]) + 1;
    std::int64_t dz = static_cast<std::int64_t>((max_p[2] - min_p[2]) * inverse_leaf_size_[2]) + 1;

    if ((dx * dy * dz) > static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::max())) {
      // pcl::PCL_WARN("[pcl::applyFilter] Leaf size is too small for the input dataset. Integer
      // indices would overflow.\n");
      output = *input_;
      return;
    }

    // Compute the minimum and maximum bounding box values
    min_b_[0] = static_cast<int>(std::floor(min_p[0] * inverse_leaf_size_[0]));
    max_b_[0] = static_cast<int>(std::floor(max_p[0] * inverse_leaf_size_[0]));
    min_b_[1] = static_cast<int>(std::floor(min_p[1] * inverse_leaf_size_[1]));
    max_b_[1] = static_cast<int>(std::floor(max_p[1] * inverse_leaf_size_[1]));
    min_b_[2] = static_cast<int>(std::floor(min_p[2] * inverse_leaf_size_[2]));
    max_b_[2] = static_cast<int>(std::floor(max_p[2] * inverse_leaf_size_[2]));

    // Compute the number of divisions needed along all axis
    div_b_ = max_b_ - min_b_ + Eigen::Vector4i::Ones();
    div_b_[3] = 0;

    // Set up the division multiplier
    divb_mul_ = Eigen::Vector4i(1, div_b_[0], div_b_[0] * div_b_[1], 0);

    // Storage for mapping leaf and pointcloud indexes
    std::vector<cloud_point_index_idx> index_vector;
    index_vector.reserve(indices_->size());

    // If we don't want to process the entire cloud, but rather filter points far away from the
    // viewpoint first...
    if (!filter_field_name_.empty()) {
      // Get the distance field index
      std::vector<pcl::PCLPointField> fields;
      int distance_idx = pcl::getFieldIndex<PointT>(filter_field_name_, fields);
      if (distance_idx == -1)

        // First pass: go over all points and insert them into the index_vector vector
        // with calculated idx. Points with the same idx value will contribute to the
        // same point of resulting CloudPoint
        for (const auto & index : (*indices_)) {
          if (!input_->is_dense)
            // Check if the point is invalid
            if (!pcl::isXYZFinite((*input_)[index])) continue;

          // Get the distance value
          const std::uint8_t * pt_data = reinterpret_cast<const std::uint8_t *>(&(*input_)[index]);
          float distance_value = 0;
          memcpy(&distance_value, pt_data + fields[distance_idx].offset, sizeof(float));

          if (filter_limit_negative_) {
            // Use a threshold for cutting out points which inside the interval
            if ((distance_value < filter_limit_max_) && (distance_value > filter_limit_min_))
              continue;
          } else {
            // Use a threshold for cutting out points which are too close/far away
            if ((distance_value > filter_limit_max_) || (distance_value < filter_limit_min_))
              continue;
          }

          int ijk0 = static_cast<int>(
            std::floor((*input_)[index].x * inverse_leaf_size_[0]) - static_cast<float>(min_b_[0]));
          int ijk1 = static_cast<int>(
            std::floor((*input_)[index].y * inverse_leaf_size_[1]) - static_cast<float>(min_b_[1]));
          int ijk2 = static_cast<int>(
            std::floor((*input_)[index].z * inverse_leaf_size_[2]) - static_cast<float>(min_b_[2]));

          // Compute the centroid leaf index
          int idx = ijk0 * divb_mul_[0] + ijk1 * divb_mul_[1] + ijk2 * divb_mul_[2];
          index_vector.emplace_back(static_cast<unsigned int>(idx), index);
        }
    } else {
      // No distance filtering, process all data
      // First pass: go over all points and insert them into the index_vector vector
      // with calculated idx. Points with the same idx value will contribute to the
      // same point of resulting CloudPoint
      for (const auto & index : (*indices_)) {
        if (!input_->is_dense)
          // Check if the point is invalid
          if (!pcl::isXYZFinite((*input_)[index])) continue;

        int ijk0 = static_cast<int>(
          std::floor((*input_)[index].x * inverse_leaf_size_[0]) - static_cast<float>(min_b_[0]));
        int ijk1 = static_cast<int>(
          std::floor((*input_)[index].y * inverse_leaf_size_[1]) - static_cast<float>(min_b_[1]));
        int ijk2 = static_cast<int>(
          std::floor((*input_)[index].z * inverse_leaf_size_[2]) - static_cast<float>(min_b_[2]));

        // Compute the centroid leaf index
        int idx = ijk0 * divb_mul_[0] + ijk1 * divb_mul_[1] + ijk2 * divb_mul_[2];
        index_vector.emplace_back(static_cast<unsigned int>(idx), index);
      }
    }

    // Second pass: sort the index_vector vector using value representing target cell as index
    // in effect all points belonging to the same output cell will be next to each other
    auto rightshift_func = [](const cloud_point_index_idx & x, const unsigned offset) {
      return x.idx >> offset;
    };
    boost::sort::spreadsort::integer_sort(
      index_vector.begin(), index_vector.end(), rightshift_func);

    // Third pass: count output cells
    // we need to skip all the same, adjacent idx values
    unsigned int total = 0;
    unsigned int index = 0;
    // first_and_last_indices_vector[i] represents the index in index_vector of the first point in
    // index_vector belonging to the voxel which corresponds to the i-th output point,
    // and of the first point not belonging to.
    std::vector<std::pair<unsigned int, unsigned int>> first_and_last_indices_vector;
    // Worst case size
    first_and_last_indices_vector.reserve(index_vector.size());
    while (index < index_vector.size()) {
      unsigned int i = index + 1;
      while (i < index_vector.size() && index_vector[i].idx == index_vector[index].idx) ++i;
      if (i - index >= min_points_per_voxel_) {
        ++total;
        first_and_last_indices_vector.emplace_back(index, i);
      }
      index = i;
    }

    // Fourth pass: compute centroids, insert them into their final position
    output.resize(total);
    if (save_leaf_layout_) {
      try {
        // Resizing won't reset old elements to -1.  If leaf_layout_ has been used previously, it
        // needs to be re-initialized to -1
        std::uint32_t new_layout_size = div_b_[0] * div_b_[1] * div_b_[2];
        // This is the number of elements that need to be re-initialized to -1
        std::uint32_t reinit_size = std::min(
          static_cast<unsigned int>(new_layout_size),
          static_cast<unsigned int>(leaf_layout_.size()));
        for (std::uint32_t i = 0; i < reinit_size; i++) {
          leaf_layout_[i] = -1;
        }
        leaf_layout_.resize(new_layout_size, -1);
      } catch (std::bad_alloc &) {
        throw pcl::PCLException(
          "VoxelGrid bin size is too low; impossible to allocate memory for layout",
          "voxel_grid.hpp", "applyFilter");
      } catch (std::length_error &) {
        throw pcl::PCLException(
          "VoxelGrid bin size is too low; impossible to allocate memory for layout",
          "voxel_grid.hpp", "applyFilter");
      }
    }

    index = 0;
    for (const auto & cp : first_and_last_indices_vector) {
      // calculate centroid - sum values from all input points, that have the same idx value in
      // index_vector array
      unsigned int first_index = cp.first;
      unsigned int last_index = cp.second;

      // index is centroid final position in resulting PointCloud
      if (save_leaf_layout_) leaf_layout_[index_vector[first_index].idx] = index;

      // Limit downsampling to coords
      if (!downsample_all_data_) {
        Eigen::Vector4f centroid(Eigen::Vector4f::Zero());

        for (unsigned int li = first_index; li < last_index; ++li)
          centroid += (*input_)[index_vector[li].cloud_point_index].getVector4fMap();

        centroid /= static_cast<float>(last_index - first_index);
        output[index].getVector4fMap() = centroid;
      } else {
        pcl::CentroidPoint<PointT> centroid;

        // fill in the accumulator with leaf points
        for (unsigned int li = first_index; li < last_index; ++li)
          centroid.add((*input_)[index_vector[li].cloud_point_index]);

        centroid.get(output[index]);
      }

      ++index;
    }
    output.width = output.size();
  }

private:
  Eigen::Vector4f min_p, max_p;
};

}  // namespace compare_map_segmentation

#endif  // COMPARE_MAP_SEGMENTATION__MULTI_VOXEL_GRID_MAP_UPDATE_HPP_
