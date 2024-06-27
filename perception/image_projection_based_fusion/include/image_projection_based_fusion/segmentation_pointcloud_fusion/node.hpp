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

#ifndef IMAGE_PROJECTION_BASED_FUSION__SEGMENTATION_POINTCLOUD_FUSION__NODE_HPP_
#define IMAGE_PROJECTION_BASED_FUSION__SEGMENTATION_POINTCLOUD_FUSION__NODE_HPP_

#include "image_projection_based_fusion/fusion_node.hpp"

#include <image_projection_based_fusion/utils/utils.hpp>

#include <string>
#include <vector>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

namespace image_projection_based_fusion
{
class SegmentPointCloudFusionNode : public FusionNode<PointCloud2, PointCloud2, Image>
{
private:
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_pointcloud_ptr_;
  std::vector<bool> filter_semantic_label_target_;
  float filter_distance_threshold_;
  /* data */
public:
  explicit SegmentPointCloudFusionNode(const rclcpp::NodeOptions & options);

protected:
  void preprocess(PointCloud2 & pointcloud_msg) override;

  void postprocess(PointCloud2 & pointcloud_msg) override;

  void fuseOnSingleImage(
    const PointCloud2 & input_pointcloud_msg, const std::size_t image_id, const Image & input_mask,
    const CameraInfo & camera_info, PointCloud2 & output_pointcloud_msg) override;

  bool out_of_scope(const PointCloud2 & filtered_cloud);
  inline void copyPointCloud(
    const PointCloud2 & input, const int point_step, const size_t global_offset,
    PointCloud2 & output, size_t & output_pointcloud_size)
  {
    std::memcpy(&output.data[output_pointcloud_size], &input.data[global_offset], point_step);
    output_pointcloud_size += point_step;
  }
};

}  // namespace image_projection_based_fusion
#endif  // IMAGE_PROJECTION_BASED_FUSION__SEGMENTATION_POINTCLOUD_FUSION__NODE_HPP_
