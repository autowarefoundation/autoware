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

#ifndef IMAGE_PROJECTION_BASED_FUSION__ROI_POINTCLOUD_FUSION__NODE_HPP_
#define IMAGE_PROJECTION_BASED_FUSION__ROI_POINTCLOUD_FUSION__NODE_HPP_

#include "image_projection_based_fusion/fusion_node.hpp"

#include <string>
#include <vector>
namespace image_projection_based_fusion
{
class RoiPointCloudFusionNode
: public FusionNode<sensor_msgs::msg::PointCloud2, DetectedObjectWithFeature>
{
private:
  int min_cluster_size_{1};
  bool fuse_unknown_only_{true};
  double cluster_2d_tolerance_;

  rclcpp::Publisher<DetectedObjectsWithFeature>::SharedPtr pub_objects_ptr_;
  std::vector<DetectedObjectWithFeature> output_fused_objects_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cluster_debug_pub_;

  /* data */
public:
  explicit RoiPointCloudFusionNode(const rclcpp::NodeOptions & options);

protected:
  void preprocess(sensor_msgs::msg::PointCloud2 & pointcloud_msg) override;

  void postprocess(sensor_msgs::msg::PointCloud2 & pointcloud_msg) override;

  void fuseOnSingleImage(
    const PointCloud2 & input_pointcloud_msg, const std::size_t image_id,
    const DetectedObjectsWithFeature & input_roi_msg,
    const sensor_msgs::msg::CameraInfo & camera_info, PointCloud2 & output_pointcloud_msg) override;
  bool out_of_scope(const DetectedObjectWithFeature & obj);
};

}  // namespace image_projection_based_fusion
#endif  // IMAGE_PROJECTION_BASED_FUSION__ROI_POINTCLOUD_FUSION__NODE_HPP_
