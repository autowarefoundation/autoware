// Copyright 2020 TIER IV, Inc.
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

#ifndef IMAGE_PROJECTION_BASED_FUSION__ROI_CLUSTER_FUSION__NODE_HPP_
#define IMAGE_PROJECTION_BASED_FUSION__ROI_CLUSTER_FUSION__NODE_HPP_

#include "image_projection_based_fusion/fusion_node.hpp"

#include <memory>

namespace image_projection_based_fusion
{

class RoiClusterFusionNode
: public FusionNode<DetectedObjectsWithFeature, DetectedObjectWithFeature>
{
public:
  explicit RoiClusterFusionNode(const rclcpp::NodeOptions & options);

protected:
  void preprocess(DetectedObjectsWithFeature & output_cluster_msg) override;

  void fuseOnSingleImage(
    const DetectedObjectsWithFeature & input_cluster_msg, const std::size_t image_id,
    const DetectedObjectsWithFeature & input_roi_msg,
    const sensor_msgs::msg::CameraInfo & camera_info,
    DetectedObjectsWithFeature & output_cluster_msg) override;

  bool use_iou_x_{false};
  bool use_iou_y_{false};
  bool use_iou_{false};
  bool use_cluster_semantic_type_{false};
  float iou_threshold_{0.0f};

  bool out_of_scope(const DetectedObjectWithFeature & obj);
};

}  // namespace image_projection_based_fusion

#endif  // IMAGE_PROJECTION_BASED_FUSION__ROI_CLUSTER_FUSION__NODE_HPP_
