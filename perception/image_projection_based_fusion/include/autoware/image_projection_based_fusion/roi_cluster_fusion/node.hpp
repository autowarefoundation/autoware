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

#ifndef AUTOWARE__IMAGE_PROJECTION_BASED_FUSION__ROI_CLUSTER_FUSION__NODE_HPP_
#define AUTOWARE__IMAGE_PROJECTION_BASED_FUSION__ROI_CLUSTER_FUSION__NODE_HPP_

#include "autoware/image_projection_based_fusion/fusion_node.hpp"

#include <map>
#include <memory>
#include <string>
namespace autoware::image_projection_based_fusion
{
const std::map<std::string, uint8_t> IOU_MODE_MAP{{"iou", 0}, {"iou_x", 1}, {"iou_y", 2}};

class RoiClusterFusionNode
: public FusionNode<
    DetectedObjectsWithFeature, DetectedObjectWithFeature, DetectedObjectsWithFeature>
{
public:
  explicit RoiClusterFusionNode(const rclcpp::NodeOptions & options);

protected:
  void preprocess(DetectedObjectsWithFeature & output_cluster_msg) override;
  void postprocess(DetectedObjectsWithFeature & output_cluster_msg) override;

  void fuseOnSingleImage(
    const DetectedObjectsWithFeature & input_cluster_msg, const std::size_t image_id,
    const DetectedObjectsWithFeature & input_roi_msg,
    const sensor_msgs::msg::CameraInfo & camera_info,
    DetectedObjectsWithFeature & output_cluster_msg) override;

  std::string trust_object_iou_mode_{"iou"};
  bool use_cluster_semantic_type_{false};
  bool only_allow_inside_cluster_{false};
  double roi_scale_factor_{1.1};
  double iou_threshold_{0.0};
  double unknown_iou_threshold_{0.0};
  const float min_roi_existence_prob_ =
    0.1;  // keep small value to lessen affect on merger object stage
  bool remove_unknown_;
  double fusion_distance_;
  double trust_object_distance_;
  std::string non_trust_object_iou_mode_{"iou_x"};
  bool is_far_enough(const DetectedObjectWithFeature & obj, const double distance_threshold);
  bool out_of_scope(const DetectedObjectWithFeature & obj);
  double cal_iou_by_mode(
    const sensor_msgs::msg::RegionOfInterest & roi_1,
    const sensor_msgs::msg::RegionOfInterest & roi_2, const std::string iou_mode);
  // bool CheckUnknown(const DetectedObjectsWithFeature & obj);
};

}  // namespace autoware::image_projection_based_fusion

#endif  // AUTOWARE__IMAGE_PROJECTION_BASED_FUSION__ROI_CLUSTER_FUSION__NODE_HPP_
