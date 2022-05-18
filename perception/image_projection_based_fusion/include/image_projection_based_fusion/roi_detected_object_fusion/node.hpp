// Copyright 2022 TIER IV, Inc.
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

#ifndef IMAGE_PROJECTION_BASED_FUSION__ROI_DETECTED_OBJECT_FUSION__NODE_HPP_
#define IMAGE_PROJECTION_BASED_FUSION__ROI_DETECTED_OBJECT_FUSION__NODE_HPP_

#include "image_projection_based_fusion/fusion_node.hpp"

#include <map>
#include <memory>
#include <vector>

namespace image_projection_based_fusion
{

class RoiDetectedObjectFusionNode : public FusionNode<DetectedObjects, DetectedObject>
{
public:
  explicit RoiDetectedObjectFusionNode(const rclcpp::NodeOptions & options);

protected:
  void fuseOnSingleImage(
    const DetectedObjects & input_object_msg, const std::size_t image_id,
    const DetectedObjectsWithFeature & input_roi_msg,
    const sensor_msgs::msg::CameraInfo & camera_info, DetectedObjects & output_object_msg) override;

  void generateDetectedObjectRois(
    const std::vector<DetectedObject> & input_objects, const double image_width,
    const double image_height, const Eigen::Affine3d & object2camera_affine,
    const Eigen::Matrix4d & camera_projection,
    std::map<std::size_t, sensor_msgs::msg::RegionOfInterest> & object_roi_map);

  void updateDetectedObjectClassification(
    const std::vector<DetectedObjectWithFeature> & image_rois,
    const std::map<std::size_t, sensor_msgs::msg::RegionOfInterest> & object_roi_map,
    std::vector<DetectedObject> & output_objects);

  bool use_iou_{false};
  bool use_iou_x_{false};
  bool use_iou_y_{false};
  float iou_threshold_{0.0f};

  bool out_of_scope(const DetectedObject & obj);
};

}  // namespace image_projection_based_fusion

#endif  // IMAGE_PROJECTION_BASED_FUSION__ROI_DETECTED_OBJECT_FUSION__NODE_HPP_
