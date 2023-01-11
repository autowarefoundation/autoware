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
#include "tier4_autoware_utils/ros/debug_publisher.hpp"

#include <map>
#include <memory>
#include <vector>

namespace image_projection_based_fusion
{

using sensor_msgs::msg::RegionOfInterest;

class RoiDetectedObjectFusionNode : public FusionNode<DetectedObjects, DetectedObject>
{
public:
  explicit RoiDetectedObjectFusionNode(const rclcpp::NodeOptions & options);

protected:
  void preprocess(DetectedObjects & output_msg) override;

  void fuseOnSingleImage(
    const DetectedObjects & input_object_msg, const std::size_t image_id,
    const DetectedObjectsWithFeature & input_roi_msg,
    const sensor_msgs::msg::CameraInfo & camera_info, DetectedObjects & output_object_msg) override;

  std::map<std::size_t, RegionOfInterest> generateDetectedObjectRoIs(
    const std::vector<DetectedObject> & input_objects, const double image_width,
    const double image_height, const Eigen::Affine3d & object2camera_affine,
    const Eigen::Matrix4d & camera_projection);

  void fuseObjectsOnImage(
    const std::vector<DetectedObject> & objects,
    const std::vector<DetectedObjectWithFeature> & image_rois,
    const std::map<std::size_t, sensor_msgs::msg::RegionOfInterest> & object_roi_map);

  void publish(const DetectedObjects & output_msg) override;

  bool out_of_scope(const DetectedObject & obj);

private:
  struct
  {
    double passthrough_lower_bound_probability_threshold{};
    double min_iou_threshold{};
    bool use_roi_probability{};
    double roi_probability_threshold{};
  } fusion_params_;

  std::vector<bool> passthrough_object_flags_, fused_object_flags_, ignored_object_flags_;
};

}  // namespace image_projection_based_fusion

#endif  // IMAGE_PROJECTION_BASED_FUSION__ROI_DETECTED_OBJECT_FUSION__NODE_HPP_
