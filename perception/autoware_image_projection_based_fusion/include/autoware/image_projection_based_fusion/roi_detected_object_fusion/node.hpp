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

#ifndef AUTOWARE__IMAGE_PROJECTION_BASED_FUSION__ROI_DETECTED_OBJECT_FUSION__NODE_HPP_
#define AUTOWARE__IMAGE_PROJECTION_BASED_FUSION__ROI_DETECTED_OBJECT_FUSION__NODE_HPP_

#include "autoware/image_projection_based_fusion/fusion_node.hpp"
#include "autoware/universe_utils/ros/debug_publisher.hpp"

#include <autoware/image_projection_based_fusion/utils/utils.hpp>

#include "autoware_perception_msgs/msg/object_classification.hpp"

#include <map>
#include <memory>
#include <vector>

namespace autoware::image_projection_based_fusion
{

using sensor_msgs::msg::RegionOfInterest;

class RoiDetectedObjectFusionNode
: public FusionNode<DetectedObjects, DetectedObject, DetectedObjectsWithFeature>
{
public:
  explicit RoiDetectedObjectFusionNode(const rclcpp::NodeOptions & options);

protected:
  void preprocess(DetectedObjects & output_msg) override;

  void fuseOnSingleImage(
    const DetectedObjects & input_object_msg, const std::size_t image_id,
    const DetectedObjectsWithFeature & input_roi_msg,
    const sensor_msgs::msg::CameraInfo & camera_info, DetectedObjects & output_object_msg) override;

  std::map<std::size_t, DetectedObjectWithFeature> generateDetectedObjectRoIs(
    const DetectedObjects & input_object_msg, const double image_width, const double image_height,
    const Eigen::Affine3d & object2camera_affine,
    const image_geometry::PinholeCameraModel & pinhole_camera_model);

  void fuseObjectsOnImage(
    const DetectedObjects & input_object_msg,
    const std::vector<DetectedObjectWithFeature> & image_rois,
    const std::map<std::size_t, DetectedObjectWithFeature> & object_roi_map);

  void publish(const DetectedObjects & output_msg) override;

  bool out_of_scope(const DetectedObject & obj);

private:
  struct
  {
    std::vector<double> passthrough_lower_bound_probability_thresholds{};
    std::vector<double> trust_distances{};
    double min_iou_threshold{};
    bool use_roi_probability{};
    double roi_probability_threshold{};
    Eigen::MatrixXi can_assign_matrix;
  } fusion_params_;

  std::map<int64_t, std::vector<bool>> passthrough_object_flags_map_, fused_object_flags_map_,
    ignored_object_flags_map_;
};

}  // namespace autoware::image_projection_based_fusion

#endif  // AUTOWARE__IMAGE_PROJECTION_BASED_FUSION__ROI_DETECTED_OBJECT_FUSION__NODE_HPP_
