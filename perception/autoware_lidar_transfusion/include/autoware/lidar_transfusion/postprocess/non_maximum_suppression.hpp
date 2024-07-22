// Copyright 2024 TIER IV, Inc.
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

#ifndef AUTOWARE__LIDAR_TRANSFUSION__POSTPROCESS__NON_MAXIMUM_SUPPRESSION_HPP_
#define AUTOWARE__LIDAR_TRANSFUSION__POSTPROCESS__NON_MAXIMUM_SUPPRESSION_HPP_

#include "autoware/lidar_transfusion/ros_utils.hpp"

#include <Eigen/Eigen>

#include <autoware_perception_msgs/msg/detected_object.hpp>

#include <string>
#include <vector>

namespace autoware::lidar_transfusion
{
using autoware_perception_msgs::msg::DetectedObject;

enum class NMS_TYPE {
  IoU_BEV
  // IoU_3D
  // Distance_2D
  // Distance_3D
};

struct NMSParams
{
  NMS_TYPE nms_type_{};
  std::vector<std::string> target_class_names_{};
  double search_distance_2d_{};
  double iou_threshold_{};
  // double distance_threshold_{};
};

std::vector<bool> classNamesToBooleanMask(const std::vector<std::string> & class_names)
{
  std::vector<bool> mask;
  constexpr std::size_t num_object_classification = 8;
  mask.resize(num_object_classification);
  for (const auto & class_name : class_names) {
    const auto semantic_type = getSemanticType(class_name);
    mask.at(semantic_type) = true;
  }

  return mask;
}

class NonMaximumSuppression
{
public:
  void setParameters(const NMSParams &);

  std::vector<DetectedObject> apply(const std::vector<DetectedObject> &);

private:
  bool isTargetLabel(const std::uint8_t);

  bool isTargetPairObject(const DetectedObject &, const DetectedObject &);

  Eigen::MatrixXd generateIoUMatrix(const std::vector<DetectedObject> &);

  NMSParams params_{};
  std::vector<bool> target_class_mask_{};
};

}  // namespace autoware::lidar_transfusion

#endif  // AUTOWARE__LIDAR_TRANSFUSION__POSTPROCESS__NON_MAXIMUM_SUPPRESSION_HPP_
