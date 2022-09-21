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

#ifndef LIDAR_CENTERPOINT__DETECTION_CLASS_REMAPPER_HPP_
#define LIDAR_CENTERPOINT__DETECTION_CLASS_REMAPPER_HPP_

#include <Eigen/Core>

#include <autoware_auto_perception_msgs/msg/detected_object_kinematics.hpp>
#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>
#include <autoware_auto_perception_msgs/msg/object_classification.hpp>
#include <autoware_auto_perception_msgs/msg/shape.hpp>

#include <vector>

namespace centerpoint
{

class DetectionClassRemapper
{
public:
  void setParameters(
    const std::vector<int64_t> & allow_remapping_by_area_matrix,
    const std::vector<double> & min_area_matrix, const std::vector<double> & max_area_matrix);
  void mapClasses(autoware_auto_perception_msgs::msg::DetectedObjects & msg);

protected:
  Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> allow_remapping_by_area_matrix_;
  Eigen::MatrixXd min_area_matrix_;
  Eigen::MatrixXd max_area_matrix_;
  int num_labels_;
};

}  // namespace centerpoint

#endif  // LIDAR_CENTERPOINT__DETECTION_CLASS_REMAPPER_HPP_
