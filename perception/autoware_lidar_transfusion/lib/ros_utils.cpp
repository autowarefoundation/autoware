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

#include "autoware/lidar_transfusion/ros_utils.hpp"

#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware/universe_utils/math/constants.hpp>
#include <object_recognition_utils/object_recognition_utils.hpp>

namespace autoware::lidar_transfusion
{

using Label = autoware_perception_msgs::msg::ObjectClassification;

void box3DToDetectedObject(
  const Box3D & box3d, const std::vector<std::string> & class_names,
  autoware_perception_msgs::msg::DetectedObject & obj)
{
  obj.existence_probability = box3d.score;

  // classification
  autoware_perception_msgs::msg::ObjectClassification classification;
  classification.probability = 1.0f;
  if (box3d.label >= 0 && static_cast<size_t>(box3d.label) < class_names.size()) {
    classification.label = getSemanticType(class_names[box3d.label]);
  } else {
    classification.label = Label::UNKNOWN;
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("lidar_transfusion"), "Unexpected label: UNKNOWN is set.");
  }

  if (object_recognition_utils::isCarLikeVehicle(classification.label)) {
    obj.kinematics.orientation_availability =
      autoware_perception_msgs::msg::DetectedObjectKinematics::SIGN_UNKNOWN;
  }

  obj.classification.emplace_back(classification);

  // pose and shape
  // mmdet3d yaw format to ros yaw format
  float yaw = box3d.yaw + autoware::universe_utils::pi / 2;
  obj.kinematics.pose_with_covariance.pose.position =
    autoware::universe_utils::createPoint(box3d.x, box3d.y, box3d.z);
  obj.kinematics.pose_with_covariance.pose.orientation =
    autoware::universe_utils::createQuaternionFromYaw(yaw);
  obj.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  obj.shape.dimensions =
    autoware::universe_utils::createTranslation(box3d.length, box3d.width, box3d.height);
}

uint8_t getSemanticType(const std::string & class_name)
{
  if (class_name == "CAR") {
    return Label::CAR;
  } else if (class_name == "TRUCK") {
    return Label::TRUCK;
  } else if (class_name == "BUS") {
    return Label::BUS;
  } else if (class_name == "TRAILER") {
    return Label::TRAILER;
  } else if (class_name == "MOTORCYCLE") {
    return Label::MOTORCYCLE;
  } else if (class_name == "BICYCLE") {
    return Label::BICYCLE;
  } else if (class_name == "PEDESTRIAN") {
    return Label::PEDESTRIAN;
  } else {  // CONSTRUCTION_VEHICLE, BARRIER, TRAFFIC_CONE
    return Label::UNKNOWN;
  }
}

}  // namespace autoware::lidar_transfusion
