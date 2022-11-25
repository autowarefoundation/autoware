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

#ifndef LIDAR_CENTERPOINT__SINGLE_INFERENCE_NODE_HPP_
#define LIDAR_CENTERPOINT__SINGLE_INFERENCE_NODE_HPP_

#include <Eigen/Core>
#include <lidar_centerpoint/centerpoint_trt.hpp>
#include <lidar_centerpoint/detection_class_remapper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_perception_msgs/msg/detected_object_kinematics.hpp>
#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>
#include <autoware_auto_perception_msgs/msg/object_classification.hpp>
#include <autoware_auto_perception_msgs/msg/shape.hpp>

#include <memory>
#include <string>
#include <vector>

namespace centerpoint
{

class SingleInferenceLidarCenterPointNode : public rclcpp::Node
{
public:
  explicit SingleInferenceLidarCenterPointNode(const rclcpp::NodeOptions & node_options);

private:
  void detect(const std::string & pcd_path, const std::string & detections_path);
  std::vector<Eigen::Vector3d> getVertices(
    const autoware_auto_perception_msgs::msg::Shape & shape, const Eigen::Affine3d & pose) const;
  void dumpDetectionsAsMesh(
    const autoware_auto_perception_msgs::msg::DetectedObjects & objects_msg,
    const std::string & output_path) const;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_{tf_buffer_};

  float score_threshold_{0.0};
  std::vector<std::string> class_names_;
  bool rename_car_to_truck_and_bus_{false};
  bool has_twist_{false};

  DetectionClassRemapper detection_class_remapper_;

  std::unique_ptr<CenterPointTRT> detector_ptr_{nullptr};
};

}  // namespace centerpoint

#endif  // LIDAR_CENTERPOINT__SINGLE_INFERENCE_NODE_HPP_
