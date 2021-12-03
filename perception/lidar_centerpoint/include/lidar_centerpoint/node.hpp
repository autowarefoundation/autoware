// Copyright 2021 Tier IV, Inc.
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

#ifndef LIDAR_CENTERPOINT__NODE_HPP_
#define LIDAR_CENTERPOINT__NODE_HPP_

#include <centerpoint_trt.hpp>
#include <config.hpp>
#include <pointcloud_densification.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_perception_msgs/msg/detected_object_kinematics.hpp>
#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>
#include <autoware_auto_perception_msgs/msg/object_classification.hpp>
#include <autoware_auto_perception_msgs/msg/shape.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <memory>
#include <string>
#include <vector>

namespace centerpoint
{
using Label = autoware_auto_perception_msgs::msg::ObjectClassification;

class LidarCenterPointNode : public rclcpp::Node
{
public:
  explicit LidarCenterPointNode(const rclcpp::NodeOptions & node_options);

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_pointcloud_msg);

  static uint8_t getSemanticType(const std::string & class_name);
  static bool isCarLikeVehicleLabel(const uint8_t label);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Publisher<autoware_auto_perception_msgs::msg::DetectedObjects>::SharedPtr objects_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;

  float score_threshold_{0.0};
  std::string densification_base_frame_;
  int densification_past_frames_{0};
  bool use_encoder_trt_{false};
  bool use_head_trt_{false};
  std::string trt_precision_;

  std::string encoder_onnx_path_;
  std::string encoder_engine_path_;
  std::string encoder_pt_path_;
  std::string head_onnx_path_;
  std::string head_engine_path_;
  std::string head_pt_path_;

  std::vector<std::string> class_names_;
  bool rename_car_to_truck_and_bus_{false};

  std::unique_ptr<PointCloudDensification> densification_ptr_{nullptr};
  std::unique_ptr<CenterPointTRT> detector_ptr_{nullptr};
};

}  // namespace centerpoint

#endif  // LIDAR_CENTERPOINT__NODE_HPP_
