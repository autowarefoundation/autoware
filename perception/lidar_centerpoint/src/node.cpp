// Copyright 2021 TIER IV, Inc.
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

#include "lidar_centerpoint/node.hpp"

#include <config.hpp>
#include <pcl_ros/transforms.hpp>
#include <pointcloud_densification.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/math/constants.hpp>
#include <utils.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <memory>
#include <string>
#include <vector>

namespace centerpoint
{
LidarCenterPointNode::LidarCenterPointNode(const rclcpp::NodeOptions & node_options)
: Node("lidar_center_point", node_options), tf_buffer_(this->get_clock())
{
  score_threshold_ = this->declare_parameter("score_threshold", 0.4);
  std::string densification_world_frame_id =
    this->declare_parameter("densification_world_frame_id", "map");
  int densification_num_past_frames = this->declare_parameter("densification_num_past_frames", 1);
  std::string trt_precision = this->declare_parameter("trt_precision", "fp16");
  std::string encoder_onnx_path = this->declare_parameter("encoder_onnx_path", "");
  std::string encoder_engine_path = this->declare_parameter("encoder_engine_path", "");
  std::string head_onnx_path = this->declare_parameter("head_onnx_path", "");
  std::string head_engine_path = this->declare_parameter("head_engine_path", "");
  class_names_ = this->declare_parameter<std::vector<std::string>>("class_names");
  rename_car_to_truck_and_bus_ = this->declare_parameter("rename_car_to_truck_and_bus", false);

  NetworkParam encoder_param(encoder_onnx_path, encoder_engine_path, trt_precision);
  NetworkParam head_param(head_onnx_path, head_engine_path, trt_precision);
  DensificationParam densification_param(
    densification_world_frame_id, densification_num_past_frames);
  detector_ptr_ = std::make_unique<CenterPointTRT>(
    class_names_.size(), score_threshold_, encoder_param, head_param, densification_param);

  pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "~/input/pointcloud", rclcpp::SensorDataQoS{}.keep_last(1),
    std::bind(&LidarCenterPointNode::pointCloudCallback, this, std::placeholders::_1));
  objects_pub_ = this->create_publisher<autoware_auto_perception_msgs::msg::DetectedObjects>(
    "~/output/objects", rclcpp::QoS{1});
}

void LidarCenterPointNode::pointCloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_pointcloud_msg)
{
  const auto objects_sub_count =
    objects_pub_->get_subscription_count() + objects_pub_->get_intra_process_subscription_count();
  if (objects_sub_count < 1) {
    return;
  }

  std::vector<Box3D> det_boxes3d;
  bool is_success = detector_ptr_->detect(*input_pointcloud_msg, tf_buffer_, det_boxes3d);
  if (!is_success) {
    return;
  }

  autoware_auto_perception_msgs::msg::DetectedObjects output_msg;
  output_msg.header = input_pointcloud_msg->header;
  for (const auto & box3d : det_boxes3d) {
    if (box3d.score < score_threshold_) {
      continue;
    }
    autoware_auto_perception_msgs::msg::DetectedObject obj;
    box3DToDetectedObject(box3d, obj);
    output_msg.objects.emplace_back(obj);
  }

  if (objects_sub_count > 0) {
    objects_pub_->publish(output_msg);
  }
}

void LidarCenterPointNode::box3DToDetectedObject(
  const Box3D & box3d, autoware_auto_perception_msgs::msg::DetectedObject & obj)
{
  // TODO(yukke42): the value of classification confidence of DNN, not probability.
  obj.existence_probability = box3d.score;

  // classification
  autoware_auto_perception_msgs::msg::ObjectClassification classification;
  classification.probability = 1.0f;
  if (box3d.label >= 0 && static_cast<size_t>(box3d.label) < class_names_.size()) {
    classification.label = getSemanticType(class_names_[box3d.label]);
  } else {
    classification.label = Label::UNKNOWN;
  }

  float l = box3d.length;
  float w = box3d.width;
  if (classification.label == Label::CAR && rename_car_to_truck_and_bus_) {
    // Note: object size is referred from multi_object_tracker
    if ((w * l > 2.2 * 5.5) && (w * l <= 2.5 * 7.9)) {
      classification.label = Label::TRUCK;
    } else if (w * l > 2.5 * 7.9) {
      classification.label = Label::BUS;
    }
  }

  if (isCarLikeVehicleLabel(classification.label)) {
    obj.kinematics.orientation_availability =
      autoware_auto_perception_msgs::msg::DetectedObjectKinematics::SIGN_UNKNOWN;
  }

  obj.classification.emplace_back(classification);

  // pose and shape
  // mmdet3d yaw format to ros yaw format
  float yaw = -box3d.yaw - tier4_autoware_utils::pi / 2;
  obj.kinematics.pose_with_covariance.pose.position =
    tier4_autoware_utils::createPoint(box3d.x, box3d.y, box3d.z);
  obj.kinematics.pose_with_covariance.pose.orientation =
    tier4_autoware_utils::createQuaternionFromYaw(yaw);
  obj.shape.type = autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX;
  obj.shape.dimensions =
    tier4_autoware_utils::createTranslation(box3d.length, box3d.width, box3d.height);

  // twist
  float vel_x = box3d.vel_x;
  float vel_y = box3d.vel_y;
  geometry_msgs::msg::Twist twist;
  twist.linear.x = std::sqrt(std::pow(vel_x, 2) + std::pow(vel_y, 2));
  twist.angular.z = 2 * (std::atan2(vel_y, vel_x) - yaw);
  obj.kinematics.twist_with_covariance.twist = twist;
  obj.kinematics.has_twist = true;
}

uint8_t LidarCenterPointNode::getSemanticType(const std::string & class_name)
{
  if (class_name == "CAR") {
    return Label::CAR;
  } else if (class_name == "TRUCK") {
    return Label::TRUCK;
  } else if (class_name == "BUS") {
    return Label::BUS;
  } else if (class_name == "TRAILER") {
    return Label::TRAILER;
  } else if (class_name == "BICYCLE") {
    return Label::BICYCLE;
  } else if (class_name == "MOTORBIKE") {
    return Label::MOTORCYCLE;
  } else if (class_name == "PEDESTRIAN") {
    return Label::PEDESTRIAN;
  } else {
    return Label::UNKNOWN;
  }
}

bool LidarCenterPointNode::isCarLikeVehicleLabel(const uint8_t label)
{
  return label == Label::CAR || label == Label::TRUCK || label == Label::BUS ||
         label == Label::TRAILER;
}

}  // namespace centerpoint

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(centerpoint::LidarCenterPointNode)
