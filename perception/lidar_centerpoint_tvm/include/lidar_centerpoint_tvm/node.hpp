// Copyright 2021-2022 AutoCore Ltd., TIER IV, Inc.
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

#ifndef LIDAR_CENTERPOINT_TVM__NODE_HPP_
#define LIDAR_CENTERPOINT_TVM__NODE_HPP_

#include <lidar_centerpoint_tvm/centerpoint_tvm.hpp>
#include <lidar_centerpoint_tvm/visibility_control.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/ros/debug_publisher.hpp>
#include <tier4_autoware_utils/system/stop_watch.hpp>

#include <autoware_auto_perception_msgs/msg/detected_object_kinematics.hpp>
#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>
#include <autoware_auto_perception_msgs/msg/object_classification.hpp>
#include <autoware_auto_perception_msgs/msg/shape.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware
{
namespace perception
{
namespace lidar_centerpoint_tvm
{
class LIDAR_CENTERPOINT_TVM_PUBLIC LidarCenterPointTVMNode : public rclcpp::Node
{
public:
  explicit LidarCenterPointTVMNode(const rclcpp::NodeOptions & node_options);

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_pointcloud_msg);

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_{tf_buffer_};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Publisher<autoware_auto_perception_msgs::msg::DetectedObjects>::SharedPtr objects_pub_;

  float score_threshold_{0.0};
  std::vector<std::string> class_names_;
  bool rename_car_to_truck_and_bus_{false};
  bool has_twist_{false};

  std::unique_ptr<CenterPointTVM> detector_ptr_{nullptr};

  // debugger
  std::unique_ptr<tier4_autoware_utils::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_{
    nullptr};
  std::unique_ptr<tier4_autoware_utils::DebugPublisher> debug_publisher_ptr_{nullptr};
};

}  // namespace lidar_centerpoint_tvm
}  // namespace perception
}  // namespace autoware

#endif  // LIDAR_CENTERPOINT_TVM__NODE_HPP_
