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

#include "static_centerline_generator/centerline_source/bag_ego_trajectory_based_centerline.hpp"

#include "rclcpp/serialization.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "static_centerline_generator/static_centerline_generator_node.hpp"

#include <nav_msgs/msg/odometry.hpp>

namespace static_centerline_generator
{
std::vector<TrajectoryPoint> generate_centerline_with_bag(rclcpp::Node & node)
{
  const auto bag_filename = node.declare_parameter<std::string>("bag_filename");

  // open rosbag
  rosbag2_cpp::Reader bag_reader;
  bag_reader.open(bag_filename);

  // extract 2D position of ego's trajectory from rosbag
  rclcpp::Serialization<nav_msgs::msg::Odometry> bag_serialization;
  std::vector<TrajectoryPoint> centerline_traj_points;
  while (bag_reader.has_next()) {
    const rosbag2_storage::SerializedBagMessageSharedPtr msg = bag_reader.read_next();

    if (msg->topic_name != "/localization/kinematic_state") {
      continue;
    }

    rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
    const auto ros_msg = std::make_shared<nav_msgs::msg::Odometry>();

    bag_serialization.deserialize_message(&serialized_msg, ros_msg.get());

    if (!centerline_traj_points.empty()) {
      constexpr double epsilon = 1e-1;
      if (
        std::abs(centerline_traj_points.back().pose.position.x - ros_msg->pose.pose.position.x) <
          epsilon &&
        std::abs(centerline_traj_points.back().pose.position.y - ros_msg->pose.pose.position.y) <
          epsilon) {
        continue;
      }
    }
    TrajectoryPoint centerline_traj_point;
    centerline_traj_point.pose.position = ros_msg->pose.pose.position;
    centerline_traj_points.push_back(centerline_traj_point);
  }

  RCLCPP_INFO(node.get_logger(), "Extracted centerline from the bag.");

  // calculate rough orientation of centerline trajectory points
  for (size_t i = 0; i < centerline_traj_points.size(); ++i) {
    if (i == centerline_traj_points.size() - 1) {
      if (i != 0) {
        centerline_traj_points.at(i).pose.orientation =
          centerline_traj_points.at(i - 1).pose.orientation;
      }
    } else {
      const double yaw_angle = tier4_autoware_utils::calcAzimuthAngle(
        centerline_traj_points.at(i).pose.position, centerline_traj_points.at(i + 1).pose.position);
      centerline_traj_points.at(i).pose.orientation =
        tier4_autoware_utils::createQuaternionFromYaw(yaw_angle);
    }
  }

  return centerline_traj_points;
}
}  // namespace static_centerline_generator
