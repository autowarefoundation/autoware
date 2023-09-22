// Copyright 2023 Autoware Foundation
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

#include "landmark_tf_caster/landmark_tf_caster_core.hpp"

#include "lanelet2_extension/utility/message_conversion.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <lanelet2_core/LaneletMap.h>
#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#endif

LandmarkTfCaster::LandmarkTfCaster() : Node("landmark_tf_caster")
{
  // Parameters
  volume_threshold_ = this->declare_parameter("volume_threshold", 1e-5);

  // tf
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);

  // Subscribers
  map_bin_sub_ = this->create_subscription<autoware_auto_mapping_msgs::msg::HADMapBin>(
    "~/input/lanelet2_map", rclcpp::QoS(10).durability(rclcpp::DurabilityPolicy::TransientLocal),
    std::bind(&LandmarkTfCaster::map_bin_callback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "finish setup");
}

void LandmarkTfCaster::map_bin_callback(
  const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr & msg)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "msg->format_version: " << msg->format_version);
  RCLCPP_INFO_STREAM(this->get_logger(), "msg->map_format: " << msg->map_format);
  RCLCPP_INFO_STREAM(this->get_logger(), "msg->map_version: " << msg->map_version);
  RCLCPP_INFO_STREAM(this->get_logger(), "msg->data.size(): " << msg->data.size());
  lanelet::LaneletMapPtr lanelet_map_ptr{std::make_shared<lanelet::LaneletMap>()};
  lanelet::utils::conversion::fromBinMsg(*msg, lanelet_map_ptr);
  for (const auto & poly : lanelet_map_ptr->polygonLayer) {
    const std::string type{poly.attributeOr(lanelet::AttributeName::Type, "none")};
    if (type != "pose_marker") {
      continue;
    }
    publish_tf(poly);
  }
}

void LandmarkTfCaster::publish_tf(const lanelet::Polygon3d & poly)
{
  // Get marker_id
  const std::string marker_id = poly.attributeOr("marker_id", "none");

  // Get 4 vertices
  const auto & vertices = poly.basicPolygon();
  if (vertices.size() != 4) {
    RCLCPP_WARN_STREAM(this->get_logger(), "vertices.size() (" << vertices.size() << ") != 4");
    return;
  }

  // 4 vertices represent the marker vertices in counterclockwise order
  // Calculate the volume by considering it as a tetrahedron
  const auto & v0 = vertices[0];
  const auto & v1 = vertices[1];
  const auto & v2 = vertices[2];
  const auto & v3 = vertices[3];
  const double volume = (v1 - v0).cross(v2 - v0).dot(v3 - v0) / 6.0;
  RCLCPP_INFO_STREAM(this->get_logger(), "volume = " << volume);
  if (volume > volume_threshold_) {
    RCLCPP_WARN_STREAM(
      this->get_logger(),
      "volume (" << volume << ") > threshold (" << volume_threshold_ << "), This is not plane.");
    return;
  }

  // Calculate the center of the quadrilateral
  const auto center = (v0 + v1 + v2 + v3) / 4.0;

  // Define axes
  const auto x_axis = (v1 - v0).normalized();
  const auto y_axis = (v2 - v1).normalized();
  const auto z_axis = x_axis.cross(y_axis).normalized();

  // Construct rotation matrix and convert it to quaternion
  Eigen::Matrix3d rotation_matrix;
  rotation_matrix << x_axis, y_axis, z_axis;
  const Eigen::Quaterniond q{rotation_matrix};

  // Create transform
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = this->now();
  tf.header.frame_id = "map";
  tf.child_frame_id = "tag_" + marker_id;
  tf.transform.translation.x = center.x();
  tf.transform.translation.y = center.y();
  tf.transform.translation.z = center.z();
  tf.transform.rotation.x = q.x();
  tf.transform.rotation.y = q.y();
  tf.transform.rotation.z = q.z();
  tf.transform.rotation.w = q.w();

  // Publish transform
  tf_broadcaster_->sendTransform(tf);
  RCLCPP_INFO_STREAM(this->get_logger(), "id: " << marker_id);
  RCLCPP_INFO_STREAM(
    this->get_logger(), "(x, y, z) = " << tf.transform.translation.x << ", "
                                       << tf.transform.translation.y << ", "
                                       << tf.transform.translation.z);
  RCLCPP_INFO_STREAM(
    this->get_logger(), "q = " << tf.transform.rotation.x << ", " << tf.transform.rotation.y << ", "
                               << tf.transform.rotation.z << ", " << tf.transform.rotation.w);
}
