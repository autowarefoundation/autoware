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

#include "geo_pose_projector.hpp"

#include <geography_utils/height.hpp>
#include <geography_utils/projection.hpp>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <string>

GeoPoseProjector::GeoPoseProjector()
: Node("geo_pose_projector"), publish_tf_(declare_parameter<bool>("publish_tf"))
{
  // Subscribe to map_projector_info topic
  const auto adaptor = component_interface_utils::NodeAdaptor(this);
  adaptor.init_sub(
    sub_map_projector_info_,
    [this](const MapProjectorInfo::Message::ConstSharedPtr msg) { projector_info_ = *msg; });

  // Subscribe to geo_pose topic
  geo_pose_sub_ = create_subscription<GeoPoseWithCovariance>(
    "input_geo_pose", 10,
    [this](const GeoPoseWithCovariance::ConstSharedPtr msg) { on_geo_pose(msg); });

  // Publish pose topic
  pose_pub_ = create_publisher<PoseWithCovariance>("output_pose", 10);

  // Publish tf
  if (publish_tf_) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    parent_frame_ = declare_parameter<std::string>("parent_frame");
    child_frame_ = declare_parameter<std::string>("child_frame");
  }
}

void GeoPoseProjector::on_geo_pose(const GeoPoseWithCovariance::ConstSharedPtr msg)
{
  if (!projector_info_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000 /* ms */, "map_projector_info is not received yet.");
    return;
  }

  // get position
  geographic_msgs::msg::GeoPoint gps_point;
  gps_point.latitude = msg->pose.pose.position.latitude;
  gps_point.longitude = msg->pose.pose.position.longitude;
  gps_point.altitude = msg->pose.pose.position.altitude;
  geometry_msgs::msg::Point position =
    geography_utils::project_forward(gps_point, projector_info_.value());
  position.z = geography_utils::convert_height(
    position.z, gps_point.latitude, gps_point.longitude, MapProjectorInfo::Message::WGS84,
    projector_info_.value().vertical_datum);

  // Convert geo_pose to pose
  PoseWithCovariance projected_pose;
  projected_pose.header = msg->header;
  projected_pose.pose.pose.position = position;
  projected_pose.pose.pose.orientation = msg->pose.pose.orientation;
  projected_pose.pose.covariance = msg->pose.covariance;

  // Covariance in GeoPoseWithCovariance is in Lat/Lon/Alt coordinate.
  // TODO(TIER IV): This swap may be invalid when using other projector type.
  projected_pose.pose.covariance[0] = msg->pose.covariance[7];
  projected_pose.pose.covariance[7] = msg->pose.covariance[0];

  pose_pub_->publish(projected_pose);

  // Publish tf
  if (publish_tf_) {
    tf2::Transform transform;
    transform.setOrigin(tf2::Vector3(
      projected_pose.pose.pose.position.x, projected_pose.pose.pose.position.y,
      projected_pose.pose.pose.position.z));
    const auto localization_quat = tf2::Quaternion(
      projected_pose.pose.pose.orientation.x, projected_pose.pose.pose.orientation.y,
      projected_pose.pose.pose.orientation.z, projected_pose.pose.pose.orientation.w);
    transform.setRotation(localization_quat);

    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header = msg->header;
    transform_stamped.header.frame_id = parent_frame_;
    transform_stamped.child_frame_id = child_frame_;
    transform_stamped.transform = tf2::toMsg(transform);
    tf_broadcaster_->sendTransform(transform_stamped);
  }
}
