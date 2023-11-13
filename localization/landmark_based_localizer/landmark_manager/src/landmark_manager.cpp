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

#include "landmark_manager/landmark_manager.hpp"

#include "lanelet2_extension/utility/message_conversion.hpp"

#include <Eigen/Core>
#include <tf2_eigen/tf2_eigen.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Polygon.h>

namespace landmark_manager
{

std::vector<Landmark> parse_landmarks(
  const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr & msg,
  const std::string & target_subtype, const rclcpp::Logger & logger)
{
  RCLCPP_INFO_STREAM(logger, "msg->format_version: " << msg->format_version);
  RCLCPP_INFO_STREAM(logger, "msg->map_format: " << msg->map_format);
  RCLCPP_INFO_STREAM(logger, "msg->map_version: " << msg->map_version);
  RCLCPP_INFO_STREAM(logger, "msg->data.size(): " << msg->data.size());
  lanelet::LaneletMapPtr lanelet_map_ptr{std::make_shared<lanelet::LaneletMap>()};
  lanelet::utils::conversion::fromBinMsg(*msg, lanelet_map_ptr);

  std::vector<Landmark> landmarks;

  for (const auto & poly : lanelet_map_ptr->polygonLayer) {
    const std::string type{poly.attributeOr(lanelet::AttributeName::Type, "none")};
    if (type != "pose_marker") {
      continue;
    }
    const std::string subtype{poly.attributeOr(lanelet::AttributeName::Subtype, "none")};
    if (subtype != target_subtype) {
      continue;
    }

    // Get marker_id
    const std::string marker_id = poly.attributeOr("marker_id", "none");

    // Get 4 vertices
    const auto & vertices = poly.basicPolygon();
    if (vertices.size() != 4) {
      RCLCPP_WARN_STREAM(logger, "vertices.size() (" << vertices.size() << ") != 4");
      continue;
    }

    // 4 vertices represent the marker vertices in counterclockwise order
    // Calculate the volume by considering it as a tetrahedron
    const auto & v0 = vertices[0];
    const auto & v1 = vertices[1];
    const auto & v2 = vertices[2];
    const auto & v3 = vertices[3];
    const double volume = (v1 - v0).cross(v2 - v0).dot(v3 - v0) / 6.0;
    RCLCPP_INFO_STREAM(logger, "volume = " << volume);
    const double volume_threshold = 1e-5;
    if (volume > volume_threshold) {
      RCLCPP_WARN_STREAM(
        logger,
        "volume (" << volume << ") > threshold (" << volume_threshold << "), This is not plane.");
      continue;
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

    // Create Pose
    geometry_msgs::msg::Pose pose;
    pose.position.x = center.x();
    pose.position.y = center.y();
    pose.position.z = center.z();
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

    // Add
    landmarks.push_back(Landmark{marker_id, pose});
    RCLCPP_INFO_STREAM(logger, "id: " << marker_id);
    RCLCPP_INFO_STREAM(
      logger,
      "(x, y, z) = " << pose.position.x << ", " << pose.position.y << ", " << pose.position.z);
    RCLCPP_INFO_STREAM(
      logger, "q = " << pose.orientation.x << ", " << pose.orientation.y << ", "
                     << pose.orientation.z << ", " << pose.orientation.w);
  }

  return landmarks;
}

visualization_msgs::msg::MarkerArray convert_landmarks_to_marker_array_msg(
  const std::vector<Landmark> & landmarks)
{
  int32_t id = 0;
  visualization_msgs::msg::MarkerArray marker_array;
  for (const auto & [id_str, pose] : landmarks) {
    // publish cube as a thin board
    visualization_msgs::msg::Marker cube_marker;
    cube_marker.header.frame_id = "map";
    cube_marker.header.stamp = rclcpp::Clock().now();
    cube_marker.ns = "landmark_cube";
    cube_marker.id = id;
    cube_marker.type = visualization_msgs::msg::Marker::CUBE;
    cube_marker.action = visualization_msgs::msg::Marker::ADD;
    cube_marker.pose = pose;
    cube_marker.scale.x = 1.0;
    cube_marker.scale.y = 2.0;
    cube_marker.scale.z = 0.1;
    cube_marker.color.a = 0.5;
    cube_marker.color.r = 0.0;
    cube_marker.color.g = 1.0;
    cube_marker.color.b = 0.0;
    marker_array.markers.push_back(cube_marker);

    // publish text
    visualization_msgs::msg::Marker text_marker;
    text_marker.header.frame_id = "map";
    text_marker.header.stamp = rclcpp::Clock().now();
    text_marker.ns = "landmark_text";
    text_marker.id = id;
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;
    text_marker.pose = pose;
    text_marker.text = "(" + id_str + ")";
    text_marker.scale.z = 0.5;
    text_marker.color.a = 1.0;
    text_marker.color.r = 1.0;
    text_marker.color.g = 0.0;
    text_marker.color.b = 0.0;
    marker_array.markers.push_back(text_marker);

    id++;
  }
  return marker_array;
}

}  // namespace landmark_manager
