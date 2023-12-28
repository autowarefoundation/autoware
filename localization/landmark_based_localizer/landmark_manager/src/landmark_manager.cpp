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
#include "localization_util/util_func.hpp"
#include "tier4_autoware_utils/geometry/geometry.hpp"

#include <Eigen/Core>
#include <tf2_eigen/tf2_eigen.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Polygon.h>

namespace landmark_manager
{

void LandmarkManager::parse_landmarks(
  const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr & msg,
  const std::string & target_subtype)
{
  std::vector<lanelet::Polygon3d> landmarks =
    lanelet::localization::parseLandmarkPolygons(msg, target_subtype);
  for (const lanelet::Polygon3d & poly : landmarks) {
    // Get landmark_id
    const std::string landmark_id = poly.attributeOr("marker_id", "none");

    // Get 4 vertices
    const auto & vertices = poly.basicPolygon();
    if (vertices.size() != 4) {
      continue;
    }

    // 4 vertices represent the landmark vertices in counterclockwise order
    // Calculate the volume by considering it as a tetrahedron
    const auto & v0 = vertices[0];
    const auto & v1 = vertices[1];
    const auto & v2 = vertices[2];
    const auto & v3 = vertices[3];
    const double volume = (v1 - v0).cross(v2 - v0).dot(v3 - v0) / 6.0;
    const double volume_threshold = 1e-5;
    if (volume > volume_threshold) {
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
    landmarks_map_[landmark_id].push_back(pose);
  }
}

visualization_msgs::msg::MarkerArray LandmarkManager::get_landmarks_as_marker_array_msg() const
{
  int32_t id = 0;
  visualization_msgs::msg::MarkerArray marker_array;
  for (const auto & [landmark_id_str, landmark_poses] : landmarks_map_) {
    for (const auto & pose : landmark_poses) {
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
      text_marker.text = "(" + landmark_id_str + ")";
      text_marker.scale.z = 0.5;
      text_marker.color.a = 1.0;
      text_marker.color.r = 1.0;
      text_marker.color.g = 0.0;
      text_marker.color.b = 0.0;
      marker_array.markers.push_back(text_marker);

      id++;
    }
  }
  return marker_array;
}

geometry_msgs::msg::Pose LandmarkManager::calculate_new_self_pose(
  const std::vector<landmark_manager::Landmark> & detected_landmarks,
  const geometry_msgs::msg::Pose & self_pose, const bool consider_orientation) const
{
  using Pose = geometry_msgs::msg::Pose;

  Pose min_new_self_pose;
  double min_distance = std::numeric_limits<double>::max();

  for (const landmark_manager::Landmark & landmark : detected_landmarks) {
    // Firstly, landmark pose is base_link
    const Pose & detected_landmark_on_base_link = landmark.pose;

    // convert base_link to map
    const Pose detected_landmark_on_map =
      tier4_autoware_utils::transformPose(detected_landmark_on_base_link, self_pose);

    // match to map
    if (landmarks_map_.count(landmark.id) == 0) {
      continue;
    }

    // check all poses
    for (const Pose mapped_landmark_on_map : landmarks_map_.at(landmark.id)) {
      // check distance
      const double curr_distance = tier4_autoware_utils::calcDistance3d(
        mapped_landmark_on_map.position, detected_landmark_on_map.position);
      if (curr_distance > min_distance) {
        continue;
      }

      if (consider_orientation) {
        const Eigen::Affine3d landmark_pose = pose_to_affine3d(mapped_landmark_on_map);
        const Eigen::Affine3d landmark_to_base_link =
          pose_to_affine3d(detected_landmark_on_base_link).inverse();
        const Eigen::Affine3d new_self_pose_eigen = landmark_pose * landmark_to_base_link;

        const Pose new_self_pose = matrix4f_to_pose(new_self_pose_eigen.matrix().cast<float>());

        // update
        min_distance = curr_distance;
        min_new_self_pose = new_self_pose;
      } else {
        const double diff_x =
          mapped_landmark_on_map.position.x - detected_landmark_on_map.position.x;
        const double diff_y =
          mapped_landmark_on_map.position.y - detected_landmark_on_map.position.y;
        const double diff_z =
          mapped_landmark_on_map.position.z - detected_landmark_on_map.position.z;
        Pose new_self_pose = self_pose;
        new_self_pose.position.x += diff_x;
        new_self_pose.position.y += diff_y;
        new_self_pose.position.z += diff_z;

        // update
        min_distance = curr_distance;
        min_new_self_pose = new_self_pose;
      }
    }
  }

  return min_new_self_pose;
}

}  // namespace landmark_manager
