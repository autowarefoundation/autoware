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

#include "image_projection_based_fusion/roi_detected_object_fusion/node.hpp"

#include <image_projection_based_fusion/utils/geometry.hpp>
#include <image_projection_based_fusion/utils/utils.hpp>

namespace image_projection_based_fusion
{

RoiDetectedObjectFusionNode::RoiDetectedObjectFusionNode(const rclcpp::NodeOptions & options)
: FusionNode<DetectedObjects, DetectedObject>("roi_detected_object_fusion", options)
{
  use_iou_x_ = declare_parameter("use_iou_x", false);
  use_iou_y_ = declare_parameter("use_iou_y", false);
  use_iou_ = declare_parameter("use_iou", false);
  iou_threshold_ = declare_parameter("iou_threshold", 0.1);
}

void RoiDetectedObjectFusionNode::fuseOnSingleImage(
  const DetectedObjects & input_object_msg, const std::size_t image_id,
  const DetectedObjectsWithFeature & input_roi_msg,
  const sensor_msgs::msg::CameraInfo & camera_info, DetectedObjects & output_object_msg)
{
  Eigen::Affine3d object2camera_affine;
  {
    const auto transform_stamped_optional = getTransformStamped(
      tf_buffer_, /*target*/ camera_info.header.frame_id,
      /*source*/ input_object_msg.header.frame_id, camera_info.header.stamp);
    if (!transform_stamped_optional) {
      return;
    }
    object2camera_affine = transformToEigen(transform_stamped_optional.value().transform);
  }

  Eigen::Matrix4d camera_projection;
  camera_projection << camera_info.p.at(0), camera_info.p.at(1), camera_info.p.at(2),
    camera_info.p.at(3), camera_info.p.at(4), camera_info.p.at(5), camera_info.p.at(6),
    camera_info.p.at(7), camera_info.p.at(8), camera_info.p.at(9), camera_info.p.at(10),
    camera_info.p.at(11);

  std::map<std::size_t, sensor_msgs::msg::RegionOfInterest> object_roi_map;
  generateDetectedObjectRois(
    input_object_msg.objects, static_cast<double>(camera_info.width),
    static_cast<double>(camera_info.height), object2camera_affine, camera_projection,
    object_roi_map);
  updateDetectedObjectClassification(
    input_roi_msg.feature_objects, object_roi_map, output_object_msg.objects);

  if (debugger_) {
    debugger_->image_rois_.reserve(input_roi_msg.feature_objects.size());
    for (std::size_t roi_i = 0; roi_i < input_roi_msg.feature_objects.size(); ++roi_i) {
      debugger_->image_rois_.push_back(input_roi_msg.feature_objects.at(roi_i).feature.roi);
    }
    debugger_->publishImage(image_id, input_roi_msg.header.stamp);
  }
}

void RoiDetectedObjectFusionNode::generateDetectedObjectRois(
  const std::vector<DetectedObject> & input_objects, const double image_width,
  const double image_height, const Eigen::Affine3d & object2camera_affine,
  const Eigen::Matrix4d & camera_projection,
  std::map<std::size_t, sensor_msgs::msg::RegionOfInterest> & object_roi_map)
{
  for (std::size_t obj_i = 0; obj_i < input_objects.size(); ++obj_i) {
    std::vector<Eigen::Vector3d> vertices_camera_coord;
    {
      const auto & object = input_objects.at(obj_i);

      // filter point out of scope
      if (debugger_ && out_of_scope(object)) {
        continue;
      }

      std::vector<Eigen::Vector3d> vertices;
      objectToVertices(object.kinematics.pose_with_covariance.pose, object.shape, vertices);
      transformPoints(vertices, object2camera_affine, vertices_camera_coord);
    }

    double min_x(std::numeric_limits<double>::max()), min_y(std::numeric_limits<double>::max()),
      max_x(std::numeric_limits<double>::min()), max_y(std::numeric_limits<double>::min());
    std::size_t point_on_image_cnt = 0;
    for (const auto & point : vertices_camera_coord) {
      if (point.z() <= 0.0) {
        continue;
      }

      Eigen::Vector2d proj_point;
      {
        Eigen::Vector4d proj_point_hom =
          camera_projection * Eigen::Vector4d(point.x(), point.y(), point.z(), 1.0);
        proj_point = Eigen::Vector2d(
          proj_point_hom.x() / (proj_point_hom.z()), proj_point_hom.y() / (proj_point_hom.z()));
      }

      min_x = std::min(proj_point.x(), min_x);
      min_y = std::min(proj_point.y(), min_y);
      max_x = std::max(proj_point.x(), max_x);
      max_y = std::max(proj_point.y(), max_y);

      if (
        proj_point.x() >= 0 && proj_point.x() <= image_width - 1 && proj_point.y() >= 0 &&
        proj_point.y() <= image_height - 1) {
        point_on_image_cnt++;

        if (debugger_) {
          debugger_->obstacle_points_.push_back(proj_point);
        }
      }
    }
    if (point_on_image_cnt == 0) {
      continue;
    }

    min_x = std::max(min_x, 0.0);
    min_y = std::max(min_y, 0.0);
    max_x = std::min(max_x, image_width - 1);
    max_y = std::min(max_y, image_height - 1);

    // build roi
    sensor_msgs::msg::RegionOfInterest roi;
    roi.x_offset = static_cast<std::uint32_t>(min_x);
    roi.y_offset = static_cast<std::uint32_t>(min_y);
    roi.width = static_cast<std::uint32_t>(max_x) - static_cast<std::uint32_t>(min_x);
    roi.height = static_cast<std::uint32_t>(max_y) - static_cast<std::uint32_t>(min_y);
    object_roi_map.insert(std::make_pair(obj_i, roi));

    if (debugger_) {
      debugger_->obstacle_rois_.push_back(roi);
    }
  }
}

void RoiDetectedObjectFusionNode::updateDetectedObjectClassification(
  const std::vector<DetectedObjectWithFeature> & image_rois,
  const std::map<std::size_t, sensor_msgs::msg::RegionOfInterest> & object_roi_map,
  std::vector<DetectedObject> & output_objects)
{
  for (const auto & image_roi : image_rois) {
    std::size_t object_i = 0;
    double max_iou = 0.0;
    for (const auto & object_map : object_roi_map) {
      double iou(0.0), iou_x(0.0), iou_y(0.0);
      if (use_iou_) {
        iou = calcIoU(object_map.second, image_roi.feature.roi);
      }
      if (use_iou_x_) {
        iou_x = calcIoUX(object_map.second, image_roi.feature.roi);
      }
      if (use_iou_y_) {
        iou_y = calcIoUY(object_map.second, image_roi.feature.roi);
      }

      if (iou + iou_x + iou_y > max_iou) {
        object_i = object_map.first;
        max_iou = iou + iou_x + iou_y;
      }
    }

    if (
      max_iou > iou_threshold_ &&
      output_objects.at(object_i).existence_probability <= image_roi.object.existence_probability) {
      output_objects.at(object_i).classification = image_roi.object.classification;
    }
  }
}

bool RoiDetectedObjectFusionNode::out_of_scope(const DetectedObject & obj)
{
  bool is_out = true;
  auto pose = obj.kinematics.pose_with_covariance.pose;

  auto valid_point = [](float p, float min_num, float max_num) -> bool {
    return (p > min_num) && (p < max_num);
  };

  if (!valid_point(pose.position.x, filter_scope_minx_, filter_scope_maxx_)) {
    return is_out;
  }
  if (!valid_point(pose.position.y, filter_scope_miny_, filter_scope_maxy_)) {
    return is_out;
  }
  if (!valid_point(pose.position.z, filter_scope_minz_, filter_scope_maxz_)) {
    return is_out;
  }

  is_out = false;
  return is_out;
}

}  // namespace image_projection_based_fusion

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(image_projection_based_fusion::RoiDetectedObjectFusionNode)
