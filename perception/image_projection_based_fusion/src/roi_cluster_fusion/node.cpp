// Copyright 2020 TIER IV, Inc.
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

#include "image_projection_based_fusion/roi_cluster_fusion/node.hpp"

#include <image_projection_based_fusion/utils/geometry.hpp>
#include <image_projection_based_fusion/utils/utils.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#endif

namespace image_projection_based_fusion
{

RoiClusterFusionNode::RoiClusterFusionNode(const rclcpp::NodeOptions & options)
: FusionNode<DetectedObjectsWithFeature, DetectedObjectWithFeature>("roi_cluster_fusion", options)
{
  use_iou_x_ = declare_parameter("use_iou_x", true);
  use_iou_y_ = declare_parameter("use_iou_y", false);
  use_iou_ = declare_parameter("use_iou", false);
  use_cluster_semantic_type_ = declare_parameter("use_cluster_semantic_type", false);
  iou_threshold_ = declare_parameter("iou_threshold", 0.1);
}

void RoiClusterFusionNode::preprocess(DetectedObjectsWithFeature & output_cluster_msg)
{
  // reset cluster semantic type
  if (!use_cluster_semantic_type_) {
    for (auto & feature_object : output_cluster_msg.feature_objects) {
      feature_object.object.classification.front().label =
        autoware_auto_perception_msgs::msg::ObjectClassification::UNKNOWN;
      feature_object.object.existence_probability = 0.0;
    }
  }
}

void RoiClusterFusionNode::fuseOnSingleImage(
  const DetectedObjectsWithFeature & input_cluster_msg, const std::size_t image_id,
  const DetectedObjectsWithFeature & input_roi_msg,
  const sensor_msgs::msg::CameraInfo & camera_info, DetectedObjectsWithFeature & output_cluster_msg)
{
  std::vector<sensor_msgs::msg::RegionOfInterest> debug_image_rois;
  std::vector<sensor_msgs::msg::RegionOfInterest> debug_pointcloud_rois;
  std::vector<Eigen::Vector2d> debug_image_points;

  Eigen::Matrix4d projection;
  projection << camera_info.p.at(0), camera_info.p.at(1), camera_info.p.at(2), camera_info.p.at(3),
    camera_info.p.at(4), camera_info.p.at(5), camera_info.p.at(6), camera_info.p.at(7),
    camera_info.p.at(8), camera_info.p.at(9), camera_info.p.at(10), camera_info.p.at(11);

  // get transform from cluster frame id to camera optical frame id
  geometry_msgs::msg::TransformStamped transform_stamped;
  {
    const auto transform_stamped_optional = getTransformStamped(
      tf_buffer_, /*target*/ camera_info.header.frame_id,
      /*source*/ input_cluster_msg.header.frame_id, camera_info.header.stamp);
    if (!transform_stamped_optional) {
      return;
    }
    transform_stamped = transform_stamped_optional.value();
  }

  std::map<std::size_t, RegionOfInterest> m_cluster_roi;
  for (std::size_t i = 0; i < input_cluster_msg.feature_objects.size(); ++i) {
    if (input_cluster_msg.feature_objects.at(i).feature.cluster.data.empty()) {
      continue;
    }

    // filter point out of scope
    if (debugger_ && out_of_scope(input_cluster_msg.feature_objects.at(i))) {
      continue;
    }

    sensor_msgs::msg::PointCloud2 transformed_cluster;
    tf2::doTransform(
      input_cluster_msg.feature_objects.at(i).feature.cluster, transformed_cluster,
      transform_stamped);

    int min_x(camera_info.width), min_y(camera_info.height), max_x(0), max_y(0);
    std::vector<Eigen::Vector2d> projected_points;
    projected_points.reserve(transformed_cluster.data.size());
    for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(transformed_cluster, "x"),
         iter_y(transformed_cluster, "y"), iter_z(transformed_cluster, "z");
         iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      if (*iter_z <= 0.0) {
        continue;
      }

      Eigen::Vector4d projected_point =
        projection * Eigen::Vector4d(*iter_x, *iter_y, *iter_z, 1.0);
      Eigen::Vector2d normalized_projected_point = Eigen::Vector2d(
        projected_point.x() / projected_point.z(), projected_point.y() / projected_point.z());
      if (
        0 <= static_cast<int>(normalized_projected_point.x()) &&
        static_cast<int>(normalized_projected_point.x()) <=
          static_cast<int>(camera_info.width) - 1 &&
        0 <= static_cast<int>(normalized_projected_point.y()) &&
        static_cast<int>(normalized_projected_point.y()) <=
          static_cast<int>(camera_info.height) - 1) {
        min_x = std::min(static_cast<int>(normalized_projected_point.x()), min_x);
        min_y = std::min(static_cast<int>(normalized_projected_point.y()), min_y);
        max_x = std::max(static_cast<int>(normalized_projected_point.x()), max_x);
        max_y = std::max(static_cast<int>(normalized_projected_point.y()), max_y);
        projected_points.push_back(normalized_projected_point);
        debug_image_points.push_back(normalized_projected_point);
      }
    }
    if (projected_points.empty()) {
      continue;
    }

    sensor_msgs::msg::RegionOfInterest roi;
    // roi.do_rectify = m_camera_info_.at(id).do_rectify;
    roi.x_offset = min_x;
    roi.y_offset = min_y;
    roi.width = max_x - min_x;
    roi.height = max_y - min_y;
    m_cluster_roi.insert(std::make_pair(i, roi));
    debug_pointcloud_rois.push_back(roi);
  }

  for (const auto & feature_obj : input_roi_msg.feature_objects) {
    int index = 0;
    double max_iou = 0.0;
    for (const auto & cluster_map : m_cluster_roi) {
      double iou(0.0), iou_x(0.0), iou_y(0.0);
      if (use_iou_) {
        iou = calcIoU(cluster_map.second, feature_obj.feature.roi);
      }
      if (use_iou_x_) {
        iou_x = calcIoUX(cluster_map.second, feature_obj.feature.roi);
      }
      if (use_iou_y_) {
        iou_y = calcIoUY(cluster_map.second, feature_obj.feature.roi);
      }
      if (max_iou < iou + iou_x + iou_y) {
        index = cluster_map.first;
        max_iou = iou + iou_x + iou_y;
      }
    }
    if (
      iou_threshold_ < max_iou &&
      output_cluster_msg.feature_objects.at(index).object.existence_probability <=
        feature_obj.object.existence_probability &&
      feature_obj.object.classification.front().label !=
        autoware_auto_perception_msgs::msg::ObjectClassification::UNKNOWN) {
      output_cluster_msg.feature_objects.at(index).object.classification =
        feature_obj.object.classification;
    }
    debug_image_rois.push_back(feature_obj.feature.roi);
  }

  if (debugger_) {
    debugger_->image_rois_ = debug_image_rois;
    debugger_->obstacle_rois_ = debug_pointcloud_rois;
    debugger_->obstacle_points_ = debug_image_points;
    debugger_->publishImage(image_id, input_roi_msg.header.stamp);
  }
}

bool RoiClusterFusionNode::out_of_scope(const DetectedObjectWithFeature & obj)
{
  auto cluster = obj.feature.cluster;
  bool is_out = false;
  auto valid_point = [](float p, float min_num, float max_num) -> bool {
    return (p > min_num) && (p < max_num);
  };

  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(cluster, "x"), iter_y(cluster, "y"),
       iter_z(cluster, "z");
       iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    if (!valid_point(*iter_x, filter_scope_minx_, filter_scope_maxx_)) {
      is_out = true;
      break;
    }

    if (!valid_point(*iter_y, filter_scope_miny_, filter_scope_maxy_)) {
      is_out = true;
      break;
    }

    if (!valid_point(*iter_z, filter_scope_minz_, filter_scope_maxz_)) {
      is_out = true;
      break;
    }
  }

  return is_out;
}

}  // namespace image_projection_based_fusion

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(image_projection_based_fusion::RoiClusterFusionNode)
