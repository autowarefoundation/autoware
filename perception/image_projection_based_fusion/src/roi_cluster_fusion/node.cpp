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

// cspell: ignore minx, maxx, miny, maxy, minz, maxz

namespace image_projection_based_fusion
{

RoiClusterFusionNode::RoiClusterFusionNode(const rclcpp::NodeOptions & options)
: FusionNode<DetectedObjectsWithFeature, DetectedObjectWithFeature>("roi_cluster_fusion", options)
{
  trust_object_iou_mode_ = declare_parameter<std::string>("trust_object_iou_mode");
  non_trust_object_iou_mode_ = declare_parameter<std::string>("non_trust_object_iou_mode");
  use_cluster_semantic_type_ = declare_parameter<bool>("use_cluster_semantic_type");
  only_allow_inside_cluster_ = declare_parameter<bool>("only_allow_inside_cluster");
  roi_scale_factor_ = declare_parameter<double>("roi_scale_factor");
  iou_threshold_ = declare_parameter<double>("iou_threshold");
  unknown_iou_threshold_ = declare_parameter<double>("unknown_iou_threshold");
  remove_unknown_ = declare_parameter<bool>("remove_unknown");
  fusion_distance_ = declare_parameter<double>("fusion_distance");
  trust_object_distance_ = declare_parameter<double>("trust_object_distance");
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

void RoiClusterFusionNode::postprocess(DetectedObjectsWithFeature & output_cluster_msg)
{
  if (!remove_unknown_) {
    return;
  }
  DetectedObjectsWithFeature known_objects;
  known_objects.feature_objects.reserve(output_cluster_msg.feature_objects.size());
  for (auto & feature_object : output_cluster_msg.feature_objects) {
    bool is_roi_label_known = feature_object.object.classification.front().label !=
                              autoware_auto_perception_msgs::msg::ObjectClassification::UNKNOWN;
    if (
      is_roi_label_known ||
      feature_object.object.existence_probability >= min_roi_existence_prob_) {
      known_objects.feature_objects.push_back(feature_object);
    }
  }
  output_cluster_msg.feature_objects = known_objects.feature_objects;
}

void RoiClusterFusionNode::fuseOnSingleImage(
  const DetectedObjectsWithFeature & input_cluster_msg, const std::size_t image_id,
  const DetectedObjectsWithFeature & input_roi_msg,
  const sensor_msgs::msg::CameraInfo & camera_info, DetectedObjectsWithFeature & output_cluster_msg)
{
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
      RCLCPP_WARN_STREAM(
        get_logger(), "Failed to get transform from " << input_cluster_msg.header.frame_id << " to "
                                                      << camera_info.header.frame_id);
      return;
    }
    transform_stamped = transform_stamped_optional.value();
  }

  std::map<std::size_t, RegionOfInterest> m_cluster_roi;
  for (std::size_t i = 0; i < input_cluster_msg.feature_objects.size(); ++i) {
    if (input_cluster_msg.feature_objects.at(i).feature.cluster.data.empty()) {
      continue;
    }

    if (is_far_enough(input_cluster_msg.feature_objects.at(i), fusion_distance_)) {
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
        if (debugger_) debugger_->obstacle_points_.push_back(normalized_projected_point);
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
    if (debugger_) debugger_->obstacle_rois_.push_back(roi);
  }

  for (const auto & feature_obj : input_roi_msg.feature_objects) {
    int index = -1;
    bool associated = false;
    double max_iou = 0.0;
    bool is_roi_label_known =
      feature_obj.object.classification.front().label != ObjectClassification::UNKNOWN;
    for (const auto & cluster_map : m_cluster_roi) {
      double iou(0.0);
      bool is_use_non_trust_object_iou_mode = is_far_enough(
        input_cluster_msg.feature_objects.at(cluster_map.first), trust_object_distance_);
      auto image_roi = feature_obj.feature.roi;
      auto cluster_roi = cluster_map.second;
      sanitizeROI(image_roi, camera_info.width, camera_info.height);
      sanitizeROI(cluster_roi, camera_info.width, camera_info.height);
      if (is_use_non_trust_object_iou_mode || is_roi_label_known) {
        iou = cal_iou_by_mode(cluster_roi, image_roi, non_trust_object_iou_mode_);
      } else {
        iou = cal_iou_by_mode(cluster_roi, image_roi, trust_object_iou_mode_);
      }

      const bool passed_inside_cluster_gate =
        only_allow_inside_cluster_ ? is_inside(image_roi, cluster_roi, roi_scale_factor_) : true;
      if (max_iou < iou && passed_inside_cluster_gate) {
        index = cluster_map.first;
        max_iou = iou;
        associated = true;
      }
    }

    if (!associated) {
      continue;
    }

    if (!output_cluster_msg.feature_objects.empty()) {
      bool is_roi_existence_prob_higher =
        output_cluster_msg.feature_objects.at(index).object.existence_probability <=
        feature_obj.object.existence_probability;
      if (iou_threshold_ < max_iou && is_roi_existence_prob_higher && is_roi_label_known) {
        output_cluster_msg.feature_objects.at(index).object.classification =
          feature_obj.object.classification;

        // Update existence_probability for fused objects
        if (
          output_cluster_msg.feature_objects.at(index).object.existence_probability <
          min_roi_existence_prob_) {
          output_cluster_msg.feature_objects.at(index).object.existence_probability =
            min_roi_existence_prob_;
        }
      }

      // fuse with unknown roi

      if (unknown_iou_threshold_ < max_iou && is_roi_existence_prob_higher && !is_roi_label_known) {
        output_cluster_msg.feature_objects.at(index).object.classification =
          feature_obj.object.classification;
        // Update existence_probability for fused objects
        if (
          output_cluster_msg.feature_objects.at(index).object.existence_probability <
          min_roi_existence_prob_) {
          output_cluster_msg.feature_objects.at(index).object.existence_probability =
            min_roi_existence_prob_;
        }
      }
    }
    if (debugger_) debugger_->image_rois_.push_back(feature_obj.feature.roi);
    if (debugger_) debugger_->max_iou_for_image_rois_.push_back(max_iou);
  }

  // note: debug objects are safely cleared in fusion_node.cpp
  if (debugger_) {
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

bool RoiClusterFusionNode::is_far_enough(
  const DetectedObjectWithFeature & obj, const double distance_threshold)
{
  const auto & position = obj.object.kinematics.pose_with_covariance.pose.position;
  return position.x * position.x + position.y * position.y >
         distance_threshold * distance_threshold;
}

double RoiClusterFusionNode::cal_iou_by_mode(
  const sensor_msgs::msg::RegionOfInterest & roi_1,
  const sensor_msgs::msg::RegionOfInterest & roi_2, const std::string iou_mode)
{
  switch (IOU_MODE_MAP.at(iou_mode)) {
    case 0 /* use iou mode */:
      return calcIoU(roi_1, roi_2);

    case 1 /* use iou_x mode */:
      return calcIoUX(roi_1, roi_2);

    case 2 /* use iou_y mode */:
      return calcIoUY(roi_1, roi_2);
    default:
      return 0.0;
  }
}

}  // namespace image_projection_based_fusion

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(image_projection_based_fusion::RoiClusterFusionNode)
