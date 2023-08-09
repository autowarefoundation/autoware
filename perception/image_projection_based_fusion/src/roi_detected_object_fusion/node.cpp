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

#include "object_recognition_utils/object_recognition_utils.hpp"

#include <image_projection_based_fusion/utils/geometry.hpp>
#include <image_projection_based_fusion/utils/utils.hpp>

// cspell: ignore minx, maxx, miny, maxy, minz, maxz

namespace image_projection_based_fusion
{

RoiDetectedObjectFusionNode::RoiDetectedObjectFusionNode(const rclcpp::NodeOptions & options)
: FusionNode<DetectedObjects, DetectedObject>("roi_detected_object_fusion", options)
{
  fusion_params_.passthrough_lower_bound_probability_thresholds =
    declare_parameter<std::vector<double>>("passthrough_lower_bound_probability_thresholds");
  fusion_params_.min_iou_threshold = declare_parameter<double>("min_iou_threshold");
  fusion_params_.trust_distances = declare_parameter<std::vector<double>>("trust_distances");
  fusion_params_.use_roi_probability = declare_parameter<bool>("use_roi_probability");
  fusion_params_.roi_probability_threshold = declare_parameter<double>("roi_probability_threshold");
  {
    const auto can_assign_vector_tmp = declare_parameter<std::vector<int64_t>>("can_assign_matrix");
    std::vector<int> can_assign_vector(can_assign_vector_tmp.begin(), can_assign_vector_tmp.end());
    const int label_num = static_cast<int>(std::sqrt(can_assign_vector.size()));
    Eigen::Map<Eigen::MatrixXi> can_assign_matrix_tmp(
      can_assign_vector.data(), label_num, label_num);
    fusion_params_.can_assign_matrix = can_assign_matrix_tmp.transpose();
  }
}

void RoiDetectedObjectFusionNode::preprocess(DetectedObjects & output_msg)
{
  std::vector<bool> passthrough_object_flags, fused_object_flags, ignored_object_flags;
  passthrough_object_flags.resize(output_msg.objects.size());
  fused_object_flags.resize(output_msg.objects.size());
  ignored_object_flags.resize(output_msg.objects.size());
  for (std::size_t obj_i = 0; obj_i < output_msg.objects.size(); ++obj_i) {
    const auto & object = output_msg.objects.at(obj_i);
    const auto label = object_recognition_utils::getHighestProbLabel(object.classification);
    const auto pos = object_recognition_utils::getPose(object).position;
    const auto object_sqr_dist = pos.x * pos.x + pos.y * pos.y;
    const auto prob_threshold =
      fusion_params_.passthrough_lower_bound_probability_thresholds.at(label);
    const auto trust_sqr_dist =
      fusion_params_.trust_distances.at(label) * fusion_params_.trust_distances.at(label);
    if (object.existence_probability > prob_threshold || object_sqr_dist > trust_sqr_dist) {
      passthrough_object_flags.at(obj_i) = true;
    }
  }

  int64_t timestamp_nsec =
    output_msg.header.stamp.sec * (int64_t)1e9 + output_msg.header.stamp.nanosec;
  passthrough_object_flags_map_.insert(std::make_pair(timestamp_nsec, passthrough_object_flags));
  fused_object_flags_map_.insert(std::make_pair(timestamp_nsec, fused_object_flags));
  ignored_object_flags_map_.insert(std::make_pair(timestamp_nsec, ignored_object_flags));
}

void RoiDetectedObjectFusionNode::fuseOnSingleImage(
  const DetectedObjects & input_object_msg, const std::size_t image_id,
  const DetectedObjectsWithFeature & input_roi_msg,
  const sensor_msgs::msg::CameraInfo & camera_info,
  DetectedObjects & output_object_msg __attribute__((unused)))
{
  Eigen::Affine3d object2camera_affine;
  {
    const auto transform_stamped_optional = getTransformStamped(
      tf_buffer_, /*target*/ input_roi_msg.header.frame_id,
      /*source*/ input_object_msg.header.frame_id, input_roi_msg.header.stamp);
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

  const auto object_roi_map = generateDetectedObjectRoIs(
    input_object_msg, static_cast<double>(camera_info.width),
    static_cast<double>(camera_info.height), object2camera_affine, camera_projection);
  fuseObjectsOnImage(input_object_msg, input_roi_msg.feature_objects, object_roi_map);

  if (debugger_) {
    debugger_->image_rois_.reserve(input_roi_msg.feature_objects.size());
    for (std::size_t roi_i = 0; roi_i < input_roi_msg.feature_objects.size(); ++roi_i) {
      debugger_->image_rois_.push_back(input_roi_msg.feature_objects.at(roi_i).feature.roi);
    }
    debugger_->publishImage(image_id, input_roi_msg.header.stamp);
  }
}

std::map<std::size_t, DetectedObjectWithFeature>
RoiDetectedObjectFusionNode::generateDetectedObjectRoIs(
  const DetectedObjects & input_object_msg, const double image_width, const double image_height,
  const Eigen::Affine3d & object2camera_affine, const Eigen::Matrix4d & camera_projection)
{
  std::map<std::size_t, DetectedObjectWithFeature> object_roi_map;
  int64_t timestamp_nsec =
    input_object_msg.header.stamp.sec * (int64_t)1e9 + input_object_msg.header.stamp.nanosec;
  if (passthrough_object_flags_map_.size() == 0) {
    return object_roi_map;
  }
  if (passthrough_object_flags_map_.count(timestamp_nsec) == 0) {
    return object_roi_map;
  }
  const auto & passthrough_object_flags = passthrough_object_flags_map_.at(timestamp_nsec);
  for (std::size_t obj_i = 0; obj_i < input_object_msg.objects.size(); ++obj_i) {
    std::vector<Eigen::Vector3d> vertices_camera_coord;
    const auto & object = input_object_msg.objects.at(obj_i);

    if (passthrough_object_flags.at(obj_i)) {
      continue;
    }

    // filter point out of scope
    if (debugger_ && out_of_scope(object)) {
      continue;
    }

    {
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
    if (point_on_image_cnt < 3) {
      continue;
    }

    min_x = std::max(min_x, 0.0);
    min_y = std::max(min_y, 0.0);
    max_x = std::min(max_x, image_width - 1);
    max_y = std::min(max_y, image_height - 1);

    DetectedObjectWithFeature object_roi;
    object_roi.feature.roi.x_offset = static_cast<std::uint32_t>(min_x);
    object_roi.feature.roi.y_offset = static_cast<std::uint32_t>(min_y);
    object_roi.feature.roi.width =
      static_cast<std::uint32_t>(max_x) - static_cast<std::uint32_t>(min_x);
    object_roi.feature.roi.height =
      static_cast<std::uint32_t>(max_y) - static_cast<std::uint32_t>(min_y);
    object_roi.object = object;
    object_roi_map.insert(std::make_pair(obj_i, object_roi));

    if (debugger_) {
      debugger_->obstacle_rois_.push_back(object_roi.feature.roi);
    }
  }

  return object_roi_map;
}

void RoiDetectedObjectFusionNode::fuseObjectsOnImage(
  const DetectedObjects & input_object_msg,
  const std::vector<DetectedObjectWithFeature> & image_rois,
  const std::map<std::size_t, DetectedObjectWithFeature> & object_roi_map)
{
  int64_t timestamp_nsec =
    input_object_msg.header.stamp.sec * (int64_t)1e9 + input_object_msg.header.stamp.nanosec;
  if (fused_object_flags_map_.size() == 0 || ignored_object_flags_map_.size() == 0) {
    return;
  }
  if (
    fused_object_flags_map_.count(timestamp_nsec) == 0 ||
    ignored_object_flags_map_.count(timestamp_nsec) == 0) {
    return;
  }
  auto & fused_object_flags = fused_object_flags_map_.at(timestamp_nsec);
  auto & ignored_object_flags = ignored_object_flags_map_.at(timestamp_nsec);

  for (const auto & object_pair : object_roi_map) {
    const auto & obj_i = object_pair.first;
    if (fused_object_flags.at(obj_i)) {
      continue;
    }

    float roi_prob = 0.0f;
    float max_iou = 0.0f;
    for (const auto & image_roi : image_rois) {
      const auto & object_roi = object_pair.second;
      const auto object_roi_label =
        object_recognition_utils::getHighestProbLabel(object_roi.object.classification);
      const auto image_roi_label =
        object_recognition_utils::getHighestProbLabel(image_roi.object.classification);
      if (!fusion_params_.can_assign_matrix(object_roi_label, image_roi_label)) {
        continue;
      }

      const double iou = calcIoU(object_roi.feature.roi, image_roi.feature.roi);
      if (iou > max_iou) {
        max_iou = iou;
        roi_prob = image_roi.object.existence_probability;
      }
    }

    if (max_iou > fusion_params_.min_iou_threshold) {
      if (fusion_params_.use_roi_probability) {
        if (roi_prob > fusion_params_.roi_probability_threshold) {
          fused_object_flags.at(obj_i) = true;
        } else {
          ignored_object_flags.at(obj_i) = true;
        }
      } else {
        fused_object_flags.at(obj_i) = true;
      }
    } else {
      ignored_object_flags.at(obj_i) = true;
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

void RoiDetectedObjectFusionNode::publish(const DetectedObjects & output_msg)
{
  if (pub_ptr_->get_subscription_count() < 1) {
    return;
  }

  int64_t timestamp_nsec =
    output_msg.header.stamp.sec * (int64_t)1e9 + output_msg.header.stamp.nanosec;
  if (
    passthrough_object_flags_map_.size() == 0 || fused_object_flags_map_.size() == 0 ||
    ignored_object_flags_map_.size() == 0) {
    return;
  }
  if (
    passthrough_object_flags_map_.count(timestamp_nsec) == 0 ||
    fused_object_flags_map_.count(timestamp_nsec) == 0 ||
    ignored_object_flags_map_.count(timestamp_nsec) == 0) {
    return;
  }
  auto & passthrough_object_flags = passthrough_object_flags_map_.at(timestamp_nsec);
  auto & fused_object_flags = fused_object_flags_map_.at(timestamp_nsec);
  auto & ignored_object_flags = ignored_object_flags_map_.at(timestamp_nsec);

  DetectedObjects output_objects_msg, debug_fused_objects_msg, debug_ignored_objects_msg;
  output_objects_msg.header = output_msg.header;
  debug_fused_objects_msg.header = output_msg.header;
  debug_ignored_objects_msg.header = output_msg.header;
  for (std::size_t obj_i = 0; obj_i < output_msg.objects.size(); ++obj_i) {
    const auto & obj = output_msg.objects.at(obj_i);
    if (passthrough_object_flags.at(obj_i)) {
      output_objects_msg.objects.emplace_back(obj);
    }
    if (fused_object_flags.at(obj_i)) {
      output_objects_msg.objects.emplace_back(obj);
      debug_fused_objects_msg.objects.emplace_back(obj);
    }
    if (ignored_object_flags.at(obj_i)) {
      debug_ignored_objects_msg.objects.emplace_back(obj);
    }
  }

  pub_ptr_->publish(output_objects_msg);

  debug_publisher_->publish<DetectedObjects>("debug/fused_objects", debug_fused_objects_msg);
  debug_publisher_->publish<DetectedObjects>("debug/ignored_objects", debug_ignored_objects_msg);

  passthrough_object_flags_map_.erase(timestamp_nsec);
  fused_object_flags_map_.erase(timestamp_nsec);
  ignored_object_flags_map_.erase(timestamp_nsec);
}

}  // namespace image_projection_based_fusion

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(image_projection_based_fusion::RoiDetectedObjectFusionNode)
