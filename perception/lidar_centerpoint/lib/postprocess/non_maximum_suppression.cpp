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

#include "lidar_centerpoint/postprocess/non_maximum_suppression.hpp"

#include "object_recognition_utils/geometry.hpp"
#include "object_recognition_utils/object_recognition_utils.hpp"
#include "tier4_autoware_utils/geometry/geometry.hpp"
namespace centerpoint
{

void NonMaximumSuppression::setParameters(const NMSParams & params)
{
  assert(params.search_distance_2d_ >= 0.0);
  assert(params.iou_threshold_ >= 0.0 && params.iou_threshold_ <= 1.0);

  params_ = params;
  target_class_mask_ = classNamesToBooleanMask(params.target_class_names_);
}

bool NonMaximumSuppression::isTargetLabel(const uint8_t label)
{
  if (label >= target_class_mask_.size()) {
    return false;
  }
  return target_class_mask_.at(label);
}

bool NonMaximumSuppression::isTargetPairObject(
  const DetectedObject & object1, const DetectedObject & object2)
{
  const auto label1 = object_recognition_utils::getHighestProbLabel(object1.classification);
  const auto label2 = object_recognition_utils::getHighestProbLabel(object2.classification);

  if (isTargetLabel(label1) && isTargetLabel(label2)) {
    return true;
  }

  const auto search_sqr_dist_2d = params_.search_distance_2d_ * params_.search_distance_2d_;
  const auto sqr_dist_2d = tier4_autoware_utils::calcSquaredDistance2d(
    object_recognition_utils::getPose(object1), object_recognition_utils::getPose(object2));
  return sqr_dist_2d <= search_sqr_dist_2d;
}

Eigen::MatrixXd NonMaximumSuppression::generateIoUMatrix(
  const std::vector<DetectedObject> & input_objects)
{
  // NOTE: row = target objects to be suppressed, col = source objects to be compared
  Eigen::MatrixXd triangular_matrix =
    Eigen::MatrixXd::Zero(input_objects.size(), input_objects.size());
  for (std::size_t target_i = 0; target_i < input_objects.size(); ++target_i) {
    for (std::size_t source_i = 0; source_i < target_i; ++source_i) {
      const auto & target_obj = input_objects.at(target_i);
      const auto & source_obj = input_objects.at(source_i);
      if (!isTargetPairObject(target_obj, source_obj)) {
        continue;
      }

      if (params_.nms_type_ == NMS_TYPE::IoU_BEV) {
        const double iou = object_recognition_utils::get2dIoU(target_obj, source_obj);
        triangular_matrix(target_i, source_i) = iou;
        // NOTE: If the target object has any objects with iou > iou_threshold, it
        // will be suppressed regardless of later results.
        if (iou > params_.iou_threshold_) {
          break;
        }
      }
    }
  }

  return triangular_matrix;
}

std::vector<DetectedObject> NonMaximumSuppression::apply(
  const std::vector<DetectedObject> & input_objects)
{
  Eigen::MatrixXd iou_matrix = generateIoUMatrix(input_objects);

  std::vector<DetectedObject> output_objects;
  output_objects.reserve(input_objects.size());
  for (std::size_t i = 0; i < input_objects.size(); ++i) {
    const auto value = iou_matrix.row(i).maxCoeff();
    if (params_.nms_type_ == NMS_TYPE::IoU_BEV) {
      if (value <= params_.iou_threshold_) {
        output_objects.emplace_back(input_objects.at(i));
      }
    }
  }

  return output_objects;
}
}  // namespace centerpoint
