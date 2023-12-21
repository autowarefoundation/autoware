// Copyright 2020 Tier IV, Inc.
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

#include "object_merger/node.hpp"

#include "object_merger/utils/utils.hpp"
#include "object_recognition_utils/object_recognition_utils.hpp"
#include "tier4_autoware_utils/geometry/geometry.hpp"

#include <boost/optional.hpp>

#include <chrono>
#include <unordered_map>

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

using Label = autoware_auto_perception_msgs::msg::ObjectClassification;

namespace
{
bool isUnknownObjectOverlapped(
  const autoware_auto_perception_msgs::msg::DetectedObject & unknown_object,
  const autoware_auto_perception_msgs::msg::DetectedObject & known_object,
  const double precision_threshold, const double recall_threshold,
  std::map<int, double> distance_threshold_map,
  const std::map<int, double> generalized_iou_threshold_map)
{
  const double generalized_iou_threshold = generalized_iou_threshold_map.at(
    object_recognition_utils::getHighestProbLabel(known_object.classification));
  const double distance_threshold = distance_threshold_map.at(
    object_recognition_utils::getHighestProbLabel(known_object.classification));
  const double sq_distance_threshold = std::pow(distance_threshold, 2.0);
  const double sq_distance = tier4_autoware_utils::calcSquaredDistance2d(
    unknown_object.kinematics.pose_with_covariance.pose,
    known_object.kinematics.pose_with_covariance.pose);
  if (sq_distance_threshold < sq_distance) return false;
  const auto precision = object_recognition_utils::get2dPrecision(unknown_object, known_object);
  const auto recall = object_recognition_utils::get2dRecall(unknown_object, known_object);
  const auto generalized_iou =
    object_recognition_utils::get2dGeneralizedIoU(unknown_object, known_object);
  return precision > precision_threshold || recall > recall_threshold ||
         generalized_iou > generalized_iou_threshold;
}
}  // namespace

namespace
{
std::map<int, double> convertListToClassMap(const std::vector<double> & distance_threshold_list)
{
  std::map<int /*class label*/, double /*distance_threshold*/> distance_threshold_map;
  int class_label = 0;
  for (const auto & distance_threshold : distance_threshold_list) {
    distance_threshold_map.insert(std::make_pair(class_label, distance_threshold));
    class_label++;
  }
  return distance_threshold_map;
}
}  // namespace

namespace object_association
{
ObjectAssociationMergerNode::ObjectAssociationMergerNode(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("object_association_merger_node", node_options),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_),
  object0_sub_(this, "input/object0", rclcpp::QoS{1}.get_rmw_qos_profile()),
  object1_sub_(this, "input/object1", rclcpp::QoS{1}.get_rmw_qos_profile())
{
  // Parameters
  base_link_frame_id_ = declare_parameter<std::string>("base_link_frame_id", "base_link");
  priority_mode_ = static_cast<PriorityMode>(
    declare_parameter<int>("priority_mode", static_cast<int>(PriorityMode::Confidence)));
  sync_queue_size_ = declare_parameter<int>("sync_queue_size", 20);
  remove_overlapped_unknown_objects_ =
    declare_parameter<bool>("remove_overlapped_unknown_objects", true);
  overlapped_judge_param_.precision_threshold =
    declare_parameter<double>("precision_threshold_to_judge_overlapped");
  overlapped_judge_param_.recall_threshold =
    declare_parameter<double>("recall_threshold_to_judge_overlapped", 0.5);
  overlapped_judge_param_.generalized_iou_threshold =
    convertListToClassMap(declare_parameter<std::vector<double>>("generalized_iou_threshold"));

  // get distance_threshold_map from distance_threshold_list
  /** TODO(Shin-kyoto):
   *  this implementation assumes index of vector shows class_label.
   *  if param supports map, refactor this code.
   */
  overlapped_judge_param_.distance_threshold_map =
    convertListToClassMap(declare_parameter<std::vector<double>>("distance_threshold_list"));

  const auto tmp = this->declare_parameter<std::vector<int64_t>>("can_assign_matrix");
  const std::vector<int> can_assign_matrix(tmp.begin(), tmp.end());
  const auto max_dist_matrix = this->declare_parameter<std::vector<double>>("max_dist_matrix");
  const auto max_rad_matrix = this->declare_parameter<std::vector<double>>("max_rad_matrix");
  const auto min_iou_matrix = this->declare_parameter<std::vector<double>>("min_iou_matrix");
  data_association_ = std::make_unique<DataAssociation>(
    can_assign_matrix, max_dist_matrix, max_rad_matrix, min_iou_matrix);

  // Create publishers and subscribers
  using std::placeholders::_1;
  using std::placeholders::_2;
  sync_ptr_ = std::make_shared<Sync>(SyncPolicy(sync_queue_size_), object0_sub_, object1_sub_);
  sync_ptr_->registerCallback(
    std::bind(&ObjectAssociationMergerNode::objectsCallback, this, _1, _2));

  merged_object_pub_ = create_publisher<autoware_auto_perception_msgs::msg::DetectedObjects>(
    "output/object", rclcpp::QoS{1});
}

void ObjectAssociationMergerNode::objectsCallback(
  const autoware_auto_perception_msgs::msg::DetectedObjects::ConstSharedPtr & input_objects0_msg,
  const autoware_auto_perception_msgs::msg::DetectedObjects::ConstSharedPtr & input_objects1_msg)
{
  // Guard
  if (merged_object_pub_->get_subscription_count() < 1) {
    return;
  }

  /* transform to base_link coordinate */
  autoware_auto_perception_msgs::msg::DetectedObjects transformed_objects0, transformed_objects1;
  if (
    !object_recognition_utils::transformObjects(
      *input_objects0_msg, base_link_frame_id_, tf_buffer_, transformed_objects0) ||
    !object_recognition_utils::transformObjects(
      *input_objects1_msg, base_link_frame_id_, tf_buffer_, transformed_objects1)) {
    return;
  }

  // build output msg
  autoware_auto_perception_msgs::msg::DetectedObjects output_msg;
  output_msg.header = input_objects0_msg->header;

  /* global nearest neighbor */
  std::unordered_map<int, int> direct_assignment, reverse_assignment;
  const auto & objects0 = transformed_objects0.objects;
  const auto & objects1 = transformed_objects1.objects;
  Eigen::MatrixXd score_matrix =
    data_association_->calcScoreMatrix(transformed_objects1, transformed_objects0);
  data_association_->assign(score_matrix, direct_assignment, reverse_assignment);

  for (size_t object0_idx = 0; object0_idx < objects0.size(); ++object0_idx) {
    const auto & object0 = objects0.at(object0_idx);
    if (direct_assignment.find(object0_idx) != direct_assignment.end()) {  // found and merge
      const auto & object1 = objects1.at(direct_assignment.at(object0_idx));
      switch (priority_mode_) {
        case PriorityMode::Object0:
          output_msg.objects.push_back(object0);
          break;
        case PriorityMode::Object1:
          output_msg.objects.push_back(object1);
          break;
        case PriorityMode::Confidence:
          if (object1.existence_probability <= object0.existence_probability)
            output_msg.objects.push_back(object0);
          else
            output_msg.objects.push_back(object1);
          break;
      }
    } else {  // not found
      output_msg.objects.push_back(object0);
    }
  }
  for (size_t object1_idx = 0; object1_idx < objects1.size(); ++object1_idx) {
    const auto & object1 = objects1.at(object1_idx);
    if (reverse_assignment.find(object1_idx) != reverse_assignment.end()) {  // found
    } else {                                                                 // not found
      output_msg.objects.push_back(object1);
    }
  }

  // Remove overlapped unknown object
  if (remove_overlapped_unknown_objects_) {
    std::vector<autoware_auto_perception_msgs::msg::DetectedObject> unknown_objects, known_objects;
    unknown_objects.reserve(output_msg.objects.size());
    known_objects.reserve(output_msg.objects.size());
    for (const auto & object : output_msg.objects) {
      if (object_recognition_utils::getHighestProbLabel(object.classification) == Label::UNKNOWN) {
        unknown_objects.push_back(object);
      } else {
        known_objects.push_back(object);
      }
    }
    output_msg.objects.clear();
    output_msg.objects = known_objects;
    for (const auto & unknown_object : unknown_objects) {
      bool is_overlapped = false;
      for (const auto & known_object : known_objects) {
        if (isUnknownObjectOverlapped(
              unknown_object, known_object, overlapped_judge_param_.precision_threshold,
              overlapped_judge_param_.recall_threshold,
              overlapped_judge_param_.distance_threshold_map,
              overlapped_judge_param_.generalized_iou_threshold)) {
          is_overlapped = true;
          break;
        }
      }
      if (!is_overlapped) {
        output_msg.objects.push_back(unknown_object);
      }
    }
  }

  // publish output msg
  merged_object_pub_->publish(output_msg);
}
}  // namespace object_association

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(object_association::ObjectAssociationMergerNode)
