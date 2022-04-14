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

#define EIGEN_MPL2_ONLY

#include "image_projection_based_fusion/fusion_node.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <tier4_perception_msgs/msg/detected_object_with_feature.hpp>

#include <boost/optional.hpp>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace image_projection_based_fusion
{

template <class Msg>
FusionNode<Msg>::FusionNode(const std::string & node_name, const rclcpp::NodeOptions & options)
: Node(node_name, options), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
{
  rois_number_ = static_cast<std::size_t>(declare_parameter("rois_number", 1));
  if (rois_number_ < 1) {
    RCLCPP_WARN(
      this->get_logger(), "minimum rois_number is 1. current rois_number is %zu", rois_number_);
    rois_number_ = 1;
  }
  if (rois_number_ > 8) {
    RCLCPP_WARN(
      this->get_logger(), "maximum rois_number is 8. current rois_number is %zu", rois_number_);
    rois_number_ = 8;
  }

  // subscribers
  sub_.subscribe(this, "input", rclcpp::QoS(1).get_rmw_qos_profile());

  camera_info_subs_.resize(rois_number_);
  for (std::size_t roi_i = 0; roi_i < rois_number_; ++roi_i) {
    std::function<void(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)> fnc =
      std::bind(&FusionNode::cameraInfoCallback, this, std::placeholders::_1, roi_i);
    camera_info_subs_.at(roi_i) = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "input/camera_info" + std::to_string(roi_i), rclcpp::QoS{1}.best_effort(), fnc);
  }

  rois_subs_.resize(rois_number_);
  for (std::size_t roi_i = 0; roi_i < rois_number_; ++roi_i) {
    rois_subs_.at(roi_i) =
      std::make_shared<message_filters::Subscriber<DetectedObjectsWithFeature>>(
        this, "input/rois" + std::to_string(roi_i), rclcpp::QoS{1}.get_rmw_qos_profile());
  }

  // add dummy callback to enable passthrough filter
  rois_subs_.at(0)->registerCallback(
    std::bind(&FusionNode::dummyCallback, this, std::placeholders::_1));
  switch (rois_number_) {
    case 1:
      sync_ptr_ = std::make_shared<Sync>(
        SyncPolicy(10), sub_, *rois_subs_.at(0), passthrough_, passthrough_, passthrough_,
        passthrough_, passthrough_, passthrough_, passthrough_);
      break;
    case 2:
      sync_ptr_ = std::make_shared<Sync>(
        SyncPolicy(10), sub_, *rois_subs_.at(0), *rois_subs_.at(1), passthrough_, passthrough_,
        passthrough_, passthrough_, passthrough_, passthrough_);
      break;
    case 3:
      sync_ptr_ = std::make_shared<Sync>(
        SyncPolicy(10), sub_, *rois_subs_.at(0), *rois_subs_.at(1), *rois_subs_.at(2), passthrough_,
        passthrough_, passthrough_, passthrough_, passthrough_);
      break;
    case 4:
      sync_ptr_ = std::make_shared<Sync>(
        SyncPolicy(10), sub_, *rois_subs_.at(0), *rois_subs_.at(1), *rois_subs_.at(2),
        *rois_subs_.at(3), passthrough_, passthrough_, passthrough_, passthrough_);
      break;
    case 5:
      sync_ptr_ = std::make_shared<Sync>(
        SyncPolicy(10), sub_, *rois_subs_.at(0), *rois_subs_.at(1), *rois_subs_.at(2),
        *rois_subs_.at(3), *rois_subs_.at(4), passthrough_, passthrough_, passthrough_);
      break;
    case 6:
      sync_ptr_ = std::make_shared<Sync>(
        SyncPolicy(10), sub_, *rois_subs_.at(0), *rois_subs_.at(1), *rois_subs_.at(2),
        *rois_subs_.at(3), *rois_subs_.at(4), *rois_subs_.at(5), passthrough_, passthrough_);
      break;
    case 7:
      sync_ptr_ = std::make_shared<Sync>(
        SyncPolicy(10), sub_, *rois_subs_.at(0), *rois_subs_.at(1), *rois_subs_.at(2),
        *rois_subs_.at(3), *rois_subs_.at(4), *rois_subs_.at(5), *rois_subs_.at(6), passthrough_);
      break;
    case 8:
      sync_ptr_ = std::make_shared<Sync>(
        SyncPolicy(10), sub_, *rois_subs_.at(0), *rois_subs_.at(1), *rois_subs_.at(2),
        *rois_subs_.at(3), *rois_subs_.at(4), *rois_subs_.at(5), *rois_subs_.at(6),
        *rois_subs_.at(7));
    default:
      return;
  }

  sync_ptr_->registerCallback(std::bind(
    &FusionNode::fusionCallback, this, std::placeholders::_1, std::placeholders::_2,
    std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6,
    std::placeholders::_7, std::placeholders::_8, std::placeholders::_9));

  // publisher
  pub_ptr_ = this->create_publisher<Msg>("output", rclcpp::QoS{1});

  // debugger
  if (declare_parameter("debug_mode", false)) {
    debugger_ = std::make_shared<Debugger>(this, rois_number_);
  }
}

template <class Msg>
void FusionNode<Msg>::cameraInfoCallback(
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr input_camera_info_msg,
  const std::size_t camera_id)
{
  camera_info_map_[camera_id] = *input_camera_info_msg;
}

template <class Msg>
void FusionNode<Msg>::preprocess(Msg & ouput_msg __attribute__((unused)))
{
  // do nothing by default
}

template <class Msg>
void FusionNode<Msg>::fusionCallback(
  typename Msg::ConstSharedPtr input_msg, DetectedObjectsWithFeature::ConstSharedPtr input_roi0_msg,
  DetectedObjectsWithFeature::ConstSharedPtr input_roi1_msg,
  DetectedObjectsWithFeature::ConstSharedPtr input_roi2_msg,
  DetectedObjectsWithFeature::ConstSharedPtr input_roi3_msg,
  DetectedObjectsWithFeature::ConstSharedPtr input_roi4_msg,
  DetectedObjectsWithFeature::ConstSharedPtr input_roi5_msg,
  DetectedObjectsWithFeature::ConstSharedPtr input_roi6_msg,
  DetectedObjectsWithFeature::ConstSharedPtr input_roi7_msg)
{
  if (pub_ptr_->get_subscription_count() < 1) {
    return;
  }

  Msg output_msg = *input_msg;

  preprocess(output_msg);

  for (std::size_t image_id = 0; image_id < rois_subs_.size(); ++image_id) {
    DetectedObjectsWithFeature::ConstSharedPtr input_roi_msg;
    switch (image_id) {
      case 0:
        input_roi_msg = input_roi0_msg;
        break;
      case 1:
        input_roi_msg = input_roi1_msg;
        break;
      case 2:
        input_roi_msg = input_roi2_msg;
        break;
      case 3:
        input_roi_msg = input_roi3_msg;
        break;
      case 4:
        input_roi_msg = input_roi4_msg;
        break;
      case 5:
        input_roi_msg = input_roi5_msg;
        break;
      case 6:
        input_roi_msg = input_roi6_msg;
        break;
      case 7:
        input_roi_msg = input_roi7_msg;
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "invalid image id. id is %zu", image_id);
        return;
    }

    if (camera_info_map_.find(image_id) == camera_info_map_.end()) {
      RCLCPP_WARN(this->get_logger(), "no camera info. id is %zu", image_id);
      continue;
    }
    if (debugger_) {
      debugger_->clear();
    }

    fuseOnSingleImage(
      *input_msg, image_id, *input_roi_msg, camera_info_map_.at(image_id), output_msg);
  }

  postprocess();

  publish(output_msg);
}

template <class Msg>
void FusionNode<Msg>::postprocess()
{
  // do nothing by default
}

template <class Msg>
void FusionNode<Msg>::publish(const Msg & output_msg)
{
  pub_ptr_->publish(output_msg);
}

template class FusionNode<DetectedObjects>;
template class FusionNode<DetectedObjectsWithFeature>;
}  // namespace image_projection_based_fusion
