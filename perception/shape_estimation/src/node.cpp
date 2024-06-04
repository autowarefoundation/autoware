// Copyright 2018 Autoware Foundation. All rights reserved.
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

#include "shape_estimation/shape_estimator.hpp"

#include <node.hpp>
#include <tier4_autoware_utils/math/unit_conversion.hpp>

#include <autoware_perception_msgs/msg/object_classification.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <memory>
#include <string>

using Label = autoware_perception_msgs::msg::ObjectClassification;

ShapeEstimationNode::ShapeEstimationNode(const rclcpp::NodeOptions & node_options)
: Node("shape_estimation", node_options)
{
  using std::placeholders::_1;
  sub_ = create_subscription<DetectedObjectsWithFeature>(
    "input", rclcpp::QoS{1}, std::bind(&ShapeEstimationNode::callback, this, _1));

  pub_ = create_publisher<DetectedObjectsWithFeature>("objects", rclcpp::QoS{1});
  bool use_corrector = declare_parameter<bool>("use_corrector");
  bool use_filter = declare_parameter<bool>("use_filter");
  use_vehicle_reference_yaw_ = declare_parameter<bool>("use_vehicle_reference_yaw");
  use_vehicle_reference_shape_size_ = declare_parameter<bool>("use_vehicle_reference_shape_size");
  bool use_boost_bbox_optimizer = declare_parameter<bool>("use_boost_bbox_optimizer");
  fix_filtered_objects_label_to_unknown_ =
    declare_parameter<bool>("fix_filtered_objects_label_to_unknown");
  RCLCPP_INFO(this->get_logger(), "using boost shape estimation : %d", use_boost_bbox_optimizer);
  estimator_ =
    std::make_unique<ShapeEstimator>(use_corrector, use_filter, use_boost_bbox_optimizer);

  processing_time_publisher_ =
    std::make_unique<tier4_autoware_utils::DebugPublisher>(this, "shape_estimation");
  stop_watch_ptr_ = std::make_unique<tier4_autoware_utils::StopWatch<std::chrono::milliseconds>>();
  stop_watch_ptr_->tic("cyclic_time");
  stop_watch_ptr_->tic("processing_time");
  published_time_publisher_ = std::make_unique<tier4_autoware_utils::PublishedTimePublisher>(this);
}

static autoware_perception_msgs::msg::ObjectClassification::_label_type get_label(
  const autoware_perception_msgs::msg::DetectedObject::_classification_type & classification)
{
  if (classification.empty()) {
    return Label::UNKNOWN;
  }
  return classification.front().label;
}

static bool label_is_vehicle(
  const autoware_perception_msgs::msg::ObjectClassification::_label_type & label)
{
  return Label::CAR == label || Label::TRUCK == label || Label::BUS == label ||
         Label::TRAILER == label;
}

void ShapeEstimationNode::callback(const DetectedObjectsWithFeature::ConstSharedPtr input_msg)
{
  stop_watch_ptr_->toc("processing_time", true);
  // Guard
  if (pub_->get_subscription_count() < 1) {
    return;
  }

  // Create output msg
  DetectedObjectsWithFeature output_msg;
  output_msg.header = input_msg->header;

  // Estimate shape for each object and pack msg
  for (const auto & feature_object : input_msg->feature_objects) {
    const auto & object = feature_object.object;
    const auto label = get_label(object.classification);
    const auto is_vehicle = label_is_vehicle(label);
    const auto & feature = feature_object.feature;
    // convert ros to pcl
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(feature.cluster, *cluster);

    // check cluster data
    if (cluster->empty()) {
      continue;
    }

    // estimate shape and pose
    autoware_perception_msgs::msg::Shape shape;
    geometry_msgs::msg::Pose pose;
    boost::optional<ReferenceYawInfo> ref_yaw_info = boost::none;
    boost::optional<ReferenceShapeSizeInfo> ref_shape_size_info = boost::none;
    if (use_vehicle_reference_yaw_ && is_vehicle) {
      ref_yaw_info = ReferenceYawInfo{
        static_cast<float>(tf2::getYaw(object.kinematics.pose_with_covariance.pose.orientation)),
        tier4_autoware_utils::deg2rad(10)};
    }
    if (use_vehicle_reference_shape_size_ && is_vehicle) {
      ref_shape_size_info = ReferenceShapeSizeInfo{object.shape, ReferenceShapeSizeInfo::Mode::Min};
    }
    const bool estimated_success = estimator_->estimateShapeAndPose(
      label, *cluster, ref_yaw_info, ref_shape_size_info, shape, pose);

    // If the shape estimation fails, change to Unknown object.
    if (!fix_filtered_objects_label_to_unknown_ && !estimated_success) {
      continue;
    }
    output_msg.feature_objects.push_back(feature_object);
    if (!estimated_success) {
      output_msg.feature_objects.back().object.classification.front().label = Label::UNKNOWN;
    }

    output_msg.feature_objects.back().object.shape = shape;
    output_msg.feature_objects.back().object.kinematics.pose_with_covariance.pose = pose;
  }

  // Publish
  pub_->publish(output_msg);
  published_time_publisher_->publish_if_subscribed(pub_, output_msg.header.stamp);
  processing_time_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
    "debug/cyclic_time_ms", stop_watch_ptr_->toc("cyclic_time", true));
  processing_time_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
    "debug/processing_time_ms", stop_watch_ptr_->toc("processing_time", true));
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ShapeEstimationNode)
