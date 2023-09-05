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

#include <autoware_auto_perception_msgs/msg/object_classification.hpp>

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

using Label = autoware_auto_perception_msgs::msg::ObjectClassification;

ShapeEstimationNode::ShapeEstimationNode(const rclcpp::NodeOptions & node_options)
: Node("shape_estimation", node_options)
{
  using std::placeholders::_1;
  sub_ = create_subscription<DetectedObjectsWithFeature>(
    "input", rclcpp::QoS{1}, std::bind(&ShapeEstimationNode::callback, this, _1));

  pub_ = create_publisher<DetectedObjectsWithFeature>("objects", rclcpp::QoS{1});
  bool use_corrector = declare_parameter("use_corrector", true);
  bool use_filter = declare_parameter("use_filter", true);
  use_vehicle_reference_yaw_ = declare_parameter("use_vehicle_reference_yaw", true);
  use_vehicle_reference_shape_size_ = declare_parameter("use_vehicle_reference_shape_size", true);
  bool use_boost_bbox_optimizer = declare_parameter("use_boost_bbox_optimizer", false);
  RCLCPP_INFO(this->get_logger(), "using boost shape estimation : %d", use_boost_bbox_optimizer);
  estimator_ =
    std::make_unique<ShapeEstimator>(use_corrector, use_filter, use_boost_bbox_optimizer);
}

void ShapeEstimationNode::callback(const DetectedObjectsWithFeature::ConstSharedPtr input_msg)
{
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
    const auto & label = object.classification.front().label;
    const auto & feature = feature_object.feature;
    const bool is_vehicle = Label::CAR == label || Label::TRUCK == label || Label::BUS == label ||
                            Label::TRAILER == label;

    // convert ros to pcl
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(feature.cluster, *cluster);

    // check cluster data
    if (cluster->empty()) {
      continue;
    }

    // estimate shape and pose
    autoware_auto_perception_msgs::msg::Shape shape;
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

    // If the shape estimation fails, ignore it.
    if (!estimated_success) {
      continue;
    }

    output_msg.feature_objects.push_back(feature_object);
    output_msg.feature_objects.back().object.shape = shape;
    output_msg.feature_objects.back().object.kinematics.pose_with_covariance.pose = pose;
  }

  // Publish
  pub_->publish(output_msg);
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ShapeEstimationNode)
