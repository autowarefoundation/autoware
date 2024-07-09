// Copyright 2024 TIER IV, Inc.
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

#include "low_intensity_cluster_filter_node.hpp"

#include "autoware/universe_utils/geometry/geometry.hpp"

#include <pcl_ros/transforms.hpp>

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <pcl_conversions/pcl_conversions.h>

namespace autoware::low_intensity_cluster_filter
{
LowIntensityClusterFilter::LowIntensityClusterFilter(const rclcpp::NodeOptions & node_options)
: Node("low_intensity_cluster_filter_node", node_options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  intensity_threshold_ = declare_parameter<double>("intensity_threshold");
  existence_probability_threshold_ = declare_parameter<double>("existence_probability_threshold");
  max_x_ = declare_parameter<double>("max_x");
  min_x_ = declare_parameter<double>("min_x");
  max_y_ = declare_parameter<double>("max_y");
  min_y_ = declare_parameter<double>("min_y");

  filter_target_.UNKNOWN = declare_parameter<bool>("filter_target_label.UNKNOWN");
  filter_target_.CAR = declare_parameter<bool>("filter_target_label.CAR");
  filter_target_.TRUCK = declare_parameter<bool>("filter_target_label.TRUCK");
  filter_target_.BUS = declare_parameter<bool>("filter_target_label.BUS");
  filter_target_.TRAILER = declare_parameter<bool>("filter_target_label.TRAILER");
  filter_target_.MOTORCYCLE = declare_parameter<bool>("filter_target_label.MOTORCYCLE");
  filter_target_.BICYCLE = declare_parameter<bool>("filter_target_label.BICYCLE");
  filter_target_.PEDESTRIAN = declare_parameter<bool>("filter_target_label.PEDESTRIAN");

  using std::placeholders::_1;
  // Set publisher/subscriber
  object_sub_ = this->create_subscription<tier4_perception_msgs::msg::DetectedObjectsWithFeature>(
    "input/objects", rclcpp::QoS{1},
    std::bind(&LowIntensityClusterFilter::objectCallback, this, _1));
  object_pub_ = this->create_publisher<tier4_perception_msgs::msg::DetectedObjectsWithFeature>(
    "output/objects", rclcpp::QoS{1});
  // initialize debug tool
  {
    using autoware::universe_utils::DebugPublisher;
    using autoware::universe_utils::StopWatch;
    stop_watch_ptr_ = std::make_unique<StopWatch<std::chrono::milliseconds>>();
    debug_publisher_ptr_ =
      std::make_unique<DebugPublisher>(this, "low_intensity_cluster_filter_node");
    stop_watch_ptr_->tic("cyclic_time");
    stop_watch_ptr_->tic("processing_time");
  }
}

void LowIntensityClusterFilter::objectCallback(
  const tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr input_msg)
{
  // Guard
  stop_watch_ptr_->toc("processing_time", true);
  if (object_pub_->get_subscription_count() < 1) return;

  tier4_perception_msgs::msg::DetectedObjectsWithFeature output_object_msg;
  output_object_msg.header = input_msg->header;
  geometry_msgs::msg::TransformStamped transform_stamp;
  try {
    transform_stamp = tf_buffer_.lookupTransform(
      input_msg->header.frame_id, base_link_frame_id_, tf2_ros::fromMsg(input_msg->header.stamp));
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(get_logger(), "Failed to lookup transform: %s", ex.what());
    return;
  }
  geometry_msgs::msg::Pose min_range;
  min_range.position.x = min_x_;
  min_range.position.y = min_y_;
  geometry_msgs::msg::Pose max_pose;
  max_pose.position.x = max_x_;
  max_pose.position.y = max_y_;
  auto min_ranged_transformed = autoware::universe_utils::transformPose(min_range, transform_stamp);
  auto max_range_transformed = autoware::universe_utils::transformPose(max_pose, transform_stamp);
  for (const auto & feature_object : input_msg->feature_objects) {
    const auto & object = feature_object.object;
    const auto & label = object.classification.front().label;
    const auto & feature = feature_object.feature;
    const auto & cluster = feature.cluster;
    const auto existence_probability = object.existence_probability;
    const auto & position = object.kinematics.pose_with_covariance.pose.position;
    bool is_inside_validation_range = min_ranged_transformed.position.x < position.x &&
                                      position.x < max_range_transformed.position.x &&
                                      min_ranged_transformed.position.y < position.x &&
                                      position.y < max_range_transformed.position.y;
    int intensity_index = pcl::getFieldIndex(cluster, "intensity");
    if (
      intensity_index != -1 && filter_target_.isTarget(label) && is_inside_validation_range &&
      !isValidatedCluster(cluster) && existence_probability < existence_probability_threshold_) {
      continue;
    }
    output_object_msg.feature_objects.emplace_back(feature_object);
  }
  object_pub_->publish(output_object_msg);
  if (debug_publisher_ptr_ && stop_watch_ptr_) {
    const double cyclic_time_ms = stop_watch_ptr_->toc("cyclic_time", true);
    const double processing_time_ms = stop_watch_ptr_->toc("processing_time", true);
    debug_publisher_ptr_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/cyclic_time_ms", cyclic_time_ms);
    debug_publisher_ptr_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/processing_time_ms", processing_time_ms);
  }
}
bool LowIntensityClusterFilter::isValidatedCluster(const sensor_msgs::msg::PointCloud2 & cluster)
{
  double mean_intensity = 0.0;
  if (cluster.point_step < 16) {
    RCLCPP_WARN(get_logger(), "Invalid point cloud data. point_step is less than 16.");
    return true;
  }
  for (sensor_msgs::PointCloud2ConstIterator<uint8_t> iter(cluster, "intensity");
       iter != iter.end(); ++iter) {
    mean_intensity += static_cast<float>(*iter);
  }
  const size_t num_points = cluster.width * cluster.height;
  mean_intensity /= static_cast<double>(num_points);
  if (mean_intensity > intensity_threshold_) {
    return true;
  }
  return false;
}

}  // namespace autoware::low_intensity_cluster_filter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::low_intensity_cluster_filter::LowIntensityClusterFilter)
