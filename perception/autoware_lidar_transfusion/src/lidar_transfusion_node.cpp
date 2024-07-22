// Copyright 2024 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "autoware/lidar_transfusion/lidar_transfusion_node.hpp"

#include "autoware/lidar_transfusion/utils.hpp"

namespace autoware::lidar_transfusion
{

LidarTransfusionNode::LidarTransfusionNode(const rclcpp::NodeOptions & options)
: Node("lidar_transfusion", options), tf_buffer_(this->get_clock())
{
  auto descriptor = rcl_interfaces::msg::ParameterDescriptor{}.set__read_only(true);
  // network
  class_names_ = this->declare_parameter<std::vector<std::string>>("class_names", descriptor);
  const std::string trt_precision =
    this->declare_parameter<std::string>("trt_precision", descriptor);
  const std::size_t cloud_capacity =
    this->declare_parameter<std::int64_t>("cloud_capacity", descriptor);
  const auto voxels_num = this->declare_parameter<std::vector<int64_t>>("voxels_num", descriptor);
  const auto point_cloud_range =
    this->declare_parameter<std::vector<double>>("point_cloud_range", descriptor);
  const auto voxel_size = this->declare_parameter<std::vector<double>>("voxel_size", descriptor);
  const std::size_t num_proposals =
    this->declare_parameter<std::int64_t>("num_proposals", descriptor);
  const std::string onnx_path = this->declare_parameter<std::string>("onnx_path", descriptor);
  const std::string engine_path = this->declare_parameter<std::string>("engine_path", descriptor);

  if (point_cloud_range.size() != 6) {
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("lidar_transfusion"),
      "The size of point_cloud_range != 6: use the default parameters.");
  }
  if (voxel_size.size() != 3) {
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("lidar_transfusion"),
      "The size of voxel_size != 3: use the default parameters.");
  }

  // pre-process
  const std::string densification_world_frame_id =
    this->declare_parameter<std::string>("densification_world_frame_id", descriptor);
  const int densification_num_past_frames =
    this->declare_parameter<int64_t>("densification_num_past_frames", descriptor);

  // post-process
  const float circle_nms_dist_threshold =
    static_cast<float>(this->declare_parameter<double>("circle_nms_dist_threshold", descriptor));
  {  // IoU NMS
    NMSParams p;
    p.nms_type_ = NMS_TYPE::IoU_BEV;
    p.target_class_names_ =
      this->declare_parameter<std::vector<std::string>>("iou_nms_target_class_names", descriptor);
    p.search_distance_2d_ =
      this->declare_parameter<double>("iou_nms_search_distance_2d", descriptor);
    p.iou_threshold_ = this->declare_parameter<double>("iou_nms_threshold", descriptor);
    iou_bev_nms_.setParameters(p);
  }
  const auto yaw_norm_thresholds =
    this->declare_parameter<std::vector<double>>("yaw_norm_thresholds", descriptor);
  const float score_threshold =
    static_cast<float>(this->declare_parameter<double>("score_threshold", descriptor));

  NetworkParam network_param(onnx_path, engine_path, trt_precision);
  DensificationParam densification_param(
    densification_world_frame_id, densification_num_past_frames);
  TransfusionConfig config(
    cloud_capacity, voxels_num, point_cloud_range, voxel_size, num_proposals,
    circle_nms_dist_threshold, yaw_norm_thresholds, score_threshold);

  const auto allow_remapping_by_area_matrix =
    this->declare_parameter<std::vector<int64_t>>("allow_remapping_by_area_matrix", descriptor);
  const auto min_area_matrix =
    this->declare_parameter<std::vector<double>>("min_area_matrix", descriptor);
  const auto max_area_matrix =
    this->declare_parameter<std::vector<double>>("max_area_matrix", descriptor);
  detection_class_remapper_.setParameters(
    allow_remapping_by_area_matrix, min_area_matrix, max_area_matrix);

  detector_ptr_ = std::make_unique<TransfusionTRT>(network_param, densification_param, config);

  cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "~/input/pointcloud", rclcpp::SensorDataQoS{}.keep_last(1),
    std::bind(&LidarTransfusionNode::cloudCallback, this, std::placeholders::_1));

  objects_pub_ = this->create_publisher<autoware_perception_msgs::msg::DetectedObjects>(
    "~/output/objects", rclcpp::QoS(1));

  published_time_pub_ = std::make_unique<autoware::universe_utils::PublishedTimePublisher>(this);

  // initialize debug tool
  {
    using autoware::universe_utils::DebugPublisher;
    using autoware::universe_utils::StopWatch;
    stop_watch_ptr_ = std::make_unique<StopWatch<std::chrono::milliseconds>>();
    debug_publisher_ptr_ = std::make_unique<DebugPublisher>(this, this->get_name());
    stop_watch_ptr_->tic("cyclic");
    stop_watch_ptr_->tic("processing/total");
  }

  if (this->declare_parameter<bool>("build_only", false, descriptor)) {
    RCLCPP_INFO(this->get_logger(), "TensorRT engine is built and shutdown node.");
    rclcpp::shutdown();
  }
}

void LidarTransfusionNode::cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  const auto objects_sub_count =
    objects_pub_->get_subscription_count() + objects_pub_->get_intra_process_subscription_count();
  if (objects_sub_count < 1) {
    return;
  }

  if (stop_watch_ptr_) {
    stop_watch_ptr_->toc("processing/total", true);
  }

  std::vector<Box3D> det_boxes3d;
  std::unordered_map<std::string, double> proc_timing;
  bool is_success = detector_ptr_->detect(*msg, tf_buffer_, det_boxes3d, proc_timing);
  if (!is_success) {
    return;
  }

  std::vector<autoware_perception_msgs::msg::DetectedObject> raw_objects;
  raw_objects.reserve(det_boxes3d.size());
  for (const auto & box3d : det_boxes3d) {
    autoware_perception_msgs::msg::DetectedObject obj;
    box3DToDetectedObject(box3d, class_names_, obj);
    raw_objects.emplace_back(obj);
  }

  autoware_perception_msgs::msg::DetectedObjects output_msg;
  output_msg.header = msg->header;
  output_msg.objects = iou_bev_nms_.apply(raw_objects);

  detection_class_remapper_.mapClasses(output_msg);

  if (objects_sub_count > 0) {
    objects_pub_->publish(output_msg);
    published_time_pub_->publish_if_subscribed(objects_pub_, output_msg.header.stamp);
  }

  // add processing time for debug
  if (debug_publisher_ptr_ && stop_watch_ptr_) {
    const double cyclic_time_ms = stop_watch_ptr_->toc("cyclic", true);
    const double processing_time_ms = stop_watch_ptr_->toc("processing/total", true);
    const double pipeline_latency_ms =
      std::chrono::duration<double, std::milli>(
        std::chrono::nanoseconds(
          (this->get_clock()->now() - output_msg.header.stamp).nanoseconds()))
        .count();
    debug_publisher_ptr_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/cyclic_time_ms", cyclic_time_ms);
    debug_publisher_ptr_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/pipeline_latency_ms", pipeline_latency_ms);
    debug_publisher_ptr_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/processing_time/total_ms", processing_time_ms);
    for (const auto & [topic, time_ms] : proc_timing) {
      debug_publisher_ptr_->publish<tier4_debug_msgs::msg::Float64Stamped>(topic, time_ms);
    }
  }
}

}  // namespace autoware::lidar_transfusion

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::lidar_transfusion::LidarTransfusionNode)
