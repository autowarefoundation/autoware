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

#ifndef ROI_CLUSTER_FUSION__ROI_CLUSTER_FUSION_NODELET_HPP_
#define ROI_CLUSTER_FUSION__ROI_CLUSTER_FUSION_NODELET_HPP_

#define EIGEN_MPL2_ONLY

#include <Eigen/Core>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <tier4_perception_msgs/msg/detected_objects_with_feature.hpp>

#include <boost/circular_buffer.hpp>

#include <cv_bridge/cv_bridge.h>
#include <message_filters/pass_through.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <map>
#include <memory>
#include <vector>

namespace roi_cluster_fusion
{
class Debugger
{
public:
  explicit Debugger(rclcpp::Node * node, const int camera_num);
  ~Debugger() = default;
  rclcpp::Node * node_;
  void showImage(
    const int id, const rclcpp::Time & time,
    const std::vector<sensor_msgs::msg::RegionOfInterest> & image_rois,
    const std::vector<sensor_msgs::msg::RegionOfInterest> & pointcloud_rois,
    const std::vector<Eigen::Vector2d> & points);

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & input_image_msg, const int id);
  std::shared_ptr<image_transport::ImageTransport> image_transport_;
  std::vector<image_transport::Subscriber> image_subs_;
  std::vector<image_transport::Publisher> image_pubs_;
  std::vector<boost::circular_buffer<sensor_msgs::msg::Image::ConstSharedPtr>> image_buffers_;
};

class RoiClusterFusionNodelet : public rclcpp::Node
{
public:
  explicit RoiClusterFusionNodelet(const rclcpp::NodeOptions & options);

private:
  void fusionCallback(
    tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr input_cluster_msg,
    tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr input_roi0_msg,
    tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr input_roi1_msg,
    tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr input_roi2_msg,
    tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr input_roi3_msg,
    tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr input_roi4_msg,
    tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr input_roi5_msg,
    tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr input_roi6_msg,
    tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr input_roi7_msg);
  void cameraInfoCallback(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr input_camera_info_msg, const int id);
  double calcIoU(
    const sensor_msgs::msg::RegionOfInterest & roi_1,
    const sensor_msgs::msg::RegionOfInterest & roi_2);
  double calcIoUX(
    const sensor_msgs::msg::RegionOfInterest & roi_1,
    const sensor_msgs::msg::RegionOfInterest & roi_2);
  double calcIoUY(
    const sensor_msgs::msg::RegionOfInterest & roi_1,
    const sensor_msgs::msg::RegionOfInterest & roi_2);

  rclcpp::Publisher<tier4_perception_msgs::msg::DetectedObjectsWithFeature>::SharedPtr
    labeled_cluster_pub_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr> v_camera_info_sub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_ptr_;
  message_filters::Subscriber<tier4_perception_msgs::msg::DetectedObjectsWithFeature> cluster_sub_;
  std::vector<std::shared_ptr<
    message_filters::Subscriber<tier4_perception_msgs::msg::DetectedObjectsWithFeature>>>
    v_roi_sub_;
  message_filters::PassThrough<tier4_perception_msgs::msg::DetectedObjectsWithFeature> passthrough_;
  typedef message_filters::sync_policies::ApproximateTime<
    tier4_perception_msgs::msg::DetectedObjectsWithFeature,
    tier4_perception_msgs::msg::DetectedObjectsWithFeature,
    tier4_perception_msgs::msg::DetectedObjectsWithFeature,
    tier4_perception_msgs::msg::DetectedObjectsWithFeature,
    tier4_perception_msgs::msg::DetectedObjectsWithFeature,
    tier4_perception_msgs::msg::DetectedObjectsWithFeature,
    tier4_perception_msgs::msg::DetectedObjectsWithFeature,
    tier4_perception_msgs::msg::DetectedObjectsWithFeature,
    tier4_perception_msgs::msg::DetectedObjectsWithFeature>
    SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Sync;
  std::shared_ptr<Sync> sync_ptr_;
  inline void dummyCallback(
    tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr input)
  {
    auto dummy = input;
    passthrough_.add(dummy);
  }
  // ROS Parameters
  bool use_iou_x_;
  bool use_iou_y_;
  bool use_iou_;
  bool use_cluster_semantic_type_;
  double iou_threshold_;
  int rois_number_;
  std::map<int, sensor_msgs::msg::CameraInfo> m_camera_info_;
  std::shared_ptr<Debugger> debugger_;
};

}  // namespace roi_cluster_fusion

#endif  // ROI_CLUSTER_FUSION__ROI_CLUSTER_FUSION_NODELET_HPP_
