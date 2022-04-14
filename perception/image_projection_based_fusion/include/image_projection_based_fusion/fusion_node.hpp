// Copyright 2021 TIER IV, Inc.
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

#ifndef IMAGE_PROJECTION_BASED_FUSION__FUSION_NODE_HPP_
#define IMAGE_PROJECTION_BASED_FUSION__FUSION_NODE_HPP_

#include <image_projection_based_fusion/debugger.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tier4_perception_msgs/msg/detected_objects_with_feature.hpp>

#include <message_filters/pass_through.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace image_projection_based_fusion
{
using autoware_auto_perception_msgs::msg::DetectedObject;
using autoware_auto_perception_msgs::msg::DetectedObjects;
using tier4_perception_msgs::msg::DetectedObjectsWithFeature;
using tier4_perception_msgs::msg::DetectedObjectWithFeature;

template <class Msg>
class FusionNode : public rclcpp::Node
{
public:
  explicit FusionNode(const std::string & node_name, const rclcpp::NodeOptions & options);

protected:
  void cameraInfoCallback(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr input_camera_info_msg,
    const std::size_t camera_id);

  void fusionCallback(
    typename Msg::ConstSharedPtr input_msg,
    DetectedObjectsWithFeature::ConstSharedPtr input_roi0_msg,
    DetectedObjectsWithFeature::ConstSharedPtr input_roi1_msg,
    DetectedObjectsWithFeature::ConstSharedPtr input_roi2_msg,
    DetectedObjectsWithFeature::ConstSharedPtr input_roi3_msg,
    DetectedObjectsWithFeature::ConstSharedPtr input_roi4_msg,
    DetectedObjectsWithFeature::ConstSharedPtr input_roi5_msg,
    DetectedObjectsWithFeature::ConstSharedPtr input_roi6_msg,
    DetectedObjectsWithFeature::ConstSharedPtr input_roi7_msg);

  virtual void preprocess(Msg & output_msg);

  virtual void fuseOnSingleImage(
    const Msg & input_msg, const std::size_t image_id,
    const DetectedObjectsWithFeature & input_roi_msg,
    const sensor_msgs::msg::CameraInfo & camera_info, Msg & output_msg) = 0;

  // set args if you need
  virtual void postprocess();

  void publish(const Msg & output_msg);

  std::size_t rois_number_{1};
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // camera_info
  std::map<std::size_t, sensor_msgs::msg::CameraInfo> camera_info_map_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr> camera_info_subs_;

  // fusion
  typename message_filters::Subscriber<Msg> sub_;
  message_filters::PassThrough<DetectedObjectsWithFeature> passthrough_;
  std::vector<std::shared_ptr<message_filters::Subscriber<DetectedObjectsWithFeature>>> rois_subs_;
  inline void dummyCallback(DetectedObjectsWithFeature::ConstSharedPtr input)
  {
    passthrough_.add(input);
  }
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    Msg, DetectedObjectsWithFeature, DetectedObjectsWithFeature, DetectedObjectsWithFeature,
    DetectedObjectsWithFeature, DetectedObjectsWithFeature, DetectedObjectsWithFeature,
    DetectedObjectsWithFeature, DetectedObjectsWithFeature>;
  using Sync = message_filters::Synchronizer<SyncPolicy>;
  typename std::shared_ptr<Sync> sync_ptr_;

  // output
  typename rclcpp::Publisher<Msg>::SharedPtr pub_ptr_;

  // debugger
  std::shared_ptr<Debugger> debugger_;
};

}  // namespace image_projection_based_fusion

#endif  // IMAGE_PROJECTION_BASED_FUSION__FUSION_NODE_HPP_
