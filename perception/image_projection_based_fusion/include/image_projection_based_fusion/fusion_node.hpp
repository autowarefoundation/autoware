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

#ifndef IMAGE_PROJECTION_BASED_FUSION__FUSION_NODE_HPP_
#define IMAGE_PROJECTION_BASED_FUSION__FUSION_NODE_HPP_

#include <image_projection_based_fusion/debugger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/ros/debug_publisher.hpp>
#include <tier4_autoware_utils/system/stop_watch.hpp>

#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tier4_perception_msgs/msg/detected_objects_with_feature.hpp>

#include <message_filters/pass_through.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace image_projection_based_fusion
{
using autoware_auto_perception_msgs::msg::DetectedObject;
using autoware_auto_perception_msgs::msg::DetectedObjects;
using sensor_msgs::msg::PointCloud2;
using tier4_perception_msgs::msg::DetectedObjectsWithFeature;
using tier4_perception_msgs::msg::DetectedObjectWithFeature;

template <class Msg, class ObjType>
class FusionNode : public rclcpp::Node
{
public:
  /** \brief constructor. */
  explicit FusionNode(const std::string & node_name, const rclcpp::NodeOptions & options);
  /** \brief constructor.
   * \param queue_size the maximum queue size
   */
  explicit FusionNode(
    const std::string & node_name, const rclcpp::NodeOptions & options, int queue_size);

protected:
  void cameraInfoCallback(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr input_camera_info_msg,
    const std::size_t camera_id);

  virtual void preprocess(Msg & output_msg);

  // callback for Msg subscription
  virtual void subCallback(const typename Msg::ConstSharedPtr input_msg);

  // callback for roi subscription
  virtual void roiCallback(
    const DetectedObjectsWithFeature::ConstSharedPtr input_roi_msg, const std::size_t roi_i);

  virtual void fuseOnSingleImage(
    const Msg & input_msg, const std::size_t image_id,
    const DetectedObjectsWithFeature & input_roi_msg,
    const sensor_msgs::msg::CameraInfo & camera_info, Msg & output_msg) = 0;

  // set args if you need
  virtual void postprocess(Msg & output_msg);

  void publish(const Msg & output_msg);

  void timer_callback();
  void setPeriod(const int64_t new_period);

  std::size_t rois_number_{1};
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // camera_info
  std::map<std::size_t, sensor_msgs::msg::CameraInfo> camera_info_map_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr> camera_info_subs_;

  rclcpp::TimerBase::SharedPtr timer_;
  double timeout_ms_{};
  double match_threshold_ms_{};

  /** \brief A vector of subscriber. */
  typename rclcpp::Subscription<Msg>::SharedPtr sub_;
  std::vector<rclcpp::Subscription<DetectedObjectsWithFeature>::SharedPtr> rois_subs_;

  /** \brief Input point cloud topics. */
  std::vector<std::string> input_topics_;
  std::vector<double> input_offset_ms_;

  // cache for fusion
  std::vector<bool> is_fused_;
  std::pair<int64_t, typename Msg::SharedPtr> sub_stdpair_;
  std::vector<std::map<int64_t, DetectedObjectsWithFeature::ConstSharedPtr>> roi_stdmap_;
  std::mutex mutex_;

  // output publisher
  typename rclcpp::Publisher<Msg>::SharedPtr pub_ptr_;

  // debugger
  std::shared_ptr<Debugger> debugger_;
  virtual bool out_of_scope(const ObjType & obj) = 0;
  float filter_scope_minx_;
  float filter_scope_maxx_;
  float filter_scope_miny_;
  float filter_scope_maxy_;
  float filter_scope_minz_;
  float filter_scope_maxz_;

  /** \brief processing time publisher. **/
  std::unique_ptr<tier4_autoware_utils::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_;
  std::unique_ptr<tier4_autoware_utils::DebugPublisher> debug_publisher_;
};

}  // namespace image_projection_based_fusion

#endif  // IMAGE_PROJECTION_BASED_FUSION__FUSION_NODE_HPP_
