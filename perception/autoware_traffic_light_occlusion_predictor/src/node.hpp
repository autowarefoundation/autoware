// Copyright 2023 Tier IV, Inc.
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

#ifndef NODE_HPP_
#define NODE_HPP_

#include "occlusion_predictor.hpp"

#include <perception_utils/prime_synchronizer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <traffic_light_utils/traffic_light_utils.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tier4_perception_msgs/msg/traffic_light_array.hpp>
#include <tier4_perception_msgs/msg/traffic_light_roi_array.hpp>

#if __has_include(<image_geometry/pinhole_camera_model.hpp>)
#include <image_geometry/pinhole_camera_model.hpp>  // for ROS 2 Jazzy or newer
#else
#include <image_geometry/pinhole_camera_model.h>  // for ROS 2 Humble or older
#endif

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <map>
#include <memory>
#include <mutex>
#include <vector>

namespace autoware::traffic_light
{
class TrafficLightOcclusionPredictorNode : public rclcpp::Node
{
public:
  explicit TrafficLightOcclusionPredictorNode(const rclcpp::NodeOptions & node_options);

private:
  struct Config
  {
    double azimuth_occlusion_resolution_deg;
    double elevation_occlusion_resolution_deg;
    double max_valid_pt_dist;
    double max_image_cloud_delay;
    double max_wait_t;
    int max_occlusion_ratio;
  };

private:
  /**
   * @brief receive the lanelet2 map
   *
   * @param input_msg
   */
  void mapCallback(const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr input_msg);
  /**
   * @brief subscribers
   *
   */
  void syncCallback(
    const tier4_perception_msgs::msg::TrafficLightArray::ConstSharedPtr in_signal_msg,
    const tier4_perception_msgs::msg::TrafficLightRoiArray::ConstSharedPtr in_roi_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr in_cam_info_msg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr in_cloud_msg,
    const uint8_t traffic_light_type);

  rclcpp::Subscription<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr map_sub_;
  /**
   * @brief publishers
   *
   */
  rclcpp::Publisher<tier4_perception_msgs::msg::TrafficLightArray>::SharedPtr signal_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::map<lanelet::Id, tf2::Vector3> traffic_light_position_map_;
  Config config_;
  /**
   * @brief main class for calculating the occlusion probability
   *
   */
  std::shared_ptr<CloudOcclusionPredictor> cloud_occlusion_predictor_;
  typedef perception_utils::PrimeSynchronizer<
    tier4_perception_msgs::msg::TrafficLightArray, tier4_perception_msgs::msg::TrafficLightRoiArray,
    sensor_msgs::msg::CameraInfo, sensor_msgs::msg::PointCloud2>
    SynchronizerType;

  std::shared_ptr<SynchronizerType> synchronizer_;
  std::shared_ptr<SynchronizerType> synchronizer_ped_;

  std::vector<bool> subscribed_;
  std::vector<int> occlusion_ratios_;
  tier4_perception_msgs::msg::TrafficLightArray out_msg_;
};
}  // namespace autoware::traffic_light
#endif  // NODE_HPP_
