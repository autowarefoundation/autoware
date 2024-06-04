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

#ifndef DETECTED_OBJECT_VALIDATION__DETECTED_OBJECT_FILTER__OBJECT_LANELET_FILTER_HPP_
#define DETECTED_OBJECT_VALIDATION__DETECTED_OBJECT_FILTER__OBJECT_LANELET_FILTER_HPP_

#include "detected_object_validation/utils/utils.hpp"

#include <lanelet2_extension/utility/utilities.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/ros/debug_publisher.hpp>
#include <tier4_autoware_utils/ros/published_time_publisher.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/detected_objects.hpp>

#include <lanelet2_core/Forward.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>

namespace object_lanelet_filter
{
using tier4_autoware_utils::LinearRing2d;
using tier4_autoware_utils::MultiPoint2d;
using tier4_autoware_utils::Point2d;
using tier4_autoware_utils::Polygon2d;

class ObjectLaneletFilterNode : public rclcpp::Node
{
public:
  explicit ObjectLaneletFilterNode(const rclcpp::NodeOptions & node_options);

private:
  void objectCallback(const autoware_perception_msgs::msg::DetectedObjects::ConstSharedPtr);
  void mapCallback(const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr);

  rclcpp::Publisher<autoware_perception_msgs::msg::DetectedObjects>::SharedPtr object_pub_;
  rclcpp::Subscription<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr map_sub_;
  rclcpp::Subscription<autoware_perception_msgs::msg::DetectedObjects>::SharedPtr object_sub_;
  std::unique_ptr<tier4_autoware_utils::DebugPublisher> debug_publisher_{nullptr};

  lanelet::LaneletMapPtr lanelet_map_ptr_;
  lanelet::ConstLanelets road_lanelets_;
  lanelet::ConstLanelets shoulder_lanelets_;
  std::string lanelet_frame_id_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  utils::FilterTargetLabel filter_target_;
  struct FilterSettings
  {
    bool polygon_overlap_filter;
    bool lanelet_direction_filter;
    double lanelet_direction_filter_velocity_yaw_threshold;
    double lanelet_direction_filter_object_speed_threshold;
  } filter_settings_;

  bool filterObject(
    const autoware_perception_msgs::msg::DetectedObject & transformed_object,
    const autoware_perception_msgs::msg::DetectedObject & input_object,
    const lanelet::ConstLanelets & intersected_road_lanelets,
    const lanelet::ConstLanelets & intersected_shoulder_lanelets,
    autoware_perception_msgs::msg::DetectedObjects & output_object_msg);
  LinearRing2d getConvexHull(const autoware_perception_msgs::msg::DetectedObjects &);
  lanelet::ConstLanelets getIntersectedLanelets(
    const LinearRing2d &, const lanelet::ConstLanelets &);
  bool isObjectOverlapLanelets(
    const autoware_perception_msgs::msg::DetectedObject & object,
    const lanelet::ConstLanelets & intersected_lanelets);
  bool isPolygonOverlapLanelets(const Polygon2d &, const lanelet::ConstLanelets &);
  bool isSameDirectionWithLanelets(
    const lanelet::ConstLanelets & lanelets,
    const autoware_perception_msgs::msg::DetectedObject & object);
  geometry_msgs::msg::Polygon setFootprint(const autoware_perception_msgs::msg::DetectedObject &);

  std::unique_ptr<tier4_autoware_utils::PublishedTimePublisher> published_time_publisher_;
};

}  // namespace object_lanelet_filter

#endif  // DETECTED_OBJECT_VALIDATION__DETECTED_OBJECT_FILTER__OBJECT_LANELET_FILTER_HPP_
