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

#ifndef DETECTED_OBJECT_FILTER__OBJECT_LANELET_FILTER_HPP_
#define DETECTED_OBJECT_FILTER__OBJECT_LANELET_FILTER_HPP_

#include "utils/utils.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

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
  void objectCallback(const autoware_auto_perception_msgs::msg::DetectedObjects::ConstSharedPtr);
  void mapCallback(const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr);

  rclcpp::Publisher<autoware_auto_perception_msgs::msg::DetectedObjects>::SharedPtr object_pub_;
  rclcpp::Subscription<autoware_auto_mapping_msgs::msg::HADMapBin>::SharedPtr map_sub_;
  rclcpp::Subscription<autoware_auto_perception_msgs::msg::DetectedObjects>::SharedPtr object_sub_;

  lanelet::LaneletMapPtr lanelet_map_ptr_;
  lanelet::ConstLanelets road_lanelets_;
  std::string lanelet_frame_id_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  utils::FilterTargetLabel filter_target_;

  LinearRing2d getConvexHull(const autoware_auto_perception_msgs::msg::DetectedObjects &);
  lanelet::ConstLanelets getIntersectedLanelets(
    const LinearRing2d &, const lanelet::ConstLanelets &);
  bool isPolygonOverlapLanelets(const Polygon2d &, const lanelet::ConstLanelets &);
};

}  // namespace object_lanelet_filter

#endif  // DETECTED_OBJECT_FILTER__OBJECT_LANELET_FILTER_HPP_
